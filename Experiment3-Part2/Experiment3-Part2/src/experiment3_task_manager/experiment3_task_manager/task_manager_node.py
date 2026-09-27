#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from vision_msgs.msg import Detection2DArray
from experiment3_interfaces.action import PickAndPlace


class Experiment3TaskManager(Node):

    def __init__(self):
        super().__init__('experiment3_task_manager')

        # --------------------------------------------------
        # Configurable image size.
        # Part 1 currently uses the agreed 640x480 grid model.
        # These parameters can be changed later without changing code.
        # --------------------------------------------------
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)

        self.image_width = int(
            self.get_parameter('image_width').value
        )
        self.image_height = int(
            self.get_parameter('image_height').value
        )

        # --------------------------------------------------
        # Part 1 -> Part 2 class mapping.
        #
        # IMPORTANT:
        # Part 1 already converts YOLO internal classes.
        # Published /detections uses:
        #   "0" = Square
        #   "1" = Circle
        #
        # DO NOT invert again here.
        # --------------------------------------------------
        self.class_to_bin = {
            '0': 1,   # Square -> Bin1
            '1': 2,   # Circle -> Bin2
        }

        self.class_names = {
            '0': 'Square',
            '1': 'Circle',
        }

        # --------------------------------------------------
        # State machine
        # --------------------------------------------------
        self.state = 'WAITING'

        # Grids already successfully processed.
        self.processed_grids = set()

        # Prevent multiple simultaneous Action requests.
        self.busy = False

        self.current_grid = None
        self.current_bin = None
        self.current_class = None

        # --------------------------------------------------
        # PickAndPlace Action client
        # --------------------------------------------------
        self.action_client = ActionClient(
            self,
            PickAndPlace,
            '/pick_and_place'
        )

        # --------------------------------------------------
        # Detection subscriber
        # --------------------------------------------------
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10
        )

        self.get_logger().info(
            '=============================================='
        )
        self.get_logger().info(
            'Experiment 3 Task Manager READY'
        )
        self.get_logger().info(
            f'Grid image size: '
            f'{self.image_width}x{self.image_height}'
        )
        self.get_logger().info(
            'Class mapping: 0=Square->Bin1, 1=Circle->Bin2'
        )
        self.get_logger().info(
            'State: WAITING'
        )
        self.get_logger().info(
            '=============================================='
        )

    # ======================================================
    # Utility: state transition
    # ======================================================
    def set_state(self, new_state):
        old_state = self.state
        self.state = new_state

        self.get_logger().info(
            f'STATE: {old_state} -> {new_state}'
        )

    # ======================================================
    # bbox center -> Grid ID
    #
    # Layout:
    #
    #   Grid1 | Grid2 | Grid3
    #   ------ | ----- | -----
    #   Grid4 | Grid5 | Grid6
    #
    # ======================================================
    def bbox_to_grid(self, x, y):

        if (
            x < 0 or
            y < 0 or
            x >= self.image_width or
            y >= self.image_height
        ):
            self.get_logger().error(
                f'OUT_OF_RANGE: bbox center '                 f'({x:.1f}, {y:.1f}) outside '                 f'{self.image_width}x{self.image_height}'
            )
            return None

        # Final Part 2 simulation calibration for the angled
        # 640x480 overhead camera.
        grid_centers = {
            1: (371.2, 218.1),
            2: (317.7, 207.8),
            3: (268.0, 194.5),
            4: (347.8, 256.6),
            5: (290.9, 242.3),
            6: (242.6, 227.5),
        }

        nearest_grid = None
        nearest_dist2 = float('inf')

        for grid_id, (gx, gy) in grid_centers.items():
            dist2 = (x - gx) ** 2 + (y - gy) ** 2
            if dist2 < nearest_dist2:
                nearest_dist2 = dist2
                nearest_grid = grid_id

        # Accept detections only when they are sufficiently close
        # to one of the calibrated pickup-grid centers.
        if nearest_dist2 > 30.0 ** 2:
            return None

        return nearest_grid

    # ======================================================
    # Detection callback
    # ======================================================
    def detection_callback(self, msg):

        # Action is already executing.
        if self.busy:
            return

        # All six grids have completed.
        if len(self.processed_grids) >= 6:
            if self.state != 'DONE':
                self.set_state('DONE')
                self.get_logger().info(
                    '=============================================='
                )
                self.get_logger().info(
                    'DONE: all 6 grids processed successfully.'
                )
                self.get_logger().info(
                    '=============================================='
                )
            return

        # No object detected in this frame.
        if not msg.detections:
            return

        # --------------------------------------------------
        # Collect valid, unprocessed detections.
        # The final scheduler handles grids in fixed G1->G6 order.
        # --------------------------------------------------
        candidates = []

        for detection in msg.detections:

            if not detection.results:
                continue

            result = detection.results[0]

            class_id = str(
                result.hypothesis.class_id
            )

            score = float(
                result.hypothesis.score
            )

            # Part 1 publishes bbox center through:
            # bbox.center.position.x / y
            x = float(
                detection.bbox.center.position.x
            )
            y = float(
                detection.bbox.center.position.y
            )

            grid_id = self.bbox_to_grid(x, y)

            # Ignore detections that do not belong to a valid pickup Grid.
            if grid_id is None:
                continue

            # Unknown class exception.
            if class_id not in self.class_to_bin:
                self.get_logger().error(
                    f'UNKNOWN: class_id={class_id} '
                    f'at Grid {grid_id}'
                )
                continue

            # Do not process a grid twice.
            if grid_id in self.processed_grids:
                continue

            candidates.append(
                (
                    score,
                    grid_id,
                    class_id,
                    x,
                    y
                )
            )

        if not candidates:
            return

        # Fixed execution order:
        # G1 -> G2 -> G3 -> G4 -> G5 -> G6
        expected_grid = next(
            (
                grid
                for grid in (1, 2, 3, 4, 5, 6)
                if grid not in self.processed_grids
            ),
            None
        )

        if expected_grid is None:
            return

        # Only accept detections belonging to the current expected Grid.
        ordered_candidates = [
            item
            for item in candidates
            if item[1] == expected_grid
        ]

        if not ordered_candidates:
            return

        # If several detections fall inside the same expected Grid,
        # use the highest-confidence one only inside that Grid.
        ordered_candidates.sort(
            key=lambda item: item[0],
            reverse=True
        )

        score, grid_id, class_id, x, y = ordered_candidates[0]

        bin_id = self.class_to_bin[class_id]
        class_name = self.class_names[class_id]

        self.current_grid = grid_id
        self.current_bin = bin_id
        self.current_class = class_id

        self.set_state('TARGET_SELECTED')

        self.get_logger().info(
            f'TARGET_SELECTED | '
            f'Grid {grid_id} | '
            f'{class_name} | '
            f'class_id={class_id} | '
            f'Bin {bin_id} | '
            f'score={score:.3f} | '
            f'bbox=({x:.1f},{y:.1f})'
        )

        self.send_pick_and_place(
            grid_id,
            bin_id
        )

    # ======================================================
    # Send PickAndPlace Action
    # ======================================================
    def send_pick_and_place(self, grid_id, bin_id):

        if not self.action_client.server_is_ready():
            self.get_logger().warning(
                '/pick_and_place Action server not ready.'
            )
            self.set_state('WAITING')
            return

        self.busy = True
        self.set_state('REQUEST_PICK')

        goal = PickAndPlace.Goal()
        goal.grid_id = int(grid_id)
        goal.bin_id = int(bin_id)

        self.get_logger().info(
            f'REQUEST_PICK | '
            f'Grid {grid_id} -> Bin {bin_id}'
        )

        future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(
            self.goal_response_callback
        )

    # ======================================================
    # Action goal response
    # ======================================================
    def goal_response_callback(self, future):

        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(
                f'ACTION_ERROR: {exc}'
            )
            self.busy = False
            self.set_state('WAITING')
            return

        if not goal_handle.accepted:
            self.get_logger().error(
                'GRASP_FAILED: Action goal rejected'
            )
            self.busy = False
            self.set_state('GRASP_FAILED')
            self.set_state('WAITING')
            return

        self.set_state('WAIT_RESULT')

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.result_callback
        )

    # ======================================================
    # Action feedback
    # ======================================================
    def feedback_callback(self, feedback_msg):

        feedback = feedback_msg.feedback

        self.get_logger().info(
            f'ACTION_FEEDBACK | {feedback.state}'
        )

    # ======================================================
    # Action result
    # ======================================================
    def result_callback(self, future):

        grid_id = self.current_grid
        bin_id = self.current_bin

        try:
            wrapped_result = future.result()
            result = wrapped_result.result

        except Exception as exc:
            self.get_logger().error(
                f'ACTION_RESULT_ERROR: {exc}'
            )

            self.busy = False
            self.set_state('GRASP_FAILED')
            self.set_state('WAITING')
            return

        if result.success:

            self.processed_grids.add(grid_id)

            self.set_state('SUCCESS')

            self.get_logger().info(
                f'SUCCESS | '
                f'Grid {grid_id} -> Bin {bin_id} | '
                f'processed={sorted(self.processed_grids)}'
            )

            self.busy = False

            if len(self.processed_grids) >= 6:
                self.set_state('DONE')

                self.get_logger().info(
                    '=============================================='
                )
                self.get_logger().info(
                    'DONE: 6/6 grids processed.'
                )
                self.get_logger().info(
                    '=============================================='
                )
            else:
                self.set_state('WAITING')

        else:

            message = result.message

            self.get_logger().error(
                f'GRASP_FAILED | '
                f'Grid {grid_id} -> Bin {bin_id} | '
                f'{message}'
            )

            self.busy = False
            self.set_state('GRASP_FAILED')

            # Failed grid is NOT added to processed_grids.
            # A later detection may retry it.
            self.set_state('WAITING')


def main(args=None):

    rclpy.init(args=args)

    node = Experiment3TaskManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
