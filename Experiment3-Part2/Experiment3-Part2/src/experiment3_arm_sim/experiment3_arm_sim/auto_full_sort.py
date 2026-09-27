#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from experiment3_interfaces.action import PickAndPlace


class AutoFullSort(Node):

    def __init__(self):
        super().__init__('auto_full_sort')

        self.client = ActionClient(
            self,
            PickAndPlace,
            '/pick_and_place'
        )

        self.tasks = [
            (1, 1),
            (2, 1),
            (3, 1),
            (4, 2),
            (5, 2),
            (6, 2),
        ]

        self.index = 0
        self.success_count = 0

        self.get_logger().info(
            'AUTO FULL SORT READY | G1-G6'
        )

        self.client.wait_for_server()

        self.get_logger().info(
            'PickAndPlace Action server connected.'
        )

        self.send_next()

    def send_next(self):

        if self.index >= len(self.tasks):
            self.get_logger().info(
                '========================================'
            )
            self.get_logger().info(
                f'DONE: {self.success_count}/6 objects sorted.'
            )
            self.get_logger().info(
                'G1 G2 G3 -> Bin1'
            )
            self.get_logger().info(
                'G4 G5 G6 -> Bin2'
            )
            self.get_logger().info(
                'Robot returned HOME.'
            )
            self.get_logger().info(
                'PART 2 FULL SORT COMPLETE'
            )
            self.get_logger().info(
                '========================================'
            )

            rclpy.shutdown()
            return

        grid_id, bin_id = self.tasks[self.index]

        self.get_logger().info(
            f'START {self.index + 1}/6 | '
            f'Grid {grid_id} -> Bin {bin_id}'
        )

        goal = PickAndPlace.Goal()
        goal.grid_id = grid_id
        goal.bin_id = bin_id

        future = self.client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(
                f'TASK {self.index + 1} REJECTED'
            )
            rclpy.shutdown()
            return

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.result_callback
        )

    def feedback_callback(self, feedback_msg):

        self.get_logger().info(
            f'Feedback: {feedback_msg.feedback.state}'
        )

    def result_callback(self, future):

        result = future.result().result

        grid_id, bin_id = self.tasks[self.index]

        if not result.success:
            self.get_logger().error(
                f'FAILED: Grid {grid_id} -> Bin {bin_id}'
            )
            self.get_logger().error(
                result.message
            )
            self.get_logger().error(
                'Automatic full sequence stopped.'
            )

            rclpy.shutdown()
            return

        self.success_count += 1

        self.get_logger().info(
            f'SUCCESS: Grid {grid_id} -> Bin {bin_id}'
        )

        self.get_logger().info(
            f'Progress: {self.success_count}/6'
        )

        self.index += 1
        self.send_next()


def main(args=None):

    rclpy.init(args=args)

    node = AutoFullSort()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
