#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from experiment3_interfaces.action import PickAndPlace


class AutoCircleSort(Node):

    def __init__(self):
        super().__init__('auto_circle_sort')

        self.client = ActionClient(
            self,
            PickAndPlace,
            '/pick_and_place'
        )

        # G4 -> Bin2, G5 -> Bin2, G6 -> Bin2
        self.tasks = [
            (4, 2),
            (5, 2),
            (6, 2),
        ]

        self.task_index = 0

        self.get_logger().info(
            'AUTO CIRCLE SORT READY | G4 -> G5 -> G6'
        )

        self.client.wait_for_server()

        self.get_logger().info(
            'PickAndPlace Action server connected.'
        )

        self.send_next_task()

    def send_next_task(self):

        if self.task_index >= len(self.tasks):
            self.get_logger().info(
                '======================================'
            )
            self.get_logger().info(
                'DONE: G4, G5, G6 sorting completed.'
            )
            self.get_logger().info(
                'Robot returned HOME after final object.'
            )
            self.get_logger().info(
                '======================================'
            )

            rclpy.shutdown()
            return

        grid_id, bin_id = self.tasks[self.task_index]

        self.get_logger().info(
            f'START TASK {self.task_index + 1}/3 | '
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
                f'TASK {self.task_index + 1} REJECTED'
            )
            rclpy.shutdown()
            return

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.result_callback
        )

    def feedback_callback(self, feedback_msg):

        state = feedback_msg.feedback.state

        self.get_logger().info(
            f'Feedback: {state}'
        )

    def result_callback(self, future):

        result = future.result().result

        grid_id, bin_id = self.tasks[self.task_index]

        if not result.success:
            self.get_logger().error(
                f'FAILED: Grid {grid_id} -> Bin {bin_id}'
            )
            self.get_logger().error(
                result.message
            )
            self.get_logger().error(
                'Automatic sequence stopped for safety.'
            )

            rclpy.shutdown()
            return

        self.get_logger().info(
            f'SUCCESS: Grid {grid_id} -> Bin {bin_id}'
        )

        self.get_logger().info(
            f'Grid {grid_id} finished and robot is HOME.'
        )

        self.task_index += 1

        self.send_next_task()


def main(args=None):

    rclpy.init(args=args)

    node = AutoCircleSort()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
