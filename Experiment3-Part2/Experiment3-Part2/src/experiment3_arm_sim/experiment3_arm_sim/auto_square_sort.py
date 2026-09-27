#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from experiment3_interfaces.action import PickAndPlace


class AutoSquareSort(Node):

    def __init__(self):
        super().__init__('auto_square_sort')

        self.client = ActionClient(
            self,
            PickAndPlace,
            '/pick_and_place'
        )

        # G1 -> Bin1, G2 -> Bin1, G3 -> Bin1
        self.tasks = [
            (1, 1),
            (2, 1),
            (3, 1),
        ]

        self.task_index = 0

        self.get_logger().info(
            'AUTO SQUARE SORT READY | G1 -> G2 -> G3'
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
                'DONE: G1, G2, G3 sorting completed.'
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

        self.get_logger().info(
            f'TASK {self.task_index + 1} ACCEPTED'
        )

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

        if not result.success:
            grid_id, bin_id = self.tasks[self.task_index]

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

        grid_id, bin_id = self.tasks[self.task_index]

        self.get_logger().info(
            f'SUCCESS: Grid {grid_id} -> Bin {bin_id}'
        )

        # The Action server has already returned the robot HOME
        # before reporting success.
        self.get_logger().info(
            f'Grid {grid_id} finished and robot is HOME.'
        )

        self.task_index += 1

        self.send_next_task()


def main(args=None):

    rclpy.init(args=args)

    node = AutoSquareSort()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
