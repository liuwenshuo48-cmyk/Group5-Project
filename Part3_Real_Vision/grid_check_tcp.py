#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray


class GridCheck(Node):

    def __init__(self):
        super().__init__('real_grid_check')

        self.subscription = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.callback,
            10
        )

        self.candidate_key = None
        self.stable_count = 0
        self.last_printed_key = None

        # 连续稳定多少帧后才确认
        self.required_stable_frames = 8

        self.get_logger().info(
            'Real Grid checker started. '
            'Stable detection mode enabled. '
            'This node does NOT control the robot.'
        )

    def bbox_to_grid(self, x, y):

        if x < 140 or x > 677 or y < 29 or y > 587:
            return None

        if y < 308:
            if x < 315:
                return 1
            elif x < 494:
                return 2
            else:
                return 3
        else:
            if x < 315:
                return 4
            elif x < 494:
                return 5
            else:
                return 6

    def callback(self, msg):

        if not msg.detections:
            self.candidate_key = None
            self.stable_count = 0
            return

        # 当前测试只取置信度最高的检测
        best_det = None
        best_score = -1.0

        for det in msg.detections:
            if not det.results:
                continue

            score = det.results[0].hypothesis.score

            if score > best_score:
                best_score = score
                best_det = det

        if best_det is None:
            return

        hypothesis = best_det.results[0].hypothesis

        class_id = hypothesis.class_id
        score = hypothesis.score

        x = best_det.bbox.center.position.x
        y = best_det.bbox.center.position.y

        grid_id = self.bbox_to_grid(x, y)

        if grid_id is None:
            self.candidate_key = None
            self.stable_count = 0
            return

        if class_id == '0':
            class_name = 'Square'
        elif class_id == '1':
            class_name = 'Circle'
        else:
            class_name = f'Unknown({class_id})'

        current_key = (class_id, grid_id)

        # 连续帧稳定判断
        if current_key == self.candidate_key:
            self.stable_count += 1
        else:
            self.candidate_key = current_key
            self.stable_count = 1

        # 连续稳定足够多帧，并且不是刚刚打印过的结果
        if (
            self.stable_count >= self.required_stable_frames
            and current_key != self.last_printed_key
        ):
            self.get_logger().info(
                f'{class_name} '
                f'score={score:.3f} '
                f'center=({x:.1f},{y:.1f}) '
                f'-> G{grid_id}'
            )

            self.last_printed_key = current_key


def main():

    rclpy.init()

    node = GridCheck()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
