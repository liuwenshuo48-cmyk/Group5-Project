#!/usr/bin/env python3

import cv2
import json
import rclpy

from pathlib import Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class GridCalibrator(Node):

    def __init__(self):
        super().__init__('grid_calibrator')

        self.bridge = CvBridge()
        self.frame = None
        self.points = []

        self.output_file = (
            Path.home()
            / "Exp_3_group_5"
            / "Part3_Real_Vision"
            / "real_grid_points.json"
        )

        self.subscription = self.create_subscription(
            Image,
            '/image_rectified',
            self.image_callback,
            qos_profile_sensor_data
        )

        cv2.namedWindow('Grid Calibration')
        cv2.setMouseCallback(
            'Grid Calibration',
            self.mouse_callback
        )

        self.get_logger().info(
            'Click grid centers in order: '
            'G1 G2 G3 G4 G5 G6'
        )

    def image_callback(self, msg):

        self.frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        display = self.frame.copy()

        # Draw clicked points
        for i, (x, y) in enumerate(self.points):
            grid_id = i + 1

            cv2.circle(
                display,
                (x, y),
                8,
                (0, 0, 255),
                -1
            )

            cv2.putText(
                display,
                f'G{grid_id}',
                (x + 10, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        cv2.putText(
            display,
            'Click: G1 G2 G3 G4 G5 G6',
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2
        )

        cv2.putText(
            display,
            'R = reset, S = save, Q = quit',
            (15, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2
        )

        cv2.imshow(
            'Grid Calibration',
            display
        )

        key = cv2.waitKey(1) & 0xFF

        if key == ord('r'):
            self.points = []
            self.get_logger().info(
                'Grid points reset.'
            )

        elif key == ord('s'):
            self.save_points()

        elif key == ord('q') or key == 27:
            rclpy.shutdown()

    def mouse_callback(
        self,
        event,
        x,
        y,
        flags,
        param
    ):

        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if len(self.points) >= 6:
            self.get_logger().warn(
                'Already have 6 points. '
                'Press R to reset.'
            )
            return

        self.points.append((x, y))

        grid_id = len(self.points)

        self.get_logger().info(
            f'G{grid_id} = ({x}, {y})'
        )

    def save_points(self):

        if len(self.points) != 6:
            self.get_logger().warn(
                f'Need 6 points, currently have '
                f'{len(self.points)}.'
            )
            return

        data = {}

        for i, (x, y) in enumerate(self.points):
            data[f'G{i+1}'] = {
                'x': int(x),
                'y': int(y)
            }

        self.output_file.write_text(
            json.dumps(
                data,
                indent=2
            )
        )

        self.get_logger().info(
            f'Saved to {self.output_file}'
        )


def main():

    rclpy.init()

    node = GridCalibrator()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()
        cv2.destroyAllWindows()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
