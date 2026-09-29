#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ArucoCheck(Node):

    def __init__(self):
        super().__init__('aruco_check')

        self.bridge = CvBridge()

        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )

        self.parameters = cv2.aruco.DetectorParameters()

        self.detector = cv2.aruco.ArucoDetector(
            self.dictionary,
            self.parameters
        )

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        self.last_ids = None

        self.get_logger().info(
            'ArUco checker started. Listening to /image_raw'
        )

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = self.detector.detectMarkers(gray)

        detected_ids = []

        if ids is not None:

            detected_ids = sorted(ids.flatten().tolist())

            cv2.aruco.drawDetectedMarkers(
                frame,
                corners,
                ids
            )

            for marker_corners, marker_id in zip(
                corners,
                ids.flatten()
            ):

                pts = marker_corners.reshape(4, 2)

                cx = int(pts[:, 0].mean())
                cy = int(pts[:, 1].mean())

                cv2.putText(
                    frame,
                    f'ID {marker_id}',
                    (cx - 30, cy - 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )

        if detected_ids != self.last_ids:
            self.get_logger().info(
                f'Detected marker IDs: {detected_ids}'
            )
            self.last_ids = detected_ids

        cv2.putText(
            frame,
            f'Detected: {detected_ids}',
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2
        )

        cv2.imshow('ArUco Check', frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q') or key == 27:
            rclpy.shutdown()


def main():

    rclpy.init()

    node = ArucoCheck()

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
