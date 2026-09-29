#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ArucoRectifyPublisher(Node):

    def __init__(self):
        super().__init__('aruco_rectify_publisher')

        self.bridge = CvBridge()

        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )

        self.detector = cv2.aruco.ArucoDetector(
            self.dictionary,
            cv2.aruco.DetectorParameters()
        )

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        self.publisher = self.create_publisher(
            Image,
            '/image_rectified',
            10
        )

        self.output_width = 800
        self.output_height = 600

        self.last_H = None

        self.get_logger().info(
            'ArUco rectification publisher started.'
        )

        self.get_logger().info(
            'Input: /image_raw'
        )

        self.get_logger().info(
            'Output: /image_rectified'
        )

    def order_points(self, pts):

        pts = np.asarray(
            pts,
            dtype=np.float32
        )

        rect = np.zeros(
            (4, 2),
            dtype=np.float32
        )

        s = pts.sum(axis=1)

        diff = np.diff(
            pts,
            axis=1
        ).reshape(-1)

        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]

        return rect

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        clean_frame = frame.copy()

        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY
        )

        corners, ids, rejected = \
            self.detector.detectMarkers(gray)

        marker_centers = {}

        if ids is not None:

            for marker_corners, marker_id in zip(
                corners,
                ids.flatten()
            ):

                marker_id = int(marker_id)

                # Ignore all accidental IDs.
                if marker_id not in [0, 1, 2, 3]:
                    continue

                pts = marker_corners.reshape(
                    4,
                    2
                )

                center = pts.mean(axis=0)

                marker_centers[marker_id] = center

                cv2.polylines(
                    frame,
                    [pts.astype(np.int32)],
                    True,
                    (0, 255, 0),
                    2
                )

                cx = int(center[0])
                cy = int(center[1])

                cv2.putText(
                    frame,
                    f'ID {marker_id}',
                    (cx - 30, cy - 12),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )

        if len(marker_centers) == 4:

            points = np.array(
                list(marker_centers.values()),
                dtype=np.float32
            )

            src = self.order_points(
                points
            )

            # Expand the calibrated tabletop region outward.
            # This gives objects near the edges more visible margin.
            center = src.mean(axis=0)
            scale = 1.30
            src = center + (src - center) * scale

            dst = np.array([
                [0, 0],
                [self.output_width - 1, 0],
                [
                    self.output_width - 1,
                    self.output_height - 1
                ],
                [
                    0,
                    self.output_height - 1
                ]
            ], dtype=np.float32)

            self.last_H = \
                cv2.getPerspectiveTransform(
                    src,
                    dst
                )

            status = 'CALIBRATED'

        else:

            status = (
                'Markers: '
                + str(
                    sorted(
                        marker_centers.keys()
                    )
                )
            )

        cv2.putText(
            frame,
            status,
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2
        )

        cv2.imshow(
            'Original Camera',
            frame
        )

        if self.last_H is not None:

            warped = cv2.warpPerspective(
                clean_frame,
                self.last_H,
                (
                    self.output_width,
                    self.output_height
                )
            )

            # Publish the clean rectified image.
            out_msg = self.bridge.cv2_to_imgmsg(
                warped,
                encoding='bgr8'
            )

            out_msg.header = msg.header
            out_msg.header.frame_id = \
                'rectified_tabletop'

            self.publisher.publish(
                out_msg
            )

            cv2.imshow(
                'Rectified Tabletop',
                warped
            )

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q') or key == 27:
            rclpy.shutdown()


def main():

    rclpy.init()

    node = ArucoRectifyPublisher()

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
