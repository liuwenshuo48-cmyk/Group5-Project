#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ArucoRectify(Node):

    def __init__(self):
        super().__init__('aruco_rectify')

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

        # 输出的统一桌面坐标系
        self.output_width = 800
        self.output_height = 600

        # 保存最近一次有效的透视变换
        self.last_H = None

        self.get_logger().info(
            'ArUco tabletop rectification started.'
        )

    def order_points(self, pts):
        """
        将四个点自动排列成：
        top-left, top-right, bottom-right, bottom-left
        """

        pts = np.asarray(pts, dtype=np.float32)

        rect = np.zeros((4, 2), dtype=np.float32)

        s = pts.sum(axis=1)
        diff = np.diff(pts, axis=1).reshape(-1)

        rect[0] = pts[np.argmin(s)]       # TL
        rect[2] = pts[np.argmax(s)]       # BR
        rect[1] = pts[np.argmin(diff)]    # TR
        rect[3] = pts[np.argmax(diff)]    # BL

        return rect

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY
        )

        corners, ids, rejected = self.detector.detectMarkers(gray)

        marker_centers = {}

        if ids is not None:

            for marker_corners, marker_id in zip(
                corners,
                ids.flatten()
            ):

                marker_id = int(marker_id)

                # 只接受我们自己的四个 Marker
                if marker_id not in [0, 1, 2, 3]:
                    continue

                pts = marker_corners.reshape(4, 2)

                center = pts.mean(axis=0)

                marker_centers[marker_id] = center

                # 原图显示
                cv2.polylines(
                    frame,
                    [pts.astype(np.int32)],
                    True,
                    (0, 255, 0),
                    2
                )

                cx = int(center[0])
                cy = int(center[1])

                cv2.circle(
                    frame,
                    (cx, cy),
                    5,
                    (0, 0, 255),
                    -1
                )

                cv2.putText(
                    frame,
                    f'ID {marker_id}',
                    (cx - 30, cy - 12),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )

        # 四个 Marker 同时出现时重新计算透视变换
        if len(marker_centers) == 4:

            points = np.array(
                list(marker_centers.values()),
                dtype=np.float32
            )

            src = self.order_points(points)

            dst = np.array([
                [0, 0],
                [self.output_width - 1, 0],
                [self.output_width - 1, self.output_height - 1],
                [0, self.output_height - 1]
            ], dtype=np.float32)

            self.last_H = cv2.getPerspectiveTransform(
                src,
                dst
            )

            status_text = 'CALIBRATED - 4 markers'

        else:

            status_text = (
                f'Visible markers: '
                f'{sorted(marker_centers.keys())}'
            )

        cv2.putText(
            frame,
            status_text,
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

        # 只要曾经成功得到 H，
        # 后面即使临时掉一个 Marker 也继续使用
        if self.last_H is not None:

            warped = cv2.warpPerspective(
                frame,
                self.last_H,
                (
                    self.output_width,
                    self.output_height
                )
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

    node = ArucoRectify()

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
