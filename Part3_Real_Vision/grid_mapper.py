#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection2DArray


GRIDS = {
    "G1": (224.6,131.7),
    "G2": (404.3,130.4),
    "G3": (589.4,121.9),
    "G4": (202.1,435.1),
    "G5": (403.2,349.0),
    "G6": (603.5,347.8)
}


class GridMapper(Node):

    def __init__(self):
        super().__init__("grid_mapper")

        self.sub = self.create_subscription(
            Detection2DArray,
            "/detections",
            self.callback,
            10
        )

        self.get_logger().info(
            "Grid mapper started"
        )


    def callback(self,msg):

        for det in msg.detections:

            if len(det.results)==0:
                continue

            cls = det.results[0].hypothesis.class_id
            score = det.results[0].hypothesis.score

            x = det.bbox.center.position.x
            y = det.bbox.center.position.y


            best_grid=None
            best_dist=99999


            for name,(gx,gy) in GRIDS.items():

                d=((x-gx)**2+(y-gy)**2)**0.5

                if d < best_dist:
                    best_dist=d
                    best_grid=name


            self.get_logger().info(
                f"class={cls}, score={score:.2f}, "
                f"pos=({x:.1f},{y:.1f}) -> {best_grid}, "
                f"distance={best_dist:.1f}px"
            )


def main():

    rclpy.init()

    node=GridMapper()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__=="__main__":
    main()
