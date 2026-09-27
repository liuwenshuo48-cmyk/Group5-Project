import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import os
import cv2

MODEL_TO_GROUP_ID = {0: 1, 1: 0}
MODEL_TO_GROUP_NAME = {0: 'Circle', 1: 'Square'}

class DetectorNode(Node):
    def __init__(self):
        super().__init__('experiment3_detector')

        default_weights = os.path.join(
            get_package_share_directory('experiment3_vision'),
            'weights',
            'best.pt'
        )
        self.declare_parameter('weights_path', default_weights)
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('detection_topic', '/detections')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('show_window', True)

        weights_path = self.get_parameter('weights_path').value
        image_topic = self.get_parameter('image_topic').value
        detection_topic = self.get_parameter('detection_topic').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.show_window = self.get_parameter('show_window').value

        self.get_logger().info(f'Loading model from {weights_path}')
        self.model = YOLO(weights_path)
        self.bridge = CvBridge()

        self.sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.pub = self.create_publisher(Detection2DArray, detection_topic, 10)

        self.get_logger().info(f'Subscribed to {image_topic}, publishing to {detection_topic}')

    def image_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        annotated = cv_image.copy()

        results = self.model.predict(cv_image, conf=self.conf_threshold, verbose=False)

        detection_array = Detection2DArray()
        detection_array.header = msg.header

        for r in results:
            for box in r.boxes:
                model_class_id = int(box.cls[0])
                confidence = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
                w, h = x2 - x1, y2 - y1

                group_class_id = MODEL_TO_GROUP_ID[model_class_id]
                group_class_name = MODEL_TO_GROUP_NAME[model_class_id]

                det = Detection2D()
                det.header = msg.header
                det.bbox.center.position.x = cx
                det.bbox.center.position.y = cy
                det.bbox.center.theta = 0.0
                det.bbox.size_x = w
                det.bbox.size_y = h

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(group_class_id)
                hyp.hypothesis.score = confidence
                det.results.append(hyp)
                detection_array.detections.append(det)

                cv2.rectangle(
                    annotated,
                    (int(x1), int(y1)),
                    (int(x2), int(y2)),
                    (0, 255, 0),
                    1
                )

                label = f'{group_class_name} {confidence:.2f}'

                cv2.putText(
                    annotated,
                    label,
                    (int(x1), max(int(y1) - 4, 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.38,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA
                )

        self.pub.publish(detection_array)

        if self.show_window:
            cv2.imshow('Experiment3 Detection', annotated)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
