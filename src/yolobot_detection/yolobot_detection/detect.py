import torch
import numpy as np
import os
import cv2
from time import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int8


from yolobot_interfaces.msg import Cord, Cords, Labels

bridge = CvBridge()


class Detectron(Node):
    def __init__(self):
        super().__init__('detectron_node')
        self.i = 1

        self.declare_parameters(
            namespace='',
            parameters=[
                ('detections', "apples"),
                ('apple_weights', "/home/bresilla/ROS/resources/yolov5s.pt")
            ])

        detections_param = self.get_parameter('apple_weights').get_parameter_value().string_value

        self.model = torch.hub.load(
            'ultralytics/yolov5',
            'custom',
            path=detections_param,
            force_reload=True
        )
        self.classes = self.model.names
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        print("Using Device: ", self.device)

        self.labeled_image = self.create_publisher(Image, 'image_labeled', 10)
        self.labels = self.create_publisher(Labels, 'labels', 10)
        self.cords = self.create_publisher(Cords, 'cordinates', 10)
        self.fps = self.create_publisher(Int8, 'fps_processing', 10)
        self.detections = self.create_publisher(Int8, 'detections', 10)
        self.subscription = self.create_subscription(
            Image,
            'color/image_raw',
            self.callback,
            10)

    def score_frame(self, frame):
        self.model.to(self.device)
        frame = [frame]
        results = self.model(frame)
        labels, cord = results.xyxyn[0][:, -1], results.xyxyn[0][:, :-1]
        return labels, cord

    def class_to_label(self, x):
        return self.classes[int(x)]

    def pub_each(self, results, frame):
        n = len(results[0])
        detections = Int8()
        detections.data = n
        self.detections.publish(detections)

        labels_msg = Labels()
        cords = Cords()
        cord = Cord()
        x_shape, y_shape = frame.shape[1], frame.shape[0]
        for i in range(n):
            row = results[1][i]
            if row[4] >= 0.3:
                x1, y1, x2, y2 = int(row[0]*x_shape), int(row[1]*y_shape), int(row[2]*x_shape), int(row[3]*y_shape)
                bgr = (0, 255, 0)
                cord.x1 = x1
                cord.x2 = x2
                cord.y1 = y1
                cord.y2 = y2
                cords.cords.append(cord)
                cv2.rectangle(frame, (x1, y1), (x2, y2), bgr, 2)
                label_string = self.class_to_label(results[0][i])
                labels_msg.labels.append(label_string)
                cv2.putText(frame, label_string, (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.9, bgr, 2)
        self.labels.publish(labels_msg)
        self.cords.publish(cords)
        return frame

    def callback(self, data):
        self.i += 1
        print("checking " + str(self.i))
        frame = bridge.imgmsg_to_cv2(data, "bgr8")

        start_time = time()
        results = self.score_frame(frame)
        frame = self.pub_each(results, frame)
        end_time = time()

        labeled_frame = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.labeled_image.publish(labeled_frame)

        fps_data = Int8()
        fps_data.data = int(1/np.round(end_time - start_time, 2))
        self.fps.publish(fps_data)


def main():
    rclpy.init(args=None)
    print('Hi from yolobot_detection.')
    detector = Detectron()
    rclpy.spin(detector)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
