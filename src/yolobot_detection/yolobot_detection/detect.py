import torch
import numpy as np
import cv2
from time import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int8

bridge = CvBridge()

class Detectron(Node):
    def __init__(self):
        super().__init__('detectron_node')
        self.i = 1
        self.labels = None
        self.cords = None

        self.model = torch.hub.load(
            'ultralytics/yolov5',
            'custom', 
            path='/home/ubuntu/ROS/resources/yolov5s.pt',
            force_reload=True
        )
        self.classes = self.model.names
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        print("Using Device: ", self.device)

        self.labeled_image = self.create_publisher(Image, 'image_labeled', 10)
        self.fps = self.create_publisher(Int8, 'fps_processing', 10)
        self.detections = self.create_publisher(Int8, 'detections', 10)
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
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

    def plot_boxes(self, results, frame):
        labels, cord = results
        n = len(labels)
        x_shape, y_shape = frame.shape[1], frame.shape[0]
        for i in range(n):
            row = cord[i]
            if row[4] >= 0.3:
                x1, y1, x2, y2 = int(row[0]*x_shape), int(row[1]*y_shape), int(row[2]*x_shape), int(row[3]*y_shape)
                bgr = (0, 255, 0)
                cv2.rectangle(frame, (x1, y1), (x2, y2), bgr, 2)
                cv2.putText(frame, self.class_to_label(labels[i]), (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.9, bgr, 2)

        return frame

    def pub_each(self, results, frame):
        labels, cord = results
        print(cord)
 
        n = len(labels)
        detections = Int8()
        detections.data = n
        self.detections.publish(detections)


        x_shape, y_shape = frame.shape[1], frame.shape[0]
        for i in range(n):
            row = cord[i]
            if row[4] >= 0.3:
                x1, y1, x2, y2 = int(row[0]*x_shape), int(row[1]*y_shape), int(row[2]*x_shape), int(row[3]*y_shape)
                bgr = (0, 255, 0)
                cv2.rectangle(frame, (x1, y1), (x2, y2), bgr, 2)
                cv2.putText(frame, self.class_to_label(labels[i]), (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.9, bgr, 2)




    def callback(self, data):
        self.i += 1
        print("checking " + str(self.i))
        frame = bridge.imgmsg_to_cv2(data, "bgr8")

        start_time = time()
        results = self.score_frame(frame)
        frame = self.plot_boxes(results, frame)
        end_time = time()

        self.pub_each(results, frame)

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
