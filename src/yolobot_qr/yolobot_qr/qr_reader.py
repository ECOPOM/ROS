import cv2
import numpy as np
from pyzbar.pyzbar import decode

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

 
bridge = CvBridge()

class Detectron(Node):
    def __init__(self):
        super().__init__('qr_node')
 
        self.subscription = self.create_subscription(
                Image,
                'color/image_raw',
                self.callback,
                10)
        self.labeled_image = self.create_publisher(Image, 'qr_image', 10)
        self.labels = self.create_publisher(String, 'qr_label', 10)




    def qr_it(self, img):
        barcodes = decode(img)
        labels = []
        for barcode in barcodes:
            labels.append(barcode.data.decode('utf-8'))
            pts = np.array([barcode.polygon], np.int32)
            pts = pts.reshape((-1,1,2))
            cv2.polylines(img, [pts], True, (255,0,255), 5)
        return img, labels


    def callback(self, data):
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
        frame, labels = self.qr_it(frame)
        if len(labels) == 1:
            labeled_frame = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.labeled_image.publish(labeled_frame)
            label_str = String()
            label_str.data = labels[0]
            self.labels.publish(label_str)


def main():
    rclpy.init(args=None)
    print('Hi from yolobot_detection.')
    detector = Detectron()
    rclpy.spin(detector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
