import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from sensor_msgs.msg import Image
from yolobot_interfaces.msg import Cord, Cords, Labels
 
class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('triggered')
        self.cords_msg = Cords()
        self.labels_msg = Labels()
        self.raw_image_msg = Image()
        self.labeled_image_msg = Image()
 
        self.subscription = self.create_subscription(Int64, '/yolobot/trigger', self.listener_callback, 10)
        self.labeled_image = self.create_subscription(Image, '/yolobot/image_labeled', self.labeled_image_callback, 10)
        self.raw_image = self.create_subscription(Image,'/yolobot/color/image_raw', self.raw_image_callback, 10)
        self.labels = self.create_subscription(Labels, '/yolobot/labels', self.labels_callback, 10)
        self.cords = self.create_subscription(Cords, '/yolobot/cordinates', self.cords_callback, 10)
 
        self.labeled_image_pub = self.create_publisher(Image, '/triggered/image_labeled',10)
        self.raw_image_pub = self.create_publisher(Image,'/triggered/image_raw', 10)
        self.labels_pub = self.create_publisher(Labels, '/triggered/labels', 10)
        self.cords_pub = self.create_publisher(Cords, '/triggered/cordinates', 10)


    def labeled_image_callback(self, msg):
        self.labeled_image_msg = msg
    def raw_image_callback(self, msg):
        self.raw_image_msg = msg
    def labels_callback(self, msg):
        self.labels_msg = msg
    def cords_callback(self, msg):
        self.cords_msg = msg

    def listener_callback(self, msg):
        self.labeled_image_pub.publish(self.labeled_image_msg)
        self.raw_image_pub.publish(self.raw_image_msg)
        self.labels_pub.publish(self.labels_msg)
        self.cords_pub.publish(self.cords_msg)
        self.get_logger().info('--> triggered')


def main(args=None):
  rclpy.init(args=args)
  minimal_subscriber = MinimalSubscriber()
 
  rclpy.spin(minimal_subscriber)
  minimal_subscriber.destroy_node()

  rclpy.shutdown()
 
if __name__ == '__main__':
  main()
