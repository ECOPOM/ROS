import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
 
class MinimalSubscriber(Node):
  def __init__(self):
    # Initiate the Node class's constructor and give it a name
    super().__init__('trigger_node_sub')
 
    # The node subscribes to messages of type std_msgs/String, over a topic named: /trigger and queued messages of 10.
    self.subscription = self.create_subscription(Int8, 'trigger', self.listener_callback, 10)
    self.subscription  # prevent unused variable warning
 
  def listener_callback(self, msg):
    # Display a message on the console every time a message is received on the
    # addison topic
    self.get_logger().info('I heard: "%s"' % msg.data)
 
def main(args=None):
 
  # Initialize the rclpy library
  rclpy.init(args=args)
 
  # Create a subscriber
  minimal_subscriber = MinimalSubscriber()
 
  # Spin the node so the callback function is called.
  # Pull messages from any topics this node is subscribed to.
  rclpy.spin(minimal_subscriber)
 
  # Destroy the node explicitly
  minimal_subscriber.destroy_node()

  # Shutdown the ROS client library for Python
  rclpy.shutdown()
 
if __name__ == '__main__':
  main()
