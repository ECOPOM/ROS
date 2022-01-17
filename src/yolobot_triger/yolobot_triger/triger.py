import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
 
class MinimalPublisher(Node):
  def __init__(self):
    # Initiate the Node class's constructor and give it a name
    super().__init__('trigger_node')

    # Create the publisher. This publisher will publish a Int8 message to the trigger topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(Int8, 'trigger', 10)

    # We will publish a message every x seconds
    timer_period = 1  # seconds

    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)

    # Initialize a counter variable
    self.i = 1
 
  def timer_callback(self):
    # Create a Int8 message
    msg = Int8()
 
    # Set the Int8 message's data
    msg.data = self.i

    # Publish the message to the topic
    self.publisher_.publish(msg)

    # Display the message on the console
    self.get_logger().info('Publishing: "%s"' % msg.data)
 
class MinimalPublisher_2(Node):
  def __init__(self):
    # Initiate the Node class's constructor and give it a name
    super().__init__('counter_node')

    # Create the publisher. This publisher will publish a Int8 message to the trigger topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(Int8, 'counter', 10)

    # We will publish a message every x seconds
    timer_period = 1  # seconds

    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)

    # Initialize a counter variable
    self.i = 0
 
  def timer_callback(self):
    # Create a Int8 message
    msg = Int8()
 
    # Set the Int8 message's data
    msg.data = self.i

    # Publish the message to the topic
    self.publisher_.publish(msg)

    # Display the message on the console
    self.get_logger().info('Publishing: "%s"' % msg.data)

    self.i += 1
 

def main(args=None):
  rclpy.init(args=args)
 
  # Create the node
  counter_node = MinimalPublisher_2()
  trigger_node = MinimalPublisher()
 
  # Spin the node so the callback function is called.
  # rclpy.spin(trigger_node)
  try:
    while rclpy.ok():
        counter_node.timer_callback()
        trigger_node.timer_callback()
        time.sleep(1)
  except KeyboardInterrupt:
    pass
  counter_node.destroy_node()
  trigger_node.destroy_node()
 
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
 
if __name__ == '__main__':
  main()
