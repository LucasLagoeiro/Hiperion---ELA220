import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the LaserScan module from sensor_msgs interface
from geometry_msgs.msg import Twist
# import Quality of Service library, to set the correct profile and reliability to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile
import time


class VelPublisher(Node):

    def __init__(self):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('VelPublisher')
        # create the publisher object
        # in this case, the publisher will publish on /cmd_vel Topic with a queue size of 10 messages.
        # use the Twist module for /cmd_vel Topic
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # define the timer period for 0.5 seconds
        timer_period = 0.5
        # create a timer sending two parameters:
        # - the duration between 2 callbacks (0.5 seconds)
        # - the timer function (timer_callback)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Here you have the callback method
        # create a Twist message
        msg = Twist()
        # define the linear x-axis velocity of /cmd_vel Topic parameter to 0.5
        msg.linear.x = -40.0
        # define the angular z-axis velocity of /cmd_vel Topic parameter to 0.5
        msg.angular.z = -40.0
        # Publish the message to the Topic
        self.publisher_.publish(msg)
        # Display the message on the console
        self.get_logger().info('Publishing: "%s"' % msg)


        


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    simple_velPublisher= VelPublisher()
    while rclpy.ok():
        rclpy.spin(simple_velPublisher)
    
    # Explicity destroy the node
    simple_velPublisher.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
