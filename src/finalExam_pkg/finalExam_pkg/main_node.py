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
        qos_profile = QoSProfile(depth=50)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', qos_profile)
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
        self.get_logger().info('Moving foward for 5 seconds')
        msg.linear.x = -40.0
        msg.angular.z = -40.0
        self.publisher_.publish(msg)
        time.sleep(5.0)

        self.get_logger().info('Turning left for 3 seconds')
        msg.linear.x = 20.0
        msg.angular.z = -20.0
        self.publisher_.publish(msg)
        time.sleep(3.0)

        self.get_logger().info('Moving foward for 5 seconds')
        msg.linear.x = -40.0
        msg.angular.z = -40.0
        self.publisher_.publish(msg)
        time.sleep(5.0)

        self.get_logger().info('Turning right for 3 seconds')
        msg.linear.x = -20.0
        msg.angular.z = 20.0
        self.publisher_.publish(msg)
        time.sleep(5.0)

        self.get_logger().info('Moving foward for 5 seconds')
        msg.linear.x = -40.0
        msg.angular.z = -40.0
        self.publisher_.publish(msg)
        time.sleep(5.0)

        self.get_logger().info('Turning left for 3 seconds')
        msg.linear.x = 20.0
        msg.angular.z = -20.0
        self.publisher_.publish(msg)
        time.sleep(3.0)

        self.get_logger().info('Moving foward for 5 seconds')
        msg.linear.x = -40.0
        msg.angular.z = -40.0
        self.publisher_.publish(msg)
        time.sleep(5.0)

        # Display the message on the console
        self.get_logger().info('I will catch the trash :D!')
        msg.linear.x =  0.0
        msg.linear.y =  22.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        time.sleep(30.0)


        


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
