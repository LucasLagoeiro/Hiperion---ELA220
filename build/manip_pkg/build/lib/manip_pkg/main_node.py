import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the LaserScan module from sensor_msgs interface
from geometry_msgs.msg import Twist
# import Quality of Service library, to set the correct profile and reliability to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile
from .submodules.robot import Robot
import time


class SimpleSubscriber(Node):

    def __init__(self):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        self.robot = Robot()
        super().__init__('simple_subscriber')
        # create the subscriber object
        # in this case, the subscriptor will be subscribed on /scan topic with a queue size of 10 messages.
        # use the LaserScan module for /scan topic
        # send the received info to the listener_callback method.
        qos_profile = QoSProfile(depth=100)
        self.subscription_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            qos_profile)
        self.subscription_cmd_vel




    def listener_callback(self, msg):
        # print the log info in the terminal     
        self.get_logger().info('I receive1: "%s"' % str(msg.linear.x))
        if(msg.linear.y > 0):
            self.get_logger().info('I will catch!')
            self.robot.set_all_joints_angle([90,90,90,90])
            self.robot.set_all_joints_angle([60,10,190,90])
            self.robot.set_all_joints_angle([60,10,190,0])
            time.sleep(2)
            self.robot.set_all_joints_angle([140,10,190,0])
        


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    simple_subscriber = SimpleSubscriber()
    while rclpy.ok():
        rclpy.spin(simple_subscriber)
    
    # Explicity destroy the node
    simple_subscriber.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
