from robot import Robot
import time


r3_robot = Robot()
r3_robot.home_position()
# r3_robot.set_all_joints_angle([0, 0, 0])
# time.sleep(2)
# r3_robot.set_all_joints_angle([90, 90, 90])
# time.sleep(2)
# r3_robot.set_all_joints_angle([180, 180, 180])
# time.sleep(2)
r3_robot.stop_robot()

