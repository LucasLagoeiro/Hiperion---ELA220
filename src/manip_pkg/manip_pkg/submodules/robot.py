from board import SCL, SDA
import busio

from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

import numpy as np
from time import sleep

UPPER_LIMIT_PWM = 2000
LOWER_LIMIT_PWM = 10000

class Robot:
    def __init__(self):
        self.pca = PCA9685(busio.I2C(SCL, SDA))
        self.pca.reference_clock_speed = 26624000
        self.pca.frequency = 60
        self.joints_number = 4
        self.velocity = 100.0
        
        self.home_angles = [90,90,90,90]
        self.limits = ((0, 180), (40,180), (0, 180), (0, 180))
        self.atual_angles = [90,90,90,90]
        self.home_position()

    def validate(self, joint_number, angle_degrees):
        return angle_degrees
        upper_limit = self.limits[joint_number-1][1]
        lower_limit = self.limits[joint_number-1][0]

        if joint_number == 3:
            if self.atual_angles[1] <= 130:
                upper_limit = 140
                lower_limit = abs(self.atual_angles[1]-155)
            else:
                upper_limit = abs(self.atual_angles[1]-280)
                lower_limit = self.atual_angles[1]-70

            if angle_degrees < lower_limit:
                return lower_limit
            elif angle_degrees > upper_limit:
                return upper_limit
            
            return angle_degrees
            
        else:
            if angle_degrees < lower_limit:
                print('1')
                return lower_limit
            elif angle_degrees > upper_limit:
                print('2')
                return upper_limit

            print('3')

            return angle_degrees
        
        
    def degrees_to_pwm(self, angle_degrees):
        angle_pwm = int((UPPER_LIMIT_PWM - LOWER_LIMIT_PWM)/(180 - 0)*angle_degrees + LOWER_LIMIT_PWM)
        return angle_pwm


    def set_single_joint_angle(self, joint_number, angle_degrees):

        angle_degrees = self.validate(joint_number, angle_degrees)
        
        if self.atual_angles[joint_number-1] == angle_degrees:
            return


        increment = (angle_degrees - self.atual_angles[joint_number-1])/self.velocity 

        for angle in np.arange(self.atual_angles[joint_number-1], angle_degrees, increment):
            angle_pwm = self.degrees_to_pwm(angle)
            self.pca.channels[joint_number-1].duty_cycle = angle_pwm
            sleep(0.01)

        
        angle_pwm = self.degrees_to_pwm(angle_degrees)
        self.pca.channels[joint_number-1].duty_cycle = angle_pwm

        self.atual_angles[joint_number-1] = angle_degrees
        print(f'Set Joint {joint_number}: {angle_degrees}\t | All angles: {self.atual_angles}')


    def set_all_joints_angle(self, list_angle_degrees):
        angles_data = list()
        for i in range(4):
            angles_data.append([self.atual_angles[i], (list_angle_degrees[i] - self.atual_angles[i])/self.velocity])

        for i in range(100):
            for joint in range(4):
                angle_pwm = self.degrees_to_pwm(angles_data[joint][0] + angles_data[joint][1] * i)
                self.pca.channels[joint].duty_cycle = angle_pwm
            sleep(0.01)
        
        for joint in range(4):
            angle_pwm = self.degrees_to_pwm(list_angle_degrees[joint])
            self.pca.channels[joint].duty_cycle = angle_pwm
            self.atual_angles[joint] = list_angle_degrees[joint]



    def stop_robot(self):
        self.pca.deinit()

    def home_position(self):
        self.set_all_joints_angle(self.home_angles)

    def trajectory(self, angles_list):
        for angles in angles_list:
            self.set_all_joints_angle(angles)


