# Copyright 2022 Gaia Platform, LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Includes code adapted from Freenove under Creative Commons Licence 3.0
# 
# See:
#   https://github.com/Freenove/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi
#   https://creativecommons.org/licenses/by/3.0/
#

import rclpy
from rclpy.node import Node
from gaia_bot_interfaces.msg import WheelSpeeds
from .PCA9685 import PCA9685

class Motors:
    def __init__(self):
        self.pwm = PCA9685(0x40, debug=True)
        self.pwm.setPWMFreq(50)

    def __del__(self):
        self.setMotorModel(0, 0, 0, 0)

    def getDuty(self, speed):
        if speed > 100.0:
            return -4095
        if speed < -100.0:
            return 4095
        return int(speed * -4095.0 / 100.0)

    def wheel(self, chan1, chan2, speed):
        duty = self.getDuty(speed)
        if duty > 0:
            self.pwm.setMotorPwm(chan1, 0)
            self.pwm.setMotorPwm(chan2, duty)
        elif duty < 0:
            self.pwm.setMotorPwm(chan2, 0)
            self.pwm.setMotorPwm(chan1, abs(duty))
        else:
            self.pwm.setMotorPwm(chan1, 4095)
            self.pwm.setMotorPwm(chan2, 4095)

    def setMotorModel(self, front_left_speed, front_right_speed, rear_left_speed, rear_right_speed):
        self.wheel(0, 1, front_left_speed)
        self.wheel(3, 2, rear_left_speed)
        self.wheel(6, 7, front_right_speed)
        self.wheel(4, 5, rear_right_speed)

class WheelsNode(Node):
    motors = Motors()

    def __init__(self):
        super().__init__('wheels')
        self.subscription = self.create_subscription(
            WheelSpeeds,
            'wheels',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.motors.setMotorModel(msg.front_left, msg.front_right, msg.rear_left, msg.rear_right)

def main(args=None):
    rclpy.init(args=args)

    wheels = WheelsNode()

    rclpy.spin(wheels)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wheels.destroy_node()
    rclpy.shutdown()
