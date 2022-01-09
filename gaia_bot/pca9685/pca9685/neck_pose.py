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
 
import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory

from .PCA9685 import PCA9685
class Servo:
    def __init__(self):
        self.PwmServo = PCA9685(0x40, debug=True)
        self.PwmServo.setPWMFreq(50)
        self.PwmServo.setServoPulse(8,1500)
        self.PwmServo.setServoPulse(9,1500)
    def setServoPwm(self,channel,angle,error=10):
        angle=int(angle)
        if channel=='rotation':
            self.PwmServo.setServoPulse(8,2500-int((angle+error)/0.09))
        elif channel=='lift':
            self.PwmServo.setServoPulse(9,500+int((angle+error)/0.09))

class NeckNode(Node):
    pwm = Servo()
    def __init__(self):
        super().__init__('neck_pose')
        self.subscription = self.create_subscription(
            JointTrajectory,
            'neck_pose',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info('Incoming message')
        for x in range(0, len(msg.joint_names)):
            self.get_logger().info(msg.joint_names[x])
            self.get_logger().info(str(msg.points[x].positions[0]))
            self.pwm.setServoPwm(msg.joint_names[x],msg.points[x].positions[0])


def main(args=None):
    rclpy.init(args=args)

    neck = NeckNode()

    rclpy.spin(neck)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    neck.destroy_node()
    rclpy.shutdown()
