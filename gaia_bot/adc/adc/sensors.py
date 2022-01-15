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
from sensor_msgs.msg import BatteryState, Illuminance
from .ADC import Adc
import time

adc=Adc()

class SensorsNode(Node):

    def __init__(self):
        super().__init__('sensors')
        self.publisher_battery = self.create_publisher(BatteryState, 'battery', 10)
        self.publisher_left_light = self.create_publisher(Illuminance, 'left_light', 10)
        self.publisher_right_light = self.create_publisher(Illuminance, 'right_light', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        battery = BatteryState()
        battery.voltage = float(adc.recvADC(2)*3)
        self.publisher_battery.publish(battery)

        left = Illuminance()
        left.illuminance = float(adc.recvADC(0))
        self.publisher_left_light.publish(left)

        right = Illuminance()
        right.illuminance = float(adc.recvADC(1))
        self.publisher_right_light.publish(right)

def main(args=None):
    rclpy.init(args=args)

    sensors = SensorsNode()

    rclpy.spin(sensors)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    range.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
