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
from gaia_bot_interfaces.msg import Leds
from rpi_ws281x import *
import time

# LED strip configuration:
LED_COUNT      = 8      # Number of LED pixels.
LED_PIN        = 18      # GPIO pin connected to the pixels (18 uses PWM!).
LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA        = 10      # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 255     # Set to 0 for darkest and 255 for brightest
LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL    = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53

class LedStrip:
    def __init__(self):
        #Control the sending order of color data
        self.ORDER = "RGB"  
        # Create NeoPixel object with appropriate configuration.
        self.strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
        # Intialize the library (must be called once before other functions).
        self.strip.begin()

    def LED_TYPR(self, order, R_G_B):
        B=R_G_B & 255
        G=R_G_B >> 8 & 255
        R=R_G_B >> 16 & 255 
        Led_type=["GRB","GBR","RGB", "RBG","BRG","BGR"]
        color = [Color(G,R,B),Color(G,B,R),Color(R,G,B),Color(R,B,G),Color(B,R,G),Color(B,G,R)]
        if order in Led_type:
            return color[Led_type.index(order)]

    def set(self, bmp, R, G, B):
        color=self.LED_TYPR(self.ORDER,Color(R,G,B))
        for i in range(LED_COUNT):
            if bmp & 0x01 == 1:
                self.strip.setPixelColor(i, color)
                self.strip.show()
            bmp = bmp >> 1

class LedsNode(Node):
    ledStrip = LedStrip()

    def __init__(self):
        super().__init__('leds')
        self.subscription = self.create_subscription(
            Leds,
            'leds',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        print('test')
        self.ledStrip.set(msg.led_bmp, msg.red, msg.green, msg.blue)

def main(args=None):
    rclpy.init(args=args)

    leds = LedsNode()

    rclpy.spin(leds)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    leds.destroy_node()
    rclpy.shutdown()
