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
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection3D
import cv2
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import os

class FacesNode(Node):

    def __init__(self):
        super().__init__('faces')
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        data_file_name = os.path.join(get_package_share_directory('faces'), 'resource', 'haarcascade_frontalface_default.xml')
        self.face_cascade = cv2.CascadeClassifier(data_file_name)

        self.publisher_ = self.create_publisher(Detection3D, 'face', 10)


    def listener_callback(self, data):
        # Convert ROS Image message to OpenCV image
        current_frame = self.bridge.imgmsg_to_cv2(data)
        frame_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        frame_gray = cv2.equalizeHist(frame_gray)
        detected_faces = self.face_cascade.detectMultiScale(frame_gray)
        if(len(detected_faces)):
            for (x, y, w, h) in detected_faces:
                face = Detection3D()
                face.bbox.center.position.x = (2 * x + w) / data.width - 1.0
                face.bbox.center.position.y = -1.0 * ((2 * y + h) / data.height - 1.0)
                face.bbox.center.position.z = 0.0
                face.bbox.size.x = float(2 * w / data.width)
                face.bbox.size.y = float(2 * h / data.height)
                face.bbox.size.z = 0.0
                self.publisher_.publish(face)
                cv2.rectangle(frame_gray, (x, y), (x+w, y+h), (0, 255, 0), 2)
        
        # Display image
        cv2.imshow("camera", frame_gray)        
        cv2.waitKey(3)


def main(args=None):
    rclpy.init(args=args)

    faces = FacesNode()

    rclpy.spin(faces)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    faces.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
