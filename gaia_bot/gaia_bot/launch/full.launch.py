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

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with a single component."""
    gaia_bot_container = ComposableNodeContainer(
            name='gaia_bot_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='gaia_bot',
                    plugin='gaia_bot::GaiaBot',
                    name='gaia_bot'),
            ],
            output='screen',
    )

    camera = Node(
        package = 'v4l2_camera',
        namespace='',
        executable='v4l2_camera_node',
        name='v4l2_camera'
    )

    faces = Node(
        package = 'faces',
        namespace='',
        executable='faces',
        name='faces',
    )

    leds = Node(
        package = 'leds',
        namespace='',
        executable='leds',
        name='leds'
    )

    range = Node(
        package = 'range',
        namespace='',
        executable='range',
        name='range'
    )

    sensors = Node(
        package = 'adc',
        namespace='',
        executable='sensors',
        name='sensors'
    )

    wheels = Node(
        package = 'pca9685',
        namespace='',
        executable='wheels',
        name='wheels'
    )

    neck_pose = Node(
        package = 'pca9685',
        namespace='',
        executable='neck_pose',
        name='neck_pose'
    )

    return launch.LaunchDescription([
        gaia_bot_container,
        camera,
        faces,
#        leds,
#        range,
#        sensors,
#        wheels,
        neck_pose,
    ])
