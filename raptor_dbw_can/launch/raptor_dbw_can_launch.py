# Copyright (c) 2019 AutonomouStuff, LLC
# Copyright (c) 2020 New Eagle, LLC
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from ament_index_python import get_package_share_directory

def generate_launch_description():
    params_file = LaunchConfiguration(
        'params',
        default=[ThisLaunchFileDir(), '/launch_params.yaml'])

    #make sure the dbc file gets installed with the launch file
    dbc_file_path = get_package_share_directory('raptor_dbw_can') + "/launch/New_Eagle_DBW_3.3.388.dbc"
    
    return LaunchDescription(
        [
            Node(
                package='raptor_dbw_can',
                node_executable='raptor_dbw_can_node',
                output='screen',
                node_namespace='raptor_dbw_interface',
                parameters=[
                    {"dbw_dbc_file": dbc_file_path}
                ],
            ),
            Node(
                package='kvaser_interface',
                node_executable='kvaser_can_bridge',
                output='screen',
                node_namespace='can_rx_tx',
                parameters=[params_file],
            ),
        ]
    )

generate_launch_description()