# MIT License

# Copyright (c) 2023 Derek King

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


import launch

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import (LaunchConfiguration,
                                  PythonExpression,
                                  TextSubstitution)


def generate_launch_description():
    img_fn_launch_arg = DeclareLaunchArgument(
        "img_fn", default_value=TextSubstitution(text="")
    )

    img_fns_launch_arg = DeclareLaunchArgument(
        "img_fns", default_value=TextSubstitution(text="")
    )

    # this expression is not super useful on command line because of how
    # difficult it is to provide a comma separated list of filenames
    img_fns_expr = PythonExpression(
        ["'", LaunchConfiguration("img_fns"), "'", '.split(",")'])

    thread_num_launch_arg = DeclareLaunchArgument(
        "thread_num", default_value=TextSubstitution(text="3"))

    pub_period_launch_arg = DeclareLaunchArgument(
        "pub_period", default_value=TextSubstitution(text="0.1"))

    output_map_fn_launch_arg = DeclareLaunchArgument(
        "output_map_fn", default_value=TextSubstitution(text=""))

    display_debug_launch_arg = DeclareLaunchArgument(
        "display_debug", default_value=TextSubstitution(text="True"))

    display_wait_duration_launch_arg = DeclareLaunchArgument(
        "display_wait_duration", default_value=TextSubstitution(text="1.0"))

    image_loader_component = ComposableNode(
        package='ar_slam',
        plugin='ar_slam::ImageLoader',
        name='image_loader',
        parameters=[{
            'img_fn': LaunchConfiguration("img_fn"),
            'img_fns': img_fns_expr,
            'pub_period': LaunchConfiguration("pub_period"),
        }],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    aruco_4x4_detctor_component = ComposableNode(
        package='ar_slam',
        plugin='ar_slam::ArucoDetector',
        name='aruco_detector',
        parameters=[{
            # 'aruco_dict': '5X5_100',
            'aruco_dict': '4X4_50',
            'id_offset': 0,
        }],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    aruco_5x5_detctor_component = ComposableNode(
        package='ar_slam',
        plugin='ar_slam::ArucoDetector',
        name='aruco_detector2',
        parameters=[{
            'aruco_dict': '5X5_100',
            # 'aruco_dict': '4X4_50',
            'id_offset': 1000,
        }],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    merge_component = ComposableNode(
        package='ar_slam',
        plugin='ar_slam::MergeDetections',
        name='merge_detections',
        parameters=[{
            'expected_detectors': [
                'aruco_4X4_50',
                'aruco_5X5_100',
            ]
        }],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    ar_slam_component = ComposableNode(
        package='ar_slam',
        plugin='ar_slam::ArSlam',
        name='ar_slam',
        parameters=[{
            'output_map_fn': LaunchConfiguration('output_map_fn'),
            'display_debug': LaunchConfiguration('display_debug'),
            'display_wait_duration':
            LaunchConfiguration('display_wait_duration'),
        }],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    container = ComposableNodeContainer(
        name='ar_slam_container',
        namespace='',
        package='rclcpp_components',
        # executable='component_container',
        executable='component_container_mt',
        # prefix=['gdbserver localhost:3000'],
        output='screen',
        parameters=[{
            'thread_num': LaunchConfiguration("thread_num")
        }],
        composable_node_descriptions=[
            image_loader_component,
            aruco_4x4_detctor_component,
            aruco_5x5_detctor_component,
            merge_component,
            ar_slam_component,
        ],
    )

    return launch.LaunchDescription([
        img_fn_launch_arg,
        img_fns_launch_arg,
        thread_num_launch_arg,
        pub_period_launch_arg,
        output_map_fn_launch_arg,
        display_debug_launch_arg,
        display_wait_duration_launch_arg,
        container,
    ])
