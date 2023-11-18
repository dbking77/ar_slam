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


import os
import launch

from ament_index_python.packages import get_package_share_directory

from launch.actions import (EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo,
                            RegisterEventHandler, TimerAction)
from launch.substitutions import (FindExecutable, LocalSubstitution)
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                   OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ar_slam_dir = get_package_share_directory('ar_slam')
    ar_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(ar_slam_dir, 'launch', 'ar_slam_launch.py')
        ]),
        launch_arguments={'output_map_fn': '/tmp/map.yaml'}.items(),
    )

    img_fns = ['img1.jpg', 'img2.jpg', 'img3.jpg']
    img_dir = os.path.join(ar_slam_dir, 'resources', 'images')
    img_paths = [os.path.join(img_dir, fn) for fn in img_fns]

    load_images = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' run ar_slam load_images_client ' + ' '.join(img_paths)
        ]],
        shell=True,
    )

    on_start = RegisterEventHandler(
        OnProcessStart(
            target_action=load_images,
            on_start=[
                LogInfo(msg='---------- Client Started ----------'),
            ]
        )
    )

    on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=load_images,
            on_exit=[
                LogInfo(msg='---------- Cleint Exits ----------'),
                TimerAction(
                    period=10.0,
                    actions=[
                        EmitEvent(event=Shutdown(reason='client ended')),
                    ],
                )
            ]
        )
    )

    on_complete = RegisterEventHandler(
        OnExecutionComplete(
            target_action=ar_slam,
            on_completion=[
                LogInfo(msg='---------- ARxSlam Complete ----------'),
            ]
        )
    )

    on_shutdown = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                LogInfo(msg=[
                    "---------- Shutting Down ",
                    LocalSubstitution('event.reason')])
            ]
        )
    )

    return launch.LaunchDescription([
        ar_slam,
        on_start,
        on_exit,
        on_complete,
        on_shutdown,
        load_images,
    ])
