---
title: ROS2 Launch
description: 
---

## Concepts

The [launch](https://github.com/ros2/launch) and [launch_ros](https://github.com/ros2/launch_ros) packages provide a set of API to model the configuration and starting of ROS2 nodes. The "launch" package provides the base classes and the "launch_ros" package provides ROS2 specific extensions.

* **launch.LaunchDescription**: this is the main class used to describe the launch configuration. It is a list of "actions" that are executed sequentially. The launch configuration is executed by the launch service.
* **launch.Action**: this class represents the various user intentions.
    - **launch.actions.IncludeLaunchDescription**: the action to include another launch file
    - **launch.actions.DeclareLaunchArgument**: the action to declare a launch argument
    - **launch.actions.SetEnvironmentVariable**: the action to set an environment variable by name
    - **launch.actions.AppendEnvironmentVariable**: the action to append a value to an environment variable
    - **launch.actions.GroupAction**: the action to group other actions, and it can be associated with conditions
    - **launch.actions.TimerAction**: the action to start other actions after a specified amount of time
    - **launch.actions.ExecuteProcess**: the action to execute a process with path and argutments

There are a few more actions defined in the launch package and you can find more information in the [documentation](file:///home/rdu/Workspace/code_study/launch/launch/doc/build/html/architecture.html). A commonly used action extended from launch and implemented in launch_ros is **from launch_ros.actions.Node**. You can express the intention of starting a ROS node with this action.

You can expect the outline of a typical ROS2 launch file to be like: 

```python
from launch import LaunchDescription
from launch.actions import ABC
from launch.actions import XYZ

def generate_launch_description():
    action_abc = ABC()
    action_xyz = XYZ()

    ld = LaunchDescription()
    ld.add_action(action_abc)
    ld.add_action(action_xyz)

    return ld
```

* **launch.Substitution**: this class represents the various ways to substitute a string. The substitution is performed when the launch configuration is executed by the launch service. Substitutions can help to make the launch configuration more flexible and easier to be reused.
    - **launch.substitutions.Text**: the substitution to get the given string when evaluated
    - **launch.substitutions.LaunchConfiguration**: the substitution to get the value of a launch argument
    - **launch.substitutions.EnvironmentVariable**: the substitution to get the value of an environment variable

In most cases, what you need to do to create a launch file is to use substitutions to construct actions and then use actions to construct the launch description.

* **launch.LaunchService**: "Launch descriptions, and the actions contained therein, can either be introspected directly or launched by a launch.LaunchService. A launch service is a long running activity that handles the event loop and dispatches actions."[3] This means other than creating a LaunchDescription() object and launching with "ros2 launch", you can also manually create a LaunchService object and run with the LaunchDescription object from a plain Python script.
* **launch.EventHandler**: "Event handlers can be registered for specific events and can be useful for monitoring the state of processes." [4] Predefined event handlers from the launch package includes (but not limited to):
    - **launch.event_handlers.OnExecutionComplete**
    - **launch.event_handlers.OnProcessStart**
    - **launch.event_handlers.OnProcessExit**
    - **launch.event_handlers.OnProcessIO**
    - **launch.event_handlers.OnShutdown**

## Sample Launch

The following launch file is copied from the ROS2 documentation [5] with minor modifications.

```python
# example.launch.py

import os

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction

from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    # args that can be set from the command line or a default will be used
    background_r_launch_arg = DeclareLaunchArgument(
        "background_r", default_value=TextSubstitution(text="0")
    )
    background_g_launch_arg = DeclareLaunchArgument(
        "background_g", default_value=TextSubstitution(text="255")
    )
    background_b_launch_arg = DeclareLaunchArgument(
        "background_b", default_value=TextSubstitution(text="0")
    )
    chatter_ns_launch_arg = DeclareLaunchArgument(
        "chatter_ns", default_value=TextSubstitution(text="my/chatter/ns")
    )

    # include another launch file
    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('demo_nodes_cpp'),
                'launch/topics/talker_listener.launch.py'))
    )
    # include another launch file in the chatter_ns namespace
    launch_include_with_namespace = GroupAction(
        actions=[
            # push_ros_namespace to set namespace of included nodes
            PushRosNamespace('chatter_ns'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('demo_nodes_cpp'),
                        'launch/topics/talker_listener.launch.py'))
            ),
        ]
    )

    # start a turtlesim_node in the turtlesim1 namespace
    turtlesim_node = Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        )

    # start another turtlesim_node in the turtlesim2 namespace
    # and use args to set parameters
    turtlesim_node_with_parameters = Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim',
            parameters=[{
                "background_r": LaunchConfiguration('background_r'),
                "background_g": LaunchConfiguration('background_g'),
                "background_b": LaunchConfiguration('background_b'),
            }]
        )

    # perform remap so both turtles listen to the same command topic
    forward_turtlesim_commands_to_second_turtlesim_node = Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )

    return LaunchDescription([
        background_r_launch_arg,
        background_g_launch_arg,
        background_b_launch_arg,
        chatter_ns_launch_arg,
        launch_include,
        launch_include_with_namespace,
        turtlesim_node,
        turtlesim_node_with_parameters,
        forward_turtlesim_commands_to_second_turtlesim_node,
    ])
```

## Typical Use Cases

### Launch a node

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim',
            output='screen',
            remapping=[
                ('origin1', 'other1'),
                ('origin2', 'other2')
            ]
        )
    ])
```

You can find more arguments to the Node() class in the source code [7].

### Launch a launch file

```python
from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    colors = {
        'background_r': '200'
    }

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('launch_tutorial'),
                    'example_substitutions.launch.py'
                ])
            ]),
            launch_arguments={
                'turtlesim_ns': 'turtlesim2',
                'use_provided_red': 'True',
                'new_background_r': TextSubstitution(text=str(colors['background_r']))
            }.items()
        )
    ])
```

## Reference

* [1] https://docs.ros.org/en/humble/Concepts/About-Domain-ID.html
* [2] https://github.com/chargerKong/learning_ros2_launch_by_example
* [3] https://github.com/ros2/launch/blob/humble/launch/doc/source/architecture.rst
* [4] https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Event-Handlers.html#
* [5] https://docs.ros.org/en/humble/How-To-Guides/Launch-file-different-formats.html
* [6] https://answers.ros.org/question/322874/ros2-what-is-different-between-declarelaunchargument-and-launchconfiguration/
* [7] https://github.com/ros2/launch_ros/blob/humble/launch_ros/launch_ros/actions/node.py#L187