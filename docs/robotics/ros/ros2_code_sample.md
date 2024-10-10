# ROS2 Code Sample

This note contains code samples from ROS Humble for quick reference. The code can be found from the Github repository [ros2_sample](https://github.com/rxdu/ros2_sample). Most of them are from the ROS official documentation. More references can be found from the official examples repository [ros2/examples](https://github.com/ros2/examples/tree/humble/rclcpp)

## Python Launch File

More information about ROS2 launch can be found at [ROS2 Launch](../ros2_launch)

#### Sample my_app.launch.py

```python
from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import (EnvironmentVariable, FindExecutable)
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ## arguments
    
    ## local variables
    map_path = PathJoinSubstitution([
                    FindPackageShare('robot_map'),
                    "map",
                    "my_office"
                ])

    rviz_config_file = PathJoinSubstitution([
                    FindPackageShare('robot_bringup'),
                    "rviz",
                    "autoware_default_view.rviz"
                ])

    ## actions
    launch_py_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robot_module'),
                    "launch",
                    "module_launch.launch.py"
                ])
            ]),    
        launch_arguments={
            'start_rviz2': 'false'
        }.items())

    launch_xml_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robot_nav'),
                    "launch",
                    'subsystem',
                    "map.launch.xml"
                ])
            ]),    
        launch_arguments={
            'map_path': map_path,
        }.items())
    
    start_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_sample',
        arguments=['-d', rviz_config_file],
        output='screen')

    ## LaunchDescription
    return LaunchDescription([
        launch_py_launch,
        launch_xml_launch,
        start_rviz_node
    ])
```

## Cmake with ament_cmake

#### Sample CMakeLists.txt

ament_cmake has defined many extension functions to the standar CMake. In general, you should follow the template to make sure everything is set up properly.

```cmake
cmake_minimum_required(VERSION 3.10.2)
project(my_project VERSION 0.1.0)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

## Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Set compiler to use c++ 14 features
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Additional cmake module path
list(APPEND CMAKE_PREFIX_PATH "/usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu/cmake")
list(APPEND CMAKE_PREFIX_PATH "/opt/my_dep_lib/lib/cmake")

## Find depdenencies and add targets
# standard cmake dependencies
find_package(Eigen3 REQUIRED)

# ros2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# my targets
add_library(my_library SHARED
    src/my_source.cpp
)
target_link_libraries(my_library PUBLIC Eigen3::Eigen)
ament_target_dependencies(my_library PUBLIC rclcpp)
target_include_directories(my_library
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

add_executable(my_executable src/main.cpp)
target_link_libraries(my_executable PUBLIC my_library)

# the install and export configuration
install(
  TARGETS my_library
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# targets that you don't want to export
install(TARGETS
    my_executable
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(some_dependency)

ament_package()
```

## Publisher/Subscriber

#### Create a ROS2 publisher

```cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

 private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

```

#### Create a ROS2 subscriber

```cpp
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node {
 public:
  MinimalSubscriber() : Node("minimal_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

 private:
  void topic_callback(const std_msgs::msg::String& msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

## Service/Client

#### Create a ROS2 server

```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <memory>

void add(
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "Incoming request\na: %ld"
              " b: %ld",
              request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]",
              (long int)response->sum);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("add_two_ints_server");

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
      node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints",
                                                                &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
```

#### Create a ROS2 client

```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  if (argc != 3) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
    return 1;
  }

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("add_two_ints_client");
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client =
      node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

  auto request =
      std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = atoll(argv[1]);
  request->b = atoll(argv[2]);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}
```

## Custom ROS2 msg/srv/action

By convension, the custom type definition files are put in msg/srv/action folder inside the package respectively.

#### Num.msg

```
int64 num
```

#### AddTreeInts.srv

```
int64 a
int64 b
int64 c
---
int64 sum
```

#### CMakeLists.txt

```cmake
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# the library name must match ${PROJECT_NAME}
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "msg/Sphere.msg"
  "srv/AddThreeInts.srv"
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)
```

#### package.xml

```xml
<depend>geometry_msgs</depend>

<build_depend>rosidl_default_generators</build_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

## Handle Parameters

#### Sample parameter file

```yml
/lidar_ns:
  lidar_node_name:
    ros__parameters:
      lidar_name: foo
      id: 10
imu:
  ros__parameters:
    ports: [2438, 2439, 2440]
/**:
  ros__parameters:
    debug: true
```

The use of wildcards (/**) to indicate that the parameters listed in this section are set on any node in any namespace.

#### Set parameters from command line

```bash
$ ros2 run package_name executable_name --ros-args -p param_name:=param_value
# example
$ ros2 run demo_nodes_cpp parameter_blackboard --ros-args -p some_int:=42 -p "a_string:=Hello world" -p "some_lists.some_integers:=[1, 2, 3, 4]" -p "some_lists.some_doubles:=[3.14, 2.718]"
```

#### Set parameters from Launch file

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="params",
            executable="param_node",
            name="param_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"my_parameter": "my_value"}
            ]
        )
    ])
```

#### Load parameter file on node startup

```bash
$ ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
# example
$ ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim.yaml
```

#### Load parameter file from Launch file

```python
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    vesc_config=PathJoinSubstitution([
        FindPackageShare('vesc_driver'),
        "params",
        "vesc_config.yaml"
    ])

    return LaunchDescription([
        Node(
            package='vesc_driver',
            executable='vesc_driver_can_node',
            name='vesc_driver_can_node',
            parameters=[vesc_config]
        )
    ])
```

#### Use parameters in a C++ class

```cpp
#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class MinimalParam : public rclcpp::Node
{
public:
  MinimalParam()
  : Node("minimal_param_node")
  {
    // simple paramter
    this->declare_parameter("my_parameter", "world");

    // specify parameter descriptor
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "This parameter is mine!";

    this->declare_parameter("my_parameter2", "world", param_desc);

    timer_ = this->create_wall_timer(
      1000ms, std::bind(&MinimalParam::timer_callback, this));
  }

  void timer_callback()
  {
    std::string my_param =
      this->get_parameter("my_parameter").get_parameter_value().get<std::string>();

    RCLCPP_INFO(this->get_logger(), "Hello %s!", my_param.c_str());

    std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "world")};
    this->set_parameters(all_new_parameters);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalParam>());
  rclcpp::shutdown();
  return 0;
}
```

## Composable Nodes

#### Create a composable publisher

```cpp
#ifndef MINIMAL_COMPOSITION__PUBLISHER_NODE_HPP_
#define MINIMAL_COMPOSITION__PUBLISHER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "minimal_composition/visibility.h"

class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode(rclcpp::NodeOptions options);

private:
  void on_timer();
  size_t count_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // MINIMAL_COMPOSITION__PUBLISHER_NODE_HPP_
```

```cpp
#include <chrono>

#include "minimal_composition/publisher_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

PublisherNode::PublisherNode(rclcpp::NodeOptions options)
: Node("publisher_node", options), count_(0)
{
  publisher_ = create_publisher<std_msgs::msg::String>("topic", 10);
  timer_ = create_wall_timer(
    500ms, std::bind(&PublisherNode::on_timer, this));
}

void PublisherNode::on_timer()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publisher: '%s'", message.data.c_str());
  publisher_->publish(message);
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(PublisherNode)
```

#### Package.xml for the package 

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format2.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="2">
  <name>examples_rclcpp_minimal_composition</name>
  <version>0.15.1</version>
  <description>Minimalist examples of composing nodes in the same
  process</description>
  <maintainer email="sloretz@openrobotics.org">Shane Loretz</maintainer>
  <maintainer email="aditya.pande@openrobotics.org">Aditya Pande</maintainer>
  <license>Apache License 2.0</license>
  <author>Mikael Arguedas</author>
  <author>Morgan Quigley</author>
  <author email="jacob@openrobotics.org">Jacob Perron</author>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <build_depend>rclcpp</build_depend>
  <build_depend>rclcpp_components</build_depend>
  <build_depend>std_msgs</build_depend>

  <exec_depend>rclcpp</exec_depend>
  <exec_depend>rclcpp_components</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

#### CMakeLists.txt for the package 

The below cmake file provides a complete example of how to build composable nodes. Please refer to `Sample CMakeLists.txt` above for the general cmake setup for ROS2 packages.

```cmake
cmake_minimum_required(VERSION 3.5)
project(examples_rclcpp_minimal_composition)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(composition_nodes SHARED
            src/publisher_node.cpp
            src/subscriber_node.cpp)
target_compile_definitions(composition_nodes
  PRIVATE "MINIMAL_COMPOSITION_DLL")
ament_target_dependencies(composition_nodes rclcpp rclcpp_components std_msgs)

rclcpp_components_register_node(composition_nodes
  PLUGIN "PublisherNode"
  EXECUTABLE publisher_node
)

install(TARGETS
  composition_nodes publisher_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  composition_publisher
  composition_subscriber
  composition_composed
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

## Reference

* https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html