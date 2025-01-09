# Gazebo Simulator

!!! info 

    This note was written with the assistance of ChatGPT.

Gazebo can be used as a standalone robot simulator. But in practice, it's mostly used together with ROS. The setup described in this note is mainly in the ROS context.

* **Gazebo**: Ignition Gazebo (Fortress or later)
* **ROS**: Humble or later

## Robot Kinematics

Robot kinematics is handled with TF in ROS. During the real robot and simulation setup, the following ROS nodes may be used and often get confused:

* `robot_state_publisher`: subscribes to joint_states and uses the robot’s URDF to compute and publish the TF transforms of each link in the robot.
* `joint_state_publisher`: publishes the values of each joint (positions, optionally velocities/efforts) on the `joint_states` topic.
* `joint_state_broadcaster`: publishes the values of joint position, velocity, and/or effort on the `joint_states` topic

Differences between `joint_state_publisher` and `joint_state_broadcaster`:

* The `joint_state_broadcaster` is used in a ros2_control-based setup:
    - Real joint states come from your hardware drivers (or simulation).
    - Those are read by the ros2_control infrastructure.
    - The joint_state_broadcaster publishes them on /joint_states.
* The `joint_state_publisher` node (and its GUI version `joint_state_publisher_gui`) is generally used for simpler, static, or manually-driven demos rather than for reading real hardware data. It doesn't have the knowledge of the real or simulated robot and it just publishes the specified or default values from the configuration file or ROS parameters.

Relationship between `robot_state_publisher` and `joint_state_publisher`/`joint_state_publisher`:

![](./figures/ros_joint_state.jpg)

* Both `joint_state_publisher` and `joint_state_broadcaster` can publish to the /joint_states topic but they have different use cases:
    * The `joint_state_publisher` reads the `robot_description` parameter from the parameter server or configuration file, finds all of the non-fixed joints and publishes a JointState message with all those joints defined. It's often used during the development of the robot model (i.e. writing the urdf/xacro).
    * The `joint_state_broadcaster` works as part of the ros2_control framework and publish to the /joint_states topic with information acquired from the real robot or the simulator.
    * In general, you don't need both in one setup. `joint_state_broadcaster` can dynamically update and publish joint states, while `joint_state_publisher` mainly publishes configurable but mostly static joint states. 
* If no one is publishing joint states, the `robot_state_publisher` can’t do much as its TF frames will be at default or uninitialized states.



## Reference:

* https://gazebosim.org/docs/fortress/ros2_interop/