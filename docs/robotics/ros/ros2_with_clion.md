# ROS2 Development with CLion

This is a quick reference note on how to develop ROS2 packages with CLion IDE. The main reference is the official documentation[1].

* Create your ROS2 workspace and package as usual

```bash
$ mkdir -p ros2_ws/src && cd ros2_ws/src
$ ros2 pkg create --build-type ament_cmake sample_package
```

* Build the workspace with the "CMAKE_EXPORT_COMPILE_COMMANDS" option ON. This will generate a JSON compilation database which can be used by CLion to find the dependent files.

```bash
$ cd ros2_ws
$ colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja
```

* Source the colcon workspace and launch CLion from the same terminal

```bash
$ cd ros2_ws
$ source install/setup.bash
$ clion 
```

* Open the project and adjust root directory in CLion, from the main menu

```
File | Open -> Open File or Project 
-> Select the "compile_commands.json" file -> Open as Project
```

```
Tools | Compilation Database | Change Project Root
```

## Reference

* [1] https://www.jetbrains.com/help/clion/ros2-tutorial.html