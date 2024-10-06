# Setup

### We are using ROS 2 Humble, for Ubuntu Jammy 22.04
 - Install and configure git
    ```
    sudo apt-get git
    git config --global user.name GITHUB_USERNAME
    git config --global user.email GITHUB_EMAIL
    ```

 - Install **ROS 2**
    - https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

 - Add ROS to bash
   - ```nano ~/.bashrc```

      At the bottom add

      ```
      source /opt/ros/humble/setup.bash
      source ~/ros2_ws/install/setup.bash
      ```

 - Install **Colcon** and **Ninja** for building
    - ```
      sudo apt install python3-colcon-common-extensions
      apt-get install ninja-build
      ```

 - Install **rosdep** for dependency management
   - ```
      sudo apt-get install python3-rosdep
      rosdep init
      rosdep update
      ```
 - Create a **ros2 workspace** at home and go to its **src** folder
    - ```
        mkdir -p ~/ros2_ws/src
        cd ~/ros2_ws/src
        ```
- Clone this repository
   - ```git clone https://github.com/Esteb37/dextrous_hand```

- Resolve dependencies
  - ```
      cd ..
      rosdep install -i --from-path src --rosdistro humble -y
      ```
 - Build from within ```ros2_ws```
   - ```colcon build --packages-select dextrous_hand```

 - Test nodes
   - ```ros2 run dextrous_hand basic_node```

         [INFO] [1728218504.785549319] [basic_node]: Hello, ROS 2 (C++)!
   - ```ros2 run dextrous_hand basic_node.py```

         [INFO] [1728219147.878266379] [basic_node]: Hello, ROS 2 (Python)!


# Adding nodes
## C++
   - Add the **.cpp** file at **/src**
   - Add the **.hpp** file at **/include** if necessary
   - In **CMakeList.txt**

      ```
      add_executable(node_name src/node_name.cpp)
      ament_target_dependencies(node_name rclcpp)
      ...
      install(TARGETS
         basic_node
         node_name <-- Add node name here
         DESTINATION lib/${PROJECT_NAME})
      ```
## Python
   - Add the **.py** file at **/scripts**
   - In **CMakeList.txt**

      ```
      install(PROGRAMS
      scripts/basic_node.py
      scripts/node_name.py <-- Add file name here
      DESTINATION lib/${PROJECT_NAME})
      ```

# File Structure
 - **/src** - C++ node source files
 - **/include** - C++ header files
 - **/scripts** - Python node files
 - **/dextrous_hand** - Python module files
 - **/launch** - ROS Launch files
 - **/test** - Any code that does not correspond to a node (i.e. test code and experiments)
 - **/data** - Images, text-files, ML models etc
 - **/.vscode** - For easy workspace configuration
 - **/local** - Optional, needs to be created. For anything that should not be shared to the repository.
