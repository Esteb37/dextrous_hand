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

 - Test node
   - ```ros2 run dextrous_hand basic_node.py```

         [INFO] [1728219147.878266379] [basic_node]: Hello, ROS 2 (Python)!

 - Add Motor library
   - ```sudo apt-get install ros-humble-dynamixel-sdk```

 - Add pip
   - ```sudo apt install python3-pip```

 - Add teleop control library
   - ```pip install pynput```

# Adding files

## Nodes
- Add the **.py** file at **/src**
- In setup.py
   ```
    entry_points={
        'console_scripts': [
            'basic_node = src.basic_node:main',
            'your_node = src.your_node:main', <-- Add node here
        ],
    },
   ```
## Modules
- Just add the **.py** file at **/dextrous_hand**
   - When importing, do not forget to append ```dextrous_hand```

      ```import dextrous_hand.my_module as my_module```

# File Structure
 - **/src** - Python node files
 - **/dextrous_hand** - Python module files
 - **/launch** - ROS Launch files
 - **/test** - Any code that does not correspond to a node (i.e. test code and experiments)
 - **/data** - Images, text-files, ML models etc