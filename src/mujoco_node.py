#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import mujoco
import mujoco.viewer
from std_msgs.msg import Float32MultiArray


class MujocoNode(Node):
    def __init__(self):
        super().__init__('mujoco_node')
        self.get_logger().info('mujoco_node started.')
        self.joint_command_subscriber = self.create_subscription(Float32MultiArray, 'finger_positions', self.joint_command_callback, 10)

        xml_file = "/home/atharva/rwr_ws/src/dextrous_hand/data/assets/hh_hand.xml"
        self.m = mujoco.MjModel.from_xml_path(xml_file)
        self.d = mujoco.MjData(self.m)
        self.new_joint_angles = None

        # Launch the viewer in a non-blocking way
        self.viewer = mujoco.viewer.launch_passive(self.m, self.d)

        self.create_timer(0.01, self.update_simulation)  # 100 Hz update rate

    def update_simulation(self):
        if self.viewer.is_running():
            if self.new_joint_angles is not None:
                for i in range(len(self.new_joint_angles.data)):
                    self.d.qpos[i] = self.new_joint_angles.data[i]
                self.new_joint_angles = None  # Reset after applying

            mujoco.mj_step(self.m, self.d)
            self.viewer.sync()  # Sync the viewer with the new simulation state
 

    def joint_command_callback(self, msg):
        # Mapping function to be removed
        self.new_joint_angles = msg
        print("New joint command recevied")
    
    def destroy(self):
        self.viewer.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MujocoNode()
    rclpy.spin(node)
    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()