#!/usr/bin/env python3

import rclpy
import mujoco
import mujoco.viewer
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from dextrous_hand.utils.utils import parent_dir
from dextrous_hand.utils.HandConfig import HandConfig


class MujocoControllerNode(Node):
    def __init__(self):
        super().__init__('mujoco_controller_node')

        self.config_publisher = self.create_publisher(Float32MultiArray, 'hand_config', 10)

        xml_path = parent_dir() + "/data/assets/hh_hand_arm.xml"

        self.model = mujoco.MjModel.from_xml_path(xml_path) # type: ignore
        self.data = mujoco.MjData(self.model) # type: ignore

        self.config = HandConfig()

        # Launch the viewer in a non-blocking way
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

        self.joint_names = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(self.model.njnt)] # type: ignore

        self.create_timer(0.01, self.update_simulation)  # 100 Hz update rate

        self.get_logger().warn("Mujoco Node started")

    def update_simulation(self):
        if self.viewer.is_running():

            self.config.from_mujoco_angles(self.data.ctrl)

            self.data.qpos[:3] = self.config.POSITION
            self.data.qpos[3:6] = self.config.ORIENTATION_XYZ
            mujoco.mj_step(self.model, self.data) # type: ignore
            self.viewer.sync()  # Sync the viewer with the new simulation state

            self.config_publisher.publish(self.config.as_msg())


    def destroy(self):
        self.viewer.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MujocoControllerNode()
    rclpy.spin(node)
    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()