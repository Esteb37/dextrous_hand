#!/usr/bin/env python3

import rclpy
import mujoco
import mujoco.viewer
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from dextrous_hand.utils.utils import parent_dir
from dextrous_hand.utils.HandConfig import HandConfig


class MujocoNode(Node):
    def __init__(self):
        super().__init__('mujoco_node')
        self.get_logger().info('mujoco_node started.')
        self.joint_command_subscriber = self.create_subscription(Float32MultiArray, 'hand_config', self.joint_command_callback, 10)


        xml_path = parent_dir() + "/data/assets/hand_p4_arm.xml"

        self.model = mujoco.MjModel.from_xml_path(xml_path) # type: ignore
        self.data = mujoco.MjData(self.model) # type: ignore

        self.config = HandConfig(unrestricted=True)

        # Launch the viewer in a non-blocking way
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

        self.joint_names = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(self.model.njnt)] # type: ignore

        self.create_timer(0.01, self.update_simulation)  # 100 Hz update rate

    def update_simulation(self):
        if self.viewer.is_running():

            self.data.ctrl[:] = self.config.mujoco_angles()

            self.data.qpos[:3] = self.config.POSITION
            self.data.qpos[3:6] = self.config.ORIENTATION

            mujoco.mj_step(self.model, self.data) # type: ignore
            self.viewer.sync()  # Sync the viewer with the new simulation state

    def joint_command_callback(self, msg):
        # Mapping function to be removed
        self.config = HandConfig.from_msg(msg, unrestricted=True)
        print(self.config)

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