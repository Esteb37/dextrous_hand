#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import mujoco
import mujoco.viewer
from std_msgs.msg import Float32MultiArray
from pathlib import Path
from dextrous_hand.HandConfig import HandConfig
from dextrous_hand.ids import JOINTS

JOINT_MAP = {
    JOINTS.WRIST: "wrist_base2palm",
    JOINTS.THUMB_MCP: "root2thumb_base",
    JOINTS.THUMB_ABD: "thumb_base2pp",
    JOINTS.THUMB_PIP: "thumb_pp2mp_virt",

    JOINTS.INDEX_MCP: "root2index_pp_virt",
    JOINTS.INDEX_ABD: "root2index_pp_virt_abd",
    JOINTS.INDEX_PIP: "index_pp2mp_virt",

    JOINTS.MIDDLE_MCP: "root2middle_pp_virt",
    JOINTS.MIDDLE_ABD: "root2middle_pp_virt_abd",
    JOINTS.MIDDLE_PIP: "middle_pp2mp_virt",

    JOINTS.RING_MCP: "root2ring_pp_virt",
    JOINTS.RING_ABD: "root2ring_pp_virt_abd",
    JOINTS.RING_PIP: "ring_pp2mp_virt",

    JOINTS.PINKY_MCP: "root2pinky_pp_virt",
    JOINTS.PINKY_ABD: "root2pinky_pp_virt_abd",
    JOINTS.PINKY_PIP: "pinky_pp2mp_virt",
}

class MujocoNode(Node):
    def __init__(self):
        super().__init__('mujoco_node')
        self.get_logger().info('mujoco_node started.')
        self.joint_command_subscriber = self.create_subscription(Float32MultiArray, 'hand_config', self.joint_command_callback, 10)

        # Go back until you find the "install" folder
        current_dir = Path(__file__).resolve().parent

        # Traverse upward until "install" directory is found
        install_dir = None
        for parent in current_dir.parents:
            if parent.name == "install":
                install_dir = parent
                break

        if install_dir is None:
            raise FileNotFoundError("Could not find the 'install' directory")

        parent_dir = install_dir.parent
        xml_path = parent_dir / "src" / "dextrous_hand" / "data" / "assets" / "hh_hand.xml"

        self.model = mujoco.MjModel.from_xml_path(xml_path.as_posix())
        self.data = mujoco.MjData(self.model)

        # Launch the viewer in a non-blocking way
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

        self.joint_names = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(self.model.njnt)]

        print(self.joint_names)

        self.config = HandConfig.default()

        self.create_timer(0.01, self.update_simulation)  # 100 Hz update rate

    def update_simulation(self):
        if self.viewer.is_running():
            for config_joint, sim_joint in JOINT_MAP.items():
                joint_index = self.joint_names.index(sim_joint)
                self.data.qpos[joint_index] = self.config[config_joint]

            mujoco.mj_step(self.model, self.data)
            self.viewer.sync()  # Sync the viewer with the new simulation state


    def joint_command_callback(self, msg):
        # Mapping function to be removed
        self.config = HandConfig.from_msg(msg)
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