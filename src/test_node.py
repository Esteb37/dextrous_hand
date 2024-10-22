#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import mujoco
import mujoco.viewer
from std_msgs.msg import Float32MultiArray


class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.get_logger().info('test_node started.')
        self.joint_command_subscriber = self.create_subscription(Float32MultiArray, 'finger_positions', self.joint_command_callback, 10)

        xml_file = "/home/atharva/rwr_ws/src/dextrous_hand/data/assets/hh_hand.xml"
        self.m = mujoco.MjModel.from_xml_path(xml_file)
        self.d = mujoco.MjData(self.m)
        self.new_joint_angles = None

        with mujoco.viewer.launch_passive(self.m, self.d) as viewer:
            start = time.time()
            while viewer.is_running():
                step_start = time.time()

                # mj_step can be replaced with code that also evaluates
                # a policy and applies a control signal before stepping the physics.
                if self.new_joint_angles is not None:
                    for i in range(len(self.new_joint_angles.data)):
                        self.d.qpos[i] = self.new_joint_angles.data[i]
                        self.new_joint_angles = None  # Reset after applying

                # print(self.d.qpos.shape)
                mujoco.mj_step(self.m, self.d)
                viewer.sync()

                # Example modification of a viewer option: toggle contact points every two seconds
                # Viewer.lock() is an important way to keep visualization synchronised over the multiple threads used by MuJoCo. 
                # However, just using viewr.synch() after mujoco.mj_step also seemingly works fine ...

                # with viewer.lock():
                #     # uncomment this line if you don't want to visualise contact points
                #     viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(self.d.time % 2)

                #     # Pick up changes to the physics state, apply perturbations, update options from GUI.
                #     viewer.sync()

                #     # Rudimentary time keeping, will drift relative to wall clock.
                #     time_until_next_step = self.m.opt.timestep - (time.time() - step_start)
                #     if time_until_next_step > 0:
                #         time.sleep(time_until_next_step)

    def joint_command_callback(self, msg):
        # Mapping function to be removed
        self.new_joint_angles = msg
        print("New joint command recevied")




def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()