#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import mujoco
import mujoco.viewer

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.get_logger().info('test_node started.')
        xml_file = "/home/atharva/rwr_ws/src/dextrous_hand/data/assets/hh_hand.xml"
        m = mujoco.MjModel.from_xml_path(xml_file)
        d = mujoco.MjData(m)

        with mujoco.viewer.launch_passive(m, d) as viewer:
        # Close the viewer automatically after 30 wall-seconds.
            start = time.time()
            while viewer.is_running() and time.time() - start < 30:
                step_start = time.time()

                # mj_step can be replaced with code that also evaluates
                # a policy and applies a control signal before stepping the physics.
                mujoco.mj_step(m, d)

                # Example modification of a viewer option: toggle contact points every two seconds.
                with viewer.lock():
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

                    # Pick up changes to the physics state, apply perturbations, update options from GUI.
                    viewer.sync()

                    # Rudimentary time keeping, will drift relative to wall clock.
                    time_until_next_step = m.opt.timestep - (time.time() - step_start)
                    if time_until_next_step > 0:
                        time.sleep(time_until_next_step)




def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()