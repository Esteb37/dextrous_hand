import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState  # Assuming JointState is used, modify if different
import mujoco_py

class MujocoJointController(Node):
    def __init__(self):
        super().__init__('mujoco_joint_controller')

        # Create a subscriber for joint commands
        self.joint_command_subscriber = self.create_subscription(
            JointState, 'finger_positions', self.command_callback, 10)

        # Set up MuJoCo model and simulation
        xml_file = "../data/assets/hh_hand.xml"
        self.model = mujoco_py.load_model_from_path(xml_file)
        self.sim = mujoco_py.MjSim(self.model)

        # Initialize storage for commands
        self.joint_positions = [0.0] * self.model.nq  # nq: number of generalized coordinates

        # Create a timer to periodically step the simulation
        self.timer = self.create_timer(0.01, self.simulation_step)  # 100 Hz

    def command_callback(self, msg):
        # Assuming the incoming message contains joint positions
        # Update the joint positions from the ROS message
        self.joint_positions = msg.position

    def simulation_step(self):
        # Apply the joint positions in the MuJoCo simulation
        for i, pos in enumerate(self.joint_positions):
            self.sim.data.qpos[i] = pos

        # Step the MuJoCo simulation
        self.sim.step()

        # Optionally, log the joint positions for debugging
        self.get_logger().info(f'Updated joint positions: {self.joint_positions}')

def main(args=None):
    rclpy.init(args=args)
    node = MujocoJointController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
