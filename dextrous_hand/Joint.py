#!/usr/bin/env python3

import dextrous_hand.constants as constants
from dextrous_hand.Motor import Motor

class Joint():
    """
    A joint is a motor or a collection of motors that move together to achieve a specific angle.
    """
    # For the singleton pattern
    _instances = {}

    def __new__(cls, name, *args, **kwargs):
        """
        Singleton pattern. Make sure only one instance of each Joint is created.
        If a joint with the same name has already been created, return that instance.
        """
        if name not in cls._instances:
            cls._instances[name] = super(Joint, cls).__new__(cls)
        return cls._instances[name]

    def __init__(self, id : constants.JOINTS):
        """
        params
            id [JOINTS]: the joint's id

        raises
            Exception: if the joint has no motors
        """
        # Avoid reinitialization if already initialized
        if hasattr(self, 'initialized') and self.initialized:
            return

        self.id = id

        # The target angle of the joint
        self.target = 0

        # Check if the joint has motors in the constants.py file
        if self.id not in constants.JOINT_MOTORS or len(constants.JOINT_MOTORS[self.id]) == 0:
            raise Exception("Joint " + self.id.name + " has no motors")

        self.motors = [Motor(port) for port in constants.JOINT_MOTORS[id]]

        self.initialized = True

    def joint2motors(self, angle):
        """
        Map the angle of the joint to the angle of the motor(s)

        params
            angle: the angle of the joint

        returns
            a list of motor angles to reach the joint angle

        TODO: map depending on motor composition
        """
        motor_angles = [angle for _ in self.motors]
        return motor_angles

    def write(self, angle):
        """
        Set the joint's motor to a specific angle

        params
            angle: the angle to set the motor to

        returns
            True if the joint is at the target position, False otherwise
        """
        self.target = angle
        motor_angles = self.joint2motors(angle)
        for motor, motor_angle in zip(self.motors, motor_angles):
            motor.write(motor_angle)
        return self.at_position()

    def motors2joint(self, motor_angles):
        """
        Map the angles of the motor(s) to the angle of the joint

        params
            motor_angles: a list of motor angles

        returns
            the angle of the joint

        TODO: map depending on motor composition
        """
        joint_angle = motor_angles[0]
        return joint_angle

    def read(self):
        """
        returns:
            The joint's current angle in radians
        """
        motor_angles = [motor.read() for motor in self.motors]
        return self.motors2joint(motor_angles)

    def at_position(self):
        """
        returns
            True if all the motors in the joint are at their target positions, False otherwise
        """
        at_position = True
        for motor in self.motors:
            at_position = at_position and motor.at_position()
        return at_position

    @property
    def angle(self):
        """
        returns
            the joint's current angle in radians
        """
        return self.read()

    def __str__(self):
        return self.id.name + ": " + f"{self.angle:.3f} rad"

    def __getitem__(self, motor_index):
        """
        To support 'obj[motor_index]' for getting motors
        """
        return self.motors[motor_index]
