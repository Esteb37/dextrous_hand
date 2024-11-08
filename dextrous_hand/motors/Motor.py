#!/usr/bin/env python

import dextrous_hand.utils.ids as ids
import dextrous_hand.utils.constants as constants

class Motor():
    """
    Class to control a single motor
    """

    _instances = {}
    _ports = {}

    # Error threshold to be considered at position
    AT_ANGLE_THRESHOLD = 0.1

    def __new__(cls, motor_id, *args, **kwargs):
        """
        Singleton pattern. Make sure only one instance of each Motor is created and no motors share the same port.
        If it has already been created, return the existing instance.
        If the port is already occupied, raise an exception.

        param motor_id: the ID of the motor
        """

        if motor_id not in cls._instances:
            cls._instances[motor_id] = super(Motor, cls).__new__(cls)

        # Check if the port is already occupied
        if motor_id.value in cls._ports and cls._ports[motor_id.value] != motor_id:
            raise Exception("Port " + str(motor_id.value) + " is already occupied by motor " + str(cls._ports[motor_id.value].name))

        cls._ports[motor_id.value] = motor_id
        return cls._instances[motor_id]


    def __init__(self, motor_id : ids.MOTORS):
        """
        params
            motor_id [MOTORS]: the motor's ID

        raises
            Exception: if the motor has no limits
            Exception: if the motor port has already been occupied
        """

        # Avoid reinitialization if the instance already exists
        if hasattr(self, 'initialized') and self.initialized:
            return

        self.id = motor_id
        self.port = motor_id.value
        self.name = motor_id.name

        # Check if the motor's zero position has been defined
        self.zero = constants.MOTOR_CONSTANTS[self.name]["zero"]

        if  self.zero is None:
            raise Exception("Motor " + str(self.name) + " has no zero position")

        self.angle_limits = constants.MOTOR_CONSTANTS[self.name]["range"]

        # Check if the motor's limits have been defined
        if self.angle_limits is None or len(self.angle_limits) != 2:
            raise Exception("Motor " + str(self.name) + " has no limits or invalid limits")

        self.direction = constants.MOTOR_CONSTANTS[self.name]["direction"]

        # Check if the motor's direction has been defined
        if self.direction is None:
            raise Exception("Motor " + str(self.name) + " has no direction")

        self.target = 0.0

        self.angle = 0.0

        self.initialized = True


    def write(self, angle):
        """
        Map angle to absolute position and write to the motor

        params:
            angle: the angle in radians to set the motor to

        returns:
            True if the motor is at the target position, False otherwise
        """
        target = min(max(angle, self.angle_limits[0]), self.angle_limits[1])
        self.target = target
        return self.at_angle()

    def read(self):
        """
        returns:
            the motor's current angle in radians
        """
        return self.angle

    @property
    def target(self):
        return (self.dxl_target - self.zero) * self.direction

    @target.setter
    def target(self, target):
        target = target * self.direction + self.zero
        # Limit to dynamixel's range
        from dextrous_hand.motors.DynamixelClient import DXL_MAXIMUM_ANGLE_VALUE
        target = min(max(target, 0), DXL_MAXIMUM_ANGLE_VALUE)
        self.dxl_target = target

    @property
    def angle(self):
        return (self.dxl_angle - self.zero) * self.direction

    @angle.setter
    def angle(self, angle):
        self.dxl_angle = angle * self.direction + self.zero

    def at_angle(self):
        """
        returns:
            True if the motor is within the error threshold to the target position, False otherwise
        """
        return abs(self.target - self.angle) < self.AT_ANGLE_THRESHOLD

    def __str__(self):
        return "Motor %s" % self.name + ": " + f"angle = {self.angle:.3f}" + " rad \t target = " + f"{self.target:.3f}" + " rad \t dxl_angle = " + f"{self.dxl_angle:.3f}" + " rad \t dxl_target = " + f"{self.dxl_target:.3f}" + " rad"
