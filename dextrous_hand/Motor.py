#!/usr/bin/env python

import os
import dextrous_hand.constants as constants
import numpy as np

DXL_MAXIMUM_POSITION_VALUE  = 4095

class Motor():
    """
    Class to control a single motor
    """

    _instances = {}
    _ports = {}

    # Error threshold to be considered at position
    AT_POSITION_THRESHOLD = 10

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


    def __init__(self, motor_id : constants.MOTORS):
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

        # Check if the motor's limits have been defined
        if self.id not in constants.MOTOR_LIMITS or len(constants.MOTOR_LIMITS[self.id]) != 2:
            raise Exception("Motor " + str(self.name) + " has no limits or invalid limits")

        # Map the angle limits to hard position limits
        angle_limits = constants.MOTOR_LIMITS[self.id]
        self.min_position = (angle_limits[0] + np.pi) * DXL_MAXIMUM_POSITION_VALUE / (2 * np.pi)
        self.max_position = (angle_limits[1] + np.pi) * DXL_MAXIMUM_POSITION_VALUE / (2 * np.pi)

        self.target = 0

        self.current_position = 0

        self.initialized = True

    def write_position(self, position):
        """
        Send the absolute position to the motor

        params:
            position: the absolute position to send to the motor

        returns:
            True if the motor is at the target position, False otherwise
        """

        # Make sure the position is within the motor's limits
        position = int(max(min(position, self.max_position), self.min_position))

        # Set the target position (current position is not equal to target position because it needs time to move)
        self.target = position

        return self.at_position()

    def write(self, angle):
        """
        Map angle to absolute position and write to the motor

        params:
            angle: the angle in radians to set the motor to

        returns:
            True if the motor is at the target position, False otherwise
        """
        position = (angle + np.pi) * DXL_MAXIMUM_POSITION_VALUE / (2 * np.pi)
        return self.write_position(position)

    def read_position(self):
        """
        returns:
            the motor's current position
        """
        return self.current_position

    def read(self):
        """
        returns:
            the motor's current angle in radians
        """
        position = self.read_position()
        return (position * 2 * np.pi / DXL_MAXIMUM_POSITION_VALUE) - np.pi

    def at_position(self):
        """
        returns:
            True if the motor is within the error threshold to the target position, False otherwise
        """
        return abs(self.target - self.read_position()) < self.AT_POSITION_THRESHOLD

    @property
    def angle(self):
        """
        returns:
            the motor's current angle in radians
        """
        return self.read()

    @property
    def position(self):
        """
            returns the motor's current position
        """
        return self.read_position()

    def __str__(self):
        return "Motor %s" % self.name + ": " + f"{self.angle:.2f}" + " rad"
