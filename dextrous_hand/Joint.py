#!/usr/bin/env python3

import dextrous_hand.constants as constants

class Joint():
    # For the singleton pattern
    _name_instances = {}
    _motor_instances = {}

    _instances = {}

    def __new__(cls, name, motor_port, *args, **kwargs):
        """
        Singleton pattern. Make sure only one instance of each Joint is created.
        If a joint with the same name has already been created, return that instance.
        If a joint with the same motor_port has already been created, raise an exception so no two joints share the same motor_port.
        """

        # Check if an instance with this name already exists
        if name in cls._name_instances:
            return cls._name_instances[name]
        # Check if a motor with this motor_port already exists
        if motor_port in cls._motor_instances:
            raise Exception(f"Motor ID {motor_port} is already in use by another Joint instance.")

        # Create a new instance
        instance = super(Joint, cls).__new__(cls)
        cls._name_instances[name] = instance
        cls._motor_instances[motor_port] = instance
        return instance

    def __init__(self, name, motor_port, limits):
        # Avoid reinitialization if already initialized
        if hasattr(self, 'initialized') and self.initialized:
            return

        self.name = name
        self.motor_port = motor_port
        self.limits = limits

        self.position = 0.0
        self.velocity = 0.0

        self.initialized = True

    def send_data(self, data):
        """
        Send the motor angle to the controller

        param data: the information in a format that is understandable by the controller
        """
        pass

    def position_to_data(self, position):
        """
        Convert the motor angle to a format that can be sent to the controller

        param position: the motor angle

        return data: the information in a format that is understandable by the controller
        """
        pass

    def set_position(self, position):
        """
        Set the joint's motor to a specific angle

        param position: the angle to set the motor to
        """

        self.position = max(self.limits[0], min(self.limits[1], position))
        data = self.position_to_data(position)
        self.send_data(data)

    def print(self, verbose= False):
        if verbose:
            print(str(self))
        else:
            print(self.name, ":", self.position)

    def __str__(self):
        string = "----" + self.name + "----\n"
        string += "Motor port: " + str(self.motor_port) + "\n"
        string += "Position: " + str(self.position) + "\n"
        string += "Velocity: " + str(self.velocity) + "\n"
        string += "Limits: " + str(self.limits) + "\n"
        return string

WRIST = Joint(constants.IDS.WRIST.name,
              constants.MOTOR_PORTS[constants.IDS.WRIST],
              constants.MOTOR_LIMITS[constants.IDS.WRIST])

ABDUCTION = Joint(constants.IDS.ABDUCTION.name,
                  constants.MOTOR_PORTS[constants.IDS.ABDUCTION],
                  constants.MOTOR_LIMITS[constants.IDS.ABDUCTION])