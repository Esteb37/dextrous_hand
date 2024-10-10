#!/usr/bin/env python

# Partially copied from the Dynamixel SDK example code

#*******************************************************************************
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#*******************************************************************************


#*******************************************************************************
#***********************     Read and Write Example      ***********************
#  Required Environment to run this example :
#    - Protocol 2.0 supported DYNAMIXEL(X, P, PRO/PRO(A), MX 2.0 series)
#    - DYNAMIXEL Starter Set (U2D2, U2D2 PHB, 12V SMPS)
#  How to use the example :
#    - Select the DYNAMIXEL in use at the MY_DXL in the example code.
#    - Build and Run from proper architecture subdirectory.
#    - For ARM based SBCs such as Raspberry Pi, use linux_sbc subdirectory to build and run.
#    - https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/
#  Author: Ryu Woon Jung (Leon)
#  Maintainer : Zerom, Will Son
# *******************************************************************************

import os
import dextrous_hand.constants as constants
import numpy as np

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

import dynamixel_sdk as DXL

# Motor-specific constants
MY_DXL = 'X_SERIES'
ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132
DXL_MINIMUM_POSITION_VALUE  = 0
DXL_MAXIMUM_POSITION_VALUE  = 4095
BAUDRATE                    = 57600 # Change only if you have changed the baudrate of the motors
PROTOCOL_VERSION            = 2.0
DEVICENAME                  = constants.DEVICENAME

TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold

PORT_HANDLER = DXL.PortHandler(DEVICENAME)
PACKET_HANDLER = DXL.PortHandler(PROTOCOL_VERSION)


if constants.IS_SIMULATION:
    print("Running in simulation mode")
else:
    if PORT_HANDLER.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port " + DEVICENAME + ". Don't forget to do sudo chmod 777 "+DEVICENAME)
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    if  PORT_HANDLER.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

class Motor():

    _instances = {}
    _ports = {}

    AT_POSITION_THRESHOLD = 10

    def __new__(cls, motor_id, *args, **kwargs):
        """
        Singleton pattern. Make sure only one instance of each Motor is created and no motors share the same port.
        If it has already been created, return the existing instance.
        If the port is already occupied, raise an exception.

        param motor_id: the ID of the motor
        """

        if motor_id.value in cls._ports:
            raise Exception("Port " + str(motor_id.value) + " is already occupied by motor " + cls._ports[motor_id.value].name)

        cls._ports[motor_id.value] = motor_id

        if motor_id not in cls._instances:
            cls._instances[motor_id] = super(Motor, cls).__new__(cls)
        return cls._instances[motor_id]


    def __init__(self, motor_id):
        # Avoid reinitialization if the instance already exists
        if hasattr(self, 'initialized') and self.initialized:
            return

        self.id = motor_id

        if not constants.IS_SIMULATION:
            # Enable Dynamixel Torque
            dxl_comm_result, dxl_error = PACKET_HANDLER.write1ByteTxRx(PORT_HANDLER, self.id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE) # type: ignore
            if dxl_comm_result != DXL.COMM_SUCCESS:
                print("Motor %s: %s" % (self.id.name, PACKET_HANDLER.getTxRxResult(dxl_comm_result))) # type: ignore
            elif dxl_error != 0:
                print("Motor %s: %s" % (self.id.name, PACKET_HANDLER.getRxPacketError(dxl_error))) # type: ignore
            else:
                print("Motor %s has been successfully connected" % self.id.name)
        else:
            print("Motor %s is in simulation mode" % self.id.name)

        if self.id not in constants.MOTOR_LIMITS or len(constants.MOTOR_LIMITS[self.id]) == 0:
            raise Exception("Motor " + self.id.name + " has no limits")

        angle_limits = constants.MOTOR_LIMITS[self.id]
        self.min_position = (angle_limits[0] + np.pi) * DXL_MAXIMUM_POSITION_VALUE / (2 * np.pi)
        self.max_position = (angle_limits[1] + np.pi) * DXL_MAXIMUM_POSITION_VALUE / (2 * np.pi)

        self.target = 0

        self.initialized = True

    def write_position(self, position):
        """
            Send the position to the motor
        """
        position = max(min(position, self.max_position), self.min_position)
        self.target = position

        if not constants.IS_SIMULATION:
            dxl_comm_result, dxl_error = PACKET_HANDLER.write4ByteTxRx(PORT_HANDLER, self.id, ADDR_GOAL_POSITION, position) # type: ignore
            if dxl_comm_result != DXL.COMM_SUCCESS:
                print("Motor %s: %s" % (self.id.name, PACKET_HANDLER.getTxRxResult(dxl_comm_result))) # type: ignore
            elif dxl_error != 0:
                print("Motor %s: %s" % (self.id.name, PACKET_HANDLER.getRxPacketError(dxl_error))) # type: ignore

    def write(self, angle):
        """
            Map (-pi, pi) to (0, MAX_POS) and write the position to the motor
        """
        position = (angle + np.pi) * DXL_MAXIMUM_POSITION_VALUE / (2 * np.pi)
        self.write_position(position)


    def read_position(self):

        if constants.IS_SIMULATION:
            return self.target

        dxl_present_position, dxl_comm_result, dxl_error = PACKET_HANDLER.read4ByteTxRx(PORT_HANDLER, self.id, ADDR_PRESENT_POSITION) # type: ignore
        if dxl_comm_result != DXL.COMM_SUCCESS:
            print("Motor %s: %s" % (self.id.name, PACKET_HANDLER.getTxRxResult(dxl_comm_result))) # type: ignore
        elif dxl_error != 0:
            print("Motor %s: %s" % (self.id.name, PACKET_HANDLER.getRxPacketError(dxl_error))) # type: ignore
        return dxl_present_position

    def read(self):
        """
            Read the position of the motor and map it to (-pi, pi)
        """
        position = self.read_position()
        return (position * 2 * np.pi / DXL_MAXIMUM_POSITION_VALUE) - np.pi

    def at_position(self):
        return abs(self.target - self.read_position()) < self.AT_POSITION_THRESHOLD

    @property
    def angle(self):
        return self.read()

    @property
    def position(self):
        return self.read_position()

    def __str__(self):
        return "Motor %s" % self.id.name + ": " + f"{self.angle:.3f}" + " rad"
