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
import dextrous_hand.constants as constants
from dextrous_hand.Motor import Motor

# Motor-specific constants
MY_DXL = 'X_SERIES'
ADDR_TORQUE_ENABLE          = 64
ADDR_TARGET_POSITION          = 116
ADDR_CURRENT_POSITION       = 132
LEN_TARGET_POSITION           = 4         # Data Byte Length
LEN_CURRENT_POSITION        = 4         # Data Byte Length
DXL_MINIMUM_POSITION_VALUE  = 0
DXL_MAXIMUM_POSITION_VALUE  = 4095
BAUDRATE                    = 57600 # Change only if you have changed the baudrate of the motors
PROTOCOL_VERSION            = 2.0

TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 10    # Dynamixel moving status threshold

from rclpy import logging
def LOG_INFO(log):
    logging.get_logger('MotorBridge').info(log)

def LOG_ERROR(log):
    logging.get_logger('MotorBridge').error(log)

def LOG_WARN(log):
    logging.get_logger('MotorBridge').warn(log)

class MotorBridge():
    """
    This class serves as a handler for communicating with the Dynamixel board sending a single message and setting all motors at once, as well as reading the current positions of all motors at once.
    """

    _instance = None

    def __new__(cls):
        """
        Singleton pattern. Make sure only one instance of MotorBridge is created.
        """
        if cls._instance is None:
            cls._instance = super(MotorBridge, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        """
        Open the port and initialize the motors.
        """
        if hasattr(self, 'initialized') and self.initialized:
            return

        if constants.IS_SIMULATION:
            LOG_INFO("MotorBridge is in simulation mode")

        else:
            self.setup_port()

        self.motors = []

        # Initialize motors
        for motor_id in constants.MOTORS:

            # Only setup communication if the motors are actually connected
            if constants.IS_SIMULATION:
                LOG_INFO("Motor %s is in simulation mode" % motor_id.name)
                self.motors.append(Motor(motor_id))

            else:
                # Enable Dynamixel Torque. Also works as a check for motor connection
                dxl_comm_result, dxl_error = self.PACKET_HANDLER.write1ByteTxRx(self.PORT_HANDLER, motor_id.value, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

                # Check if connection is unsuccessful
                if dxl_comm_result != DXL.COMM_SUCCESS:
                    LOG_ERROR("Motor %s: %s" % (motor_id.name, self.PACKET_HANDLER.getTxRxResult(dxl_comm_result)))
                elif dxl_error != 0:
                    LOG_ERROR("Motor %s: %s" % (motor_id.name, self.PACKET_HANDLER.getRxPacketError(dxl_error)))
                else:
                    # Add motor to the reader instance
                    dxl_addparam_result = self.GROUP_SYNC_READ.addParam(motor_id.value)
                    if dxl_addparam_result != True:
                        LOG_ERROR("Motor %s groupSyncRead setup failed" % motor_id.name)
                    else:
                        LOG_INFO("Motor %s has been successfully connected" % motor_id.name)
                        self.motors.append(Motor(motor_id))

        self.initialized = True

    def setup_port(self):

        port_opened = False
        for DEVICENAME in constants.DEVICENAMES:
            # Initialize PortHandler instance
            # Set the port path
            # Get methods and members of PortHandlerLinux or PortHandlerWindows
            self.PORT_HANDLER = DXL.PortHandler(DEVICENAME)

            # Initialize PacketHandler instance
            # Set the protocol version
            # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
            self.PACKET_HANDLER = DXL.PacketHandler(PROTOCOL_VERSION)

            # Initialize GroupSyncWrite instance
            self.GROUP_SYNC_WRITE = DXL.GroupSyncWrite(self.PORT_HANDLER, self.PACKET_HANDLER, ADDR_TARGET_POSITION, LEN_TARGET_POSITION)

            # Initialize GroupSyncRead instace for Current Position
            self.GROUP_SYNC_READ = DXL.GroupSyncRead(self.PORT_HANDLER, self.PACKET_HANDLER, ADDR_CURRENT_POSITION, LEN_CURRENT_POSITION)

            # Open USB port
            try:
                if self.PORT_HANDLER.openPort():
                    LOG_INFO("Succeeded to open the port on %s" % DEVICENAME)
                    port_opened = True
                    break
            except:
                LOG_WARN("Failed to open port on %s" % DEVICENAME)

        if not port_opened:
            LOG_ERROR("Failed to open any USB port. Check if the motor controller is connected and remember 'sudo chmod 777 /dev/ttyUSB0 or /dev/ttyUSB1'")
            quit()

        # Set port baudrate
        if  self.PORT_HANDLER.setBaudRate(BAUDRATE):
            LOG_INFO("Succeeded to change the baudrate")
        else:
            LOG_ERROR("Failed to change the baudrate")
            quit()

    def write_targets(self):
        """
        Write the target for each motor in the synchronized writer and send a single message to all motors.
        """
        if constants.IS_SIMULATION:
            return

        for motor in self.motors:
            # Allocate target position value into byte array
            target_position = [DXL.DXL_LOBYTE(DXL.DXL_LOWORD(motor.target)),
                            DXL.DXL_HIBYTE(DXL.DXL_LOWORD(motor.target)),
                            DXL.DXL_LOBYTE(DXL.DXL_HIWORD(motor.target)),
                            DXL.DXL_HIBYTE(DXL.DXL_HIWORD(motor.target))]

            # Add target position value to the Syncwrite parameter storage
            dxl_addparam_result = self.GROUP_SYNC_WRITE.addParam(motor.port, target_position)
            if dxl_addparam_result != True:
                LOG_ERROR("Failed to add %s target to write message" % motor.name)

        # Syncwrite all target positions simultaneously
        dxl_comm_result = self.GROUP_SYNC_WRITE.txPacket()
        if dxl_comm_result != DXL.COMM_SUCCESS:
            LOG_ERROR("%s" % self.PACKET_HANDLER.getTxRxResult(dxl_comm_result))
            return False

        return True

    def update_positions(self):
        """
        Read the current encoder value for all motors simultaneously
        """
        if constants.IS_SIMULATION:
            for motor in self.motors:
                motor.current_position = motor.target
            return

        # Get the single message containing all positions
        dxl_comm_result = self.GROUP_SYNC_READ.txRxPacket()
        if dxl_comm_result != DXL.COMM_SUCCESS:
            LOG_ERROR("%s" % self.PACKET_HANDLER.getTxRxResult(dxl_comm_result))
            return False

        # Extract the current position of each motor from the single message
        for motor in self.motors:
            # Check if the motor's current position is available
            dxl_getdata_result = self.GROUP_SYNC_READ.isAvailable(motor.port, ADDR_CURRENT_POSITION, LEN_CURRENT_POSITION)
            if dxl_getdata_result != True:
                LOG_ERROR("Failed to read position from %s" % motor.name)
            else:
                # Extract the current position from the byte array
                motor.current_position = self.GROUP_SYNC_READ.getData(motor.port, ADDR_CURRENT_POSITION, LEN_CURRENT_POSITION)

        return True
