#!/usr/bin/env python3

from dextrous_hand.Subsystem import Subsystem
import dextrous_hand.constants as constants

# Singleton instance
WRIST = Subsystem(constants.SUBSYSTEMS.WRIST)
