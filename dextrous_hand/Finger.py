#!/usr/bin/env python3

from dextrous_hand.Subsystem import Subsystem
import dextrous_hand.constants as constants
from enum import Enum

# Singleton instances
THUMB = Subsystem(constants.SUBSYSTEMS.THUMB)
INDEX = Subsystem(constants.SUBSYSTEMS.INDEX)
MIDDLE = Subsystem(constants.SUBSYSTEMS.MIDDLE)
RING = Subsystem(constants.SUBSYSTEMS.RING)
PINKY = Subsystem(constants.SUBSYSTEMS.PINKY)

FINGERS = [THUMB, INDEX, MIDDLE, RING, PINKY]