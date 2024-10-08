#!/usr/bin/env python3

from dextrous_hand.Subsystem import Subsystem
import dextrous_hand.constants as constants

# Singleton instances
THUMB = Subsystem(constants.IDS.THUMB)
INDEX = Subsystem(constants.IDS.INDEX)
MIDDLE = Subsystem(constants.IDS.MIDDLE)
RING = Subsystem(constants.IDS.RING)
PINKY = Subsystem(constants.IDS.PINKY)

FINGERS = [THUMB, INDEX, MIDDLE, RING, PINKY]
