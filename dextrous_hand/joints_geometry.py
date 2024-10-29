#!/usr/bin/env python3

from numpy import pi as PI
from dextrous_hand.ids import JOINTS

GEOMETRY_TYPES = {
    "P4_FINGER_ABD":{"centers_distance":1, "T1":[0,0,1], "T2":[0,0,1], "T3":[0,0,1], "T4":[0,0,1], "length_12_0":1, "length_34_0":1, "range":[-PI/2,PI/2]},
    "P4_FINGER_MCP":{"centers_distance":1, "T1":[0,0,1], "T2":[0,0,1], "T3":[0,0,1], "T4":[0,0,1], "length_12_0":1, "length_34_0":1, "range":[0,PI/2]},
    "P4_FINGER_PIP":{"centers_distance":1, "T1":[0,0,1], "T2":[0,0,1], "T3":[0,0,1], "T4":[0,0,1], "length_12_0":1, "length_34_0":1, "range":[0,PI/2]},
    "P4_FINGER_DIP":{"centers_distance":1, "T1":[0,0,1], "T2":[0,0,1], "T3":[0,0,1], "T4":[0,0,1], "length_12_0":1, "length_34_0":1, "range":[0,PI/2]},
    "THUMB_ABD":{"radius":1, "range":[-PI/2,PI/2]}, 
    "THUMB_MCP":{"radius":1, "range":[0,PI/2]},
    "THUMB_PIP":{"centers_distance":1, "T1":[0,0,1], "T2":[0,0,1], "T3":[0,0,1], "T4":[0,0,1], "length_12_0":1, "length_34_0":1, "range":[0,PI/2]},
    "THUMB_DIP":{"centers_distance":1, "T1":[0,0,1], "T2":[0,0,1], "T3":[0,0,1], "T4":[0,0,1], "length_12_0":1, "length_34_0":1, "range":[0,PI/2]},
    "WRIST":{"radius":1}
}

JOINTS_GEOMETRY = {
    JOINTS.PINKY_ABD: GEOMETRY_TYPES["P4_FINGER_ABD"],
    JOINTS.PINKY_MCP: GEOMETRY_TYPES["P4_FINGER_MCP"],
    JOINTS.PINKY_PIP: GEOMETRY_TYPES["P4_FINGER_PIP"],
    JOINTS.PINKY_DIP: GEOMETRY_TYPES["P4_FINGER_DIP"],

    JOINTS.RING_ABD: GEOMETRY_TYPES["P4_FINGER_ABD"],
    JOINTS.RING_MCP: GEOMETRY_TYPES["P4_FINGER_MCP"],
    JOINTS.RING_PIP: GEOMETRY_TYPES["P4_FINGER_PIP"],
    JOINTS.RING_DIP: GEOMETRY_TYPES["P4_FINGER_DIP"],

    JOINTS.MIDDLE_ABD: GEOMETRY_TYPES["P4_FINGER_ABD"],
    JOINTS.MIDDLE_MCP: GEOMETRY_TYPES["P4_FINGER_MCP"],
    JOINTS.MIDDLE_PIP: GEOMETRY_TYPES["P4_FINGER_PIP"],
    JOINTS.MIDDLE_DIP: GEOMETRY_TYPES["P4_FINGER_DIP"],

    JOINTS.INDEX_ABD: GEOMETRY_TYPES["P4_FINGER_ABD"],
    JOINTS.INDEX_MCP: GEOMETRY_TYPES["P4_FINGER_MCP"],
    JOINTS.INDEX_PIP: GEOMETRY_TYPES["P4_FINGER_PIP"],
    JOINTS.INDEX_DIP: GEOMETRY_TYPES["P4_FINGER_DIP"],

    JOINTS.THUMB_ABD: GEOMETRY_TYPES["THUMB_ABD"],
    JOINTS.THUMB_MCP: GEOMETRY_TYPES["THUMB_MCP"],
    JOINTS.THUMB_PIP: GEOMETRY_TYPES["THUMB_PIP"],
    JOINTS.THUMB_DIP: GEOMETRY_TYPES["THUMB_DIP"],

    JOINTS.WRIST: GEOMETRY_TYPES["WRIST"]
}
"""
joint geometry for each type of joint and assign it to the existing joint instances
"""