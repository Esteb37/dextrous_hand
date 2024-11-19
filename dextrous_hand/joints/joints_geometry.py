#!/usr/bin/env python3

from numpy import pi as PI
from dextrous_hand.utils.ids import JOINTS
import math

# in mm
SPOOL_RADIUS = 8.5
SPOOL_RADIUS_WRIST = 4

z_offset = 0.765

# in mm
GEOMETRY_TYPES = {
    "P4_FINGER_ABD":{"centers_distance":18, "T1":[-4.042,8.041,0], "T2":[-3.982,-8.071,0], "T3":[4.042,8.041,0], "T4":[3.982,-8.071,0], "length_12_0":1.889, "length_34_0":1.889, "range":[-PI/6,PI/6]},
    "P4_FINGER_MCP":{"centers_distance":18, "T1":[-9,4.336,0], "T2":[-8.813,-1.827,0], "T3":[2.4,9.157,0], "T4":[2.563,-8.627,0], "length_12_0":11.838, "length_34_0":0.27, "range":[0,PI/2]},
    "P4_FINGER_PIP":{"centers_distance":12, "T1":[-6.42,-2.46,0], "T2":[-5.989,0.358,0], "T3":[1.443,5.606,0], "T4":[1.443,-5.571,0], "length_12_0":14.83, "length_34_0":0.823, "range":[0,PI/2]},
    "P4_FINGER_DIP":{"centers_distance":9, "T1":[-4.495,0.218,0], "T2":[-4.348,-1.16,0], "T3":[1.75,4.146,0], "T4":[1.75,-4.146,0], "length_12_0":7.623, "length_34_0":0.707, "range":[0,PI/2]},
    "THUMB_ABD":{"radius":11.6, "range":[0,PI/4+0.698]}, # 40 deg
    "THUMB_MCP":{"radius":10, "range":[0,2.356]}, # 135 deg
    "THUMB_PIP":{"centers_distance":18, "T1":[-8.917,4.495,z_offset], "T2":[-8.796,-1.905,0], "T3":[2.575,9.376,0], "T4":[2.575,-8.624,0], "length_12_0":1, "length_34_0":math.sqrt(11.601**2+z_offset**2), "range":[0,PI/2]},
    "THUMB_DIP":{"centers_distance":9, "T1":[-4.495,0.218,0], "T2":[-4.348,-1.16,0], "T3":[1.75,4.146,0], "T4":[1.75,-4.146,0], "length_12_0":7.623, "length_34_0":0.707, "range":[0,PI/2]},
    "WRIST":{"radius":16, "range":[-PI/2, PI/2]}
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