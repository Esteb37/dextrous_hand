import numpy as np
from typing import Dict

from dextrous_hand.utils.constants import RETARGETER_PARAMS

coupled_scale = RETARGETER_PARAMS["global"]["coupled_scale"]

# the information of the tendons in the hand. Each tendon represents a grouped actuation.
GC_TENDONS = {
    "root2thumb_base": {},
    "thumb_base2pp": {},
    "thumb_pp2mp_virt": {
        "thumb_pp2mp": 1,
    },
    "thumb_mp2dp_virt": {
        "thumb_mp2dp": 1,
    },

    "index_abd_virt": {"index_abd": 1},
    "root2index_pp_virt": {"root2index_pp": 1},
    "index_pp2mp_virt": {
        "index_pp2mp": 1,
        "index_mp2dp_virt": coupled_scale,
        "index_mp2dp": coupled_scale,
    },

    "middle_abd_virt": {"middle_abd": 1},
    "root2middle_pp_virt": {"root2middle_pp": 1},
    "middle_pp2mp_virt": {
        "middle_pp2mp": 1,
        "middle_mp2dp_virt": coupled_scale,
        "middle_mp2dp": coupled_scale,
    },

    "ring_abd_virt": {"ring_abd": 1},
    "root2ring_pp_virt": {"root2ring_pp": 1},
    "ring_pp2mp_virt": {
        "ring_pp2mp": 1,
        "ring_mp2dp_virt": coupled_scale,
        "ring_mp2dp": coupled_scale
    },

    "pinky_abd_virt": {"pinky_abd": 1},
    "root2pinky_pp_virt": {"root2pinky_pp": 1},
    "pinky_pp2mp_virt": {
        "pinky_pp2mp": 1,
        "pinky_mp2dp_virt": coupled_scale,
        "pinky_mp2dp": coupled_scale,
    },
}

# the mapping from fingername to the frame of the fingertip
# Use pytorch_kinematics.Chain.print_tree() to see the tip frame
FINGER_TO_TIP: Dict[str, str] = {
    "thumb": "thumb_fingertip",
    "index": "index_fingertip",
    "middle": "middle_fingertip",
    "ring": "ring_fingertip",
    "pinky": "pinky_fingertip",
}

FINGER_TO_MP: Dict[str, str] = {
    "index": "index_mp_virt",
    "middle": "middle_mp_virt",
    "ring": "ring_mp_virt",
    "pinky": "pinky_mp_virt",
}

# the mapping from fingername to the frame of the fingerbase (The base that fixed to the palm)
# Use pytorch_kinematics.Chain.print_tree() to see the base frame
FINGER_TO_BASE = {
    "thumb": "thumb_base",
    "index": "index_um_virt",
    "middle": "middle_um_virt",
    "ring": "ring_um_virt",
    "pinky": "pinky_um_virt",
}

WRIST_NAME = "retarget_palm"