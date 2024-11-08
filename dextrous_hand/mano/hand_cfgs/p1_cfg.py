import numpy as np
from typing import Dict

# the information of the tendons in the hand. Each tendon represents a grouped actuation.
GC_TENDONS = {
    "root2thumb_base": {},
    "thumb_base2pp": {},
    "thumb_pp2mp_virt": {
        "thumb_pp2mp": 1,
        "thumb_mp2dp_virt": 0.71,
        "thumb_mp2dp": 0.71,
    },
    "root2index_pp_virt": {"root2index_pp": 1},
    "index_pp2mp_virt": {
        "index_pp2mp": 1,
        "index_mp2dp_virt": 0.71,
        "index_mp2dp": 0.71,
    },
    "root2middle_pp_virt": {"root2middle_pp": 1},
    "middle_pp2mp_virt": {
        "middle_pp2mp": 1,
        "middle_mp2dp_virt": 0.71,
        "middle_mp2dp": 0.71,
    },
    "root2ring_pp_virt": {"root2ring_pp": 1},
    "ring_pp2mp_virt": {"ring_pp2mp": 1, "ring_mp2dp_virt": 0.71, "ring_mp2dp": 0.71},
    "root2pinky_pp_virt": {"root2pinky_pp": 1},
    "pinky_pp2mp_virt": {
        "pinky_pp2mp": 1,
        "pinky_mp2dp_virt": 0.71,
        "pinky_mp2dp": 0.71,
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

# the mapping from fingername to the frame of the fingerbase (The base that fixed to the palm)
# Use pytorch_kinematics.Chain.print_tree() to see the base frame
FINGER_TO_BASE = {
    "thumb": "thumb_base",
    "index": "index_pp_virt",
    "middle": "middle_pp_virt",
    "ring": "ring_pp_virt",
    "pinky": "pinky_pp_virt",
}

GC_LIMITS_LOWER = np.array(
    [
        0.0,
        -95.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]
)
GC_LIMITS_UPPER = np.array(
    [130.0, 60.0, 110.0, 95.0, 110.0, 95.0, 110.0, 95.0, 110.0, 95.0, 110.0]
)
