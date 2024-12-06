
import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from dextrous_hand.utils.constants import RETARGETER_PARAMS


class ManoHandVisualizer:
    def __init__(self, marker_publisher):
        self.marker_publisher = marker_publisher
        self.markers = []

    def reset_markers(self):
        self.markers = []

    def publish_markers(self):
        marker_array = MarkerArray()
        marker_array.markers = self.markers
        for idx, marker in enumerate(marker_array.markers):
            marker.id = idx

        self.marker_publisher.publish(marker_array)

    def generate_hand_markers(self, joints, stamp):
        markers = []

        # Create marker for joints
        joint_marker = Marker()
        joint_marker.header.frame_id = "world"
        joint_marker.header.stamp = stamp
        joint_marker.ns = "joints"
        joint_marker.type = Marker.POINTS
        joint_marker.action = Marker.ADD
        joint_marker.scale.x = 0.01  # Point width
        joint_marker.scale.y = 0.01  # Point height
        joint_marker.color.a = 1.0
        joint_marker.color.r = 1.0  # Red color

        # Add joint points
        for joint in joints:
            p = Point(x=float(joint[0].item()), y=float(joint[1].item()), z=float(joint[2].item()))
            joint_marker.points.append(p) # type: ignore

        markers.append(joint_marker)

        # Create marker for bones
        bones = [
            (0, 1),
            (1, 2),
            (2, 3),
            (3, 4),  # Thumb
            (0, 5),
            (5, 6),
            (6, 7),
            (7, 8),  # Index finger
            (0, 9),
            (9, 10),
            (10, 11),
            (11, 12),  # Middle finger
            (0, 13),
            (13, 14),
            (14, 15),
            (15, 16),  # Ring finger
            (0, 17),
            (17, 18),
            (18, 19),
            (19, 20),  # Pinky
        ]

        bone_marker = Marker()
        bone_marker.header.frame_id = "world"
        bone_marker.header.stamp = stamp
        bone_marker.ns = "bones"
        bone_marker.type = Marker.LINE_LIST
        bone_marker.action = Marker.ADD
        bone_marker.scale.x = 0.005  # Line width
        bone_marker.color.a = 1.0
        bone_marker.color.b = 1.0  # Blue color

        # Add bone lines
        for bone in bones:
            start_joint = joints[bone[0]]
            end_joint = joints[bone[1]]
            p_start = Point(x=float(start_joint[0]), y=float(start_joint[1]), z=float(start_joint[2]))
            p_end = Point(x=float(end_joint[0]), y=float(end_joint[1]), z=float(end_joint[2]))
            bone_marker.points.append(p_start) # type: ignore
            bone_marker.points.append(p_end) # type: ignore

        markers.append(bone_marker)

        self.markers.extend(markers)

    def generate_frame_markers(self, origin, x_axis, y_axis, z_axis, stamp):
        markers = []
        axes = {
            "x": (x_axis, (1.0, 0.0, 0.0)),  # Red
            "y": (y_axis, (0.0, 1.0, 0.0)),  # Green
            "z": (z_axis, (0.0, 0.0, 1.0)),  # Blue
        }
        for i, (axis_name, (axis_vector, color)) in enumerate(axes.items()):
            arrow_marker = Marker()
            arrow_marker.header.frame_id = "world"
            arrow_marker.header.stamp = stamp
            arrow_marker.ns = "frame"
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.scale.x = 0.005  # Shaft diameter
            arrow_marker.scale.y = 0.008  # Head diameter
            arrow_marker.scale.z = 0.01  # Head length
            arrow_marker.color.a = 1.0
            arrow_marker.color.r = color[0]
            arrow_marker.color.g = color[1]
            arrow_marker.color.b = color[2]

            # Start and end points of the arrow
            p_start = Point(x=float(origin[0]), y=float(origin[1]), z=float(origin[2]))
            p_end = Point(
                x=float(origin[0] + axis_vector[0]),
                y=float(origin[1] + axis_vector[1]),
                z=float(origin[2] + axis_vector[2]),
            )
            arrow_marker.points.append(p_start) # type: ignore
            arrow_marker.points.append(p_end) # type: ignore

            markers.append(arrow_marker)

        self.markers.extend(markers)

    def generate_keyvector_markers(self, fingertips, mps, palm, stamp, detach = False):

        markers = []

        if detach:
            tips = [tip_tensor.detach().cpu().numpy()[0] for tip_tensor in fingertips.values()]

            mps = [mp.detach().cpu().numpy()[0] for mp in mps.values()]

            palm = palm.detach().cpu().numpy()[0]
        else:
            tips = [tip_tensor.cpu().numpy()[0] for tip_tensor in fingertips.values()]

            mps = [mp.cpu().numpy()[0] for mp in mps.values()]

            palm = palm.cpu().numpy()[0]

        # Create marker for joints

        if detach:
            joint_marker = Marker()
            joint_marker.header.frame_id = "world"
            joint_marker.header.stamp = stamp
            joint_marker.ns = "joints"
            joint_marker.type = Marker.POINTS
            joint_marker.action = Marker.ADD
            joint_marker.scale.x = 0.01  # Point width
            joint_marker.scale.y = 0.01  # Point height
            joint_marker.color.a = 1.0
            joint_marker.color.r = 1.0  # Red color

            for joint in tips + [palm]:
                p = Point(x=float(joint[0]), y=float(joint[1]), z=float(joint[2]))
                joint_marker.points.append(p) # type: ignore

            markers.append(joint_marker)

        bone_marker = Marker()
        bone_marker.header.frame_id = "world"
        bone_marker.header.stamp = stamp
        bone_marker.ns = "palm_keyvectors"
        bone_marker.type = Marker.LINE_LIST
        bone_marker.action = Marker.ADD
        bone_marker.scale.x = 0.001  # Line width
        bone_marker.color.a = 1.0
        bone_marker.color.g = 1.0  # Blue color

        shift = np.array(RETARGETER_PARAMS["global"]["mano_shift"])

        # Add bone lines
        for tip in tips:
            if detach:
                start_joint = palm
                end_joint = tip
            else:
                start_joint = palm  + shift
                end_joint = tip + shift
            p_start = Point(x=float(start_joint[0]), y=float(start_joint[1]), z=float(start_joint[2]))
            p_end = Point(x=float(end_joint[0]), y=float(end_joint[1]), z=float(end_joint[2]))
            bone_marker.points.append(p_start) # type: ignore
            bone_marker.points.append(p_end) # type: ignore

        markers.append(bone_marker)

        bone_marker = Marker()
        bone_marker.header.frame_id = "world"
        bone_marker.header.stamp = stamp
        bone_marker.ns = "tip_keyvectors"
        bone_marker.type = Marker.LINE_LIST
        bone_marker.action = Marker.ADD
        bone_marker.scale.x = 0.001  # Line width
        bone_marker.color.a = 1.0
        bone_marker.color.g = 1.0
        bone_marker.color.r = 1.0

        for i, start_tip in enumerate(tips):
            for i, end_tip in enumerate(tips[i:]):
                if detach:
                    start_joint = start_tip
                    end_joint = end_tip
                else:
                    start_joint = start_tip  + shift
                    end_joint = end_tip + shift
                p_start = Point(x=float(start_joint[0]), y=float(start_joint[1]), z=float(start_joint[2]))
                p_end = Point(x=float(end_joint[0]), y=float(end_joint[1]), z=float(end_joint[2]))
                bone_marker.points.append(p_start) # type: ignore
                bone_marker.points.append(p_end) # type: ignore

        markers.append(bone_marker)

        bone_marker = Marker()
        bone_marker.header.frame_id = "world"
        bone_marker.header.stamp = stamp
        bone_marker.ns = "mcp_keyvectors"
        bone_marker.type = Marker.LINE_LIST
        bone_marker.action = Marker.ADD
        bone_marker.scale.x = 0.001  # Line width
        bone_marker.color.a = 1.0
        bone_marker.color.b = 1.0
        bone_marker.color.r = 1.0

        for i, start_tip in enumerate(mps):
            if i == len(mps) - 1:
                break
            end_tip = mps[i+1]
            if detach:
                start_joint = start_tip
                end_joint = end_tip
            else:
                start_joint = start_tip  + shift
                end_joint = end_tip + shift
            p_start = Point(x=float(start_joint[0]), y=float(start_joint[1]), z=float(start_joint[2]))
            p_end = Point(x=float(end_joint[0]), y=float(end_joint[1]), z=float(end_joint[2]))
            bone_marker.points.append(p_start) # type: ignore
            bone_marker.points.append(p_end) # type: ignore

        markers.append(bone_marker)

        self.markers.extend(markers)
