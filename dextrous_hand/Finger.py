#!/usr/bin/env python3

from dextrous_hand.Subsystem import Subsystem
import dextrous_hand.ids as ids

from dextrous_hand.constants import SPOOL_RADIUS

class Finger(Subsystem):
    def __init__(self, id : ids.SUBSYSTEMS):
        """
        params
            id [SUBSYSTEMS]: the finger's id
        """
        super().__init__(id)


    def find_closest_in_space(self, range_abd, range_flex, abd_angle, flex_angle):
        theta_flex_max=range_flex[1]
        theta_flex_min=range_flex[0]

        theta_abd_max=range_abd[1]
        theta_abd_min=range_abd[0] # for now we assume that the theta_abd_max==theta_abd_min

        def above(line,x,y):
            return -line[0]*x+y>line[1]

        def below_on(line,x,y):
            return -line[0]*x+y<=line[1]
        
        def closest_from(line,x,y):
            a, b = line
            x_q = (x + a * (y - b)) / (a**2 + 1)
            y_q = a * x_q + b
            return x_q, y_q

        # find closest point in the boundaries
        a = theta_flex_max/theta_abd_max
        b = theta_flex_max
        b2 = -theta_abd_max/a

        zone_1 = above((-1/a,b),abd_angle,flex_angle) and above((1/a,b),abd_angle,flex_angle)
        zone_2 = below_on((-1/a,b),abd_angle,flex_angle) and above((-1/a,b2),abd_angle,flex_angle) and above((a,b),abd_angle,flex_angle)
        zone_3 = below_on((-1/a,b2),abd_angle,flex_angle) and abd_angle<-theta_abd_max
        zone_4 = abd_angle>-theta_abd_max and abd_angle<theta_abd_max and flex_angle<0
        zone_5 = abd_angle>theta_abd_max and below_on((1/a,b2),abd_angle,flex_angle)
        zone_6 = below_on((1/a,b),abd_angle,flex_angle) and above((1/a,b2),abd_angle,flex_angle) and above((-a,b),abd_angle,flex_angle)

        if zone_1:
            return theta_flex_min, theta_flex_max
        elif zone_2:
            return closest_from((a,b), abd_angle,flex_angle)
        elif zone_3:
            return -theta_abd_max, theta_flex_min
        elif zone_4:
            return closest_from((0,0), abd_angle,flex_angle)
        elif zone_5:
            return theta_abd_max, theta_flex_min
        elif zone_6:
            return closest_from((-a,b), abd_angle,flex_angle)
        else:
          return abd_angle,flex_angle
    
    def joints2motors(self, joint_angles) -> list[float]:
        """
        Map the angles of the joints to the angles of the motors.
        Note that the number of angles that can be set is 3 and not 4 because
        DIP is not settable.

        TODO: Implement this method
        """
        assert len(joint_angles) == 3
        joint_angles[0],joint_angles[1] = self.find_closest_in_space(self.joints[0].geometry["range"], self.joints[1].geometry["range"], joint_angles[0],joint_angles[1])

        virtual_tendons_diff=[]
        for i, joint_angle in enumerate(joint_angles):
            virtual_tendons_diff.append(self.joints[i].joint2length(joint_angle))


        tendons_pair_motor_1 = (virtual_tendons_diff[0][0]+virtual_tendons_diff[1][0],virtual_tendons_diff[0][1]+virtual_tendons_diff[1][1])
        tendons_pair_motor_2 = (virtual_tendons_diff[0][1]+virtual_tendons_diff[1][0],virtual_tendons_diff[0][0]+virtual_tendons_diff[1][1])
        tendons_length_diff=[tendons_pair_motor_1, tendons_pair_motor_2]
        
        motor_angles=[]
        for tendons_diff in tendons_length_diff:
            motor_angles.append(tendons_diff[0]/SPOOL_RADIUS)
        # !!!!! adjust the sign to the direction of the motor
        # here it assumes that the the motor will pull when turning with a positive angle

        return motor_angles

    @property
    def ABD(self):
        return self.get_joint("ABD")

    @property
    def MCP(self):
        return self.get_joint("MCP")

    @property
    def PIP(self):
        return self.get_joint("PIP")

    @property
    def DIP(self):
        return self.get_joint("DIP")

    @property
    def TRBL(self):
        return self.get_motor("TRBL")

    @property
    def TLBR(self):
        return self.get_motor("TLBR")

    @property
    def TMBM(self):
        return self.get_motor("TMBM")

class Thumb(Finger):
    """
    For now, the thumb is the same as a finger.
    TODO: Implement thumb-specific methods
    """
    def __init__(self, id : ids.SUBSYSTEMS):
        """
        params
            id [SUBSYSTEMS]: the thumb's id
        """
        super().__init__(id)

    def joints2motors(self, joint_angles) -> list[float]:
        """
        Map the angles of the joints to the angles of the motors.
        Note that the number of angles that can be set is 3 and not 4 because
        DIP is not settable.

        TODO: Implement this method
        """
        assert len(joint_angles) == 3

        return joint_angles

# Singleton instances
PINKY = Finger(ids.SUBSYSTEMS.PINKY)
RING = Finger(ids.SUBSYSTEMS.RING)
MIDDLE = Finger(ids.SUBSYSTEMS.MIDDLE)
INDEX = Finger(ids.SUBSYSTEMS.INDEX)
THUMB = Thumb(ids.SUBSYSTEMS.THUMB)

FINGERS = [PINKY, RING, MIDDLE, INDEX, THUMB]
"""
 A collection of fingers for easy iteration.
"""

def finger_index(id : ids.SUBSYSTEMS) -> int:
    """
    Convert a finger id to an index in the FINGERS list
    """
    return [finger.id for finger in FINGERS].index(id)
