#!/usr/bin/env python3

import dextrous_hand.utils.ids as ids
from dextrous_hand.subsystems.Subsystem import Subsystem
from dextrous_hand.joints.joints_geometry import SPOOL_RADIUS

from dextrous_hand.utils.architecture import SUBSYSTEM_MOTORS
from dextrous_hand.joints.joints_geometry import STATIC_SLACK

class Finger(Subsystem):
    def __init__(self, id : ids.SUBSYSTEMS):
        """
        params
            id [SUBSYSTEMS]: the finger's id
        """
        super().__init__(id)
        self.motor_slack = [[0, 0]] * len(self.motors)
        self.static_slack = [STATIC_SLACK[motor_id] for motor_id in SUBSYSTEM_MOTORS[self.id]]


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
        zone_4 = abd_angle>=-theta_abd_max and abd_angle<=theta_abd_max and flex_angle<0
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

    def restrict_joint_angles(self, joint_angles):
        """
        Enforces the "triangle" restriction on the joint angles.
        """

        assert len(joint_angles) == 3

        joint_angles[0],joint_angles[1] = self.find_closest_in_space(self.joints[0].geometry["range"], self.joints[1].geometry["range"], joint_angles[0],joint_angles[1])

        joint_angles[2] = max(self.joints[2].geometry["range"][0], min(joint_angles[2], self.joints[2].geometry["range"][1]))

        return joint_angles


    def static_slack_into_account(self, motor_angle, idx):
        if motor_angle > 0 and self.motor_slack[idx][0]<0:
            motor_angle-=self.static_slack[idx][0]/SPOOL_RADIUS
        elif motor_angle < 0 and self.static_slack[idx][1]<0:
            motor_angle+=self.static_slack[idx][1]/SPOOL_RADIUS
        return motor_angle

    def slack_into_account(self, motor_angle, idx):
        if motor_angle > 0 and self.motor_slack[idx][0]<0:
            motor_angle-=self.motor_slack[idx][0]/SPOOL_RADIUS
        elif motor_angle < 0 and self.motor_slack[idx][1]<0:
            motor_angle+=self.motor_slack[idx][1]/SPOOL_RADIUS
        return motor_angle

    def update_slack(self, motor_angle, slack, idx):
        if motor_angle > 0:
            self.motor_slack[idx] = [0,slack]
        else:
            self.motor_slack[idx] = [slack,0]

    def update_static_slack(self, motor_angle, idx):
        static_slack = self.static_slack[idx][0] + self.static_slack[idx][1]
        if motor_angle > 0:
            self.static_slack[idx] = [0,static_slack]
        else:
            self.static_slack[idx] = [static_slack,0]


    def coupled_motors(self, virtual_tendon_abd, virtual_tendon_flex):

        abd_l1 = virtual_tendon_abd[0]
        abd_l2 = virtual_tendon_abd[1]
        flex_l1 = virtual_tendon_flex[0]
        flex_l2 = virtual_tendon_flex[1]

        # compute virtual slack
        s_abd = abd_l1 + abd_l2
        s_flex = flex_l1 + flex_l2

        dl_m1 = 0
        dl_m2 = 0
        s_m1 = 0
        s_m2 = 0

        # compute real tendons length
        if abd_l1 > abd_l2 and flex_l1 == flex_l2:
            # case 1
            dl_m1 = abd_l1
            dl_m2 = -abd_l1
            s_m1 = s_abd
            s_m2 = s_abd
        elif abd_l1 < abd_l2 and flex_l1 == flex_l2:
            # case 2
            dl_m1 = -abd_l2
            dl_m2 = abd_l2
            s_m1 = s_abd
            s_m2 = s_abd
        elif abd_l1 == abd_l2 and flex_l1 > flex_l2:
            # case 3
            dl_m1 = flex_l1
            dl_m2 = flex_l1
            s_m1 = s_flex
            s_m2 = s_flex
        elif abd_l1 == abd_l2 and flex_l1 < flex_l2:
            # case 4
            dl_m1 = -flex_l2
            dl_m2 = -flex_l2
            s_m1 = s_flex
            s_m2 = s_flex
        elif abd_l1 > abd_l2 and flex_l1 > flex_l2:
            # case 5
            dl_m1 = abd_l1 + flex_l1
            dl_m2 = flex_l1 - abd_l1
            s_m1 = s_abd + s_flex
            s_m2 = s_abd + s_flex
        elif abd_l1 < abd_l2 and flex_l1 < flex_l2:
            # case 6
            dl_m1 = -abd_l2 - flex_l2
            dl_m2 = -flex_l2 + abd_l2
            s_m1 = s_abd + s_flex
            s_m2 = s_abd + s_flex
        elif abd_l1 < abd_l2 and flex_l1 > flex_l2:
            # case 7
            dl_m1 = flex_l1 - abd_l2
            dl_m2 = flex_l1 + abd_l2
            s_m1 = s_abd + s_flex
            s_m2 = s_abd + s_flex
        elif abd_l1 > abd_l2 and flex_l1 < flex_l2:
            # case 8
            dl_m1 = - flex_l2 + abd_l1
            dl_m2 = - abd_l1 - flex_l2
            s_m1 = s_abd + s_flex
            s_m2 = s_abd + s_flex

        # compute motor angles
        motor_angle_1 = dl_m1/SPOOL_RADIUS
        motor_angle_2 = dl_m2/SPOOL_RADIUS


        # take salck into account
        motor_angle_1 = self.slack_into_account(motor_angle_1, 0)
        motor_angle_2 = self.slack_into_account(motor_angle_2, 1)
        """
        if motor_angle_1 > 0 and self.motor_slack[0][0]<0:
            motor_angle_1-=self.motor_slack[0][0]/SPOOL_RADIUS
        elif motor_angle_1 < 0 and self.motor_slack[0][1]<0:
            motor_angle_1+=self.motor_slack[0][1]/SPOOL_RADIUS
        if motor_angle_2 > 0 and self.motor_slack[1][0]<0:
            motor_angle_2-=self.motor_slack[1][0]/SPOOL_RADIUS
        elif motor_angle_2 < 0 and self.motor_slack[1][1]<0:
            motor_angle_2+=self.motor_slack[1][1]/SPOOL_RADIUS
        """
        # take static salck into account
        motor_angle_1 = self.static_slack_into_account(motor_angle_1, 0)
        motor_angle_2 = self.static_slack_into_account(motor_angle_2, 1)
        """
        if motor_angle_1 > 0 and self.motor_slack[0][0]<0:
            motor_angle_1-=self.static_slack[0][0]/SPOOL_RADIUS
        elif motor_angle_1 < 0 and self.static_slack[0][1]<0:
            motor_angle_1+=self.static_slack[0][1]/SPOOL_RADIUS
        if motor_angle_2 > 0 and self.motor_slack[1][0]<0:
            motor_angle_2-=self.motor_slack[1][0]/SPOOL_RADIUS
        elif motor_angle_2 < 0 and self.motor_slack[1][1]<0:
            motor_angle_2+=self.motor_slack[1][1]/SPOOL_RADIUS
        """

        # update the slack in the system
        self.update_slack(motor_angle_1, s_m1, 0)
        self.update_slack(motor_angle_2, s_m2, 1)

        """
        if motor_angle_1 > 0:
            self.motor_slack[0] = [0,s_m1]
        else:
            self.motor_slack[0] = [s_m1,0]

        if motor_angle_2 > 0:
            self.motor_slack[1] = [0,s_m2]
        else:
            self.motor_slack[1] = [s_m2,0]
        """

        # update the static slack side
        self.update_static_slack(motor_angle_1, 0)
        self.update_static_slack(motor_angle_2, 1)
        """
        static_slack = self.static_slack[0][0] + self.static_slack[0][1]
        if motor_angle_1 > 0:
            self.static_slack[0] = [0,static_slack]
        else:
            self.static_slack[0] = [static_slack,0]

        static_slack = self.static_slack[1][0] + self.static_slack[1][1]
        if motor_angle_2 > 0:
            self.static_slack[1] = [0,static_slack]
        else:
            self.static_slack[1] = [static_slack,0]
        """
        return motor_angle_1, motor_angle_2

    def single_motor(self, virtual_tendon_pip, idx):

        pip_l1 = virtual_tendon_pip[0]
        pip_l2 = virtual_tendon_pip[1]

        # compute slack
        s_pip = pip_l1 + pip_l2

        # compute real tendon length
        if pip_l1 > pip_l2:
            dl_m1 = pip_l1
        elif pip_l1 < pip_l2:
            dl_m1 = -pip_l2
        elif pip_l1 == pip_l2:
            dl_m1 = 0

        # compute motor angles
        motor_angle = dl_m1/SPOOL_RADIUS

        # take dynamic and static salck into account
        motor_angle = self.slack_into_account(motor_angle, idx)
        motor_angle = self.static_slack_into_account(motor_angle, idx)

        # update the slack in the system
        self.update_slack(motor_angle, s_pip, idx)
        self.update_static_slack(motor_angle, idx)

        return motor_angle


    def joints2motors(self, joint_angles) -> list[float]:
        """
        Map the angles of the joints to the angles of the motors.
        Note that the number of angles that can be set is 3 and not 4 because
        DIP is not settable.
        """
        assert len(joint_angles) == 3

        virtual_tendons_diff=[]
        for i, joint_angle in enumerate(joint_angles):
            virtual_tendons_diff.append(self.joints[i].joint2length(joint_angle))

        motor_angles=[0.,0.,0.]
        motor_angles[0], motor_angles[1] = self.coupled_motors(virtual_tendons_diff[0], virtual_tendons_diff[1])
        motor_angles[2] = self.single_motor(virtual_tendons_diff[2], 2)

        return motor_angles



    """
    Getters for the joints and motors of the finger
    """
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
    def FRBL(self):
        return self.get_motor("FRBL")

    @property
    def FLBR(self):
        return self.get_motor("FLBR")

    @property
    def FMBM(self):
        return self.get_motor("FMBM")

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
        """
        assert len(joint_angles) == 4

        motors_angles=[]
        pin_joints = [joint_angles[0],joint_angles[1]]
        for i, joint_angle in enumerate(pin_joints):
            motors_angles.append(self.joints[i].joint2length(joint_angle))

        motors_angles[1] *= self.joints[1].geometry["multiplier"]

        motors_angles.append(self.single_motor(self.joints[2].joint2length(joint_angles[2]), 2)  * self.joints[2].geometry["multiplier"])
        motors_angles.append(self.single_motor(self.joints[3].joint2length(joint_angles[3]), 3))

        return motors_angles

    def restrict_joint_angles(self, joint_angles):
        """
        Enforces the "triangle" restriction on the joint angles.
        """
        assert len(joint_angles) == 4
        # joint_angles[0],joint_angles[1] = self.find_closest_in_space(self.joints[0].geometry["range"], self.joints[1].geometry["range"], joint_angles[0],joint_angles[1])
        joint_angles[0] = max(self.joints[0].geometry["range"][0], min(joint_angles[0], self.joints[0].geometry["range"][1]))
        joint_angles[1] = max(self.joints[1].geometry["range"][0], min(joint_angles[1], self.joints[1].geometry["range"][1]))
        joint_angles[2] = max(self.joints[2].geometry["range"][0], min(joint_angles[2], self.joints[2].geometry["range"][1]))
        joint_angles[3] = max(self.joints[3].geometry["range"][0], min(joint_angles[3], self.joints[3].geometry["range"][1]))
        return joint_angles

# Singleton instances
PINKY = Finger(ids.SUBSYSTEMS.PINKY)
RING = Finger(ids.SUBSYSTEMS.RING)
MIDDLE = Finger(ids.SUBSYSTEMS.MIDDLE)
INDEX = Finger(ids.SUBSYSTEMS.INDEX)
THUMB = Thumb(ids.SUBSYSTEMS.THUMB)

FINGERS = [Finger(id) for id in ids.SUBSYSTEMS if id.value < ids.SUBSYSTEMS.WRIST.value]
FINGERS[ids.SUBSYSTEMS.THUMB.value] = Thumb(ids.SUBSYSTEMS.THUMB)
"""
 A collection of fingers for easy iteration.
"""
