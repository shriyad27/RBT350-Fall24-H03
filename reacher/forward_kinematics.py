import math
import numpy as np

HIP_OFFSET = 0.0335
UPPER_LEG_OFFSET = 0.10  # length of link 1
LOWER_LEG_OFFSET = 0.13  # length of link 2

def rotation_matrix(axis, angle):
    """
    Create a 3x3 rotation matrix which rotates about a specific axis.
    """
    #https://en.wikipedia.org/wiki/Rotation_matrix#Rotation_matrix_from_axis_and_angle
    cos = math.cos(angle)
    sin = math.sin(angle)

    ux, uy, uz = axis

    r_matrix = np.array([
        [ux * ux * (1 - cos) + cos, ux * uy * (1 - cos) - uz * sin, ux * uz * (1 - cos) + uy * sin],
        [uy * ux * (1 - cos) + uz * sin, uy * uy * (1 - cos) + cos, uy * uz * (1 - cos) - ux * sin],
        [uz * ux * (1 - cos) - uy * sin, uz * uy * (1 - cos) + ux * sin, uz * uz * (1 - cos) + cos]
    ])

    return r_matrix



def homogenous_transformation_matrix(axis, angle, v_A):
    """
    Create a 4x4 transformation matrix which transforms from frame A to frame B.
    """
    R = rotation_matrix(axis, angle)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = v_A
    return T

def fk_hip(joint_angles):
    """
    Calculate the xyz coordinates of the hip frame given the joint angles.
    """

def fk_shoulder(joint_angles):
    """
    Calculate the xyz coordinates of the shoulder frame given the joint angles.
    """


def fk_elbow(joint_angles):
    """
    Calculate the xyz coordinates of the elbow frame given the joint angles.
    """


def fk_foot(joint_angles):
    """
    Calculate the xyz coordinates of the foot given the joint angles.
    """
