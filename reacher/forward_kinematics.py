import math
import numpy as np

HIP_OFFSET = 0.0335
UPPER_LEG_OFFSET = 0.10  # length of link 1
LOWER_LEG_OFFSET = 0.13  # length of link 2

def rotation_matrix(axis, angle):
    """
    Create a 3x3 rotation matrix which rotates about a specific axis.
    """
    # Ensure the axis is a unit vector
    axis = np.array(axis)
    axis = axis / np.linalg.norm(axis)

    # Calculate sine and cosine of the angle
    cos_angle = np.cos(angle)
    sin_angle = np.sin(angle)

    # Create the cross-product matrix K
    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0]
    ])

    # Compute the rotation matrix
    rot_mat = np.eye(3) + sin_angle * K + (1 - cos_angle) * np.dot(K, K)
    
    return rot_mat

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
    hip_angle = joint_angles[0]
    T_hip = homogenous_transformation_matrix([0, 0, 1], hip_angle, [0, HIP_OFFSET, 0])
    return T_hip

def fk_shoulder(joint_angles):
    """
    Calculate the xyz coordinates of the shoulder frame given the joint angles.
    """
    hip_angle = joint_angles[0]
    shoulder_angle = joint_angles[1]
    T_hip = fk_hip(joint_angles)
    T_shoulder = homogenous_transformation_matrix([0, 1, 0], shoulder_angle, [UPPER_LEG_OFFSET, 0, 0])
    T_shoulder_in_base = np.dot(T_hip, T_shoulder)
    return T_shoulder_in_base

def fk_elbow(joint_angles):
    """
    Calculate the xyz coordinates of the elbow frame given the joint angles.
    """
    shoulder_angle = joint_angles[1]
    elbow_angle = joint_angles[2]
    T_shoulder = fk_shoulder(joint_angles)
    T_elbow = homogenous_transformation_matrix([0, 1, 0], elbow_angle, [LOWER_LEG_OFFSET, 0, 0])
    T_elbow_in_base = np.dot(T_shoulder, T_elbow)
    return T_elbow_in_base

def fk_foot(joint_angles):
    """
    Calculate the xyz coordinates of the foot given the joint angles.
    """
    T_elbow = fk_elbow(joint_angles)
    # Assuming the foot is at the end of the elbow segment, with no additional translation or rotation
    T_foot = np.eye(4)
    T_foot_in_base = np.dot(T_elbow, T_foot)
    return T_foot_in_base
