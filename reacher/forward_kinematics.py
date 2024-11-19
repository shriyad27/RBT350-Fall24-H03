import math
import numpy as np

HIP_OFFSET = 0.0335
UPPER_LEG_OFFSET = 0.10  # length of link 1
LOWER_LEG_OFFSET = 0.13  # length of link 2

def rotation_matrix(axis, angle):
    """
    Create a 3x3 rotation matrix which rotates about a specific axis

    Args:
        axis:  Array.  Unit vector in the direction of the axis of rotation
        angle: Number. The amount to rotate about the axis in radians

    Returns:
        3x3 rotation matrix as a numpy array
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

    # r_matrix = np.array([
    #     [cos, -sin, 0],
    #     [sin, cos, 0],
    #     [0, 0, 1]
    # ])

    return r_matrix



def homogenous_transformation_matrix(axis, angle, v_A):
    """
    Create a 4x4 transformation matrix which transforms from frame A to frame B

    Args:
        axis:  Array.  Unit vector in the direction of the axis of rotation
        angle: Number. The amount to rotate about the axis in radians
        v_A:   Vector. The vector translation from A to B defined in frame A

    Returns:
        4x4 transformation matrix as a numpy array
    """
    R = rotation_matrix(axis, angle)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = v_A
    return T

def fk_hip(joint_angles):
  """
  Use forward kinematics equations to calculate the xyz coordinates of the hip
  frame given the joint angles of the robot

  Args:
    joint_angles: numpy array of 3 elements stored in the order [hip_angle, shoulder_angle, 
                  elbow_angle]. Angles are in radians
  Returns:
    4x4 matrix representing the pose of the hip frame in the base frame
  """
  hip_angle = joint_angles[0]
  hip_frame = homogenous_transformation_matrix([0, 0, 1], hip_angle, [0, 0, 0])
  return hip_frame
    

def fk_shoulder(joint_angles):
  """
  Use forward kinematics equations to calculate the xyz coordinates of the shoulder
  joint given the joint angles of the robot

  Args:
    joint_angles: numpy array of 3 elements stored in the order [hip_angle, shoulder_angle,  
                  elbow_angle]. Angles are in radians
  Returns:
    4x4 matrix representing the pose of the shoulder frame in the base frame
  """
  hip_transform = fk_hip(joint_angles)
  shoulder_angle = joint_angles[1]
  
  # Define the shoulder frame transformation relative to the hip
  shoulder_transform = homogenous_transformation_matrix([0, 1, 0], shoulder_angle, [0, 0, 0])
  
  # Combine hip and shoulder transformations
  shoulder_frame = hip_transform @ shoulder_transform
  return shoulder_frame


def fk_elbow(joint_angles):
  """
  Use forward kinematics equations to calculate the xyz coordinates of the elbow
  joint given the joint angles of the robot

  Args:
    joint_angles: numpy array of 3 elements stored in the order [hip_angle, shoulder_angle, 
                  elbow_angle]. Angles are in radians
  Returns:
    4x4 matrix representing the pose of the elbow frame in the base frame
  """
  shoulder_transform = fk_shoulder(joint_angles)
  elbow_angle = joint_angles[2]
  
  # Define the elbow frame transformation relative to the shoulder
  elbow_transform = homogenous_transformation_matrix([0, 1, 0], elbow_angle, [0, -HIP_OFFSET, UPPER_LEG_OFFSET])
  
  # Combine shoulder and elbow transformations
  elbow_frame = shoulder_transform @ elbow_transform
  return elbow_frame

  shoulder_transform = fk_shoulder(joint_angles)


def fk_foot(joint_angles):
  """
  Use forward kinematics equations to calculate the xyz coordinates of the foot given 
  the joint angles of the robot

  Args:
    joint_angles: numpy array of 3 elements stored in the order [hip_angle, shoulder_angle, 
                  elbow_angle]. Angles are in radians
  Returns:
    4x4 matrix representing the pose of the end effector frame in the base frame
  """
  elbow_transform = fk_elbow(joint_angles)
    
    # Define the foot frame transformation relative to the elbow
  foot_transform = homogenous_transformation_matrix([0, 0, 1], 0, [0, 0, LOWER_LEG_OFFSET])
    
    # Combine elbow and foot transformations
  end_effector_frame = elbow_transform @ foot_transform
  return end_effector_frame


# def test_fk(joint_angles):
#     """
#     Test forward kinematics by computing and displaying the hip, shoulder, elbow,
#     and foot positions.
#     """
#     hip_frame = fk_hip(joint_angles)
#     print("\nHip frame:")
#     print(hip_frame)

#     shoulder_frame = fk_shoulder(joint_angles)
#     print("\nShoulder frame:")
#     print(shoulder_frame)

#     elbow_frame = fk_elbow(joint_angles)
#     print("\nElbow frame:")
#     print(elbow_frame)

#     foot_frame = fk_foot(joint_angles)
#     print("\nFoot (end effector) frame:")
#     print(foot_frame)



def get_pos(frame):
  P = np.zeros(3)
  P[0] = frame[0, 3]
  P[1] = frame[1, 3]
  P[2] = frame[2, 3]
  return P


# joint_angles = np.array([-0.331, -0.744, 0.942])  # Adjust these angles for correct movement
# test_fk(joint_angles)