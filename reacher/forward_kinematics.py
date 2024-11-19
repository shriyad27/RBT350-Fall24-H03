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
    
    rot_mat = np.zeros((3, 3))
    if axis[0] == 1:
        rot_mat[0][0] = 1
        rot_mat[0][1] = 0
        rot_mat[0][2] = 0

        rot_mat[1][0] = 0
        rot_mat[1][1] = np.cos(angle)
        rot_mat[1][2] = - np.sin(angle)

        rot_mat[2][0] = 0
        rot_mat[2][1] = np.sin(angle)
        rot_mat[2][2] = np.cos(angle)
    elif axis[1] == 1:
        rot_mat[0][0] = np.cos(angle)
        rot_mat[0][1] = 0
        rot_mat[0][2] = np.sin(angle)

        rot_mat[1][0] = 0
        rot_mat[1][1] = 1
        rot_mat[1][2] = 0

        rot_mat[2][0] = - np.sin(angle)
        rot_mat[2][1] = 0
        rot_mat[2][2] = np.cos(angle)
    elif axis[2] == 1:
        rot_mat[0][0] = np.cos(angle)
        rot_mat[0][1] = - np.sin(angle)
        rot_mat[0][2] = 0

        rot_mat[1][0] = np.sin(angle)
        rot_mat[1][1] = np.cos(angle)
        rot_mat[1][2] = 0

        rot_mat[2][0] = 0
        rot_mat[2][1] = 0
        rot_mat[2][2] = 1

    return rot_mat



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
    rot_mat = rotation_matrix(axis, angle)

    return np.block([[rot_mat, np.array([v_A]).T], [0, 0, 0, 1]])

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
  axis = np.array([0,0,1])
  v_A = np.array([0,0,0])
  hip_matrix = homogenous_transformation_matrix(axis, joint_angles[0], v_A)
  return hip_matrix
    

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
  axis = np.array([0, 0, 1])
  v_A = np.array([np.sin(joint_angles[0]) * HIP_OFFSET, -np.cos(joint_angles[0]) * HIP_OFFSET, 0])
  shoulder_mat = homogenous_transformation_matrix(axis, joint_angles[0], v_A)
  return shoulder_mat


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
  axis = np.array([0, 1, 0])
  v_A = np.array([np.sin(joint_angles[1]) * UPPER_LEG_OFFSET,
                  0,
                  np.cos(joint_angles[1]) * UPPER_LEG_OFFSET
  ])
  elbow_mat = homogenous_transformation_matrix(axis, joint_angles[1], v_A)
  shoulder_mat = fk_shoulder(joint_angles)

  elbow_frame = np.matmul(shoulder_mat, elbow_mat)

  return elbow_frame


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
  axis = np.array([0, 1, 0])
  v_A = np.array([np.sin(joint_angles[2]) * LOWER_LEG_OFFSET,
                  0,
                  np.cos(joint_angles[2]) * LOWER_LEG_OFFSET
  ])
  foot_mat = homogenous_transformation_matrix(axis, joint_angles[2], v_A)
  elbow_mat = fk_elbow(joint_angles)

  foot_frame = np.matmul(elbow_mat, foot_mat)

  return foot_frame


def get_pos(frame):
  P = np.zeros(3)
  P[0] = frame[0, 3]
  P[1] = frame[1, 3]
  P[2] = frame[2, 3]
  return P


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



# joint_angles = np.array([-0.331, -0.744, 0.942])  # Adjust these angles for correct movement
# test_fk(joint_angles)
