import math
import numpy as np
import copy
from reacher import forward_kinematics

HIP_OFFSET = 0.0335
UPPER_LEG_OFFSET = 0.10 # length of link 1
LOWER_LEG_OFFSET = 0.13 # length of link 2
TOLERANCE = 0.01 # tolerance for inverse kinematics
PERTURBATION = 0.0001 # perturbation for finite difference method
MAX_ITERATIONS = 100

def ik_cost(end_effector_pos, guess):
    """Calculates the inverse kinematics cost.

    This function computes the inverse kinematics cost, which represents the Euclidean
    distance between the desired end-effector position and the end-effector position
    resulting from the provided 'guess' joint angles.

    Args:
        end_effector_pos (numpy.ndarray), (3,): The desired XYZ coordinates of the end-effector.
            A numpy array with 3 elements.
        guess (numpy.ndarray), (3,): A guess at the joint angles to achieve the desired end-effector
            position. A numpy array with 3 elements.

    Returns:
        float: The Euclidean distance between end_effector_pos and the calculated end-effector
        position based on the guess.
    """
    # Initialize cost to zero
    guess_position = forward_kinematics.fk_foot(guess)

    x = (end_effector_pos[0] - guess_position[0][3]) * (end_effector_pos[0] - guess_position[0][3])
    y = (end_effector_pos[1] - guess_position[1][3]) * (end_effector_pos[1] - guess_position[1][3])
    z = (end_effector_pos[2] - guess_position[2][3]) * (end_effector_pos[2] - guess_position[2][3])
    return math.sqrt(x + y + z)


def calculate_jacobian_FD(joint_angles, delta):
    """
    Calculate the Jacobian matrix using finite differences.

    This function computes the Jacobian matrix for a given set of joint angles using finite differences.

    Args:
        joint_angles (numpy.ndarray), (3,): The current joint angles. A numpy array with 3 elements.
        delta (float): The perturbation value used to approximate the partial derivatives.

    Returns:
        numpy.ndarray: The Jacobian matrix. A 3x3 numpy array representing the linear mapping
        between joint velocity and end-effector linear velocity.
    """

    # Initialize Jacobian to zero
    jacobian = np.zeros((3, 3))

    # Add your solution here.
    frame = forward_kinematics.fk_foot(joint_angles)
    pos = forward_kinematics.get_pos(frame)
    angles = joint_angles

    for i in range(3):
        angles[i] += delta

        new_frame = forward_kinematics.fk_foot(angles)
        new_pos = forward_kinematics.get_pos(new_frame)

        delta_pos = (new_pos - pos) / delta
        jacobian[0, i] = delta_pos[0]
        jacobian[1, i] = delta_pos[1]
        jacobian[2, i] = delta_pos[2]

        angles[i] -= delta

    return jacobian

def calculate_inverse_kinematics(end_effector_pos, guess):
    """
    Calculate the inverse kinematics solution using the Newton-Raphson method.

    This function iteratively refines a guess for joint angles to achieve a desired end-effector position.
    It uses the Newton-Raphson method along with a finite difference Jacobian to find the solution.

    Args:
        end_effector_pos (numpy.ndarray): The desired XYZ coordinates of the end-effector.
            A numpy array with 3 elements.
        guess (numpy.ndarray): The initial guess for joint angles. A numpy array with 3 elements.

    Returns:
        numpy.ndarray: The refined joint angles that achieve the desired end-effector position.
    """

    # Initialize previous cost to infinity
    previous_cost = np.inf
    # Initialize the current cost to 0.0
    cost = 0.0

    for iters in range(MAX_ITERATIONS):
        jacobian = calculate_jacobian_FD(guess, PERTURBATION)

        residual = end_effector_pos - forward_kinematics.get_pos(forward_kinematics.fk_foot(guess))
        pseudo_inverse = np.linalg.pinv(jacobian)
        step = np.matmul(pseudo_inverse, residual)
        guess += step

        cost = ik_cost(end_effector_pos, guess)

        if abs(previous_cost - cost) < TOLERANCE:
            return guess
        previous_cost = cost

    return guess
