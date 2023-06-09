import math
import numpy as np
import copy

# Variables for FK
HIP_OFFSET = 0.05 # length of N
L1 = 0.08 # length of link 1, the hip to knee
L2 = 0.11 # length of link 2, the knee to foot
# Variables for IK, gradient descent



def calculate_forward_kinematics_robot(joint_angles):
    """Calculate xyz coordinates of end-effector given joint angles.

    Use forward kinematics equations to calculate the xyz coordinates of the end-effector
    given some joint angles.

    Args:
      joint_angles: numpy array of 3 elements [TODO names]. Numpy array of 3 elements.
    Returns:
      xyz coordinates of the end-effector in the arm frame. Numpy array of 3 elements.
    """
    def generate_y_rotation(theta):
        return np.array([
            [np.cos(theta), 0, np.sin(theta)],
            [0, 1, 0],
            [-np.sin(theta), 0, np.cos(theta)]
        ])

    def generate_z_rotation(theta):
        return np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]
        ])

    joint_angles *= -1
    r1 = np.array([0, 0, L2])
    r2 = np.array([0, 0, L1]).transpose() + \
        np.matmul(generate_y_rotation(joint_angles[2]), r1)
    r3 = np.array([0, -HIP_OFFSET, 0]).transpose() + \
        np.matmul(generate_y_rotation(joint_angles[1]), r2)
    r4 = np.matmul(generate_z_rotation(joint_angles[0]), r3)

    # print(f"guess xyz coords: {r4}")
    return r4

def ik_cost(end_effector_pos, guess):
    """Calculates the inverse kinematics loss.

    Calculate the Euclidean distance between the desired end-effector position and
    the end-effector position resulting from the given 'guess' joint angles.

    Args:
      end_effector_pos: desired xyz coordinates of end-effector. Numpy array of 3 elements.
      guess: guess at joint angles to achieve desired end-effector position. Numpy array of 3 elements.
    Returns:
      Euclidean distance between end_effector_pos and guess. Returns float.
    """
    # TODO for students: Implement this function. ~1-5 lines of code.
    cost = 0.0

    delta = np.subtract(calculate_forward_kinematics_robot(guess), end_effector_pos)
    # delta = np.subtract(calculate_forward_kinematics_robot(end_effector_pos), guess)
    
    cost = 0.5 * np.matmul(delta, delta)
    
    print(f"cost = {cost}")
    # print(f"delta: {delta}")
    print()

    return cost

def calculate_jacobian(joint_angles):
    """Calculate the jacobian of the end-effector position wrt joint angles.
    
    Calculate the jacobian, which is a matrix of the partial derivatives
    of the forward kinematics with respect to the joint angles 
    arranged as such:
    
    dx/dtheta1 dx/dtheta2 dx/dtheta3
    dy/dtheta1 dy/dtheta2 dy/dtheta3
    dz/dtheta1 dz/dtheta2 dz/dtheta3
    
    Args:
      joint_angles: joint angles of robot arm. Numpy array of 3 elements.
    
    Returns:
      Jacobian matrix. Numpy 3x3 array.
    """
    # 
    # TODO for students: Implement this function. ~5-10 lines of code.
    # h = 0.1
    # k = 1/h
    def derivatives(joint_angles, angle_index):
        h = 0.0001
        delta = np.array([0., 0., 0.])
        delta[angle_index] = h
        diff = calculate_forward_kinematics_robot(joint_angles + delta) - calculate_forward_kinematics_robot(joint_angles)
        return (diff/h)
    
    jacobian = np.array([derivatives(joint_angles, i) for i in range(3)])

    # print(f"jacobian angles = {jacobian}")

    return jacobian

# if __name__ == "__main__":
#     test_angles = calculate_jacobian(np.array([0,0,0]))

#     print("This is jacobian angles")
#     print(test_angles)



def calculate_inverse_kinematics(end_effector_pos, guess):
    """Calculates joint angles given desired xyz coordinates.

    Use gradient descent to minimize the inverse kinematics loss function. The
    joint angles that minimize the loss function are the joint angles that give 
    the smallest error from the actual resulting end-effector position to the
    desired end-effector position. You should use the jacobain function
    you wrote above.

    Args:
      end_effector_pos: Desired xyz coordinates of end-effector. Numpy array of 3 elements.
      guess: Guess at joint angles that achieve desired end-effector position. Numpy array of 3 elements.
    Returns:
      Joint angles that correspond to given desired end-effector position. Numpy array with 3 elements.
    """
    # TODO for students: Implement this function. ~10-20 lines of code.
    learning_rate = 15
    epilson = 0.005
    iterations = 0

    joint_angles = np.array([0.0, 0.0, 0.0])
    joint_angles = guess

 # calculate the jacobian
    while(ik_cost(end_effector_pos, joint_angles) > epilson and iterations < 10000):
        jacobian = calculate_jacobian(joint_angles)
        # calculate the gradient
        gradient = np.matmul(jacobian, np.transpose(calculate_forward_kinematics_robot(
            joint_angles) - end_effector_pos))
        # update the joint angles
        joint_angles -= learning_rate * gradient 
        iterations += 1

    # print(joint_angles)
    print(f"iterations = {iterations}")
    return joint_angles
    
# test_angles = np.zeros(6)
# print(calculate_inverse_kinematics(np.array([1.0, 1.0, 1.0]), test_angles))

# jointAngle = np.array([0.0, 0.52, 0.17])
jointAngle = np.array([0.0021, -0.05, 0.126])

jointGuess = np.array([0.0, 0.0, 0.0])
calculate_inverse_kinematics(jointAngle, jointGuess)

# ik_cost(jointAngle, jointGuess)

# calculate_forward_kinematics_robot(jointGuess)