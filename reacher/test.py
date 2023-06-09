import math
import numpy as np
import copy

HIP_OFFSET = 0.05 # length of N
L1 = 0.08 # length of link 1, the hip to knee
L2 = 0.11 # length of link 2, the knee to foot

##### THE CODE WE HAD LAST FRIDAY 5/19/23
# def calculate_forward_kinematics_robot(joint_angles):
#     # def generate_y_rotation(theta):
#     #     return np.array([
#     #         [np.cos(theta), 0., -np.sin(theta)],
#     #         [0., 1., 0.],
#     #         [np.sin(theta), 0., np.cos(theta)]
#     #     ])

#     # def generate_z_rotation(theta):
#     #     return np.array([
#     #         [np.cos(theta), -np.sin(theta), 0.],
#     #         [np.sin(theta), np.cos(theta), 0.],
#     #         [0., 0., 1.]
#     #     ])
#   def generate_y_rotation(joint_angles):
#       print(f"joint angles in y-rot = {joint_angles}")
#       print(f"sine = {np.sin(joint_angles)}")
#       return np.array([
#           [np.cos(joint_angles), 0., np.sin(joint_angles)],
#           [0., 1., 0.],
#           [-np.sin(joint_angles), 0., np.cos(joint_angles)]
#       ], dtype=float)


#   def generate_z_rotation(joint_angles):
#       return np.array([
#           [np.cos(joint_angles), -np.sin(joint_angles), 0],
#           [np.sin(joint_angles), np.cos(joint_angles), 0],
#           [0., 0., 1.]
#       ])

#   joint_angles *= -1
#   r1 = np.array([0., 0., L2])
#   r2 = np.array([0., 0., L1]) + \
#       np.matmul(generate_y_rotation(joint_angles[2]), np.transpose(r1))
#   r3 = np.array([0., -HIP_OFFSET, 0.]) + \
#       np.matmul(generate_y_rotation(joint_angles[1]), np.transpose(r2))
#   r4 = np.matmul(generate_z_rotation(joint_angles[0]), np.transpose(r3))

#   # print(f"r2 = {r2}")
#   # print("r3 = ", r3)
#   # print("r4 = ", r4)
#   # print()

#   return r4

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
        np.dot(generate_y_rotation(joint_angles[2]), r1)
    r3 = np.array([0, -HIP_OFFSET, 0]).transpose() + \
        np.dot(generate_y_rotation(joint_angles[1]), r2)
    r4 = np.dot(generate_z_rotation(joint_angles[0]), r3)

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
    
    cost = 0.5 * np.dot(delta, delta)
    
    print(f"cost = {cost}")
    print()
    # print(delta)
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



    # curr_pos = calculate_forward_kinematics_robot(joint_angles)

    # delta_x1 = np.array([joint_angles[0] + h, joint_angles[1], joint_angles[2], joint_angles[3], joint_angles[4], joint_angles[5]])
    # delta_y1 = np.array([joint_angles[0], joint_angles[1] + h, joint_angles[2], joint_angles[3], joint_angles[4], joint_angles[5]])
    # delta_z1 = np.array([joint_angles[0], joint_angles[1], joint_angles[2] + h, joint_angles[3], joint_angles[4], joint_angles[5]])
    # delta_x2 = np.array([joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3] + h, joint_angles[4], joint_angles[5]])
    # delta_y2 = np.array([joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3], joint_angles[4] + h, joint_angles[5]])
    # delta_z2 = np.array([joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3], joint_angles[4], joint_angles[5] + h])

    # delta_x1 = np.subtract(delta_x1, curr_pos) * 1/h
    # delta_y1 = np.subtract(delta_y1, curr_pos) * 1/h
    # delta_z1 = np.subtract(delta_z1, curr_pos) * 1/h
    # delta_x2 = np.subtract(delta_x2, curr_pos) * 1/h
    # delta_y2 = np.subtract(delta_y2, curr_pos) * 1/h
    # delta_z2 = np.subtract(delta_z2, curr_pos) * 1/h

    # jacobian = np.array([
    #         [delta_x1[0], delta_x1[1], delta_x1[2], delta_x1[3], delta_x1[4], delta_x1[5]],
    #         [delta_y1[0], delta_y1[1], delta_y1[2], delta_y1[3], delta_y1[4], delta_y1[5]],
    #         [delta_z1[0], delta_z1[1], delta_z1[2], delta_z1[3], delta_z1[4], delta_z1[5]],
    #         [delta_x2[0], delta_x2[1], delta_x2[2], delta_x2[3], delta_x2[4], delta_x2[5]],
    #         [delta_y2[0], delta_y2[1], delta_y2[2], delta_y2[3], delta_y2[4], delta_y2[5]],
    #         [delta_z2[0], delta_z2[1], delta_z2[2], delta_z2[3], delta_z2[4], delta_z2[5]]
    #     ])
    print(f"jacobian angles = {jacobian}")

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
    learning_rate = 0.01
    epilson = 0.1
    iterations = 0

    joint_angles = np.array([0.0, 0.0, 0.0])
    joint_angles = guess
    while(ik_cost(end_effector_pos, joint_angles) > epilson and iterations < 1000):
      joint_angles = joint_angles - (learning_rate * calculate_jacobian(joint_angles[0:3]))
      print(f"jacobian = {calculate_jacobian((joint_angles[0:3]))}")
      iterations += 1
      print(f"iterations = {iterations}")

    print(joint_angles)
    return joint_angles
    
# test_angles = np.zeros(6)
# print(calculate_inverse_kinematics(np.array([1.0, 1.0, 1.0]), test_angles))

jointAngle = np.array([1.0, 1.0, 1.0])
jointGuess = np.array([0.0, 0.0, 0.0])
calculate_inverse_kinematics(jointAngle, jointGuess)

# ik_cost(jointAngle, jointGuess)

# calculate_forward_kinematics_robot(jointAngle)
