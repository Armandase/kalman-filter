import numpy as np
import matplotlib.pyplot as plt
from kalman_filter import KalmanFilter
from scipy.spatial.transform import Rotation as R


def display_history(history):
    fig = plt.figure(figsize=(10, 8))

    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    true_pos = np.array(history["TRUE POSITION"]).T
    ax1.plot3D(true_pos[0], true_pos[1], true_pos[2], c='red')
    pred_pos = np.array(history["PRED POSITION"]).T
    ax1.plot3D(pred_pos[0], pred_pos[1], pred_pos[2], c='blue')
    ax1.set_title("True Position")

    ax2 = fig.add_subplot(2, 2, 2)
    speed = np.array(history['SPEED'])
    ax2.plot(speed)
    ax2.set_title("Speed")

    ax3 = fig.add_subplot(2, 2, 3)
    acceleration = np.array(history['ACCELERATION']).T
    # avg = mobile_average(acceleration[0])
    # low_pass = low_pass_filter(acceleration[0])
    # ax3.plot(avg, label="X MA")
    # ax3.plot(low_pass, label="X LP")
    ax3.plot(acceleration[0], label="X")
    ax3.plot(acceleration[1], label="Y")
    ax3.plot(acceleration[2], label="Z")
    ax3.set_title("Acceleration")
    ax3.legend()

    ax4 = fig.add_subplot(2, 2, 4)
    direction = np.array(history['DIRECTION']).T
    ax4.plot(direction[0], label="X")
    ax4.plot(direction[1], label="Y")
    ax4.plot(direction[2], label="Z")
    ax4.set_title("Direction")
    ax4.legend()

    plt.tight_layout()
    plt.show()

def display_pos_offset(history):
    fig = plt.figure(figsize=(10, 8))
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    true_pos = np.array(history["TRUE POSITION"]).T
    pred_pos = np.array(history["PRED POSITION"]).T
    ax1.plot3D(true_pos[0], true_pos[1], true_pos[2], c='green')
    ax1.plot3D(pred_pos[0], pred_pos[1], pred_pos[2], c='red')
    ax1.set_title("True Position vs Predicted Position")
    ax1.set_xlabel("X")
    ax1.set_ylabel("Y")
    ax1.set_zlabel("Z")
    ax1.legend(["True Position", "Predicted Position"])
    plt.tight_layout()
    plt.show()    

def array_to_reponse(data):
    response = ""
    for val in data:
        response += "{:.6f}".format(val)
        response += " "
    response = response.removesuffix(" ")
    return response


def rotation_matrix_from_euler(roll, pitch, yaw):
    # Rotation around X (roll)
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    # Rotation around Y (pitch)
    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    # Rotation around Z (yaw)
    R_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    # return R_z @ R_y @ R_x
    return R_x @ R_y @ R_z

def compute_velocity(euler_angles, velocity, delta_t, acceleration):
    rotation_matrix = rotation_matrix_from_euler(*euler_angles)
    
    global_accel = rotation_matrix @ acceleration
    
    new_velocity = velocity + global_accel * delta_t
    
    return new_velocity
