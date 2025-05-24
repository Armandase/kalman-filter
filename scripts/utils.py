import numpy as np
import matplotlib.pyplot as plt
from kalman_filter import KalmanFilter


def display_history(history):
    fig = plt.figure(figsize=(10, 8))

    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    true_pos = np.array(history["TRUE POSITION"]).T
    ax1.plot3D(true_pos[0], true_pos[1], true_pos[2], c='red')
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

def euler_angles_intergrate(angles):
    angular_velocity = 1 / np.cos(angles[1])

    tmp = np.array([
        [0, np.sin(angles[-1]), np.cos(angles[-1])],
        [0, np.cos(angles[-1]) * np.cos(angles[1]), -np.sin(angles[-1]) * np.cos(angles[1])],
        [np.cos(angles[1]), np.sin(angles[-1]) * np.sin(angles[1]), np.cos(angles[-1]) * np.sin(angles[1])]
    ])
    gyro = angular_velocity * tmp
    return gyro

def rotation_matrix_from_euler(roll, pitch, yaw):
    # yaw is psi, pitch is theta and roll is phi.

    # rotation around Z
    rot_psi = np.array([
        [np.cos(roll), -np.sin(roll), 0],
        [np.sin(roll), np.cos(roll), 0],
        [0, 0, 1],])

    # rotation around N which is the X axis after rot_psi
    rot_theta = np.array([
        [1, 0 ,0],
        [0, np.cos(pitch), -np.sin(pitch)],
        [0, np.sin(pitch), np.cos(pitch)]])
    
    # rotation around Z'
    rot_phi = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1],])
    
    # apply the 3 rotation
    return rot_psi @ rot_theta @ rot_phi

# return the same thing as rotation_matrix_from_euler but this way is a bit slower (based on computation time)
def change_of_basis(psi, theta, phi):
    rot = np.array([
            np.cos(psi) * np.cos(phi) - np.sin(psi) * np.cos(theta) * np.sin(phi), -np.cos(psi) * np.sin(phi) - np.sin(psi) * np.cos(theta) * np.cos(phi), np.sin(psi) * np.sin(theta),
            np.sin(psi) * np.cos(phi) + np.cos(psi) * np.cos(theta) * np.sin(phi), -np.sin(psi) * np.sin(phi) + np.cos(psi) * np.cos(theta) * np.cos(phi), -np.cos(psi) * np.sin(theta),
            np.sin(theta) * np.sin(phi), np.sin(theta) * np.cos(phi), np.cos(theta)
    ])
    return rot 

def update_pos(pos, acceleration, direction, delta_t, speed):
    rotation_matrix = rotation_matrix_from_euler(*direction)
    scaled_acceleration = acceleration * speed
    new_pos = pos + rotation_matrix @ scaled_acceleration * delta_t
    return new_pos

if __name__ == '__main__':
    gps_point = np.array([1, 2, 3])
    euler_angle = np.array([0.1, 0.2, 0.3])

    print("GPS Point:", gps_point)
    print("Euler Angles:", euler_angle)
    acceleration = np.array([1, 0, 0])
    delta_t = 0.01
    next_pos = update_pos(gps_point, acceleration, euler_angle, delta_t)
    print("Next Position:", next_pos)