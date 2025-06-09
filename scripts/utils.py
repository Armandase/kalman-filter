import numpy as np
import matplotlib.pyplot as plt
from kalman_filter import KalmanFilter
from scipy.spatial.transform import Rotation as R


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
    return rot.reshape((3, 3))

def get_rotation_matrix_from_euler(euler_angles):
    rot = R.from_euler('xyz', euler_angles, degrees=True)
    return rot.as_matrix()

def update_pos(pos, acceleration, direction, delta_t, speed):
    direction = np.array(direction)
    # rotation_matrix = rotation_matrix_from_euler(*direction)
    rotation_matrix = get_rotation_matrix_from_euler(direction)
    scaled_acceleration = acceleration * speed
    new_velocity = rotation_matrix @ scaled_acceleration * delta_t
    new_pos = pos + new_velocity * delta_t + 0.5 * scaled_acceleration * (delta_t ** 2)
    return new_pos

def compute_velocity(acceleration, euler_angles, delta_t):
    rotation_matrix = get_rotation_matrix_from_euler(euler_angles)
    velocity = rotation_matrix @ acceleration * delta_t
    return velocity

def obtain_velocity(acceleration, euler_angles, delta_t):
    roll = euler_angles[0]
    pitch = euler_angles[1]
    yaw = euler_angles[2]

    r_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])
    r_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])
    r_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])
    r = r_z @ r_y @ r_x
    a_x = r[0, 0] * acceleration[0] + r[0, 1] * acceleration[1] + r[0, 2] * acceleration[2]
    a_y = r[1, 0] * acceleration[0] + r[1, 1] * acceleration[1] + r[1, 2] * acceleration[2]
    a_z = r[2, 0] * acceleration[0] + r[2, 1] * acceleration[1] + r[2, 2] * acceleration[2]
    velocity = np.array([a_x, a_y, a_z])
    velocity *= delta_t
    return velocity

if __name__ == '__main__':
    gps_point = np.array([1, 2, 3])
    euler_angle = np.array([0.1, 0.2, 0.3])

    acceleration = np.array([1, 0, 0])
    delta_t = 0.01

    own_velocity = compute_velocity(acceleration, euler_angle, delta_t)
    print("Computed Velocity:", own_velocity)

    base_velocity = obtain_velocity(acceleration, euler_angle, delta_t)
    print("Base Velocity:", base_velocity)