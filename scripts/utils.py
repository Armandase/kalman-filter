import numpy as np
import matplotlib.pyplot as plt
from kalman_filter import KalmanFilter


def display_history(history):
    fig = plt.figure(figsize=(12, 10))

    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    true_pos = np.array(history["TRUE POSITION"]).T
    ax1.plot3D(true_pos[0], true_pos[1], true_pos[2], c='red')
    ax1.set_title("True Position")

    ax3 = fig.add_subplot(2, 3, 2)
    acceleration = np.array(history['ACCELERATION']).T
    ax3.plot(acceleration[0], label="X")
    ax3.plot(acceleration[1], label="Y")
    ax3.plot(acceleration[2], label="Z")
    ax3.set_title("Acceleration")
    ax3.legend()

    ax4 = fig.add_subplot(2, 3, 3)
    direction = np.array(history['DIRECTION']).T
    ax4.plot(direction[0], label="X")
    ax4.plot(direction[1], label="Y")
    ax4.plot(direction[2], label="Z")
    ax4.set_title("Direction")
    ax4.legend()

    # Add estimated position plot
    if "ESTIMATED POSITION" in history and len(history["ESTIMATED POSITION"]) > 0:
        ax5 = fig.add_subplot(2, 3, 5, projection='3d')
        estimated_pos = np.array(history["ESTIMATED POSITION"]).T
        ax5.plot3D(estimated_pos[0], estimated_pos[1], estimated_pos[2], c='blue')
        ax5.set_title("Estimated Position")

    plt.tight_layout()
    plt.show()

def array_to_reponse(data):
    response = ""
    for val in data:
        response += "{:.6f}".format(val)
        response += " "
    response = response.removesuffix(" ")
    print(response)
    return response