import time
import socket
import argparse
import matplotlib.pyplot as plt
import numpy as np
from kf_partial import KalmanFilter
from filters import mobile_average, low_pass_filter

def send_msg(client_socket, msg, addr=("127.0.0.1", 4242)):
    client_socket.sendto(msg.encode('UTF-8'), addr)
    return client_socket

def connect(addr="127.0.0.1", port=4242):
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client_socket.settimeout(3.0)
    complete_addr = (addr, port)
    msg = "READY"
    send_msg(client_socket, msg, addr=complete_addr) # init connexion between imu and client
    return client_socket

# def extract_state(data:dict):
    # E = np.array([data[""])

def compute_response(data:dict, client_socket, filter):
    if filter is None:
        filter = KalmanFilter(data['SPEED'], [0, 0, 0], data["ACCELERATION"])
    
    # state = extract_state(data)
    next_pos = ""
    for val in data["TRUE POSITION"]:
        next_pos += str(val)
        next_pos += " "
    next_pos = next_pos.removesuffix(" ")
    send_msg(client_socket, next_pos)

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
    avg = mobile_average(acceleration[0])
    low_pass = low_pass_filter(acceleration[0])
    ax3.plot(acceleration[0], label="X")
    ax3.plot(avg, label="X MA")
    ax3.plot(low_pass, label="X LP")
    # ax3.plot(acceleration[1], label="Y")
    # ax3.plot(acceleration[2], label="Z")
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

def get_dict():
    return  {
        "TRUE POSITION":[0, 0, 0],
        "SPEED":[0],
        "ACCELERATION":[0, 0, 0],
        "DIRECTION":[0, 0, 0],
    }

def get_empty_dict():
    return {
        "TRUE POSITION":[],
        "SPEED":[],
        "ACCELERATION":[],
        "DIRECTION":[],
    }

def read(client_socket, address="127.0.0.1", port=4242):
    parsed = get_dict()
    history = get_empty_dict()
    filter = None
    while True:
        try:
            data, server = client_socket.recvfrom(1024)
            data_decode:str = data.decode()
            if "MSG_END" in data_decode:
                compute_response(parsed, client_socket, filter)
                parsed = get_dict()
            else:
                for key in parsed.keys():
                    if key in data_decode:
                        lines = data_decode.splitlines()
                        valeurs = [float(val) for val in lines[1:]]
                        history[key].append(valeurs)
                        parsed[key] = valeurs
            print(f"Readable data:", data_decode)
            
        except socket.timeout:
            print('REQUEST TIMED OUT')
            break

    return history    

def main(address="127.0.0.1", port=4242):
    client_socket = connect(addr=address, port=port)
    history = read(client_socket, address, port)
    display_history(history)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--address', '-a', default="127.0.0.1", type=str)
    parser.add_argument('--port', '-p', default=4242, type=int)
    args = parser.parse_args()
    main(args.address, args.port)