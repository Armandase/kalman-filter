import time
import socket
import argparse
import numpy as np
# from kf_partial import KalmanFilter
from utils_v2 import display_history, array_to_reponse, display_pos_offset, compute_velocity
from kalman_filter_v2 import KalmanFilter
from real_kalman import KakalmanFilter
from constants import DELTA_T 


def send_msg(client_socket, msg, addr=("127.0.0.1", 4242)):
    client_socket.sendto(msg.encode('UTF-8'), addr)
    return client_socket

def connect(addr="127.0.0.1", port=4242):
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client_socket.settimeout(2.0)
    complete_addr = (addr, port)
    msg = "READY"
    send_msg(client_socket, msg, addr=complete_addr) # init connexion between imu and client
    return client_socket

def compute_response(data:dict, client_socket, filter):
    if filter is None:
        # filter = KakalmanFilter(true_pos=data["TRUE POSITION"], 
        filter = KalmanFilter(true_pos=data["TRUE POSITION"], 
                                acceleration=data["ACCELERATION"], 
                                speed=data['SPEED'], 
                                direction=data['DIRECTION'])

    # filter.B = filter.B_default * (np.array(data["ACCELERATION"]) * DELTA_T)
    # filter.B = filter.B_default * (np.array(data["ACCELERATION"]))
    filter.predict(np.array(data["ACCELERATION"]))

    if data["POSITION"] is None or data["POSITION"] != [0, 0, 0]:
        filter.H = np.eye(6)  # Reset observation matrix
        velocity = compute_velocity(euler_angles=data["DIRECTION"], delta_t=DELTA_T, velocity=filter.x[3:6], acceleration=data["ACCELERATION"])
        filter.update(np.concatenate((data["POSITION"], velocity)))
    else:
        filter.H = np.eye(6)  # Reset observation matrix
        filter.H[1, 1] = 0
        filter.H[2, 2] = 0
        filter.H[3, 3] = 0
        print("Position is zero, using direction and acceleration for update.")
        velocity = compute_velocity(euler_angles=data["DIRECTION"], delta_t=DELTA_T, velocity=filter.x[3:6], acceleration=data["ACCELERATION"])
        filter.update(np.concatenate((filter.x[:3], velocity)))
    next_pos = filter.x[:3]
    print("Next pos: ", next_pos)
    print("True pos: ", data["TRUE POSITION"])
    response = array_to_reponse(next_pos)
    send_msg(client_socket, response)
    return filter

def get_dict():
    return  {
        "POSITION":[0, 0, 0],
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
        "ESTIMATED POSITION":[],
        "POSITION":[],
    }

def read(client_socket):
    parsed = get_dict()
    history = get_empty_dict()
    filter = None
    counter = 0
    while True:
        try:
            data, _ = client_socket.recvfrom(1024)
            # check if the server is still available
            if not data or data == b'':
                exit(0)
            data_decode:str = data.decode()
            if "MSG_END" in data_decode:
                filter = compute_response(parsed, client_socket, filter)
                # check if pred pos is in history
                if "PRED POSITION" not in history:
                    history["PRED POSITION"] = []
                history["PRED POSITION"].append(filter.x[:3].tolist())
                parsed = get_dict()
                counter += 1
                print(f"Counter: {counter}")
                # if counter > 3:
                    # exit(0)
            else:
                print("Data received: ", data_decode)
                for key in parsed.keys():
                    if key in data_decode:
                        lines = data_decode.splitlines()
                        valeurs = [float(val) for val in lines[1:]]
                        history[key].append(valeurs)
                        parsed[key] = valeurs
        except (socket.timeout, socket.error):
            print('REQUEST TIMED OUT')
            break

    return history    

def launch_imu():
    # ./imu-sensor-stream-linux -s 42 -d 10 -p 4242 --debug
    import os
    import subprocess
    os.system("pkill imu-sensor-stream-linux")  # Kill any existing imu sensor stream
    # command = "./imu-sensor-stream-linux -s 42 -d 10 -p 4242 --debug"
    # command = "./imu-sensor-stream-linux -s 42 -d 10 -p 4242"
    command = "./imu-sensor-stream-linux -s 42 -p 4242 --debug"
    gnome_terminal_command = f"gnome-terminal -- bash -c '{command}'"
    process = subprocess.Popen(gnome_terminal_command, shell=True)
    if process.poll() is not None:
        print("Failed to launch imu sensor stream in GNOME terminal.")
        return
    print("IMU sensor stream launched in GNOME terminal.")
    sleep_time = 2
    time.sleep(sleep_time) 

def main(address="127.0.0.1", port=4242, visual=False, imu=False):
    if imu:
        launch_imu()
    client_socket = connect(addr=address, port=port)
    history = read(client_socket)
    if visual:
        # display_history(history)
        display_pos_offset(history)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--address', '-a', default="127.0.0.1", type=str)
    parser.add_argument('--port', '-p', default=4242, type=int)
    parser.add_argument('--visual', '-v', default=False, action=argparse.BooleanOptionalAction, help="Display the history of the data")
    parser.add_argument('--imu', default=False, action=argparse.BooleanOptionalAction) # enbale or disable imu
    args = parser.parse_args()
    main(args.address, args.port, args.visual, args.imu)