import time
import socket
import argparse
import numpy as np
# from kf_partial import KalmanFilter
from utils import display_history, array_to_reponse
from kalman_filter import KalmanFilter



def send_msg(client_socket, msg, addr=("127.0.0.1", 4242)):
    print("Message:", msg)
    client_socket.sendto(msg.encode('UTF-8'), addr)
    return client_socket

def connect(addr="127.0.0.1", port=4242):
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client_socket.settimeout(3.0)
    complete_addr = (addr, port)
    msg = "READY"
    send_msg(client_socket, msg, addr=complete_addr) # init connexion between imu and client
    return client_socket


def compute_response(data:dict, client_socket, filter):

    # If no Filter class is created, create one
    if filter is None:
        filter = KalmanFilter(direction=data['DIRECTION'], acceleration=data["ACCELERATION"], speed=data['SPEED'], true_pos=data["TRUE POSITION"])
        filter.measurement_update(direction=data['DIRECTION'], speed=data['SPEED'], true_pos=data["TRUE POSITION"])

    # if data["TRUE POSITION"]:
    #     print("True position:", data["TRUE POSITION"])
    #     # Update the filter with the true position
    
    
    # Update the filter
    next_pos = filter.time_update(acceleration=data["ACCELERATION"])

    response = array_to_reponse(next_pos)
    send_msg(client_socket, response)
    return filter

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
        "ESTIMATED POSITION":[],
    }

def read(client_socket, address="127.0.0.1", port=4242):
    parsed = get_dict()
    history = get_empty_dict()
    counter = 0
    filter = None
    while True:
        try:
            data, server = client_socket.recvfrom(1024)
            data_decode:str = data.decode()
            print(f"Received data: {data_decode}")
            if "MSG_END" in data_decode and counter < 3000:
                filter = compute_response(parsed, client_socket, filter)
                history["TRUE POSITION"].append(parsed["TRUE POSITION"])
                if filter is not None:
                    history["ESTIMATED POSITION"].append(filter.estim_x[:3].tolist())
                parsed = get_dict()
                counter += 1
            else:
                for key in parsed.keys():
                    if key in data_decode:
                        lines = data_decode.splitlines()
                        valeurs = [float(val) for val in lines[1:]]
                        history[key].append(valeurs)
                        parsed[key] = valeurs
            # print(f"Readable data:", data_decode)
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