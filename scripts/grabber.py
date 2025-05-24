import time
import socket
import argparse
import numpy as np
# from kf_partial import KalmanFilter
from utils import display_history, array_to_reponse, display_pos_offset
from kalman_filter import KalmanFilter
from real_kalman import KakalmanFilter


def send_msg(client_socket, msg, addr=("127.0.0.1", 4242)):
    print("Message:", msg)
    client_socket.sendto(msg.encode('UTF-8'), addr)
    return client_socket

def connect(addr="127.0.0.1", port=4242):
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client_socket.settimeout(2.0)
    complete_addr = (addr, port)
    msg = "READY"
    send_msg(client_socket, msg, addr=complete_addr) # init connexion between imu and client
    return client_socket

# def extract_state(data:dict):
    # E = np.array([data[""])

def compute_response(data:dict, client_socket, filter):
    if filter is None:
        # filter = KalmanFilter(direction=data['DIRECTION'], acceleration=data["ACCELERATION"], speed=data['SPEED'], true_pos=data["TRUE POSITION"])
        filter = KakalmanFilter(true_pos=data["TRUE POSITION"], 
                            acceleration=data["ACCELERATION"], 
                            speed=data['SPEED'], 
                            direction=data['DIRECTION'])

    
    # response = array_to_reponse(data["TRUE POSITION"])
    # x_estimated = filter.predict()
    filter.predict()
    x_estimated = filter.x
    print("x_estimated:", x_estimated)
    velocity = filter.calculate_velocity(data['SPEED'], data['DIRECTION'])
    filter.update(np.concatenate((velocity, np.array(data["ACCELERATION"]))))
    next_pos = filter.update_position(velocity=x_estimated[:3], acceleration=x_estimated[3:6])
    # print("next_pos:", next_pos)
    response = array_to_reponse(next_pos)
    # print("Response: ",response)
    print("next pos as str", next_pos)
    print('real next pos', data["TRUE POSITION"])
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
    }

def read(client_socket, address="127.0.0.1", port=4242):
    parsed = get_dict()
    history = get_empty_dict()
    filter = None
    counter = 0
    while True:
        try:
            data, server = client_socket.recvfrom(1024)
            # check if the server is still available
            if not data or data == b'':
                exit(0)
            data_decode:str = data.decode()
            # print(f"Received data: {data_decode}")
            if "MSG_END" in data_decode:
                filter = compute_response(parsed, client_socket, filter)
                # check if pred pos is in history
                # if "PRED POSITION" not in history:
                    # history["PRED POSITION"] = []
                # history["PRED POSITION"].append(filter.pos)
                parsed = get_dict()
                counter += 1
                # if counter > 3:
                    # exit(0)
            else:
                for key in parsed.keys():
                    if key in data_decode:
                        lines = data_decode.splitlines()
                        valeurs = [float(val) for val in lines[1:]]
                        history[key].append(valeurs)
                        parsed[key] = valeurs
            # print(f"Readable data:", data_decode)
        except (socket.timeout, socket.error):
            print('REQUEST TIMED OUT')
            break

    return history    

def main(address="127.0.0.1", port=4242, visual=False):
    client_socket = connect(addr=address, port=port)
    history = read(client_socket, address, port)
    if visual:
        # display_history(history)
        display_pos_offset(history)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--address', '-a', default="127.0.0.1", type=str)
    parser.add_argument('--port', '-p', default=4242, type=int)
    parser.add_argument('--visual', '-v', action='store_true', help="Display the history of the data")
    args = parser.parse_args()
    main(args.address, args.port, args.visual)