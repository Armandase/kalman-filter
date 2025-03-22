import time
import socket
import argparse

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

def compute_response(data:dict, client_socket):
    next_pos = ""
    # next_pos = str(data["TRUE POSITION"][0]) + " " + str(data["TRUE POSITION"][1]) + " " + str(data["TRUE POSITION"][2])
    for val in data["TRUE POSITION"]:
        next_pos += str(val)
        next_pos += " "
    # print("NextPos:", next_pos, end="")
    # print("RE:", next_pos[-3:-1])
    next_pos = next_pos.removesuffix(" ")
    send_msg(client_socket, next_pos)

def get_dict():
    return  {
        "TRUE POSITION":[0, 0, 0],
        "SPEED":[0],
        "ACCELERATION":[0, 0, 0],
        "DIRECTION":[0, 0, 0],
    }

def read(client_socket, address="127.0.0.1", port=4242):
    parsed = get_dict()
    while True:
        try:
            data, server = client_socket.recvfrom(1024)
            data_decode:str = data.decode()
            if "MSG_END" in data_decode:
                compute_response(parsed, client_socket)
                parsed = get_dict()
            else:
                for key in parsed.keys():
                    if key in data_decode:
                        lines = data_decode.splitlines()
                        valeurs = [float(val) for val in lines[1:]]
                        parsed[key] = valeurs
            print(f"Readable data:", data_decode)
            
        except socket.timeout:
            print('REQUEST TIMED OUT')
            exit()

def main(address="127.0.0.1", port=4242):
    client_socket = connect(addr=address, port=port)
    read(client_socket, address, port)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--address', '-a', default="127.0.0.1", type=str)
    parser.add_argument('--port', '-p', default=4242, type=int)
    args = parser.parse_args()
    main(args.address, args.port)