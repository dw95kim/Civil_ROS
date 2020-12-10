import socket
import numpy as np
import os
import struct

recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock.bind(("192.168.0.5", 9090))

type_scale = 4
total_size = 6

while True:
    recv_data = [0 for i in range(total_size)]
    received_data, addr = recv_sock.recvfrom(128)

    print(received_data)
    print(" ")

    for i in range(total_size):
        x = struct.unpack('f', received_data[0 + type_scale * i : type_scale + type_scale * i])[0]
        recv_data[i] = float(x)

    print(recv_data)