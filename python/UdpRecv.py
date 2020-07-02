#! /usr/bin/env python
import socket
import sys


if len(sys.argv) < 3:
    print("usage: python UdpRecv.py <local-ip> <local-port>")
    exit(-1)

ip = sys.argv[1]
port = int(sys.argv[2])

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((ip, port))

while True:
    data, addr = sock.recvfrom(1024)
    values = [int.from_bytes(data[i:i+2], 'big', signed=False) for i in range(0, len(data), 2)]

    print("Received from [{}:{}] {}".format(addr[0], addr[1], values))
