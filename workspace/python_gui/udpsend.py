import socket
import random
import time 

UDP_IP = "10.192.15.188"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while True:
    x = random.uniform(0, 10)
    y = random.uniform(0, 10)
    point = f"{x},{y}\n"
    sock.sendto(point.encode(), (UDP_IP, UDP_PORT))
    time.sleep(0.5)