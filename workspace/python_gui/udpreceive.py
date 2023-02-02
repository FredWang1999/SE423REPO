import socket
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import RadioButtons, Button

UDP_IP = "10.192.15.188"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.2)
plt.xlim(0, 10)
plt.ylim(0, 10)
points = []
line, = ax.plot(points, 'o')

color_rax = plt.axes([0.05, 0.5, 0.1, 0.15])
color = 'blue'
rax = RadioButtons(color_rax, ('blue', 'green', 'red'))

def update_color(label):
    global color
    color = label
rax.on_clicked(update_color)

clear_ax = plt.axes([0.8, 0.9, 0.1, 0.075])
clear_button = Button(clear_ax, 'Clear')

def clear_points(event):
    global points
    points = []
    line.set_data(list(zip(*points)))
clear_button.on_clicked(clear_points)

stop_ax = plt.axes([0.9, 0.9, 0.1, 0.075])
stop_button = Button(stop_ax, 'Stop')

stop_flag = True
clear_flag = False
def stop_receiving(event):
    global stop_flag
    stop_flag = not stop_flag
    clear_flag = True
stop_button.on_clicked(stop_receiving)

# stop_ax = plt.axes([0.9, 0.05, 0.1, 0.075])
# stop_button = Button(stop_ax, 'Stop')

# stop_flag = True

# def stop_receiving(event):
#     global stop_flag
#     stop_flag = False
#     stop_button.on_clicked(stop_receiving)


def update_plot(frame):
    try:
        data, addr = sock.recvfrom(1024)
        if stop_flag and data:
            point = data.decode().strip().split(',')
            point = [float(x) for x in point]
            if not clear_flag:
                points.append(point)
                line.set_data(list(zip(*points)))
                line.set_color(color)
    except socket.timeout:
        pass

ani = FuncAnimation(fig, update_plot, frames=None, repeat=True, blit=False, interval=100)
plt.show()
