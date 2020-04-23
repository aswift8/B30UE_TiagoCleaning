#!/usr/bin/env python

import matplotlib.pyplot as plt
import xml.etree.cElementTree
from PathGenerator import run as run_generator


def read_path(filename):
    root = xml.etree.cElementTree.parse(filename).getroot()
    paths = []
    for point in root:
        x = float(point.find("x").text)
        y = float(point.find("y").text)
        paths.append((x, y))
    return paths


def split_path_into_elements(path_list):
    x_coords = []
    y_coords = []
    for i in range(0, len(path_list)):
        x_coords.append(path_list[i][0])
        y_coords.append(path_list[i][1])
    return x_coords, y_coords


def draw_path(path_list, line_color):
    global ax
    x_coords, y_coords = split_path_into_elements(path_list)
    ax.plot(x_coords, y_coords, color=line_color)
    plt.draw()


def update_position(vector):
    global current_path, current_position, last_position
    current_position = vector.x, vector.y
    if current_position[0] != last_position[0] or current_position[1] != last_position[1]:
        current_path.append(current_position)
        last_position = current_position
        draw_path(current_path, "red")
    pass


options = ["6", "-4", "1", "30", "false", "gen_path.xml"]
run_generator(options)

test_path = read_path(options[5])
#test_path = [(0, 0), (5, 0), (5, 1), (0, 1), (0, 2), (5, 2), (5, 3), (0, 3)]

current_path = [(0, 0)]

last_position = (0, 0)
current_position = (0, 0)

fig = plt.figure()
fig.canvas.set_window_title("test readings")
fig.subplots_adjust(top=0.8)

ax = fig.add_subplot(1, 1, 1)
ax.title.set_text("Path on Plane")
ax.set_ylabel("Y Axis")
ax.set_xlabel("X Axis")

ax.set_xlim(-5, 5)
ax.set_ylim(-5, 5)

plt.ion()
plt.show()

ax.clear()

draw_path(test_path, 'blue')

while True:
    plt.pause(0.01)
