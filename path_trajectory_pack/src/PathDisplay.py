#!/usr/bin/env python

import matplotlib.pyplot as plt
#import PathControl as pc
#import PathGenerator as pg


def split_path_into_elements(path_list):
    x_coords = []
    y_coords = []
    for i in range(0, len(path_list)):
        x_coords.append(path_list[i][0])
        y_coords.append(path_list[i][1])
    return x_coords, y_coords


def draw_path(path_list, line_color):
    global ax
    my_x_coords, my_y_coords = split_path_into_elements(test_path)
    ax.plot(my_x_coords, my_y_coords, color=line_color)
    plt.draw()


test_path = [(0,0), (5,0), (5,1), (0,1), (0,2), (5,2), (5,3), (0,3)]


fig = plt.figure()
fig.canvas.set_window_title("test readings")
fig.subplots_adjust(top=0.8)

ax = fig.add_subplot()
ax.set_ylabel("Y Axis")
ax.set_xlabel("X Axis")

my_x_coords, my_y_coords = split_path_into_elements(test_path)

plt.ion()
plt.show()

ax.clear()
draw_path(test_path, 'blue')
plt.pause(0.01)



while True:
  # Update x and y coords
  #ax.clear()
  #ax.plot(my_x_coords, my_y_coords, color='blue')
  #plt.draw()
  plt.pause(0.01)
  #pass
