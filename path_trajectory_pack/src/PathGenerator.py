#!/usr/bin/env python

import sys
import math
import numpy


class Settings:
    length = 0.0
    height = 0.0
    sponge_diameter = 0.0
    theta = 0.0
    transpose = False
    xml_address = ""
    def new_settings(list):
        if len(list) != 6:
            raise IndexError
        Settings.length = float(list[0])
        Settings.height = float(list[1])
        Settings.sponge_diameter = float(list[2])
        Settings.theta = float(list[3])
        Settings.transpose = list[4].lower() == "true"
        Settings.xml_address = list[5]
    def print_variables():
        version_print("Length: "+ str(Settings.length))
        version_print("Height: "+ str(Settings.height))
        version_print("Sponge Diameter: "+ str(Settings.sponge_diameter))
        version_print("Transpose Path: "+ str(Settings.transpose))
        version_print("Save Path: "+ Settings.xml_address + "\n")


def list_to_string(list):
    length = len(list)
    string = ""
    for i in range(0, length-1):
        string = string + list[i] + " "
    string = string + list[length-1]
    return string


def save_path(path_list):
    version_print(str(path_list))

    string = "<root>\n"
    for i in range(0, len(path_list)):
        string = string + write_point(path_list[i])
    string = string + "</root>"

    f = open(Settings.xml_address, "w")
    f.write(string)
    f.close()


def write_point(point):
    tab = "  "
    string = tab + "<point>\n"
    string = string + tab + tab + "<x>" + str(point[0]) + "</x>\n"
    string = string + tab + tab + "<y>" + str(point[1]) + "</y>\n"
    string = string + tab + "</point>\n"
    return string


# change depending on what version of python you're using
def version_print(string):
    print(string)


def print_argment_format():
    version_print("Arguments: <float - length> <float - height> <float - sponge diameter> <boolean - transpose path> <string - save/path>")


def run(list):
    try:
        Settings.new_settings(list)
        Settings.print_variables()
        path = generate_path()
        path = rotate_points(path, Settings.theta)
        save_path(path)
    except IndexError:
        print("ERROR: incorrect number of arguments given - 5 arguments required")
        print_argment_format()
    except ValueError:
        print("ERROR: invalid arguments given")
        print_argment_format()


def variables_return_empty_path():
    if Settings.sponge_diameter <= 0 or (abs(Settings.length) <= Settings.sponge_diameter
    and abs(Settings.height) <= Settings.sponge_diameter):
        return True
    else:
        return False


def rotate_points(point_list, theta):
    if theta == 0:
        return point_list
    else:
        theta = math.radians(theta)
        new_list = []
        for point in point_list:
            new_list.append(rotate(point, theta))
        return new_list



def rotate(point, theta):
    p1_x, p1_y = point
    p2_x = math.cos(theta) * p1_x - math.sin(theta) * p1_y
    p2_y = math.sin(theta) * p1_x - math.cos(theta) * p1_y
    return p2_x, p2_y


def generate_path():
    if variables_return_empty_path():
        return []

    x_length = Settings.length
    if x_length > 0:
        x_length = x_length - Settings.sponge_diameter
    else:
        x_length = x_length + Settings.sponge_diameter
    y_length = Settings.height
    if y_length > 0:
        y_length = y_length - Settings.sponge_diameter
    else:
        y_length = y_length + Settings.sponge_diameter

    x_current = 0.0
    y_current = 0.0
    path = [(0.0,0.0)]

    if Settings.transpose:
        vertical_lines = math.floor(abs(x_length / Settings.sponge_diameter)) + 1
        if vertical_lines <= 1:
            path.append((0, y_length))
        else:
            x_step = Settings.length / vertical_lines
            while True:
                if y_length != 0.0:
                    if (y_current == 0.0):
                        y_current = y_length
                    else:
                        y_current = 0.0
                    path.append((x_current, y_current))

                x_current = x_current + x_step
                if (abs(x_current) > abs(x_length)):
                    break
                path.append((x_current, y_current))
    else:
         horizontal_lines = math.floor(abs(y_length / Settings.sponge_diameter)) + 1
         if horizontal_lines <= 1:
             path.append((x_length, 0))
         else:
             y_step = Settings.height / horizontal_lines
             while True:
                 if x_length != 0.0:
                     if (x_current == 0.0):
                         x_current = x_length
                     else:
                         x_current = 0.0
                     path.append((x_current, y_current))

                 y_current = y_current + y_step
                 if (abs(y_current) > abs(y_length)):
                     break
                 path.append((x_current, y_current))

    return path


if __name__ == "__main__":
    if len(sys.argv) == 1:
        version_print("ERROR: program requires arguments to function")
        print_argment_format()
    elif sys.argv[1] == "-h" or sys.argv[1] == "--help":
        print_argment_format()
    else:
        run(sys.argv[1:])
