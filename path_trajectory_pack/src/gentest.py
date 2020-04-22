#!/usr/bin/env python

import os
import PathGenerator as pg

length = 6
height = 4
sponge_diameter = 1
theta = 0
transpose = "true"
path = "file.xml"

"""
PathGenerator can be called either by:
    importing and calling "PathGenerator.run(options.split(" "))", or
    os.system("PathGenerator.py " + options)
Where:
    options = "length height sponge_diameter transpose save_location"
"""

options = length + " " + height + " " + sponge_diameter + " " + theta + " " + transpose + " " path

#os.system("PathGenerator.py " + options)
#pg.run(options.split(" "))
