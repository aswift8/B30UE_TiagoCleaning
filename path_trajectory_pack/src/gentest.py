#!/usr/bin/env python

import os
import PathGenerator as pg

"""
PathGenerator can be called either by:
    importing and calling "PathGenerator.run(options.split(" "))", or
    os.system("PathGenerator.py " + options)
Where:
    options = "length height sponge_diameter transpose save_location"
"""

options = "6 4 1 true E:\\git\\B30UE_TiagoCleaning\\path_trajectory_pack\\data\\file.xml"

#os.system("PathGenerator.py " + options)
#pg.run(options.split(" "))
