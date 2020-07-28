"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab GRAND PRIX
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
from enum import Enum, IntEnum

sys.path.insert(0, "/library")
import racecar_core
import racecar_utils as rc_utils

MAX_SPEED = 0.25

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here

########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

    rc.drive.set_max_speed(MAX_SPEED)

    # Print start message
    print(">> GRAND PRIX")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    pass


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
