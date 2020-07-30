import cv2 as cv
import numpy as np
import sys
from enum import Enum, IntEnum

sys.path.insert(0, "./library")
import racecar_utils as rc_utils

# The (min, max) degrees to consider when measuring forward and rear distances
FRONT_WINDOW = (-10, 10)
REAR_WINDOW = (170, 190)
CLOSEST = 20


class P_Wall:
    class State(IntEnum):
        standard = 0
        tooclose = 1
    
    def __init__(self):
        self.cur_state = self.State.standard

    def run_phase(self, rc, depth_image, color_image, lidar_scan):
        print(">> Running Wall Following")

            
        """
        After start() is run, this function is run every frame until the back button
        is pressed
        """
        # Use the triggers to control the car's speed left joystick for angle
        #rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
        #lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
        #speed = rt - lt
        #angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]
        # Calculate the distance 
        scan = lidar_scan

        rightDist =  rc_utils.get_lidar_average_distance(scan, 44, 10)
        leftDist = rc_utils.get_lidar_average_distance(scan, 316, 10)

        angle = rc_utils.remap_range(rightDist - leftDist, -rc.camera.get_width()//13, rc.camera.get_width()//13, -1, 1)
        angle = rc_utils.clamp(angle, -1, 1)


        rc.drive.set_speed_angle(1, angle)
