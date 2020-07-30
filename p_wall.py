import cv2 as cv
import numpy as np
import sys
from enum import Enum, IntEnum
import constants as c

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
    
    def __init__(self, prev_id):
        self.cur_state = self.State.standard

        self.is_canyon = prev_id == 1

    def run_phase(self, rc, depth_image, color_image, lidar_scan):
        


        print(">> Running Wall Following")
        if self.is_canyon:
            print("THIS IS ALSO A CANYON")
            
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

        max_wall = 0.65 #testing max speed
        hall = rc.camera.get_width()//9
        optimum = 60 

        rightDist =  rc_utils.get_lidar_average_distance(scan, 44, 10)
        leftDist = rc_utils.get_lidar_average_distance(scan, 316, 10)


        angle = rc_utils.remap_range(rightDist - leftDist, -hall, hall, -1, 1)
        angle = rc_utils.clamp(angle, -1, 1)

  
    

        # get them tags
        corners, ids = rc_utils.get_ar_markers(color_image)

        if c.ar_in_range_ID(c.CONTOUR_DETECT_RANGE, depth_image, color_image, rc) == c.ID_DIRECTION:
            dirrrrrrrrr = rc_utils.get_ar_directions(corners[0])
            if dirrrrrrrrr == rc_utils.Direction.LEFT:
                angle = -1
            elif dirrrrrrrrr == rc_utils.Direction.RIGHT:
                angle = 1
        else:
            tooClose = 80
            if rc_utils.get_depth_image_center_distance(depth_image) < tooClose:
                angle = 1


        speed = rc_utils.remap_range(abs(angle), 15, 1, 1, 0.5)#temp controls
        rc.drive.set_speed_angle(max_wall*speed, angle)
