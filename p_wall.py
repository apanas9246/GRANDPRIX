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
        self.ar_tag = False
        self.is_canyon = prev_id == 1
        self.ledge_count = 0
        self.ledge_angle = 0
        self.many = 0

    def run_phase(self, rc, depth_image, color_image, lidar_scan):
        


        #print(">> Running Wall Following")
            
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
        billy = 150
        if c.ar_in_range_ID(billy, depth_image, color_image, rc, 4/5) == c.ID_DIRECTION:
            dirrrrrrrrr = rc_utils.get_ar_direction(corners[0])
            #print(dirrrrrrrrr)
            if dirrrrrrrrr == rc_utils.Direction.LEFT:
                angle = -1
            elif dirrrrrrrrr == rc_utils.Direction.RIGHT:
                angle = 1
            self.ar_tag = True
        elif self.is_canyon:
            tooClose = 80
            if rc_utils.get_depth_image_center_distance(depth_image) < tooClose:
                angle = 1

        right_farthest = np.amax(depth_image[rc.camera.get_height() * 5 // 6, rc.camera.get_width() // 2 : rc.camera.get_width()].flatten())
        left_farthest = np.amax(depth_image[rc.camera.get_height() * 5 // 6, 0 : rc.camera.get_width() // 2].flatten())
        diff = abs(right_farthest- left_farthest)
        AAAAAAAAAH_WE_ARE_ABOUT_TO_FALL____BETTER_STOP_NOW = 100

        if self.ar_tag and self.ledge_count==0 and diff > 50 :
            if right_farthest > AAAAAAAAAH_WE_ARE_ABOUT_TO_FALL____BETTER_STOP_NOW:
                self.many+=1
                self.ledge_angle = -1
                self.ledge_count = 10
            
            elif left_farthest > AAAAAAAAAH_WE_ARE_ABOUT_TO_FALL____BETTER_STOP_NOW:
                self.many+=1
                self.ledge_angle = 1
                self.ledge_count = 10

        #print("left    ", left_farthest, "   right   ", right_farthest)
        speed = rc_utils.remap_range(abs(angle), 15, 1, 1, 0.5)#temp controls
        if self.many==3:
            self.ar_tag = False
        if self.ledge_count >0:
            angle = self.ledge_angle
            self.ledge_count-=1

        rc.drive.set_speed_angle(max_wall*speed, angle)
