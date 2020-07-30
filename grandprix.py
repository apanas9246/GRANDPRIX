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

import p_wall, p_slalom, p_lane, p_line
import constants as c

sys.path.insert(0, "./library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here

########################################################################################
# Functions
########################################################################################

class Main:

    def __init__(self, manual_mode=False):
        # Phase by value
        self.cur_id = c.ID_WALL

        self.cur_phase = p_wall.P_Wall(self.cur_id)

        self.__depth_image = None
        self.__color_image = None
        self.__lidar_scan = None

        self.__manual_mode = manual_mode

    def update_images(self):
        self.__depth_image = cv.GaussianBlur((rc.camera.get_depth_image() - 0.01) % 10000, (c.BLUR_KERNEL_SIZE, c.BLUR_KERNEL_SIZE), 0) 
        self.__color_image = rc.camera.get_color_image()
        self.__lidar_scan = rc.lidar.get_samples()

    def run_manual(self):
        # Gets speed using triggers
        rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
        lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
        speed = rt - lt

        # Gets angle from x axis of left joystick
        angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]

        # Sets speed and angle
        rc.drive.set_speed_angle(rc_utils.clamp(speed, -1, 1), rc_utils.clamp(angle, -1, 1))

    def run_main(self):
        # Gets images and lidar scan
        self.update_images()
        
        # Checks if manual mode was enabled
        if self.__manual_mode:
            self.run_manual()
        else:
            # Calls current state
            self.cur_phase.run_phase(rc, self.__depth_image, self.__color_image, self.__lidar_scan)

        # >>>> State changes
        # Finds ID of the AR tag within range
        detected_ID = c.ar_in_range_ID(c.CONTOUR_DETECT_RANGE, self.__depth_image, self.__color_image, rc)

        if detected_ID is not None and not detected_ID == self.cur_id:
            # If set to false, state will not be switched (incase color was not detected properly)
            successful = True

            if detected_ID == c.ID_LANE:
                color = ar_in_range_color(c.CONTOUR_DETECT_RANGE, self.__depth_image, self.__color_image, c.LANE_COLORS)
            elif detected_ID == c.ID_LINE:
                color = ar_in_range_color(c.CONTOUR_DETECT_RANGE, self.__depth_image, self.__color_image, c.LINE_COLORS)
            else:
                color = "Grand Pricks"
            if color is None:
                successful = False

            if successful:
                # Creates class instance based on the detectedID
                if detected_ID == c.ID_LANE:
                    self.cur_phase = p_lane.P_Lane(color)

                elif detected_ID == c.ID_LINE:
                    self.cur_phase = p_line.P_Line(color)

                elif detected_ID == c.ID_SLALOM:
                    self.cur_phase = p_slalom.P_Slalom()

                elif detected_ID == c.ID_WALL:
                    self.cur_phase = p_wall.P_Wall(self.cur_id)

                # Sets the current ID to the detected ID
                self.cur_id = detected_ID
                rc.drive.stop()

                print("STATE SWITCHED")


MAIN = None

def ar_in_range_color(RANGE, d_img, c_img, colors):
    #gets depth and ar tags, if there are some, finds center and then compares depth with 
    # RANGE. If within range, prints ids
    depth_image = d_img
    ar_image = c_img

    ar_image = rc_utils.crop(ar_image, (0,0), (rc.camera.get_height()//2, rc.camera.get_width()))
    checking_info, _ = rc_utils.get_ar_markers(ar_image)

    if checking_info:
        x =  (int)( (checking_info[0][0][0][1] + checking_info[0][0][1][1]) //2)
        y =  (int)( (checking_info[0][0][0][0] + checking_info[0][0][1][0]) //2)

        if rc_utils.get_pixel_average_distance(depth_image, (x, y))<RANGE:
            contours = [rc_utils.find_contours(ar_image, color.value[0], color.value[1]) for color in colors]
            largest_contours = [(idx, rc_utils.get_largest_contour(cont, 2000)) for idx, cont in enumerate(contours)]

            if len(largest_contours):
                if len([cont for idx, cont in largest_contours if cont is not None]):
                    return colors[max(largest_contours, key=lambda x: get_cont_area_proofed(x[1]))[0]]
                else:
                    return None
            
def get_cont_area_proofed(contour):
    if contour is None or not len(contour):
        return 0
    else:
        return rc_utils.get_contour_area(contour)


def start():
    global MAIN
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

    rc.drive.set_max_speed(c.MAX_SPEED)

    MAIN = Main()

    # Print start message
    print(">> GRAND PRIX")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    
    #ar_in_range()

    MAIN.run_main()


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
