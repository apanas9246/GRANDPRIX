import cv2 as cv
import numpy as np
import sys

import constants as c

sys.path.insert(0, "./library")
import racecar_utils as rc_utils

class P_Line:
    def __init__(self, color):
        self.color = color
        self.contour_center = None

    def updateContour(self, rc, depth_image, color_image):

        if color_image is None:
            self.contour_center = None
        else:
            # Crop the image to the floor directly in front of the car
            contour_image = rc_utils.crop(color_image, c.CROP_FLOOR[0], c.CROP_FLOOR[1])

            contours = rc_utils.find_contours(contour_image, self.color.value[0], self.color.value[1])
            
            L_contour = rc_utils.get_largest_contour(contours, c.MIN_CONTOUR_AREA)

            if L_contour is not None:

                self.contour_center = rc_utils.get_contour_center(L_contour)
                contour_area = rc_utils.get_contour_area(L_contour)
 
                # Draw contour onto the image
                #rc_utils.draw_contour(contour_image, L_contour, (0, 255, 255))
                #rc_utils.draw_circle(contour_image, self.contour_center, (0, 255, 255))

            #rc.display.show_color_image(color_image)
            
    def run_phase(self, rc, depth_image, color_image, lidar_scan):
        print(">> Running Line Following", self.color.name)

        self.updateContour(rc, depth_image, color_image)

        """ 
        if contourcenter is not none, finds angle based on contourcenter location, 
        then adjusts the goal amount to turn based on if the angle is positive or negative. 
        
        Goal amount will increase the sharpness of the turn based on how sharp it already 
        is to ensure sharp turns are followed decently.
        """
        # Notes: I think that I've tweaked the car to work consistently. Still need to
        # make a reverse function and a finding function. Also a speed function.
        if self.contour_center:

            angle = rc_utils.remap_range(self.contour_center[1], 0, rc.camera.get_width(), -1, 1)
            goal_amount = rc_utils.remap_range(abs(angle), 0, 1, 0, 500) 

            if angle > 0 :
                angle = rc_utils.remap_range(self.contour_center[1]+goal_amount, 0, rc.camera.get_width(), -1, 1)
                angle = rc_utils.clamp(angle, -1, 1)
            elif angle<0:
                angle = rc_utils.remap_range(self.contour_center[1]-goal_amount, 0, rc.camera.get_width(), -1, 1)
                angle = rc_utils.clamp(angle, -1, 1)
                
            speed = rc_utils.remap_range(abs(angle), 0, 1, 1, 0.1)  #desmos speed angle functions angle plug in for x

        else: #change this to a finding function or reverse function

            speed = 1
            angle = 0

        rc.drive.set_speed_angle(speed, angle)

