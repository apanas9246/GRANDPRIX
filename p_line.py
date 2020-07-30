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
            contour_image = rc_utils.crop(color_image, c.LINE_CROP_FLOOR[0], c.LINE_CROP_FLOOR[1])

            contours = rc_utils.find_contours(contour_image, self.color.value[0], self.color.value[1])
            
            L_contour = rc_utils.get_largest_contour(contours, c.LINE_MIN_CONTOUR_AREA)

            if L_contour is not None:

                self.contour_center = rc_utils.get_contour_center(L_contour)
                contour_area = rc_utils.get_contour_area(L_contour)
 
                # Draw contour onto the image
                rc_utils.draw_contour(contour_image, L_contour, (0, 255, 255))
                rc_utils.draw_circle(contour_image, self.contour_center, (0, 255, 255))

            rc.display.show_color_image(color_image)
            
    def run_phase(self, rc, depth_image, color_image, lidar_scan):
        print(">> Running Line Following", self.color.name)
        global prev_angle
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

            angle1 = rc_utils.remap_range(self.contour_center[1], 0, rc.camera.get_width(), -1, 1)
            goal_amount = rc_utils.remap_range(abs(angle1), 0, 1, 0, 80) 
            prev_angle = angle1
            if angle1 > 0 :
                angle1 = rc_utils.remap_range(self.contour_center[1]-goal_amount, 0, rc.camera.get_width(), -1, 1)
                angle = rc_utils.clamp(angle1, -1, 1)
                prev_angle = angle
            elif angle1<0:
                angle1 = rc_utils.remap_range(self.contour_center[1]+goal_amount, 0, rc.camera.get_width(), -1, 1)
                angle = rc_utils.clamp(angle1, -1, 1)
                prev_angle = angle
            else:
                if angle1 == 0:
                    angle = 0
                if angle1 is None:
                    angle = prev_angle

            speed = rc_utils.remap_range(abs(angle1), 0, 1, 1, 0.3)  #desmos speed angle functions angle plug in for x
    
        else: #change this to a finding function or reverse function
            speed = 1
            angle =0


        rc.drive.set_speed_angle(speed, angle)

