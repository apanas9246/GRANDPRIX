import cv2 as cv
import numpy as np
import sys
import time
from enum import Enum, IntEnum

import constants as c

sys.path.insert(0, "./library")
import racecar_utils as rc_utils

class P_Slalom:

    class State(IntEnum):
        # Align with cone and move towards it while it is visible
        APPROACH = 0
        # Once cone is no longer visible (i.e no visible contours or next contour visible is not same color)
        # Car uses previous estimates of speed to pass the cone 
        PASS = 1
        # Depending on color, turn car until contour is found. If found, go to state 0
        TURN = 2

    class Color(IntEnum):
        # Pass on the left
        BLUE = 0
        # Pass on the right
        RED = 1

    class Const(Enum):
        # Maximum distance change between frames to count as valid
        MAX_DIST_DIFF = 30

        # When passing visible cone, used as lower and higher boundaries for a proportion used to determine 
        # how far off-center of the screen to set the setpoint to not crash into the cone as the car gets close
        LOW_SET_PROPORTION = (90 / 320)
        HIGH_SET_PROPORTION = (320 / 320)
        # The distance boundaries used for the above proportion
        APPROACH_DIST_UPPER = 200
        APPROACH_DIST_LOWER = 27

        # How many cm to drive to pass a cone that went out of view
        PASS_DIST = 15
        PASS_TIME = 0.25

        # Consecutive no contours to start turn
        MAX_CONSEC_CONTOURS = 1

        

    def __init__(self):
        self.COLORS = {"BLUE": ((90, 120, 120), (120, 255, 255)), 
                       "RED": ((130, 50, 50), (179, 255, 255))}
        self.__cur_col = self.Color.RED
        self.__cur_state = self.State.APPROACH

        self.__depth_image = None
        self.__color_image = None
        self.__closest_lidar_sample = None

        # Variables for contour tracking
        self.__contour_distance = None
        self.__contour_center = None
        self.__contour = None

        # Estimated Speed of car. Only calculated in approach state
        self.__speed = None
        # Previous distance to cone
        self.__prev_distance = None
        # Used to calculate speed
        self.__prev_time = None

        # Consecutive frames where no contour was seen
        self.__consec_no_cont = 0

        self.__approach_angle = 0
        self.__approach_speed = 0
        self.__approach_setpoint = 0



    def run_phase(self, rc, depth_image, color_image, lidar_scan):
        print(">> Running Cone Slaloming")
        self.update_images(rc)
        self.update_contours(rc)
        self.auto_control(rc)


#run a function based on state
    def auto_control(self, rc):
        if self.__cur_state == self.State.APPROACH:
            print("Approach")
            self.run_approach_state(rc)
        elif self.__cur_state == self.State.PASS:
            print("Pass")
            self.run_pass_state()
        elif self.__cur_state == self.State.TURN:
            print("Turn")
            self.run_turn_state(rc)
        else:
            rc.drive.set_speed_angle(0, 0)

    #get color, depth, lidar
    def update_images(self, rc):
        self.__color_image = rc.camera.get_color_image()
        self.__depth_image = cv.GaussianBlur((rc.camera.get_depth_image() - 0.01) % 10000, (c.BLUR_KERNEL_SIZE, c.BLUR_KERNEL_SIZE), 0) 
        self.__closest_lidar_sample = rc_utils.get_lidar_closest_point((rc.lidar.get_samples() - 0.1) * 1000, (0, 360))


    def update_contours(self, rc):
        contours = rc_utils.find_contours(self.__color_image, self.COLORS[self.__cur_col.name][0], self.COLORS[self.__cur_col.name][1])
        #get contours for color, find closest one
        out = P_Slalom.get_closest_contour(contours, self.__depth_image)
        if out is not None:
            # Saves previous measured distance
            #if self.__contour_distance is not None:
                #self.__prev_distance = self.__contour_distance
            self.__prev_distance = self.__contour_distance

            
            self.__contour_distance, self.__contour, self.__contour_center = out
        
            # Draws closest contour
            rc_utils.draw_contour(self.__color_image, self.__contour)
            rc_utils.draw_circle(self.__color_image, self.__contour_center)
        else:
            self.__contour_distance = None
            self.__contour_center = None
            self.__contour = None
        
        # Displays image
        rc.display.show_color_image(self.__color_image)


    # Gets speed based on the cone being approached
    def update_speed(self):
        if self.__contour_distance is not None and self.__prev_distance is not None and self.__prev_time is not None:
            # Calculates Speed
            self.__speed = round(-(self.__contour_distance - self.__prev_distance) / (time.time() - self.__prev_time), 2)
        else:
            self.__speed = None

        # Updates previous time
        self.__prev_time = time.time()

    def get_closest_opposite_contour(self):
        # Finds closest contour of opposite color
        opp_col = self.Color.RED if self.__cur_col == self.Color.BLUE else self.Color.BLUE
        opp_conts = rc_utils.find_contours(self.__color_image, self.COLORS[opp_col.name][0], self.COLORS[opp_col.name][1])
        out = P_Slalom.get_closest_contour(opp_conts, self.__depth_image)
        
        # Gets the distance to the closest opposite color cone
        if out is not None:
            dist_opp = out[0]
        else:
            # A really high number that will always be greater than the closest desired cone
            dist_opp = 99999
        return dist_opp

    #contour none and prev not none >>when passing a cone
    #contour none if out is none
    def run_approach_state(self, rc):
        # Calculates distance change to see if cone was passed
        if self.__contour_distance is not None:
            if self.__prev_distance is None:
                self.__prev_distance = self.__contour_distance
            distance_change = self.__contour_distance - self.__prev_distance
            self.__consec_no_cont = 0
        else:
            distance_change = None
            self.__consec_no_cont += 1

        dist_opp = self.get_closest_opposite_contour()

        # If distance makes a huge jump, assume that it is now following a different cone so switch states
        if self.__consec_no_cont >= self.Const.MAX_CONSEC_CONTOURS.value or (distance_change > self.Const.MAX_DIST_DIFF.value and dist_opp < self.__contour_distance):
            self.__cur_col = self.Color.BLUE if self.__cur_col == self.Color.RED else self.Color.RED
            self.__cur_state = self.State.PASS

        else:
            # >>>> Proportional control
            # -------------------------

            # Updates speed variable
            self.update_speed()

            # Scales the center offset boundaries based on screen width
            w = rc.camera.get_width()

            low_set = self.Const.LOW_SET_PROPORTION.value * (w / 2)
            high_set = self.Const.HIGH_SET_PROPORTION.value * (w / 2)

            # Calculates new setpoint based on how close the car is to the cone.
            if self.__contour_distance is not None:
                self.__approach_setpoint = rc_utils.remap_range(self.__contour_distance, self.Const.APPROACH_DIST_UPPER.value, 
                                                self.Const.APPROACH_DIST_LOWER.value, low_set, high_set) 
                                            
            # negates setpoint if color is red. This ensures that the car passes cone on correct side
            self.__approach_setpoint *= (-1 if self.__cur_col == self.Color.RED else 1)

            """DEBUG: Draw Setpoint"""
            #rc_utils.draw_circle(self.__color_image, (rc.camera.get_height() // 2, int((w / 2) + setpoint)))

            # Gets angle from setpoint using proportional control
            kP = 3
            if self.__contour_distance is not None:
                self.__approach_angle = rc_utils.clamp(rc_utils.remap_range(self.__contour_center[1], self.__approach_setpoint, 
                                                       rc.camera.get_width() + self.__approach_setpoint, -1, 1) * kP, -1, 1)
            
            self.__approach_speed = rc_utils.remap_range(abs(self.__approach_angle), 1, 0, 0.1, 1)
            #self.__approach_speed = 1

            # Sets speed angle
            rc.drive.set_speed_angle(self.__approach_speed, self.__approach_angle)

            """DEBUG: Show image"""
            #rc.display.show_color_image(self.__color_image)
            
    def run_pass_state(self):
        if 60 < self.__closest_lidar_sample[0] < 300:
            self.__is_stamped = False
            self.__cur_state = self.State.TURN

    def run_turn_state(self, rc):
        # >>> State Switch Detection
        # --------------------

        dist_opp = self.get_closest_opposite_contour()

        # Must have a cone of the needed color visible
        # and checks if the needed cone is closer than the closest opposite color cone
        if self.__contour_distance is not None and self.__contour_distance < dist_opp:
            self.__cur_state = self.State.APPROACH

        # >>> Turn to find cone
        # ---------------------
        else:
            speed = 1
        
            # If blue cone needs to be found next, turn left. Otherwise right
            angle = -1 if self.__cur_col == self.Color.BLUE else 1

            # Turns until cone is found
            rc.drive.set_speed_angle(speed, angle)


    @staticmethod
    def get_closest_contour(contours, depth_img):
        MIN_CONTOUR_AREA = 70
        MAX_CONTOUR_DIST = 300
        # Gets centers of each contour and extracts countours based on conditions
        centers = []
        for idx, contour in enumerate(contours):
            cent = rc_utils.get_contour_center(contour)
            if cent is not None:
                dist = rc_utils.get_pixel_average_distance(depth_img, cent)
                area = rc_utils.get_contour_area(contour)
                if area > MIN_CONTOUR_AREA and dist < MAX_CONTOUR_DIST:
                    centers.append((idx, rc_utils.get_contour_center(contour)))

        indexes = [center[0] for center in centers]
        centers = [center[1] for center in centers]
        # Calculates the distance to each center
        distances = [rc_utils.get_pixel_average_distance(depth_img, (center[0], center[1])) for center in centers]

        conts = [contours[index] for index in indexes]

        # Finds smallest distance and index of that distance
        if len(conts):
            minimum = min(enumerate(distances), key=lambda x: x[1])
            # (index, min dist)

            # Returns distance to closest contour center, the contour itself, and the center position
            return (minimum[1], conts[minimum[0]], centers[minimum[0]])
        else:
            # If there is no contour, my love for humanities is returned
            return None