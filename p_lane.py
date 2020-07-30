import cv2 as cv
import numpy as np
import sys
import constants as c
from enum import IntEnum

sys.path.insert(0, "./library")
import racecar_utils as rc_utils      

class P_Lane:

    class State(IntEnum):
        HARD_STOP = 0
        SLOW = 1
        FAST = 2

    def __init__(self, color):
        self.phase = 1

        self.fast_col = color
        self.slow_col = c.Colors.ORANGE if self.fast_col == c.Colors.PURPLE else c.Colors.PURPLE

        self.cur_state = self.State.FAST

        # Used to save angle between loop calls
        self.angle = 0

        self.slow_state_angle = 0

        self.cropped_img = None

        self.stop_counter = 0

    def get_2_largest(self, img, col):
        # Gets all fast contours and sorts them by contour area
        contours = list(enumerate(rc_utils.find_contours(img, col.value[0], col.value[1])))
        areas = [rc_utils.get_contour_area(cont[1]) for cont in contours]
        contours = sorted(contours, key=lambda x: areas[x[0]], reverse=True)
        # Gets 2 largest contours
        largest = contours[:2]
        largest = [cont for cont in largest if areas[cont[0]] > c.LANE_MIN_CONTOUR_AREA]

        return largest

    def run_slow(self, rc):
        largest = self.get_2_largest(self.cropped_img, self.slow_col)

        if self.cur_state == self.State.SLOW:
            speed = c.SPEED_SLOW
        else:
            speed = -1


        # If the fast lane is visible more than the slow one, then just keep turning in the previously set direction
        largest_fast_cropped = rc_utils.get_largest_contour(rc_utils.find_contours(self.cropped_img, self.fast_col.value[0],
                                                                                                self.fast_col.value[1]))
        largest_slow_cropped = rc_utils.get_largest_contour(rc_utils.find_contours(self.cropped_img, self.slow_col.value[0],
                                                                                                self.slow_col.value[1]))
            
        slow_a = rc_utils.get_contour_area(largest_slow_cropped) if largest_slow_cropped is not None else 0
        fast_a = rc_utils.get_contour_area(largest_fast_cropped) if largest_fast_cropped is not None else 0

        if len(largest) == 2 and not slow_a < fast_a:
            self.angle = 0

        elif len(largest) == 1 and not slow_a < fast_a:
            cont = largest[0][1]

            # Finds the top point of the contour and the bottom (Estimates line slope)
            top_pt = tuple([pt[0] for pt in cont if pt[0][1] == np.amin(cont[:, :, 1])][0])
            bott_pt = tuple([pt[0] for pt in cont if pt[0][1] == np.amax(cont[:, :, 1])][0])
            
            # Slop is sloppy?????????????
            if self.slow_state_angle == 0:
                if top_pt[0] - bott_pt[0] > 0:
                    self.slow_state_angle = 1
                elif top_pt[0] - bott_pt[0] < 0:
                    self.slow_state_angle = -1
                else:
                    self.slow_state_angle = 0

            self.angle = self.slow_state_angle

            # Draws VIZHUALS
            #cv.line(self.cropped_img, top_pt, bott_pt, (255, 255, 0), thickness=2)

        #rc.display.show_color_image(self.cropped_img)

        # Sets speed and angle
        rc.drive.set_speed_angle(rc_utils.clamp(speed, -1, 1), rc_utils.clamp(self.angle, -1, 1))

        return len(largest)

    def run_fast(self, rc, dist_slow):
        # Get 2 largest fast contours
        largest = self.get_2_largest(self.cropped_img, self.fast_col)

        if len(largest) == 2:
            # Finds which contour is which (left side/ right side)
            avg_1 = np.average(largest[0][1][:, :, 0]) 
            avg_2 = np.average(largest[1][1][:, :, 0]) 

            avg_mid = (avg_2 + avg_1) / 2

            if avg_2 > avg_1:
                right_cont = largest[1][1]
                left_cont = largest[0][1]

            else:
                right_cont = largest[0][1]
                left_cont = largest[1][1]

            # Sets angle based on mid average offset from center
            self.angle = rc_utils.remap_range(avg_mid - (rc.camera.get_width() / 2), *c.ALIGN_REMAP)

            # <>>>>>>  VISZHUALS
            rc_utils.draw_contour(self.cropped_img, left_cont, color=(255, 0, 0))
            rc_utils.draw_contour(self.cropped_img, right_cont, color=(0, 255, 0))

            cv.line(self.cropped_img, (int(avg_mid), 0), (int(avg_mid), rc.camera.get_height()), (255, 255, 0))

        elif len(largest) == 1:
            # If the center of the contour is roughly in the center of the screen, tell car to turn
            offset = abs(rc_utils.get_contour_center(largest[0][1])[1] - (rc.camera.get_width() // 2)) 
            print("offset", offset)
            if offset < c.LANE_SPLIT_DETECT_MAX_OFFSET:
                self.angle = 1

        # Sets speed based on distance from sharp turn and based on angle 
        speed_approach_remap = rc_utils.remap_range(dist_slow, *c.SLOWDOWN_REMAP)
        speed_angle_remap = rc_utils.remap_range(abs(self.angle), *c.SPEED_ANGLE_REMAP)
        speed = min((speed_angle_remap, speed_approach_remap))

        # Checks ramp if y accel is below a threshold
        if rc.physics.get_linear_acceleration()[1] >= -9.59:
            speed = 1

        # Sets speed and angle
        rc.drive.set_speed_angle(rc_utils.clamp(speed, -1, 1), rc_utils.clamp(self.angle, -1, 1))

        #rc.display.show_color_image(self.cropped_img)


    def run_phase(self, rc, depth_image, color_image, lidar_scan):
        # print("FAST", self.fast_col, "SLOW", self.slow_col)
        self.cropped_img = np.copy(color_image)[rc.camera.get_height() * 2 // 3 : rc.camera.get_height(), :]

        # SLOW CONTOUR INFO GATHERING
        # Finds distance to largest slow contour >>
        largest_slow = rc_utils.get_largest_contour(rc_utils.find_contours(color_image, self.slow_col.value[0], self.slow_col.value[1]),
                                                    c.LANE_MIN_CONTOUR_AREA)
        if largest_slow is not None:
            center_slow = rc_utils.get_contour_center(largest_slow)
            dist_slow = rc_utils.get_pixel_average_distance(depth_image, center_slow)
        else:
            dist_slow = 9999
        # -------------------------------------- <<

        if self.cur_state == self.State.FAST:
            self.run_fast(rc, dist_slow)

            # If the the slow contour is within a certain range, switch states
            if dist_slow <= c.STATE_SWITCH_DIST:
                self.cur_state = self.State.HARD_STOP

        elif self.cur_state == self.State.SLOW or self.cur_state == self.State.HARD_STOP:
            if self.cur_state == self.State.HARD_STOP:
                self.stop_counter += 1
                if self.stop_counter >= 10:
                    self.stop_counter = 0
                    self.cur_state = self.State.SLOW

            # Runs function and gets output (# of slow contours visible)
            out = self.run_slow(rc)
            if out == 0:
                self.cur_state = self.State.FAST
                self.slow_state_angle = 0
        
        
        print(self.cur_state) 
            
        # If slow line area sum is big enough, align to right side of fast lane:
        
        # If no visible fast lane:
        # ------ Full turn /or/ Consider way to turn on purple line (sharp turn)

        # If only one line visible:
        # ------ Save history of left side and right side contours and determine what side the single contour is on

        """rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
        lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
        speed = rt - lt
        angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]
        rc.drive.set_speed_angle(rc_utils.clamp(speed, -1, 1), rc_utils.clamp(angle, -1, 1))"""

