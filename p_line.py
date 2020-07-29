import cv2 as cv
import numpy as np
import sys

sys.path.insert(0, "./library")
import racecar_utils as rc_utils

class P_Line:
    def __init__(self, color):
        self.color = color

    def run_phase(self, rc, depth_image, color_image, lidar_scan):
        print(">> Running Line Following", self.color.name)

        rc.drive.set_speed_angle(1, 0)
