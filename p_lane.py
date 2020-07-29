import cv2 as cv
import numpy as np
import sys

sys.path.insert(0, "./library")
import racecar_utils as rc_utils

class P_Lane:
    def __init__(self, color):
        # An enum constant 
        self.color = color

    def run_phase(self, rc, depth_image, color_image, lidar_scan):
        print(">> Running Lane Following", self.color.name)

        rc.drive.set_speed_angle(1, 0)


"""speed=0
angle =0
COLOR = None
LEFT_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()//2))
RIGHT_FLOOR = ((360, rc.camera.get_width()//2), (rc.camera.get_height(), rc.camera.get_width()))


def update_contour(i, min):
  global contour_center
  global contour_area
  global COLOR
  global angle

  if i is None:
    contour_center = None
    contour_area = 0
  else:
    for color in PRI:
      contours = rc_utils.find_contours(i, color[0], color[1])
      contour = rc_utils.get_largest_contour(contours, min)
      if contour is not None:
        COLOR = color
        contour_center =rc_utils.get_contour_center(contour)
        contour_area = rc_utils.get_contour_area(contour)
        contour_area = rc_utils.get_contour_area(contour)"""