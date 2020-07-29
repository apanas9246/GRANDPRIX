import cv2 as cv
import numpy as np
import sys
​
sys.path.insert(0, "./library")
import racecar_core
import racecar_utils as rc_utils
rc = racecar_core.create_racecar()
'''
speed=0
angle =0
contour_area =0
COLOR = None
contour_center =None
'''
LEFT= ((360, 0), (rc.camera.get_height(), rc.camera.get_width()//2))
RIGHT= ((360, rc.camera.get_width()//2), (rc.camera.get_height(), rc.camera.get_width()))
PURPLE = ((117, 50, 50),(155, 255, 255))
ORANGE = ((10, 100, 100), (20, 255, 255))
PRI = (PURPLE, ORANGE)
contour_center = None
contour_area=0
​
class P_Lane:
    def __init__(self, color):
        # An enum constant 
        self.color = color
​
    def run_phase(self, rc, depth_image, color_image, lidar_scan):
      global contour_area
      global contour_center
      
      #print(">> Running Lane Following", self.color.name)
​
      left = rc_utils.crop(color_image, LEFT[0], LEFT[1])
      right = rc_utils.crop(color_image, RIGHT[0], RIGHT[1])
      
      rc.display.show_color_image(left)
      self.update_contour(rc, color_image, 30)
      #print("before if")
      if contour_area!=0:
        self.update_contour(rc, left, 30)
        contour_area_left = contour_area
        self.update_contour(rc, right, 30)
        contour_area_right = contour_area
        print("left     ", contour_area_left, "     right     ", contour_area_right)
​
        angle = rc_utils.remap_range(contour_area_left - contour_area_right, -10000, 10000, -1, 1)
        angle = rc_utils.clamp(angle, -1, 1)
        if contour_area_right==0:
          angle = 0.6
        if contour_area_left ==0 and contour_area_right >6000:
          angle = 0.2
        #speed =0.2
        #speed = rc_utils.remap_range(abs(angle), 0, 1, 1, 0.1)
        #rc.drive.set_speed_angle(speed, angle)
​
        rc.drive.set_speed_angle(0.2, angle)
​
    def update_contour(self, rc, i, min):
      global contour_center
      global contour_area
      global COLOR
      global angle
​
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
            rc_utils.draw_contour(i, contour)
            break
          else:
            contour_center= None
            contour_area = 0
      
