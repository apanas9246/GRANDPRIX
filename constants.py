import sys
import cv2 as cv
import numpy as np
sys.path.insert(0, "./library")
import racecar_utils as rc_utils
from enum import Enum

class Colors(Enum):
    # Color Thresholds
    PURPLE = ((117, 50, 50),(155, 255, 255))
    ORANGE = ((15, 50, 50), (30, 255, 255))
    GREEN = ((35, 100, 100),(75, 255, 255))
    RED = ((165, 50, 50), (14, 255, 255))
    BLUE = ((90, 200, 200), (120, 255, 255))

# Gaussian Blue for Depth Image
BLUR_KERNEL_SIZE = 5

# Car Max Speed
MAX_SPEED = 0.5

# Colors that exist in lane/line following
LANE_COLORS = (Colors.PURPLE, Colors.ORANGE)
LINE_COLORS = (Colors.RED, Colors.GREEN, Colors.BLUE)

# Distance to detect AR tag to switch state
CONTOUR_DETECT_RANGE = 200

# Tag IDS
ID_LINE = 0
ID_LANE = 1
ID_SLALOM = 2
ID_WALL = 3
ID_DIRECTION = 199

# Crop floor size for contour line
LINE_CROP_FLOOR = ((480*3//5, 0), (480, 640)) # 480 = camera height, 640 = camera width

#Minimum contour area for largest contour function for line following
LINE_MIN_CONTOUR_AREA = 1000

""">>>>> Lane Follow"""
# When line following, how far away from the lane AR tag should priority be changed to prevent a crash
PRIORITY_SWITCH_DIST = 415

# When to do a sharp turn at the intersection
TURN_DECISION_DIST = 157

# When in fast state, dist from the slow contour needed to switch to slow state
STATE_SWITCH_DIST = 50

LANE_MIN_CONTOUR_AREA = 100

ALIGN_REMAP = (-200, 200, -1, 1)
SLOWDOWN_REMAP = (250, 30, 0.5, 0.1)
SPEED_ANGLE_REMAP = (1, 0, 0.1, 1)

DIR_NONE = 0
DIR_RIGHT = 1
DIR_LEFT = -1

TAG_ID_COLOR = 1
TAG_ID_TURN = 199

SPEED_SLOW = 0.5

# Detects if there is a fork in the road so the car is told to turn
LANE_SPLIT_DETECT_MAX_OFFSET = 50


def ar_in_range_ID(RANGE, d_img, c_img, rc, bottom= 0.5):
    #gets depth and ar tags, if there are some, finds center and then compares depth with 
    # RANGE. If within range, prints ids
    depth_image = d_img
    ar_image = c_img

    ar_image = rc_utils.crop(ar_image, (0,0), (int(rc.camera.get_height()*bottom), rc.camera.get_width()))
    checking_info, checking_info_id = rc_utils.get_ar_markers(ar_image)

    if checking_info:
        x =  (int)( (checking_info[0][0][0][1] + checking_info[0][0][1][1]) //2)
        y =  (int)( (checking_info[0][0][0][0] + checking_info[0][0][1][0]) //2)
        dis = rc_utils.get_pixel_average_distance(depth_image, (x, y))
        #print("this is dis     ",   dis)
        if dis<RANGE:
            
            return (checking_info_id)
    return None