"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Final Challenge - Time Trials
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
from enum import IntEnum

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()
 
# >> Constants
# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_AREA = 1000
 
# A crop window for the floor directly in front of the car
CROP_FLOOR = ((rc.camera.get_height()*4//5, 0), (rc.camera.get_height(), rc.camera.get_width()))
 
# Colors, stored as a pair (hsv_min, hsv_max)
GREEN = ((35, 200, 200),(75, 255, 255))
RED = ((170, 180, 180), (10, 255, 255))
ORANGE = ((10, 100, 100), (20, 255, 255))
BLUE = ((90, 200, 200), (120, 255, 255))  
#PURPLE = ((117, 100, 100), (135, 255, 255))
PURPLE = ((117, 50, 50), (155, 255, 255))

AR_PRIORITY = (PURPLE, ORANGE)
lane = False
arcount = 0
time = 0

global FIRST_PRI1
global SECOND_PRI1

FIRST_PRI = None
SECOND_PRI = None

global red_dir
global blue_dir 
global green_dir 

red_dir = 0
blue_dir = 0
green_dir = 0

# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour

global cur_state
#State Machines
class State(IntEnum):
    lineFollow = 0 #state where it follows the priority lines, and if else statement for turn choices or ar prompted turns
    laneFollow =1
    
    redConeSlalom = 2 #finds and turns for red cones, checks if multiple red
    blueConeSlalom = 3 #finds and turns for blue cones, checks if multiple blue 
    noConeSlalom = 4 #end slalom if no more cones 
 
    wallFollow =1 #follows walls, with if else statement for turn choices or ar prompted turns
 
 
class Direction(IntEnum):
    UP=1 
    RIGHT = 2
    DOWN = 3
    LEFT =4
########################################################################################
# Functions
########################################################################################
def color_dir(colorc, corners, dir): #checks if color contour center is left or right, then returns the color's ar tag's direction
    global FIRST_PRI1
    global SECOND_PRI1
    if colorc[1] < rc.camera.get_width()//2:
        if (corners[0][0][0][1]+corners[0][0][1][1])//2 > rc.camera.get_width()//2: 
            dir = rc_utils.get_ar_direction( corners[0])
            FIRST_PRI1 = dir
        else:
            dir = rc_utils.get_ar_direction( corners[1])
            FIRST_PRI1 = dir
    elif colorc[1] > rc.camera.get_width()//2:
        if (corners[0][0][0][1]+corners[0][0][1][1])//2 < rc.camera.get_width()//2: 
            dir = rc_utils.get_ar_direction( corners[0])
            SECOND_PRI1 = dir
        else:
            dir = rc_utils.get_ar_direction( corners[1]) 
            SECOND_PRI1 = dir
    else:
        dir = 0
    return dir

def update_ar(info, info_id, ar_image):
    global FIRST_PRI1
    global SECOND_PRI1
    global red_dir
    global blue_dir 
    global green_dir 
    red_dir = 0
    blue_dir = 0
    green_dir = 0


    #ar_image= rc_utils.crop(image, (0, rc.camera.get_width()//2), (rc.camera.get_height(), rc.camera.get_width()))

    
    if info is not None: #checks if there are AR markers, then gets the contour of the outlining color
        rc_utils.draw_ar_markers(ar_image, info, info_id, (0, 255, 0))
        contours_ar_red = rc_utils.find_contours(ar_image, RED[0], RED[1])
        contours_ar_blue = rc_utils.find_contours(ar_image, BLUE[0], BLUE[1])
        contours_ar_green = rc_utils.find_contours(ar_image, GREEN[0], GREEN[1])

        red_largest = rc_utils.get_largest_contour(contours_ar_red, 2000)
        blue_largest = rc_utils.get_largest_contour(contours_ar_blue, 2000)
        green_largest = rc_utils.get_largest_contour(contours_ar_green, 2000)
        
        if red_largest is not None:   
            redc = rc_utils.get_contour_center(red_largest)
            red_dir = color_dir(redc, info, red_dir)
            rc_utils.draw_contour(ar_image, red_largest) 
        if blue_largest is not None:
            bluec = rc_utils.get_contour_center(blue_largest)
            blue_dir = color_dir(bluec, info, blue_dir)
            rc_utils.draw_contour(ar_image, blue_largest)
        if green_largest is not None:
            greenc = rc_utils.get_contour_center(green_largest)
            green_dir = color_dir(greenc, info, green_dir)
            rc_utils.draw_contour(ar_image, green_largest)
        
       # if red_dir != 0 or green_dir != 0 or blue_dir != 0:
        if red_dir == 0:
            THIRD_PRI = red_dir
        elif blue_dir == 0:
            THIRD_PRI = blue_dir
        else:
            THIRD_PRI = green_dir
            
        #rc.display.show_color_image(ar_image)

def update_contour():
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area
    global cur_state
    global FIRST_PRI1
    global SECOND_PRI1
    #global THIRD_PRI
    global red_dir
    global blue_dir 
    global green_dir 

    contour_image = rc.camera.get_color_image()
    
    if contour_image is None:
        contour_center = None
        contour_area = 0
     
    else:
        # TODO (challenge 1): Search for multiple tape colors with a priority order
        # (currently we only search for blue)
 
        # Crop the image to the floor directly in front of the car
        contour_image = rc_utils.crop(contour_image, CROP_FLOOR[0], CROP_FLOOR[1])
 
        #Find all of the red contours
        contours_red = rc_utils.find_contours(contour_image, RED[0], RED[1])
        
        # Find all of the blue contours
        contours_blue = rc_utils.find_contours(contour_image, BLUE[0], BLUE[1])
 
        #Find all of the green contours
        contours_green = rc_utils.find_contours(contour_image, GREEN[0], GREEN[1])
        
 
        # Select the largest contour
        L_contour_blue = rc_utils.get_largest_contour(contours_blue, MIN_CONTOUR_AREA)
        L_contour_red = rc_utils.get_largest_contour(contours_red, MIN_CONTOUR_AREA)
        L_contour_green = rc_utils.get_largest_contour(contours_green, MIN_CONTOUR_AREA)

 
        # Priorities#####################################################################
        if FIRST_PRI1: 
            if FIRST_PRI1 == red_dir:
                FIRST_PRI = L_contour_red
                if SECOND_PRI1 == blue_dir:
                    SECOND_PRI = L_contour_blue
                    THIRD_PRI = L_contour_green
                else:
                    SECOND_PRI = L_contour_green
                    THIRD_PRI = L_contour_blue
            elif FIRST_PRI1 == blue_dir:
                FIRST_PRI = L_contour_blue
                if SECOND_PRI1 == green_dir:
                    SECOND_PRI = L_contour_green
                    THIRD_PRI = L_contour_red
                else:
                    SECOND_PRI = L_contour_red
                    THIRD_PRI = L_contour_green
            elif FIRST_PRI1 == green_dir:
                FIRST_PRI = L_contour_green
                if SECOND_PRI1 == blue_dir:
                    SECOND_PRI = L_contour_blue
                    THIRD_PRI = L_contour_red
                else:
                    SECOND_PRI = L_contour_red
                    THIRD_PRI = L_contour_blue


        if FIRST_PRI is not None: # and contour_center_first<200:
            # Calculate contour information
            contour_center = rc_utils.get_contour_center(FIRST_PRI)
            contour_area = rc_utils.get_contour_area(FIRST_PRI)
 
            # Draw contour onto the image
            rc_utils.draw_contour(contour_image, FIRST_PRI, (0, 255, 0))
            rc_utils.draw_circle(contour_image, contour_center)
        
        elif SECOND_PRI is not None:
            # Calculate contour information
            contour_center = rc_utils.get_contour_center(SECOND_PRI)
            contour_area = rc_utils.get_contour_area(SECOND_PRI)
 
            # Draw contour onto the image
            rc_utils.draw_contour(contour_image, SECOND_PRI, (0, 0, 255))
            rc_utils.draw_circle(contour_image, contour_center)
 
        elif THIRD_PRI is not None:
            # Calculate contour information
            contour_center = rc_utils.get_contour_center(THIRD_PRI)
            contour_area = rc_utils.get_contour_area(THIRD_PRI)
 
            # Draw contour onto the image
            rc_utils.draw_contour(contour_image, THIRD_PRI, (255, 0, 0))
            rc_utils.draw_circle(contour_image, contour_center)
 
        else:
            contour_center = None
            contour_area = 0
        
        #print(contour_area)
        #rc.display.show_color_image(contour_image)

def check(image, co):
    contours = rc_utils.find_contours(image, co[0], co[1])
    contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)
    return contour


def flane(color_lane):
    global contour_center
    global contour_area
    global speed
    global angle
    global lane
    global arcount
    global a
    global time
    time += rc.get_delta_time()
    
    cimage = rc.camera.get_color_image()
    if cimage is None:
        contour_center = None
        contour_area = 0
    else:
        #splits image in two so that two separate contours followed
        left = rc_utils.crop(cimage, (360,0), (rc.camera.get_height(), rc.camera.get_width()//2))
        right = rc_utils.crop(cimage, (360, rc.camera.get_width()//2), (rc.camera.get_height(), rc.camera.get_width()))
        both = [left, right]
        contour_centers = []
        contour_areas = []


        #find largest contours of left
        contours_pur = rc_utils.find_contours(left, color_lane[0], color_lane[1] )
        print( contours_pur)
        rc.display.show_color_image(left)
        print(color_lane)
        contour_left = rc_utils.get_largest_contour(contours_pur, MIN_CONTOUR_AREA)
        #if there are contours, finds center and adds to contour_centers list
        if contour_left is not None:
            left_center = rc_utils.get_contour_center(contour_left)
            contour_centers.append(left_center)
            rc_utils.draw_contour(left, contour_left, (255, 0, 0))
            rc.display.show_color_image(left)
        else:
            contour_centers.append(None)

         #find largest contours of right
        contours = rc_utils.find_contours(right, color_lane[0], color_lane[1])
        contour_right = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)
        #if there are contours, finds center and adds to contour_centers list
        if contour_right is not None:
            right_center = rc_utils.get_contour_center(contour_right)
            contour_centers.append(right_center)
            #rc_utils.draw_contour(right, contour_right)
            #rc.display.show_color_image(right)
        else:
            contour_centers.append(None)
        

        #adjusts car based on being in the center of two contours
        if None not in contour_centers:
            lane =True
            image  = rc.camera.get_color_image()
            contour_distance = (contour_centers[1][1] + rc.camera.get_width()//2) - contour_centers[0][1]
            contour_center = (contour_centers[0][0]+10, (contour_distance//2) +contour_centers[0][1])
        else:
            contour_center = None
            speed = 1
            angle = 0
        if lane == True: #catches if goes off lane, else, adjusts based off of center of contours
            if contour_centers[0]==None:
                print("turn left")
                angle = -1
            elif contour_centers[1] == None:
                print("turn right")
                angle =1
            else:
                angle = rc_utils.remap_range(contour_center[1], 0, rc.camera.get_width(), -1, 1, True)
                speed = 1
            
            ##getting ar direction
            if arcount==0 and time>2:
                a = rc.camera.get_color_image()
                a = rc_utils.crop(a, (0, rc.camera.get_width()//4), (rc.camera.get_height(), rc.camera.get_width()- (rc.camera.get_width()//4)))
                corners, ids = rc_utils.get_ar_markers(a)
                rc_utils.draw_ar_markers(a, corners, ids)
                if len(corners) >0:
                    #print("getting dir")
                    if rc_utils.get_ar_direction(corners[0])== Direction.LEFT:
                        print(rc_utils.get_ar_direction(corners[0]))
                        angle = -0.7
                    if rc_utils.get_ar_direction(corners[0]):
                        print(rc_utils.get_ar_direction(corners[0]))
                        angle = 0.7
        speed = 1
        rc.drive.set_speed_angle(0, 0)
        #rc.display.show_color_image(cimage)


def start():
    """
    This function is run once every time the start button is pressed
    """
    global cur_state
    global ar_image
    # Have the car begin at a stop
    rc.drive.stop()
    
    ar_image = rc.camera.get_color_image()
    ar_image = rc_utils.crop(ar_image, (0,0), (rc.camera.get_height()//2, rc.camera.get_width()))
    info, info_id = rc_utils.get_ar_markers(ar_image)


    update_ar(info, info_id, ar_image)

    cur_state = State.laneFollow
    #cur_state = State.lineFollow
    # Print start message
    print(">> Final Challenge - Time Trials")
    


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed
    global angle
    global cur_state
    global weird

    # Search for contours in the current color image

    # Choose an angle based on contour_center#########################################
    # If we could not find a contour, keep the previous angle (Bang Bang)
    print(cur_state)
    weird = ar_in_range()
    if cur_state == State.lineFollow:
        update_contour()
        KEEP = 0
        if weird == 2 or weird == 1:
            print("switching state")
            cur_state = State.laneFollow
        if contour_center is not None:
            #implement contour area center further in direction in proportion to necessary angle
            # TODO (warmup): Implement a smoother way to follow the line
            angle = rc_utils.remap_range(contour_center[1], 0, rc.camera.get_width(), -1, 1)
            KEEP = rc_utils.remap_range(abs(angle), 0, 1, 0, 200)
            if angle > 0 :
                angle = rc_utils.remap_range(contour_center[1]+KEEP, 0, rc.camera.get_width(), -1, 1)
                angle = rc_utils.clamp(angle, -1, 1)
            elif angle<0:
                angle = rc_utils.remap_range(contour_center[1]-KEEP, 0, rc.camera.get_width(), -1, 1)
                angle = rc_utils.clamp(angle, -1, 1)
            speed = rc_utils.remap_range(abs(angle), 0, 1, .7, 1) 
        
    if cur_state == State.laneFollow:
        if weird==2:
            flane(PURPLE)
        else:
            flane(ORANGE)
    rc.drive.set_speed_angle(speed, angle)

    
def ar_in_range():
    depth_image = rc.camera.get_depth_image()
    
    ar_image = rc.camera.get_color_image()
    ar_image = rc_utils.crop(ar_image, (0,0), (rc.camera.get_height()//2, rc.camera.get_width()))
    checking_info, checking_info_id = rc_utils.get_ar_markers(ar_image)

    if checking_info:
        x =  (int)( (checking_info[0][0][0][1] + checking_info[0][0][1][1]) //2)
        y =  (int)( (checking_info[0][0][0][0] + checking_info[0][0][1][0]) //2)
        
        if rc_utils.get_pixel_average_distance(depth_image, (x, y))<200:
            contours_ar_orange = rc_utils.find_contours(ar_image, ORANGE[0], ORANGE[1])
            contours_ar_purp =  rc_utils.find_contours(ar_image, PURPLE[0], PURPLE[1])
            orange_largest = rc_utils.get_largest_contour(contours_ar_orange, 2000)
            purp_largest = rc_utils.get_largest_contour(contours_ar_purp, 2000)
            
            if orange_largest is not None:   
                print("orange")
                return 1
            elif purp_largest is not None:
                print("purple")
                return 2
            else:
                return 0
                
        
"""
for image in images:
colorContours = []
contour =None
ora = check(image, ORANGE)
pur = check(image,PURPLE)

for color in AR_PRIORITY:
    #print(color)
    if color is ORANGE and ora is not None:
        colorContours.append(ora)
    elif color is PURPLE and pur is not None:
        colorContours.append(pur)
if colorContours != []:
    contour = colorContours[0]
    contour_centers.append(rc_utils.get_contour_center(contour))
    contour_areas.append(rc_utils.get_contour_area(contour))
else:
    contour_centers.append(None)
    contour_areas.append(0)"""

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
