from enum import Enum

class Colors(Enum):
    # Color Thresholds
    PURPLE = ((117, 50, 50),(155, 255, 255))
    ORANGE = ((10, 100, 100), (20, 255, 255))
    GREEN = ((35, 200, 200),(75, 255, 255))
    RED = ((170, 100, 100), (10, 255, 255))
    BLUE = ((90, 200, 200), (120, 255, 255))

# Gaussian Blue for Depth Image
BLUR_KERNEL_SIZE = 5

# Car Max Speed
MAX_SPEED = 1

# Colors that exist in lane/line following
LANE_COLORS = (Colors.PURPLE, Colors.ORANGE)
LINE_COLORS = (Colors.GREEN, Colors.RED, Colors.BLUE)

# Distance to detect AR tag to switch state
CONTOUR_DETECT_RANGE = 200

# Tag IDS
ID_LINE = 0
ID_LANE = 1
ID_SLALOM = 2
ID_WALL = 3
ID_DIRECTION = 199

# Crop floor size for contour line
CROP_FLOOR = ((480*3//5, 0), (480, 640)) # 480 = camera height, 640 = camera width

#Minimum contour area for largest contour function
MIN_CONTOUR_AREA = 1000