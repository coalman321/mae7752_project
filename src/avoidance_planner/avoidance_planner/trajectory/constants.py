"""
Description:

List of constants imported to functions.py script.

"""

RATIO = 400                 # The conversion ratio from the real world coordinates to GUI coordinates
CSPACE = [1.830, 0.605]     # The dimensions of the configuration space of the robot in meters

MAX_DIST = 0.025            # The maximum distance a tree expansion can extend
MAX_TREE = 4000             # The maximum iterations of the tree
CONSIDERATION = 0.050       # The radius for which to consider the rewiring of node parents
BUFFER = 0.020              # The additional buffer to consider outside of an obstacle area
EE_RAD = 0.0375
BIAS = 0.00                 # The percentage bias of the random node calculation towards the end coordinates
VISUAL = 1                  # Boolean to determine whether to show the pygame visual GUI
