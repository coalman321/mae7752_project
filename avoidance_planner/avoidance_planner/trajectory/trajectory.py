"""
ME 7752 Final Project Trajectory Planning
Returns a list of smoothed trajectory waypoints for the motion planning of a UR5 robotic arm.
Path plan created to avoid given obstacles in the available configuration space.

Author: Dylan Trainor
Email: trainor.27@osu.edu

External Sources:
www.pythonpool.com/numpy-magnitude/ - Use of numpy norm() function
mahbub029.blogspot.com/2016/01/motion-planning-algorithm-rrt-starrrt.html - General RRT* algorithm and pygame GUI setup
docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.splrep.html - SciPy curve interpolation smoothing

Companion scripts:

functions.py    --      List of primary functions called by trajectory.py script
constants.py    --      List of constants imported to trajectory.py and functions.py scripts

"""

from avoidance_planner.trajectory.functions import *


class Obstacle:

    def __init__(self, center, type):
        # Initialize center of obstacle
        self.center = center
        
        # Select radius from list of radii of each obstacle
        radii = {
            'bowl': 0.0700,
            'cup': 0.0475,
            'plate': 0.1275,
            'UR5': 0.0750
        }
        self.radius = radii[type]

def generate_trajectory(start_coords, end_coords):
    # Define list of obstacles (UR5 base, two cups, bowl, and plate)
    obstacles = [Obstacle(c.BASE_LINK, 'UR5'),
                 Obstacle([1.380, 0.530], 'cup'),
                 Obstacle([1.300, 0.155], 'cup'),
                 Obstacle([0.910, 0.340], 'bowl'),
                 Obstacle([0.390, 0.480], 'plate')]

    # Define the Z offset, starting XY coordinates, and ending XY coordinates for path planner
    # To convert to GUI frame, coordinates must be offset by UR5 base link location and cup rim offset
    z = start_coords.z
    start_coords = [-start_coords.x + c.BASE_LINK[0], -start_coords.y + c.BASE_LINK[1] + c.EE_RAD]
    end_coords = [-end_coords.x + c.BASE_LINK[0], -end_coords.y + c.BASE_LINK[1] + c.EE_RAD]

    # Compute the trajectory using the RRT star method
    trajectory = rrt_star(start_coords, end_coords, obstacles)

    # If the visual is desired, listen for GUI close event
    if c.VISUAL:
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

    # For each point in the planned trajectory
    for point in trajectory:
        # Convert XYZ back to UR5 world frame with desired Z offset
        point[0] = -(point[0] - c.BASE_LINK[0])
        point[1] = (point[1] - c.BASE_LINK[1] - c.EE_RAD)
        point.append(z)

    # Return the list of [x, y, z] points
    return trajectory




