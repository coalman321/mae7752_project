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

from functions import *


class Obstacle:

    def __init__(self, center, type):
        self.center = center

        radii = {
            'bowl': 0.0700,
            'cup': 0.0475,
            'plate': 0.1275,
            'UR5': 0.0750
        }

        self.radius = radii[type]

def main():
    # Plate is 0.255 m Diameter
    # Cup is 0.095 m Diameter
    # Bowl is 0.140 m Diameter

    obstacles = [Obstacle([0.910, 0.110], 'UR5'),
                 Obstacle([1.380, 0.530], 'cup'),
                 Obstacle([1.300, 0.155], 'cup'),
                 Obstacle([0.910, 0.340], 'bowl'),
                 Obstacle([0.390, 0.480], 'plate'),]

    start_coords = [1.745, 0.055]
    end_coords = [0.350, 0.120]

    trajectory = rrt_star(start_coords, end_coords, obstacles)

    if c.VISUAL:
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

    #trajectory = smooth_trajectory(trajectory)


if __name__ == '__main__':
    main()





