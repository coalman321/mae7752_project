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

    def __init__(self, center, radius):
        self.center = center
        self.radius = radius


def main():
    obstacles = [Obstacle([100, 100], 10),
                 Obstacle([200, 200], 30),
                 Obstacle([400, 50], 40),
                 Obstacle([700, 100], 15),
                 Obstacle([300, 150], 50)]

    start_coords = [50, 50]
    end_coords = [500, 220]

    trajectory = rrt_star(start_coords, end_coords, obstacles)

    if c.VISUAL:
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

    trajectory = smooth_trajectory(trajectory)


if __name__ == '__main__':
    main()





