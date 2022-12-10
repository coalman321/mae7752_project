"""
Description:

List of helper functions called by functions.py script. Includes the following functions:

obstacle_detection      --      A script to detect intersection with an obstacle in the list of obstacles
distance                --      A script that computes the magnitude distance between two nodes
unit_vector             --      A script that computes the 2D unit vector between two nodes
bias                    --      Computes a random set of coords in the cspace with a biased percentage reduction to end
scale                   --      Returns the scaled value or a single or set of coords from the real world to GUI window

"""

import avoidance_planner.trajectory.constants as c
import random as rand
from math import sqrt
import numpy as np


class Node:

    def __init__(self, coords):
        """
        :param coords:      the X Y Z coordinates of the node in a labeled dictionary

        :self x:            the X coordinate of the node
        :self y:            the Y coordinate of the node
        :self vec:          the numpy array vector of the node relative to the origin
        :self cost:         the cost of the node
        :self parent:       the parent node to the node
        """

        self.x = coords[0]
        self.y = coords[1]
        self.coords = [self.x, self.y]
        self.vec = np.array(self.coords)
        self.cost = 0
        self.parent = None


def obstacle_detection(origin_point, test_point, obstacles):
    """
    A script to detect whether a given node step will intersect an obstacle in the list of obstacles.

    :param origin_point:    the starting node of the node step
    :param test_point:      the ending node of the node step
    :param obstacles:       the list of obstacle objects in the configuration space
    :return collision:      the boolean outcome of the collision (FALSE == no collision)
    """

    # Initialize the collision state as no collision
    collision = False

    # Compute the unit vector between the origin point and the test point
    unit_vec = unit_vector(origin_point, test_point)

    # For a time step from 0 to 100 in increments of 1%
    for i in range(0, 10):
        # Calculate the added step vector based on the unit vector and add it to the sample point
        step = i * 0.1 * unit_vec
        sample_point = [step[0] + origin_point.x, step[1] + origin_point.y]

        # For each obstacle in the obstacle list
        for obstacle in obstacles:
            # Compute the radius as the distance between the sample point and the obstacle center
            radius = distance(Node(sample_point), Node(obstacle.center))

            # If the distance to the obstacle is less than the obstacle's radius, mark the step as a collision
            if radius < (obstacle.radius + c.BUFFER + c.EE_RAD):
                collision = True
                break

    # Return the collision boolean answer
    return collision


def distance(node1, node2):
    """
    A script that computes the magnitude distance between two nodes.

    :param node1:       the origin node
    :param node2:       the ending node
    :return delta:      the distance between the two nodes
    """

    # Compute the distance between two nodes using the pythagorean theorem hypotenuse two vector arrays
    delta = sqrt((node2.x - node1.x)*(node2.x - node1.x) + (node2.y - node1.y)*(node2.y - node1.y))

    # Return the distance between the two nodes
    return delta


def unit_vector(node1, node2):
    """
    A script that computes the 2D unit vector between two nodes.

    :param node1:       the origin node
    :param node2:       the ending node
    :return vector:     the 2D unit vector between the two nodes
    """

    # Compute the unit vector using the vector between nodes and magnitude of the distance between nodes
    vector = (node2.vec - node1.vec) / distance(node1, node2)

    # Return the unit vector between the two nodes
    return vector


def bias_rand(end_coords):
    """
    Computes a random set of XY coordinates in the configuration space with a biased percentage reduction of the
    distance between the original random point and the end coordinates.

    :param end_coords:      the ending coordinates of the trajectory
    :return:                the random XY pair in the configuration space with a bias towards the end coordinates
    """

    # Compute a random set of XY points in the configuration space
    x_rand = rand.randrange(0, 100, 1) * c.CSPACE[0]/100
    y_rand = rand.randrange(0, 100, 1) * c.CSPACE[1]/100

    # Reduce the distance from the XY points to the end coordinates by the percent bias
    x_rand = x_rand + (end_coords[0] - x_rand) * c.BIAS
    y_rand = y_rand + (end_coords[1] - y_rand) * c.BIAS

    # Return the biased random XY coordinates
    return [x_rand, y_rand]


def scale(coords):
    """
    Returns the scaled value or a single or set of coordinates from the real world to GUI window

    :param coords:      one or more coordinate values in the configuration space
    :return:            one or more coordinates scaled to the GUI window
    """

    if type(coords) is (float or int):
        coords = c.RATIO*coords
    elif type(coords) is list:
        coords = [c.RATIO*coord for coord in coords]

    return coords
