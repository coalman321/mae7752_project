"""
Description:

List of primary functions called by trajectory.py script. Includes the following functions:

rrt_star()          --      Creates list of nodes for desired trajectory using the RRT* planning algorithm in 2D space
smoothTrajectory    --      Smooths a list of trajectory waypoints using fifth order polynomial approximation method

"""

from tqdm import tqdm
import sys
import pygame
from pygame.locals import *
from helpers import *
from scipy.interpolate import splrep, splev
import matplotlib.pyplot as plt


def rrt_star(start_coords, end_coords, obstacles):
    """
    Creates a list of node objects for the desired trajectory using the RRT* motion planning algorithm in 2D space.

    :param start_coords:    the XY list of the 2D coordinates of the starting position
    :param end_coords:      the XY list of the 2D coordinates of the end position
    :param obstacles:       the list of obstacle objects for the given obstacles in the configuration space
    :return trajectory:     a list of node objects for the resultant trajectory of the RRT* planner
    """

    if c.VISUAL:
        # Create the pygame GUI
        pygame.init()                                       # Initialize pygame GUi
        screen = pygame.display.set_mode(c.CSPACE)          # Set screen size to configuration space
        pygame.display.set_caption('UR5 RRTstar Planner')   # Create a display name for the GUI
        screen.fill([255, 255, 255])                        # Fill the screen with a white background

        # For each obstacle in the list of obstacles
        for obstacle in obstacles:
            # Draw a red circle at the give obstacle center with the given obstacle radius
            pygame.draw.circle(screen, [255, 0, 0], (obstacle.center[0], obstacle.center[1]), obstacle.radius)

        # Draw a blue and green circle for the location of the start and end coordinates
        pygame.draw.circle(screen, [0, 0, 255], start_coords, 5)
        pygame.draw.circle(screen, [0, 255, 0], end_coords, 5)

    # Define the start node, end node, and node list
    start = Node(start_coords)                          # the starting node
    end = Node(end_coords)                              # the end goal node
    node_list = [start]                                 # the list of nodes, initialized with the starting node

    # For each iteration of the maximum tree size
    for i in tqdm(range(c.MAX_TREE), desc="RRT* Tree Iteration..."):
        # Define a random sampled node with a bias towards the target
        rand_sample = Node(bias_rand(end_coords))

        # Compute the distance of the random sample to each node in the node list and select the closest node
        distances = [distance(node, rand_sample) for node in node_list]
        closest_node = node_list[distances.index(min(distances))]

        # If the random node is less than the maximum distance from the closest node, define as the new node
        if min(distances) <= c.MAX_DIST:
            new_node = rand_sample

        # If the random node is greater than the maximum distance from the closest node, perform a truncated distance
        else:
            unit_vec = unit_vector(closest_node, rand_sample)       # Unit vector from closest node to random sample
            short_vec = c.MAX_DIST * unit_vec                       # Unit vector scaled to maximum distance
            new_node = Node([closest_node.x + short_vec[0],         # Define new node as max distance in unit direction
                             closest_node.y + short_vec[1]])

        # If the path from the closest node to the new node does not intersect an obstacle
        if not obstacle_detection(closest_node, new_node, obstacles):
            # For each node in the list of nodes
            for node in node_list:
                # If the path from the node to the new node does not intersect an obstacle
                if (distance(node, new_node) < c.CONSIDERATION) and not obstacle_detection(node, new_node, obstacles):
                    # Define the cost of the iterated node and closest node to the new node
                    cost_node_to_new = node.cost + distance(node, new_node)
                    cost_closest_to_new = closest_node.cost + distance(closest_node, new_node)

                    # If the distance between the node and the new node is less than the success radius, and the cost
                    # of the node to the new node is less than the cost of the closest node to the new node, update the
                    # closest node to be the node
                    if cost_node_to_new < cost_closest_to_new:
                        closest_node = node

                # Update the cost and parent of the new node
                new_node.cost = cost_closest_to_new
                new_node.parent = closest_node

            # Add the new node to the node list and draw a line on the GUI from the closest node to the new node
            node_list.append(new_node)
            if c.VISUAL:
                pygame.draw.line(screen, [20, 20, 20], closest_node.coords, new_node.coords)

            # For each node and index in the node list
            for node in node_list:
                # If the path from the node to the new node does not intersect an obstacle, the node is not the parent
                # of the new node, the distance from the node to the new node is less than the success radius, and the
                # added cost from the node to the new node is less than the original cost
                if (node is not new_node.parent) and \
                        (distance(node, new_node) < c.CONSIDERATION) and \
                        (new_node.cost + distance(node, new_node) < node.cost) and \
                        (not obstacle_detection(node, new_node, obstacles)):

                    # Update the node parent and cost
                    node.parent = new_node
                    node.cost = new_node.cost + distance(node, new_node)

                    if c.VISUAL:
                        # Erase the previous node line and replace with the new updated node line
                        pygame.draw.line(screen, [255, 255, 255], node.coords, node.parent.coords)
                        pygame.draw.line(screen, [20, 20, 20], node.coords, new_node.coords)

            if c.VISUAL:
                # Update the GUI display
                pygame.display.update()

                # For the events logged by the pygame GUI
                for event in pygame.event.get():
                    # If the user creates an action that quits the GUI, exit the script
                    if event.type == QUIT or (event.type == KEYUP and event.key == K_ESCAPE):
                        sys.exit("Leaving because you requested it.")

    # From the list of distances from the nodes in the node list to the end coordinates, find the closest end node
    distances = [distance(node, end) for node in node_list]
    end_node = node_list[distances.index(min(distances))]

    # Create the initial trajectory list
    trajectory = [end_node]

    # While the end node of the answer path is not the original starting node
    while end_node is not start:
        # Draw a line from the end node to its parent and update the end node as its parent node
        if c.VISUAL:
            pygame.draw.line(screen, [0, 255, 0], end_node.coords, end_node.parent.coords, 5)
        end_node = end_node.parent

        # Add the end node to the list of the final trajectory path nodes
        trajectory.insert(0, end_node)

    if c.VISUAL:
        # Update the GUI display
        pygame.display.update()
        print('Please exit the GUI window to continue...')

    # Return the list of nodes for the desired trajectory
    return trajectory


def smooth_trajectory(trajectory):
    """
    Smooths a list of XY trajectory waypoints in 2D space using fifth order polynomial approximation smoothing method.

    :param trajectory:      List of XY trajectory node waypoints
    :return:                Smoothed list of XY trajectory node waypoints
    """

    # Compute the domain and range lists of the XY coordinate points of the original trajectory
    domain = [node.x for node in trajectory]
    range = [-node.y for node in trajectory]

    # Compute the smoothed range values using a fifth order polynomial approximation method
    poly = np.polyfit(domain, range, 5)
    poly_y = np.poly1d(poly)(domain)

    # Plot the original XY coordinates of the trajectory and the smoothed trajectory
    plt.figure()
    plt.plot(domain, poly_y)
    plt.plot(domain, range)
    plt.show()

    # Return the smoothed trajectory as a 2xN NumPy 2D array
    smooth = np.array([domain, poly_y])
    return smooth
