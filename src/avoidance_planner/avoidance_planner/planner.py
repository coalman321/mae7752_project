import rclpy
from rclpy.node import Node
from avoidance_planner.trajectory.trajectory import *
from avoid_plan_msgs.srv import MakePlan


class PlannerNode(Node):
    def __init__(self):
        super().__init__('make_plan_node')
        # make a service server

        self.server = self.create_service(MakePlan, 'make_plan', self.make_path)

        pass

    # service server callback, calls rrt* routine
    def make_path(self, request, response):
        # request contians the path, response should contain the result at the end

        # copy the frame ID
        response.path_out.header = request.path_in.header

        # Define the start and end XYZ coordinates from the path_in request
        start_coords = request.path_in.poses[0].pose.point
        end_coords = request.path_in.poses[-1].pose.point

        # Calculate the XYZ coordinates of the path from start to end coordinates
        trajectory = generate_trajectory(start_coords, end_coords)

        # Define empty poses list
        poses = []

        # For each set of coordinates in the trajectory
        for point in trajectory:
            # Define the stamped pose structure and insert the XYZ point and original orientation
            ps = PoseStamped()
            ps.pose.point.x = point[0]
            ps.pose.point.y = point[1]
            ps.pose.point.z = point[2]
            ps.pose.orientation = start_coords.pose.orientation
            poses.append(ps)

        # push the data into the message
        response.path_out.poses = poses

        return response

    
def main():
    # starts and runs the ros node class above
    rclpy.init()
    node = PlannerNode()

    rclpy.spin(node)

    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
