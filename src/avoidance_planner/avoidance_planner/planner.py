import rclpy
from rclpy.node import Node
from avoidance_planner.trajectory.functions import rrt_star
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

        # do the planning bit

        # fill out the Z height

        # push the data into the message


        return response



def main():
    # starts and runs the ros node class above
    rclpy.init()
    node = PlannerNode()

    rclpy.spin(node)

    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
