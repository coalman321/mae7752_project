import rclpy
from rclpy.node import Node
from avoidance_planner.trajectory.functions import rrt_star


class PlannerNode(Node):
    def __init__(self):
        # make a service server

        self.server = self.create_service()

        pass

    # service server callback, calls rrt* routine
    def make_path(self):
        pass



def main():
    # starts and runs the ros node class above
    rclpy.init()
    node = PlannerNode()

    rclpy.spin(node)

    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
