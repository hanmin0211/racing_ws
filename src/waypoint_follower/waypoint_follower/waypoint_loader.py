import rclpy
from rclpy.node import Node
import yaml


class WaypointLoader(Node):

    def __init__(self):
        super().__init__('waypoint_loader')

        self.file_path = '/home/han/waypoints/all_waypoint.yaml'

        self.waypoints = []

        self.load_waypoints()

        self.get_logger().info(
            f'Loaded {len(self.waypoints)} waypoints'
        )


    def load_waypoints(self):

        with open(self.file_path, 'r') as file:
            data = yaml.safe_load(file)

        self.waypoints = data['waypoints']

        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(
                f'{i}: lat={wp["latitude"]}, lon={wp["longitude"]}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = WaypointLoader()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
