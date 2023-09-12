from example_interfaces.srv import AddTwoInts # Import service type

import rclpy # Import ros2 python client library
from rclpy.node import Node

class MinimalService(Node):
    # initialise node 'MinimalService'
    # Create a service and define the type, name, and callback.
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    # receive the request data, sum it, and return the sum as a response
    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response



# Spin the node to handle callbacks
def main():
    # Initialize the ROS 2 Python client library
    rclpy.init() 
    # Instantiate the MinimalService class to create the service node
    minimal_service = MinimalService() 
    # Initialize the ROS 2 Python client library
    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()