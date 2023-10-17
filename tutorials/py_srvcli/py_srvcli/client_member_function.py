# The client node code uses sys.argv to get access 
# to command line input arguments for the request.
import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):
    # Create a client with the same type and name as the service node
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        # Check for service matching type and name of client is available (1 Hz).
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()
    int1 = int(sys.argv[1])
    int2 = int(sys.argv[2])

    # Check future for response from the service (while system is running)
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int1, int2)
    # If the service has sent a response, write result in a log message.
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int1, int2, response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()