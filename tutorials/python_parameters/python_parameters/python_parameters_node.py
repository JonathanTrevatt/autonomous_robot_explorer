import rclpy
import rclpy.node

class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('minimal_param_node')
         # Create a parameter with the name my_parameter and a default value of world
        from rcl_interfaces.msg import ParameterDescriptor
        my_parameter_descriptor = ParameterDescriptor(description='This parameter is mine!')
        self.declare_parameter('my_parameter', 'world', my_parameter_descriptor)
        self.timer = self.create_timer(1, self.timer_callback) # Call timer_callback @ 1 Hz

    def timer_callback(self):
        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value
        # Ensure the event is logged
        self.get_logger().info('Hello %s!' % my_param)

        my_new_param = rclpy.parameter.Parameter(
            'my_parameter',
            rclpy.Parameter.Type.STRING,
            'world'
        )
        all_new_parameters = [my_new_param]
        self.set_parameters(all_new_parameters)

def main():
    rclpy.init() # ROS 2 is initialized
    node = MinimalParam() # An instance of the MinimalParam class is constructed
    rclpy.spin(node) # Start processing data from the node

if __name__ == '__main__':
    main()