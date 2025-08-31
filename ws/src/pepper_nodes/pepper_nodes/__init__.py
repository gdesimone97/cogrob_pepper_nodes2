import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class PepperIPNotSetError(Exception):
    def __init__(self, *args):
        super().__init__("You have to set Pepper IP as ROS2 param")

class PepperNode(Node):
    
    def __init__(self, node_name, *, context = None, cli_args = None, namespace = None, use_global_arguments = True, enable_rosout = True, start_parameter_services = True, parameter_overrides = None, allow_undeclared_parameters = False, automatically_declare_parameters_from_overrides = False):
        super().__init__(node_name, context=context, cli_args=cli_args, namespace=namespace, use_global_arguments=use_global_arguments, enable_rosout=enable_rosout, start_parameter_services=start_parameter_services, parameter_overrides=parameter_overrides, allow_undeclared_parameters=allow_undeclared_parameters, automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides)
        self.ip = self.declare_parameter("pepper_ip", Parameter.Type.STRING).get_parameter_value().string_value
        if self.ip is None:
            raise PepperIPNotSetError
        self.port = self.declare_parameter("pepper_port", 9559).get_parameter_value().integer_value