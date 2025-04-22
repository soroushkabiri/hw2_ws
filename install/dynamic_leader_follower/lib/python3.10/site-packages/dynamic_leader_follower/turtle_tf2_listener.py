import math
from geometry_msgs.msg import Twist
import rclpy
import numpy as np
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.parameter import Parameter
import time
from rcl_interfaces.msg import SetParametersResult


class FrameListener(Node):

    def __init__(self):
        super().__init__('turtle_tf2_listener')
        
        # we add this sleep delay to let broadcaster initialize and spawn all turtles
        time.sleep(5.0)  
        
        # Declare and acquire the number of turtles parameter
        self.num_turtles = self.declare_parameter('num_turtles', 2).get_parameter_value().integer_value
        
        # Declare and acquire `current_leader as target_frame` parameter
        # for the initial setup we consider turtle1 as leader and target frame        
        self.target_frame = self.declare_parameter('current_leader', 'turtle1').get_parameter_value().string_value
        
        # Set up parameter callback for runtime changes
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create publishers for all turtles except the leader
        self.publisher = {}
        for i in range(1, self.num_turtles + 1):
            turtlename = f'turtle{i}'
            if turtlename != self.target_frame:
                self.publisher[turtlename] = self.create_publisher(
                    Twist, f'/{turtlename}/cmd_vel', 1)
        
        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'current_leader' and param.type_ == Parameter.Type.STRING:
                # check the new current leader
                if param.value!=self.target_frame:
                    self.target_frame = param.value
                    self.get_logger().info(f"Leader changed to {self.target_frame}")
                    self.get_logger().info("Scheduling publisher rebuild...")
                    self.rebuild_publishers()
        return SetParametersResult(successful=True)
    
    def rebuild_publishers(self):
        self.get_logger().info(f"Rebuilding publishers, new leader: {self.target_frame}")
        
        #destroy previous publishers
        for pub in self.publisher.values():
            self.destroy_publisher(pub)
        self.publisher.clear()

        # make new publisher for robots except current_leader
        for i in range(1, self.num_turtles + 1):
            turtlename = f'turtle{i}'
            if turtlename != self.target_frame:
                self.publisher[turtlename] = self.create_publisher(
                    Twist, f'/{turtlename}/cmd_vel', 1)
    
    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.target_frame

        for turtlename in self.publisher.keys():
            to_frame_rel = turtlename
            try:
                t = self.tf_buffer.lookup_transform(
                    to_frame_rel, from_frame_rel, rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
                continue
            
            # make a velocity command for followers based on their frame difference with leader
            msg = Twist()
            scale_rotation_rate = 1.0
            msg.angular.z = scale_rotation_rate * math.atan2(
                        t.transform.translation.y,
                        t.transform.translation.x)
            scale_forward_speed = 0.5+int(turtlename[-1])*0.1
            msg.linear.x = scale_forward_speed * math.sqrt(
                        t.transform.translation.x ** 2 +
                        t.transform.translation.y ** 2)
            self.publisher[turtlename].publish(msg)


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
