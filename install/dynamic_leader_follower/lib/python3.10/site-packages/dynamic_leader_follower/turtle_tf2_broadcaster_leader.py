import math
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from turtlesim.msg import Pose
from rclpy.parameter import Parameter


# making quaternion from euler angles 
def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

# this node only publish leader frame
class FramePublisher_leader(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_publisher_leader')

        #parameter for initial leader selection
        # we make leadername a parameter in ros2
        self.current_leader = self.declare_parameter('current_leader', 'turtle1').get_parameter_value().string_value

        # Set up parameter callback for runtime changes
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        #show initial leader name
        self.get_logger().info(f"Initial leader set to: {self.current_leader}")
        
        # Current pose subscription
        self.setup_leader_subscription()
        
        


    def setup_leader_subscription(self):
        #Helper method to create the pose subscription
        if hasattr(self, 'subscription'):
            # prevent multi subscribtion
            self.destroy_subscription(self.subscription)
                    
        self.subscription = self.create_subscription(Pose,
                f'/{self.current_leader}/pose',
                self.leader_pose_callback,10)
                
        self.get_logger().info(f"Now tracking leader: {self.current_leader}")


    def parameters_callback(self, params):
        #Handle parameter updates at runtime (dynamic leader changing)
        for param in params:
            if param.name == 'current_leader' and param.type_ == Parameter.Type.STRING:
                self.current_leader = param.value
                self.setup_leader_subscription()
        return rclpy.node.SetParametersResult(successful=True)

    def leader_pose_callback(self, msg):
        #publish transform for the current leader
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.current_leader

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()