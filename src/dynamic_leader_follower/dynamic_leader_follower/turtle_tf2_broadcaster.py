import math
from geometry_msgs.msg import Twist, TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from turtlesim.msg import Pose
from turtlesim.srv import Spawn

# make quaternion from euler angles
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


class FramePublisher(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_publisher')

        # Declare and acquire the number of turtles parameter
        self.num_turtles = self.declare_parameter('num_turtles', 2).get_parameter_value().integer_value

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create a client to spawn a turtle
        self.spawner = self.create_client(Spawn, 'spawn')

        self.turtles = {}  # Dictionary to hold turtle state
        
        # make a dictionary for all of our robots
        for i in range(1, self.num_turtles + 1):
            turtlename = f'turtle{i}'
            self.turtles[turtlename] = {
                'spawned': False,
                'service_ready': False,
                'result': None,
                'subscription': None,
            }

        self.timer = self.create_timer(1.0, self.on_timer)


    def on_timer(self):
        for turtlename in self.turtles.keys():
            if turtlename == 'turtle1':
                # turtle1 is already spawned and we only have to make it subscribe to pos of that turtle
                self.turtles[turtlename]['spawned'] = True
                if not self.turtles[turtlename]['subscription']:
                    self.turtles[turtlename]['subscription'] = self.create_subscription(
                    Pose, f'/{turtlename}/pose',
                    lambda msg, name=turtlename: self.handle_turtle_pose(msg, name),
                    1)
                continue

            turtle_info = self.turtles[turtlename]

            if turtle_info['spawned']:
                # create subscribtion for spawned turtles
                if not turtle_info['subscription']:
                    turtle_info['subscription'] = self.create_subscription(
                        Pose,
                        f'/{turtlename}/pose',
                        lambda msg, name=turtlename: self.handle_turtle_pose(msg, name),
                        1)
                continue

            if turtle_info['service_ready']:
                # inform user that the robot spawns
                if turtle_info['result'].done():
                    self.get_logger().info(f"Spawned {turtlename}")
                    turtle_info['spawned'] = True
                else:
                    self.get_logger().info(f"Waiting for {turtlename} to spawn...")
            else:
                if self.spawner.service_is_ready():
                    request = Spawn.Request()
                    request.name = turtlename
                    request.x = float(4*np.random.random(1)[0]+1)
                    request.y = float(4*np.random.random(1)[0]+1)
                    request.theta = float(0)
                    turtle_info['result'] = self.spawner.call_async(request)
                    turtle_info['service_ready'] = True
                else:
                    self.get_logger().info(f"Spawn service not ready for {turtlename}")

    # broadcast the robots frame to tf2
    def handle_turtle_pose(self, msg, turtlename):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = turtlename
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        q = quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
