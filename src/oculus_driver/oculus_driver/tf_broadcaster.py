#!/usr/bin/env python3
"""
Minimal static TF publisher:
  map → odom → base_link → sonar_link
For testing without a real robot, odom stays at origin (dead reckoning = 0).
"""
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class StaticTF(Node):
    def __init__(self):
        super().__init__('static_tf')
        br = StaticTransformBroadcaster(self)

        def make_tf(parent, child, x=0.0, y=0.0, z=0.0):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id  = parent
            t.child_frame_id   = child
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            t.transform.rotation.w = 1.0
            return t

        br.sendTransform([
            make_tf('map',       'odom'),
            make_tf('odom',      'base_link'),
            make_tf('base_link', 'sonar_link', z=0.1),
        ])

def main(args=None):
    rclpy.init(args=args)
    node = StaticTF()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
