#!/usr/bin/env python3

# Import libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

# ROS2 Node to establish tr transforms boradcaster between "world" and "camera_init" frame
class CameraInitStaticTF(Node):
    def __init__(self):
        super().__init__('camera_init_tf_pub')
        self.broadcaster = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'aft_mapped'
        t.child_frame_id = '/wamv/wamv/base_link'
        t.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(t)
        self.get_logger().info(f'Published static transform: {t.header.frame_id} → {t.child_frame_id}')

def main():
    rclpy.init()
    node = CameraInitStaticTF()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()