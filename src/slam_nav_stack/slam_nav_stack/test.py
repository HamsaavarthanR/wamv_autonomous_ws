#!/usr/bin/env python3

# Import necessary libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2

import tf2_ros
import tf_transformations
import numpy as np


## --------------------------------------------------------------------------------------------------- ##

# ROS2 Node to log wamv_lidar points and points_enriched


# # ROS2 Node obtain extrinsic transform from IMU to LIDAR
# class ExtrinsicTFNode(Node):
#     def __init__(self):
#         super().__init__('extrinsic_tf_node')
        
#         # Create a TF2 buffer and listener
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
#         # Load tf frames
#         self.source_frame = 'wamv/wamv/imu_wamv_link/imu_wamv_sensor'
#         self.target_frame = 'wamv/wamv/base_link/lidar_wamv_sensor'
        
#         # Timer to periodically get the transform
#         self.timer = self.create_timer(1.0, self.get_transform)
        
#     def get_transform(self):
#         try:
#             self.get_logger().info(f'Looking up transform from {self.source_frame} to {self.target_frame}')
#             # Lookup the transform from source_frame to target_frame
#             if self.tf_buffer.can_transform(
#                 self.target_frame, 
#                 self.source_frame,
#                 rclpy.time.Time(),
#                 timeout=rclpy.duration.Duration(seconds=2.0)):
                
#                 # Get the transform
#                 transform = self.tf_buffer.lookup_transform(
#                     self.target_frame,
#                     self.source_frame,
#                     rclpy.time.Time())
                
#                 self.get_logger().info('TF transform available, using it to set extrinsics.')
#                 self.get_logger().info(f'Transform: {transform}')
                
#                 # Extract translation and rotation
#                 translation = transform.transform.translation
#                 rotation = transform.transform.rotation
                
#                 # Convert quaternion to rotation matrix
#                 quat = [rotation.x, rotation.y, rotation.z, rotation.w]
#                 rot_matrix = tf_transformations.quaternion_matrix(quat)[:3, :3]
                
#                 # Create transformation matrix
#                 extrinsic_T = translation
#                 extrinsic_R = rot_matrix
                
#                 self.get_logger().info(f'Extrinsic Translation: {extrinsic_T}')
#                 self.get_logger().info(f'Extrinsic Rotation:\n{extrinsic_R}')
#             else:
#                 self.get_logger().warn(f'Cannot transform from {self.source_frame} to {self.target_frame} at this time.')
#         except Exception as e:
#             self.get_logger().error(f'Error looking up transform: {e}')


# def main(args=None):
#     rclpy.init(args=args)
    
#     extrinsic_tf_node = ExtrinsicTFNode()
    
#     rclpy.spin(extrinsic_tf_node)
    
#     extrinsic_tf_node.destroy_node()
#     rclpy.shutdown()



## --------------------------------------------------------------------------------------------------- ##

# # ROS2 Node for printing LIDAR and IMU topics (Time Stamp)
# class TimeStampPrinter(Node):
#     def __init__(self):
#         super().__init__('timestamp_printer')
        
#         # LIDAR subscription
#         self.lidar_subscription = self.create_subscription(
#             PointCloud2,
#             '/wamv/sensors/lidars/lidar_wamv_sensor/points',
#             self.lidar_callback,
#             10,
#         )
        
#         # IMU subscription
#         self.imu_subscription = self.create_subscription(
#             Imu,
#             'wamv/sensors/imu/imu/data',
#             self.imu_callback,
#             10)
        
#     def lidar_callback(self, msg):
#         self.get_logger().info(f'LIDAR Time Stamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')
        
#     def imu_callback(self, msg):
#         self.get_logger().info(f'IMU Time Stamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')
        
        
        
# def main(args=None):
#     rclpy.init(args=args)
    
#     timestamp_printer = TimeStampPrinter()
    
#     rclpy.spin(timestamp_printer)
    
#     timestamp_printer.destroy_node()
#     rclpy.shutdown()
    
    
