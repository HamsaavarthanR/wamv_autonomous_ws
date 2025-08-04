#!/usr/bin/env python3

# Import necessary libraries
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import ParameterType
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
# from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import message_filters
import numpy as np
# import cv2
import tf2_ros
# import tf2_geometry_msgs
import yaml
import os
import sys
import tf_transformations
from sensor_msgs_py import point_cloud2

from autonomous_stack_interfaces.msg import StampedFloat64MultiArray

# Sensor Fusion Node
# This node fuses data from a camera and LiDAR to detect and localize obstacles in the environment.
# It uses a custom YOLOv8 model to detect obstacles in the camera feed and projects LiDAR points onto the camera image to find their positions in the world frame.
# The detected obstacles are published as PointStamped messages in the WAM-V's frame.
class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        
        # Use sim time
        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value

        # Initialize the CvBridge for converting ROS images to OpenCV format
        self.cv_bridge = CvBridge()

        # Declare and get config path from launch file
        self.declare_parameter('config_path', '')
         # Get the config path parameter
         # This allows the node to load configuration parameters from a YAML file
        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        if (not os.path.exists(config_path)):
            self.get_logger().warn('ERROR: Model path is invalid or model was not found. Make sure the model filename was entered correctly.')
            sys.exit(0)

        # Load YAML config
        with open(config_path, 'r') as f:
            self.params = yaml.safe_load(f)

        self.get_logger().info(f'Loaded config from: {config_path}')

        # Declration of camera intrinsics and distortion coefficients parameters
        # Load fallback intrinsics
        cam = self.params['sensor_fusion_node']['camera']
        self.K = np.array(cam['intrinsic_matrix']).reshape(3, 3)
        self.dist_coeffs = np.array(cam['distortion_coeffs'])
        self.P = np.array(cam['projection_matrix']).reshape(3, 4)

        # TF setup for lidar to camera transformation
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Load extrinsics from parameters or TF
        self.tf_frames = self.params['sensor_fusion_node']['lidar_to_camera']
        self.source_frame = self.tf_frames['source_frame']
        self.target_frame = self.tf_frames['target_frame']
        # self.source_frame = "wamv/wamv/base_link"
        # self.target_frame = "wamv/wamv/base_link/front_left_camera_sensor"
        self.lidar_to_camera_tf = np.eye(4)
        
        # Create timer to periodically update the TF buffer
        self.tf_timer = self.create_timer(1.0, self.update_tf)
        

        # Subscriptions
        self.image_sub = message_filters.Subscriber(self, Image, '/wamv/sensors/cameras/front_left_camera_sensor/image_raw')
        self.pc_sub = message_filters.Subscriber(self, PointCloud2, '/wamv/sensors/lidars/lidar_wamv_sensor/points')
        self.bbox_sub = message_filters.Subscriber(self, StampedFloat64MultiArray, '/wamv/sensors/cameras/front_left_camera_sensor/obstacle_centres')
        

        # Approximate time synchronizer for image, point cloud, and bounding boxes
        self.time_sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.pc_sub, self.bbox_sub], queue_size=10, slop=0.1
        )
        # Register the callback for synchronized messages
        self.time_sync.registerCallback(self.sync_callback)

        # Variable to check if TF is waiting
        self.tf_waiting = True
        
        # Publishers
        # Publisher for obstacle positions in the WAM-V frame
        self.pose_pub = self.create_publisher(PointStamped, '/wamv/obstacle_positions', 10)
    

        # # Subscribe to camera_info
        # self.camera_info_sub = self.create_subscription(
        #     CameraInfo,
        #     '/wamv/sensors/cameras/front_left_camera_sensor/camera_info',
        #     self.camera_info_callback,
        #     10
        # )

        
        self.get_logger().info('Sensor Fusion Node Initialized')

    # def camera_info_callback(self, msg):
    #     self.K = np.array(msg.k).reshape(3, 3)
    #     self.dist_coeffs = np.array(msg.d)
    #     self.get_logger().info('Camera intrinsics updated from CameraInfo topic')
    
    # Update the TF buffer periodically
    def update_tf(self):
        if self.tf_waiting:
            try:
                # Wait for the TF tree to be populated
                self.get_logger().info('Waiting for TF tree to be populated...')
                while self.tf_waiting:
                    if self.tf_buffer.can_transform(
                        self.target_frame,
                        self.source_frame,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=2.0)):
                        
                        transform = self.tf_buffer.lookup_transform(
                            self.target_frame,
                            self.source_frame,
                            rclpy.time.Time())
                        self.get_logger().info('TF transform available, using it to set extrinsics.')
                        self.get_logger().info(f'Transform: {transform}')
                        # Set tf_waiting to False to exit the loop
                        self.tf_waiting = False
                    else:
                        self.get_logger().warn('TF transform not available, using fallback from config.')
                # Extract translation and rotation from the transform
                t = transform.transform.translation
                q = transform.transform.rotation
                # translation and rotation (in quaternion form)
                trans = tf_transformations.translation_matrix([t.x, t.y, t.z])
                rot = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
                # combine translation and rotation into a single transformation matrix
                self.lidar_to_camera_tf = np.dot(trans, rot)
                self.get_logger().info('Lidar-to-camera extrinsics loaded from TF tree')
                self.get_logger().info(f'Extrinsic matrix: {self.lidar_to_camera_tf}')
            except tf2_ros.TransformException as ex:
                self.get_logger().warn(f'TF lookup failed: {ex}. Using fallback from config.')
                if 'extrinsic_matrix' in self.tf_frames:
                    self.lidar_to_camera_tf = np.array(self.tf_frames['extrinsic_matrix']).reshape(4, 4)
        
    ## TEST ##
    # def sync_callback(self, image_sub, pc_msg, bbox_msg):
    #     img_timestamp = image_sub.header.stamp
    #     pc_timestamp = pc_msg.header.stamp
    #     self.get_logger().info(f'Received synchronized messages: {img_timestamp} and {pc_timestamp} and {bbox_msg.header.stamp}')
        
        
    def sync_callback(self, image_msg, pc_msg, bbox_msg):
        self.get_logger().info('Received synchronized messages')
        # Convert ROS image to OpenCV format
        frame = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        #Obtain pointcloud points in list format
        points = list(point_cloud2.read_points(pc_msg, field_names=["x", "y", "z"], skip_nans=True))
        # self.get_logger().info(f'Point cloud has {len(points)} points') # Debugging log to check number of points
        # self.get_logger().info(f'Points: {points[:5]}...')  # Log first 5 points for debugging
        if not points:
            self.get_logger().warn('No points in the point cloud message')
            return
        # Convert points to a NumPy array
        pointcloud = np.array([[p[0], p[1], p[2]] for p in points], dtype=np.float32)
        # self.get_logger().info(f'Point cloud shape: {pointcloud.shape}') # Debugging log to check shape of pointcloud
        bboxes = np.array(bbox_msg.array.data).reshape(-1, 4)

        # Project LiDAR points to the camera image
        projected_points, in_front_mask = self.project_lidar_to_image(pointcloud)
        pointcloud = pointcloud[in_front_mask]

        for bbox in bboxes:
            self.get_logger().info(f'Processing bounding box: {bbox}')
            # Extract bounding box coordinates (normalized)
            # Assuming bbox is in the format [x, y, width, height] normalized to [0, 1]
            # Convert normalized coordinates to pixel coordinates
            x, y, w, h = bbox
            # Convert normalized coordinates to pixel coordinates
            # The pixel coordinates are calculated by multiplying the normalized coordinates by the image dimensions
            pixel_x = int(x * frame.shape[1])
            pixel_y = int(y * frame.shape[0])
            pixel_w = int(w * frame.shape[1])
            pixel_h = int(h * frame.shape[0])

            # Find projected points within the bounding box
            mask = (projected_points[:, 0] >= pixel_x - pixel_w // 2) & \
                   (projected_points[:, 0] <= pixel_x + pixel_w // 2) & \
                   (projected_points[:, 1] >= pixel_y - pixel_h // 2) & \
                   (projected_points[:, 1] <= pixel_y + pixel_h // 2)
            # Extract points within the bounding box
            object_points = pointcloud[mask]

            # If no points are found in the bounding box, skip to the next bbox
            if len(object_points) == 0:
                self.get_logger().info(f'No LiDAR points found in bbox: {bbox}')
                continue

            # Calculate the average position of the points in the bounding box
            avg_xyz = np.mean(object_points, axis=0)
            x_avg, y_avg, z_avg = avg_xyz.tolist()

            ## TEST ##
            self.get_logger().info(f'Obstacle detected at: x={x_avg}, y={y_avg}, z={z_avg}')
            
            # # Create a PointStamped message for the obstacle position
            # point_cam = PointStamped()
            # point_cam.header.frame_id = self.target_frame
            # point_cam.point.x = x_avg
            # point_cam.point.y = y_avg
            # point_cam.point.z = z_avg

            # try:
            #     point_wamv = self.tf_buffer.transform(point_cam, 'wamv', timeout=rclpy.duration.Duration(seconds=0.5))
            #     self.pose_pub.publish(point_wamv)
            # except Exception as e:
            #     self.get_logger().warn(f'TF transform failed: {e}')

    def project_lidar_to_image(self, pointcloud):
        
        # Convert [X, Y, Z] coordinates from lidar to homogeneous coordinates
        points_hom = np.hstack((pointcloud, np.ones((pointcloud.shape[0], 1))))  # Nx4

        # Transform to camera frame using the extrinsic matrix
        points_cam = (self.lidar_to_camera_tf @ points_hom.T).T[:, :3]  # Nx3
        

        # Filter out points behind the camera
        in_front = points_cam[:, 2] > 0
        points_cam = points_cam[in_front]

        # Project points to image plane using intrinsic matrix (K)
        points_2d = (self.K @ points_cam.T).T  # Nx3

        # Normalize by depth (U/Z, V/Z) => (u,v)
        points_2d = points_2d[:, :2] / points_2d[:, 2:3]  # Nx2

        return points_2d.astype(int), in_front
        

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()
