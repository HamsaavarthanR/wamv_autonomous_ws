#!/usr/bin/env python3

# Import necessary libraries
import os
import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
# from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from rclpy.clock import Clock

from autonomous_stack_interfaces.msg import StampedFloat64MultiArray


# Maritime Obstacle Detection Node
# This node uses a custom YOLOv8 model to detect obstacles in the WAM-V's front left camera feed.
# Detected obstacles are highlighted in the image and their centers are published as normalized coordinates.
class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        
        # Declare parameters
        self.declare_parameter('MODEL_PATH', '')
        MODEL_PATH = self.get_parameter('MODEL_PATH').get_parameter_value().string_value
        # Check if model file exists and is valid
        if (not os.path.exists(MODEL_PATH)):
            self.get_logger().warn(f'ERROR: Model path "{MODEL_PATH}" is invalid or model was not found. Make sure the model filename was entered correctly.')
            sys.exit(0)
        
        # Use sim time
        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value

        # Load custom YOLOv8 model
        self.model = YOLO(MODEL_PATH, task='detect')
        self.labels = self.model.names
        self.bridge = CvBridge()
        self.color_palette = [(164,120,87), (68,148,228), (93,97,209), (178,182,133),
                              (88,159,106), (96,202,231), (159,124,168), (169,162,241),
                              (98,118,150), (172,176,184)]

        # Camera Raw Image Subscriber
        self.left_camera_image_sub = self.create_subscription(
            Image,
            '/wamv/sensors/cameras/front_left_camera_sensor/image_raw',
            self.process_left_camera,
            10
        )
        #Store camera image time stamp
        self.left_camera_image_stamp = None

        # Publishers for processed image
        self.image_pub = self.create_publisher(
            Image,
            '/wamv/sensors/cameras/front_left_camera_sensor/obstacle_image',
            10
        )

        # Publisher for obstacle centers
        self.center_pub = self.create_publisher(
            StampedFloat64MultiArray,
            '/wamv/sensors/cameras/front_left_camera_sensor/obstacle_centres',
            10
        )

        self.get_logger().info('Maritime Obstacle Detector Initialized')

    # Process the left camera image (raw)
    def process_left_camera(self, msg):
        # Store the timestamp of the image
        self.left_camera_image_stamp = msg.header.stamp
        
        # Convert ROS Image message to OpenCV format
        # Use the CvBridge to convert the ROS Image message to OpenCV format
        # This is necessary for the YOLO model to process the image
        # The desired encoding is 'bgr8' for OpenCV compatibility
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image_height, image_width = frame.shape[:2]
        center_array = StampedFloat64MultiArray()

        # Run inference on frame
        results = self.model(frame, verbose=False)

        # Extract results
        detections = results[0].boxes

        # Go through each detection and get bbox coords, confidence, and class
        for i in range(len(detections)):

            # Get bounding box coordinates
            # Ultralytics returns results in Tensor format, which have to be converted to a regular Python array
            xyxy_tensor = detections[i].xyxy.cpu() # Detections in Tensor format in CPU memory
            xyxy = xyxy_tensor.numpy().squeeze() # Convert tensors to Numpy array
            xmin, ymin, xmax, ymax = xyxy.astype(int) # Extract individual coordinates and convert to int

            # Obtain Obstacle center coordinates
            x_center = (xmin + xmax) / 2
            y_center = (ymin + ymax) / 2
            # Normalize coordinates to [0, 1] range
            x_norm = x_center / image_width
            y_norm = y_center / image_height
            # Bouding box width and height normalized
            width_norm = (xmax - xmin) / image_width
            height_norm = (ymax - ymin) / image_height

            # Get bounding box class ID and name
            classidx = int(detections[i].cls.item())
            classname = self.labels[classidx]

            # Get bounding box confidence
            conf = detections[i].conf.item()

            # Draw box if confidence threshold is high enough
            if conf > 0.2:

                color = self.color_palette[classidx % len(self.color_palette)]
                cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), color, 2)

                label = f'{classname}: {int(conf*100)}% [{x_center:.2f}, {y_center:.2f}]'
                # Get font size
                labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1) 
                # Make sure not to draw label too close to top of window
                label_ymin = max(ymin, labelSize[1] + 10) 
                # Draw white box to put label text in
                cv2.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), color, cv2.FILLED) 
                # Draw label text
                cv2.putText(frame, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1) 

                # Populate center_array properly for future time synchronization
                # Add center coordinates to the array
                center_array.array.data.extend([round(x_norm, 4), round(y_norm, 4), round(width_norm, 4), round(height_norm, 4)])


        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
        if self.left_camera_image_stamp is not None:
            # Set the header for the center_array message
            center_array.header.stamp = self.left_camera_image_stamp
        else:
            # If the timestamp is not set, use the current time
            center_array.header.stamp = Clock().now().to_msg()
        center_array.header.frame_id = 'wamv/sensors/cameras/front_left_camera_sensor'
        self.center_pub.publish(center_array)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
