#!/usr/bin/env python3

# Import libraries
import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

import struct

# Node class for processing WAM-V LiDAR data   
# The node subscribes to raw PointCloud2 data, 
# computes a timestamp for each point based on its azimuth angle,
# and republishes the enriched PointCloud2 with the added timestamp field.
class LidarPreprocessor(Node):
    def __init__(self):
        super().__init__('wamv_lidar_preprocessor')

        # Declare only the scan period parameter
        # Set default to 0.1 seconds (10 Hz)
        self.declare_parameter('scan_period', 0.1)
        self.scan_period = self.get_parameter('scan_period').get_parameter_value().double_value

        # Subscriber for raw PointCloud2 (fields: x, y, z, intensity, ring)
        self.lidar_subcriber = self.create_subscription(
            PointCloud2,
            '/wamv/sensors/lidars/lidar_wamv_sensor/points',
            self.pc_callback,
            10
        )

        # Publisher for enriched PointCloud2 (adds timestamp)
        self.enriched_points_pub = self.create_publisher(
            PointCloud2,
            '/wamv/sensors/lidars/lidar_wamv_sensor/points_enriched',
            10
        )
        
        self.get_logger().info('WAM-V LiDAR preprocessor node started.')

    def pc_callback(self, msg: PointCloud2):
        # Read x, y, z, intensity, ring from incoming cloud as list
        points = list(pc2.read_points(
            msg,
            field_names=('x', 'y', 'z', 'intensity', 'ring'),
            skip_nans=True
        ))
        n_pts = len(points)
        
        
        # xyzirt dtype
        # xyzirt_dtype = np.dtype([
        #     ('x', np.float32),
        #     ('y', np.float32),
        #     ('z', np.float32),
        #     ('intensity', np.float32),
        #     ('ring', np.uint16),
        #     ('time', np.float32),  # Time in seconds
        # ], align=True)
        
        # NOTE: Define output fields (with offsets in ascending order) as per above sanity check
        # NOTE: Aligned 'PointField' points struct of 24 bytes
        fields = [
        self.make_field('x',         0,  PointField.FLOAT32, 1),
        self.make_field('y',         4,  PointField.FLOAT32, 1),
        self.make_field('z',         8,  PointField.FLOAT32, 1),
        self.make_field('intensity', 12, PointField.FLOAT32, 1),
        self.make_field('ring',      16, PointField.UINT32, 1),
        self.make_field('time', 20, PointField.FLOAT32, 1),
        ]
        
        # Prepare custom pointcloud output format
        points_enriched = PointCloud2()
        points_enriched.header = msg.header
        points_enriched.height = 16
        points_enriched.width = 1875 # (Total: 30000 points)
        points_enriched.fields = fields
        points_enriched.is_bigendian = msg.is_bigendian
        
        point_step = 24
        points_enriched.point_step = point_step
        points_enriched.row_step = point_step * points_enriched.width
        points_enriched.is_dense = msg.is_dense
        
        # Pack each point into a bytearray using struct
        # Format: little-endian < 3×float32, 1×float32, 1×uint32, 1×float32 >
        fmt = '<fff f I f'
        buffer = bytearray(point_step * n_pts)
        offset = 0
        
        # Sanity check
        # print({name: xyzirt_dtype.fields[name][1] for name in xyzirt_dtype.names})
        # OUTPUT: {'x': 0, 'y': 4, 'z': 8, 'intensity': 12, 'ring': 16, 'timestamp': 18} ==> Offsets
 
        
        # Enrich points with azimuth-based timestamp
        # Azimuth is computed as atan2(y, x) and normalized to [0, 2π)
        # The timestamp is computed as (azimuth / 2π) * scan_period
        # This gives a timestamp in seconds for each point
        # The enriched points will be in the format [x, y, z, intensity, ring, time]
        # where timestamp is the time in seconds since the start of the scan
        # This is useful for synchronizing with other sensors or for time-based processing.
        # The azimuth is computed for each point, and the "per-point" times is derived from it.
        # The enriched points are then published as a new PointCloud2 message.
        # enriched_points = []
        two_pi = 2.0 * math.pi

        for i, (x, y, z, intensity, ring) in enumerate(points):
            # Compute azimuth and normalize to [0, 2π)
            az = math.atan2(y, x)
            if az < 0.0:
                az += two_pi

            # Time fraction of full scan (in seconds)
            time = (az / two_pi) * self.scan_period
            
            
            # # Sanity check for point values
            # # Check if point values are valid (not NaN, Inf, or None)
            # if any(math.isnan(v) or math.isinf(v) or None for v in (x, y, z, intensity, time)):
            #     # self.get_logger().warn(f"NaN/Inf/None point detected: {(x, y, z, intensity, ring, time)}")
            #     continue

            # # Append only valid points to enriched_points
            # # This ensures that we only process valid points and avoid issues late
            # enriched_points.append([x, y, z, intensity, ring, time])
            
            # Pack using struct
            struct.pack_into(fmt, buffer, offset,
                             x, y, z,
                             intensity, 
                             int(ring), 
                             time)
            offset += point_step
            
            
        # # Check for invalid values in enriched points
        # # This is a sanity check to ensure that the enriched points are valid
        # bad = []
        # for idx, (x,y,z,i,ring,t) in enumerate(enriched_points):
        #     # check for non-numeric
        #     if any(v is None for v in (x,y,z,i,ring,t)):
        #         bad.append((idx, 'None', (x,y,z,i,ring,t)))
        #         continue
        #     # check for NaN/Inf in floats
        #     if any((math.isnan(v) or math.isinf(v))
        #         for v in (x,y,z,i,t)):
        #         bad.append((idx, 'nan/inf', (x,y,z,i,ring,t)))
        #         continue
        #     # check ring in uint16 range
        #     if not (0 <= ring < 2**16):
        #         bad.append((idx, 'ring out of range', ring))
        #         continue

        # if bad:
        #     self.get_logger().warn(f"Found {len(bad)} bad points:")
        #     for info in bad[:10]:
        #         print(info) 
        # else:
        #     self.get_logger().info("All points are valid.")
        #     # print("Enriched points:", enriched_points[:10])  # Print first 10 enriched points for sanity check
            
        # # Convert enriched points to numpy array with the defined dtype
        # try:
        #     structured_points = np.array(enriched_points, dtype=xyzirt_dtype)
        #     self.get_logger().info(f"Converted {len(structured_points)} enriched points to structured array.")
        # except Exception as e:
        #     self.get_logger().error(f"Error converting enriched points to structured array: {e}")
        #     return
        
        
        
        
        # Build and publish the enriched PointCloud2
        # # self.get_logger().info(structured_points)
        # cloud_out = pc2.create_cloud(msg.header, 
        #                              fields, 
        #                              structured_points)
        
        points_enriched.data = bytes(buffer)
        
        self.enriched_points_pub.publish(points_enriched)
        self.get_logger().info(f'Published enriched PointCloud2 with {n_pts} points.')
        
    # def cloud_callback(self, raw: PointCloud2):
    #     scan_period = self.scan_period

    #     # 1) Read raw points (x, y, z, intensity, ring)
    #     pts = list(pc2.read_points(
    #         raw,
    #         field_names=('x', 'y', 'z', 'intensity', 'ring'),
    #         skip_nans=True
    #     ))
    #     n_pts = len(pts)

    #     # 2) Define fields with explicit byte offsets
    #     fields = [
    #         PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
    #         PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
    #         PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
    #         PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    #         PointField(name='ring',      offset=16, datatype=PointField.UINT32,  count=1),
    #         PointField(name='time',      offset=20, datatype=PointField.FLOAT32, count=1),
    #     ]

    #     # 3) Prepare the header & top-level cloud metadata
    #     cloud = PointCloud2()
    #     cloud.header = raw.header
    #     cloud.height = 1
    #     cloud.width = n_pts
    #     cloud.fields = fields
    #     cloud.is_bigendian = False

    #     point_step = 24               # 6 fields × 4 bytes each
    #     cloud.point_step = point_step
    #     cloud.row_step = point_step * n_pts
    #     cloud.is_dense = True

    #     # 4) Pack each point into a bytearray using struct
    #     #    Format: little-endian < 3×float32, 1×float32, 1×uint32, 1×float32 >
    #     fmt = '<fff f I f'
    #     buffer = bytearray(point_step * n_pts)
    #     offset = 0

    #     for idx, (x, y, z, intensity, ring) in enumerate(pts):
    #         # per-point timestamp between 0 and scan_period
    #         t = (idx / (n_pts - 1)) * scan_period if n_pts > 1 else 0.0

    #         struct.pack_into(fmt, buffer, offset,
    #                         x, y, z,
    #                         intensity,
    #                         int(ring),
    #                         t)
    #         offset += point_step

    #     cloud.data = bytes(buffer)

    #     # 5) Publish
    #     self.enriched_points_pub.publish(cloud)
    #     self.get_logger().info(f'Published enriched PointCloud2 with {n_pts} points.')

        
    def make_field(self, name, offset, datatype, count):
        f = PointField()
        f.name     = name
        f.offset   = offset
        f.datatype = datatype
        f.count    = count
        return f

    



def main(args=None):
    rclpy.init(args=args)
    node = LidarPreprocessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
