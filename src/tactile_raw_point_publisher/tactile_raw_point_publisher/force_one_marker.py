#!/usr/bin/env python3
import math
import struct
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray

class TactileVisualizer(Node):
    def __init__(self):
        super().__init__('tactile_visualizer')
        self.force_threshold = -100

        self.sub_ = self.create_subscription(
            PointCloud2,
            '/tactile/sensor_1/raw_point_data',
            self.cb,
            10
        )
        self.sub2_ = self.create_subscription(
            PointCloud2,
            '/tactile/sensor_3/raw_point_data',
            self.cb,
            10
        )

        self.pub_ = self.create_publisher(
            PointCloud2,
            '/tactile/sensor_1/vis_cloud',
            10
        )
        self.pub_value_ = self.create_publisher(
            Float32MultiArray,
            '/tactile/sensor_1/combined',
            10,
        )
        

    def cb(self, msg):
        # Read original values (x,y,z are actually forces)
        points = list(point_cloud2.read_points(msg, field_names=('x','y','z'), skip_nans=True))

        vis_points = []
        combined_values = []
        

        for i, (x, y, z) in enumerate(points):
            # Example: put all 116 points in a 1-row grid
            px = float(i) * 0.00005    # 1 cm per taxel
            py = 0.0
            pz = float(i) * 0.00005

            # Use intensity = magnitude of force vector
            intensity = math.sqrt(x*x + y*y + z*z)
            
            if intensity<=self.force_threshold:
                continue

            # Pack new point: (x,y,z,intensity)
            vis_points.append([px, py, pz, intensity])
            
            value = math.sqrt(x * x + y * y + z * z)
            
            combined_values.append(float(value))

        # Create PointCloud2 with intensity channel
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        vis_msg = point_cloud2.create_cloud(msg.header, fields, vis_points)
        self.pub_.publish(vis_msg)

        msg_out = Float32MultiArray()
        msg_out.data=combined_values
        self.pub_value_ .publish(msg_out)
        

def main(args=None):
    rclpy.init(args=args)
    node = TactileVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
