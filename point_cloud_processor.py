import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2 
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import time

k = 10              #selects k random points from each 100 pts(sort of)
r = 5.0              #radius of visibility of the depth cam

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('point_cloud_processor')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/bcr_bot/kinect_camera/points',
            self.point_cloud_callback,
            10
        )
        self.publisher = self.create_publisher(PointCloud2, '/grouped_points', 10)
        self.get_logger().info("PointCloudProcessor node started.")        
    
    def publish_grouped_points(self, grouped_points):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "kinect_camera_optical"  # Update with your frame ID
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        # Define the correct dtype for the structured NumPy array
        dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32)]
        # Ensure `grouped_points` is converted to the structured array
        structured_points = np.array(grouped_points, dtype=dtype)
        grouped_cloud_msg = pc2.create_cloud(header, fields, structured_points)          
        self.publisher.publish(grouped_cloud_msg)
    
    def point_cloud_callback(self, msg):
        start=time.time()
        points = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
        points = np.random.choice(points, points.shape[0] //k)  
        filtered_points = self.filter_points_by_radius(points, radius=r)
        grouped_points = filtered_points
    
        self.publish_grouped_points(grouped_points)
        end=time.time()
        print("FUCKKKKKKKKKKKKKKKKKK", end-start)

    def filter_points_by_radius(self, points, radius):
        # Calculate the distance of each point from the origin (0, 0, 0)
        xyz_points = np.vstack((points['x'], points['y'], points['z'])).T
        distances = np.linalg.norm(xyz_points, axis=1)
        filtered_points = points[(distances <= radius) & ((points['y'] > 0.25) | (points['y'] < 0.23))]
        ###self.get_logger().info(f"Filtered points within {radius}m: {len(filtered_points)}")
        return filtered_points

def main():
    rclpy.init()
    node = PointCloudProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
