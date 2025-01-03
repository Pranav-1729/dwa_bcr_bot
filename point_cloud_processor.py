import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2 
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import time

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
        grouped_cloud_msg = pc2.create_cloud(header, fields, grouped_points)
        
        self.publisher.publish(grouped_cloud_msg)

    def filter_points_by_radius(self, points, radius):

        # Extract 'x', 'y', and 'z' columns into a regular 2D numpy array
        xyz_points = np.column_stack((points['x'], points['y'], points['z']))

        # Calculate the distance of each point from the origin (0, 0, 0)
        distances = np.linalg.norm(xyz_points, axis=1)
        # Filter points based on the distance being within the specified radius
        filtered_points = points[distances <= radius]
        # Log the number of points after filtering
        self.get_logger().info(f"Filtered points within {radius}m: {len(filtered_points)}")
        return filtered_points
    
    def point_cloud_callback(self, msg):
        start=time.time()
        points = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
        points = np.random.choice(points, points.shape[0] // 10)  
        filtered_points = self.filter_points_by_radius(points, radius=5.0)

        # Group nearby points
        global grouped_points
        grouped_points = self.group_points(filtered_points, cell_size=0.03)
    
        self.publish_grouped_points(grouped_points)
        end=time.time()
        print("FUCKKKKKKKKKKKKKKKKKK", end-start)
        # Log or publish the grouped points (for now, we just log the count)
        #self.get_logger().info(f"Processed {len(grouped_points)} grouped points.")
    
    def group_points(self, points, cell_size):
        
        # Assuming `points` is the output of ros2_numpy.point_cloud2.pointcloud2_to_array(msg)
        #points_array = np.stack((points['x'], points['y'], points['z']), axis=-1)
        # Quantize points to grid indices
        #Assuming `points` is a structured numpy array with fields 'x', 'y', and 'z'
        k=100/4
        xyz_points = np.column_stack((k*points['x'], k*points['y'],k* points['z']))
    
         # Apply floor function to the points
        floored_points = np.floor(xyz_points)
        floored_points = floored_points/k

        # Perform the division and quantization
        # quantized_x = np.floor(x / cell_size).astype(int)
        # quantized_y = np.floor(y / cell_size).astype(int)
        # quantized_z = np.floor(z / cell_size).astype(int)
        # # Optionally, you can stack them back together to get the quantized point coordinates
        # quantized = np.stack((quantized_x, quantized_y, quantized_z), axis=-1)
        
        # # Use a dictionary to group points by grid index
        # grouped = {}
        # for i, point in enumerate(points):
        #     key = tuple(quantized[i])  # Grid cell index
        #     if key not in grouped:
        #         grouped[key] = []
        #     grouped[key].append(point)
        
        # # Calculate representative points (e.g., mean of each group)
        # representative_points = []    
        # for group in grouped.values():
        #     group_np = np.array(group)  # Convert list of points to numpy array
            
        #     # Extract individual fields (x, y, z) from the structured array
        #     x = group_np['x']
        #     y = group_np['y']
        #     z = group_np['z']
            
        #     # Calculate the mean of each field (x, y, z)
        #     mean_x = x.mean()
        #     mean_y = y.mean()
        #     mean_z = z.mean()
            
        #     # Store the representative point as the mean of x, y, and z
        #     representative_points.append([mean_x, mean_y, mean_z])

        # Log the number of points and groups for debugging
        #self.get_logger().info(f"Input points: {len(points)}, Grouped points: {len(grouped)}") 
        # Return the list of representative points as a numpy array
        # return np.array(representative_points)
        return floored_points

def main():
    rclpy.init()
    node = PointCloudProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
