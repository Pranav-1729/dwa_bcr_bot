import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import Odometry
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2 
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import time
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose

k = 25 #factor by which you have to randomize the 3d points to project in 2d 
goal = np.array([10.0,10.0])


# Robot parameters
max_vel = 0.5
min_vel = 0.001
max_omega = 1.0
min_omega = -1.0
vel_resolution = 0.01
omega_resolution = 0.1
dt = 0.01  # Time step for simulation
predict_time = 2.0  # Prediction horizon

# Cost weights
obstacle_weight = 0.5
goal_weight = 0.3
speed_weight = 0.2

best_score = float('inf')
best_v = 0.01
best_w = 0.01
# v = 0.01
# w = 0.01



v = np.arange(min_vel, max_vel, vel_resolution)
w = np.arange(min_omega, max_omega, omega_resolution)
# Create meshgrid of velocities
Vx, Vy = np.meshgrid(v, w)
velocity_pairs = np.column_stack((Vx.flatten(), Vy.flatten()))
########toacces pair, type vel_pair[i], for particular v and w, type vel_pair[i][0] and vel_pair[i][1]






class obstacle_vel_node(Node):
    def __init__(self):
        super().__init__('obstacle_vel_node')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/grouped_points',      ###to get the grouped points from the point_cloud_processor node
            self.obstacle_callback,
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
         
        # self.odom_sub = self.create_subscription(
        #     Odometry,
        #     '/odom',
        #     self.odom_callback,
        #     10)
        self.publisher1 = self.create_publisher(PointCloud2, '/obstacles', 10)
        self.publisher2 = self.create_publisher(Twist, '/bcr_bot/cmd_vel',10)
        self.get_logger().info("Obstacle and Vel node started.") 
        self.i_position =[0.0,0.0,0.0] 
      


    def get_current_pose_from_tf(self):
                # try:
                      trans = self.tf_buffer.lookup_transform('kinect_camera', 'base_footprint', rclpy.time.Time())
                      return trans.transform.translation, trans.transform.rotation
                # except Exception as e:
                #       self.get_logger().info(f'Could not get transform: {e}')


    def obstacle_callback(self,msg):
        obstacle_msg,obstacles = self.calc_obstacle(msg)
        self.publisher1.publish(obstacle_msg)
        self.get_logger().info("Obstacles detected and obstacle msg publishedddddddd")

        linear,angular = self.get_current_pose_from_tf()
        print("pose",angular, linear)
        x = linear.x
        y = linear.y
        qx = angular.x
        qy = angular.y
        qz = angular.z
        qw = angular.w
        euler = euler_from_quaternion([qx, qy, qz, qw])
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        current_pose = [x, y, yaw]# Create a numpy array of [x, y, orientation]
        self.i_position = current_pose

        vel_msg = Twist()
        print("current_pose",current_pose)
        vel_msg = velocity_command(current_pose,obstacles=obstacles)
        print("assssssssssssssssssssssssssssssssssssssssssssssssssvel_msg",vel_msg,"fkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk",msg)

        if vel_msg is not None:
            self.publisher2.publish(vel_msg)
        else:
            self.get_logger().error("Velocity message is None. Cannot publish.")
        #self.i_position = [x+v * math.cos(yaw) * dt, y+v * math.sin(yaw) * dt, yaw+w*dt]

    def calc_obstacle(self,msg):
          # msg is a PointCloud2 message
        points = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
        points = np.random.choice(points, points.shape[0] //k)
        points['y'] = 0.24

        #add obstacles to the obstacles array
        obstacle_array = [[p[0], p[1], p[2]] for p in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))]
        global obstacles
        obstacles = np.array(obstacle_array)

        header =msg.header
        fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        
        dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32)]
        structured_points = np.array(points, dtype=dtype)
        obstacle_cloud_msg = pc2.create_cloud(header, fields, structured_points)          
        # self.publisher.publish(grouped_cloud_msg)
        return obstacle_cloud_msg,obstacles

def euler_from_quaternion(quaternion):
    """
    Converts a quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw).
    
    :param quaternion: List or tuple of quaternion components [x, y, z, w]
    :return: Tuple (roll, pitch, yaw) in radians
    """
    x, y, z, w = quaternion
    
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))  # Clamp to avoid out of range errors
    
    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw





#this funcn return an array with all the points in predicted trajectry given a v and w
def predict_trajectory(posn, v, w):
######### v and w are apair of velocities
######### posn is the current position of the robot, x,y,orientation
        
        n_steps = int(predict_time / dt)

        x_start,y_start,theta_start = posn[0],posn[1],posn[2]
        x_inc = abs(v * math.cos(theta_start) * dt)
        # if theta_start<=0.01:
        #     y_inc =0.01
        # else:    
        #     y_inc = abs(v * math.sin(theta_start) * dt)
        y_inc = abs(v * math.sin(theta_start) * dt)

        if abs(y_inc) < 1e-6:  # Threshold for "zero"
            y_inc = 1e-6  # Set to a small non-zero value
        theta_inc = w*dt

        print("x_inc",x_inc)
        print("y_inc",y_inc)
        print("theta_inc",theta_inc)
        print("n_steps",n_steps)
        print("x_start",x_start)
        print("y_start",y_start)    
        print("theta_start",theta_start)
        print("v",v)
        print("w",w)
        print("dt",dt)
        print("predict_time",predict_time)
        ###new error
        
        x = np.arange(x_start, x_start + n_steps * x_inc, x_inc)
        y = np.arange(y_start, y_start + n_steps * y_inc, y_inc)
        theta = np.arange(theta_start, theta_start + n_steps * theta_inc, theta_inc)

        # Ensure all arrays have the same length
        min_length = min(len(x), len(y), len(theta))
        x = x[:min_length]
        y = y[:min_length]
        theta = theta[:min_length]

        # Create velocity arrays with the same length
        v_array = np.full(min_length, v)
        w_array = np.full(min_length, w)

        # v_array = np.tile(v, (n_steps,1))
        # w_array = np.tile(w, (n_steps,1))
        # v_array = v_array.flatten()  # Convert to shape (200,)
        # w_array = w_array.flatten()  # Convert to shape (200,)

        
        trajectory = np.column_stack((x, y, theta,v_array,w_array))
        # assert trajectory.shape == (n_steps, 5)
        # print(trajectory)
        return trajectory


#this funcn calculates the total cost of the trajectory
#v is current speed of the robot
def calculate_cost(trajectory,obstacles):
        v = trajectory[0,3]
        w = trajectory[0,4]
        #obstacles is an array of obstacles, each obstacle is a 2d point
        #trajectory is a np array - a collection of info at all points of that traj
        costs = np.empty((0, 3))  # 1 column for total costs and other 2 for v and w
        

        #this for loop will run for all points in the trajectory and store the three costs for all of em
        for i in range(trajectory.shape[0]):
                
                # x,y = trajectory[i,:2]
                x, y = np.array(trajectory[i, :2]).astype(float)
                if ValueError:
                    x,y = trajectory[i-1,:2]
                distances = np.sqrt((obstacles[:, 0] - x)**2 + (obstacles[:, 1] - y)**2)
                min_distance = np.min(distances)

                obstacle_cost = obstacle_weight / min_distance
                vel_cost = speed_weight /v

                dx = goal[0] - x
                dy = goal[1] - y
                dist_cost = np.hypot(dx, dy)
                goal_cost = goal_weight / dist_cost
                final_cost = goal_cost + obstacle_cost + vel_cost

                # Append costs for this point
                cost_row = np.array([[final_cost,v,w]])
                costs = np.vstack((costs, cost_row))  # each row has total cost of that point and the v and w values

        #sum costs for the whole trajectory 
        total_cost = np.sum(costs[:, 0])
        return total_cost


def best_trajectory(trajectories,obstacles):

        all_costs = np.array([calculate_cost(traj,obstacles) for traj in trajectories])
        
        min_index = np.argmin(all_costs)     # Index of the minimum value
        best_trajectory = trajectories[min_index]
        best_v = best_trajectory[0,3]
        best_w = best_trajectory[0,4]

        best_attrib = [best_v,best_w]
        return best_attrib


def velocity_command(posn,obstacles): 
        # Generate all possible trajectories
        trajectories = [predict_trajectory(posn, v, w) for v, w in velocity_pairs]
        # Find the best trajectory
        best_attrib = best_trajectory(trajectories,obstacles)

        # Convert best_attrib to a Twist message
        vel_msg = Twist()
        vel_msg.linear.x = best_attrib[0]  # Set linear velocity (best_v)
        vel_msg.angular.z = best_attrib[1]  # Set angular velocity (best_w)
        return vel_msg          

def main():
    rclpy.init()
    node = obstacle_vel_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
