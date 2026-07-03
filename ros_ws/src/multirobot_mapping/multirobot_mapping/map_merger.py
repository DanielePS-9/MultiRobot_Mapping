# This code implements a ROS 2 node that merges the local maps of three robots into a single global map. 
# The node subscribes to the local maps published by each robot, transforms the coordinates of the known cells into a 
# common world frame, and publishes the merged map as an OccupancyGrid message. The merged map is dynamically sized 
# based on the explored areas of all robots.

import rclpy
import numpy as np

from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy

class CustomMapMerger(Node):
    def __init__(self):
        super().__init__('custom_map_merger')
        
        # Default spawn positions for the three robots
        self.declare_parameter('r1_x', -4.0)
        self.declare_parameter('r1_y', 0.0)
        self.declare_parameter('r2_x', -4.0)
        self.declare_parameter('r2_y', -1.0)
        self.declare_parameter('r3_x', -4.0)
        self.declare_parameter('r3_y', -2.0)
 
        self.poses = {
            'robot1': (float(self.get_parameter('r1_x').value), float(self.get_parameter('r1_y').value)),
            'robot2': (float(self.get_parameter('r2_x').value), float(self.get_parameter('r2_y').value)),
            'robot3': (float(self.get_parameter('r3_x').value), float(self.get_parameter('r3_y').value)),
        }
 
        # Initialize a dictionary to hold the latest local maps from each robot
        self.local_maps = {'robot1': None, 'robot2': None, 'robot3': None}
 
        self.create_subscription(OccupancyGrid, '/robot1/map', lambda msg: self.map_callback(msg, 'robot1'), 10)
        self.create_subscription(OccupancyGrid, '/robot2/map', lambda msg: self.map_callback(msg, 'robot2'), 10)
        self.create_subscription(OccupancyGrid, '/robot3/map', lambda msg: self.map_callback(msg, 'robot3'), 10)
 
        # Define a QoS profile with transient local durability for the merged map publisher
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        self.merged_map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        
        # Set up a map update period
        self.create_timer(1.0, self.merge_and_publish)
 
    # Callback function to store the latest local map from each robot
    def map_callback(self, msg, robot_name):
        self.local_maps[robot_name] = msg
 
    # Function to merge the local maps and publish the global map
    def merge_and_publish(self):
        if not any(self.local_maps.values()):
            return
 
        resolution = 0.05
        
        all_x_world = []
        all_y_world = []
        all_vals = []
 
        for r_name, l_map in self.local_maps.items():
            if l_map is None:
                continue
            
            spawn_x, spawn_y = self.poses[r_name]

            # Extract metadata from the local map
            l_res = l_map.info.resolution
            l_w = l_map.info.width
            l_h = l_map.info.height
            l_orig_x = l_map.info.origin.position.x
            l_orig_y = l_map.info.origin.position.y
            
            # Convert the flat data array into a 2D NumPy array
            local_grid = np.array(l_map.data, dtype=np.int8).reshape((l_h, l_w))
 
            # Find the indices of known cells in the local map
            j_indices, i_indices = np.where(local_grid != -1)
            vals = local_grid[j_indices, i_indices]
 
            if len(vals) == 0:
                continue
 
            x_local = l_orig_x + i_indices * l_res
            y_local = l_orig_y + j_indices * l_res
 
            x_world = spawn_x + x_local
            y_world = spawn_y + y_local
 
            all_x_world.append(x_world)
            all_y_world.append(y_world)
            all_vals.append(vals)
 
        if not all_x_world:
            return
 
        all_x = np.concatenate(all_x_world)
        all_y = np.concatenate(all_y_world)
        all_v = np.concatenate(all_vals)
 
        # Set the origin of the global map to the minimum x and y coordinates found
        g_origin_x = float(np.min(all_x))
        g_origin_y = float(np.min(all_y))
        
        # Convert the world coordinates to grid indices in the global map
        g_i_raw = ((all_x - g_origin_x) / resolution).astype(int)
        g_j_raw = ((all_y - g_origin_y) / resolution).astype(int)
 
        # Determine the dimensions of the global grid based on the maximum indices found
        g_width = int(np.max(g_i_raw) + 1)
        g_height = int(np.max(g_j_raw) + 1)
 
        # Initialize the global grid with unknown values (-1) 
        global_grid = np.full((g_height, g_width), -1, dtype=np.int8)
 
        # Overwrite the occupied cells (100).
        obs_mask = (all_v == 100)
        global_grid[g_j_raw[obs_mask], g_i_raw[obs_mask]] = 100
        
        # Overwrite the free cells (0).
        free_mask = (all_v == 0)
        global_grid[g_j_raw[free_mask], g_i_raw[free_mask]] = 0

        # Create and publish the merged OccupancyGrid message
        merged_msg = OccupancyGrid()
        merged_msg.header.stamp = self.get_clock().now().to_msg()
        merged_msg.header.frame_id = 'world'
        merged_msg.info.resolution = resolution
        merged_msg.info.width = g_width
        merged_msg.info.height = g_height
        merged_msg.info.origin.position.x = g_origin_x
        merged_msg.info.origin.position.y = g_origin_y
        merged_msg.data = global_grid.flatten().tolist()
        self.merged_map_pub.publish(merged_msg)
 
def main(args=None):
    rclpy.init(args=args)
    node = CustomMapMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()