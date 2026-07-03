# This code implements a ROS 2 node that coordintes the deterministic exploration of a swarm of three robots in a Gazebo simulation.
# The node subscribes to the merged global map, identifies unexplored frontiers, and assigns them to idle robots.
# It also monitors the robots's progress, detects if they are stuck, and reassigns goals as necessary. 

import rclpy
import subprocess
import sys
import math
import numpy as np
import scipy.ndimage as ndimage
import heapq 
import tf2_geometry_msgs  

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import GoalStatus
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time
from visualization_msgs.msg import Marker, MarkerArray
 
class DeterministicExplorer(Node):
    def __init__(self):
        super().__init__('deterministic_explorer')

        self.merged_map_sub = self.create_subscription(OccupancyGrid,'/map',self.map_callback,10)
        self.frontier_marker_pub = self.create_publisher(MarkerArray,'/frontier_markers',10)

        # Initialize TF buffer and listener for global transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.merged_map = None
        self.robots = ['robot1', 'robot2', 'robot3']

        # Initialize a variable necessary for map_callback
        self.initial_trigger_done = False

        self.start_time = None

        self.total_distances = {robot: 0.0 for robot in self.robots}
        self.tracking_last_poses = {robot: None for robot in self.robots}
        
        # Initialize a timer to track distances every second
        self.distance_tracker_timer = self.create_timer(1.0, self.track_distances)

        # Initialize a timer to update frontiers markers every second
        self.viz_timer = self.create_timer(1.0, self.visualization_tick)
        
        # Initialize dictionaries to manage robots
        self.action_clients = {}    # It will hold the ActionClient for each robot
        self.robot_status = {}      # It will hold the status of each robot: 'IDLE' or 'NAVIGATING'
        self.assigned_goals = {}    # It will hold the currently assigned goal for each robot
        self.last_robot_poses = {}  # It will hold the last known position for each robot
        self.stuck_counters = {}    # It will hold the counter for each robot to detect if it's stuck
        
        # Initialize the robot management dictionaries
        for robot_name in self.robots:
            action_name = f'/{robot_name}/navigate_to_pose'
            self.action_clients[robot_name] = ActionClient(self, NavigateToPose, action_name)
            self.robot_status[robot_name] = 'IDLE'
            self.assigned_goals[robot_name] = None
            self.last_robot_poses[robot_name] = None
            self.stuck_counters[robot_name] = 0

        self.get_logger().info(f"\033[94mMultiRobotExplorer node initialized. Waiting for the first merged map and TF tree to be ready...\033[0m")
       
        # Initialize a timer to check for robot stalling every 4 seconds
        self.stall_checker_timer = self.create_timer(4.0, self.check_robots_stall)

        # Initialize variables for the exploration watchdog
        self.no_frontiers_ticks = 0
        self.max_ticks_to_stop = 6

        # Initialize a timer to monitor exploration progress every 5 seconds
        self.watchdog_timer = self.create_timer(5.0, self.exploration_watchdog)


    # Functions to update frontiers markers
    def visualization_tick(self):
        if self.merged_map is None:
            return
        self.publish_frontier_markers(self.extract_frontiers())
 

    def publish_frontier_markers(self, candidate_points):
        marker_array = MarkerArray()
    
        # Clear all the old frontiers markers
        clear = Marker()
        clear.header.frame_id = 'world'
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)
    
        now = self.get_clock().now().to_msg()
    
        # Show the candidates frontiers points (Cyan Points)
        for idx, (fx, fy) in enumerate(candidate_points):
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = now
            m.ns = 'frontiers'
            m.id = idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(fx)
            m.pose.position.y = float(fy)
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.1
            m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 1.0, 0.9
            marker_array.markers.append(m)
    
        gid = 0
        for robot, goal in self.assigned_goals.items():
            if goal is None:
                continue

            # Show the goals frontiers points (Red Points)
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = now
            m.ns = 'assigned_goals'
            m.id = gid; gid += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(goal[0])
            m.pose.position.y = float(goal[1])
            m.pose.position.z = 0.15
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.2
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 1.0
            marker_array.markers.append(m)

            # Show the robots' names on the goals points
            text = Marker()
            text.header.frame_id = 'world'
            text.header.stamp = now
            text.ns = 'goal_labels'
            text.id = gid; gid += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(goal[0])
            text.pose.position.y = float(goal[1])
            text.pose.position.z = 0.5
            text.pose.orientation.w = 1.0
            text.scale.z = 0.15   
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = robot
            marker_array.markers.append(text)
    
        self.frontier_marker_pub.publish(marker_array)


    # Callback function to handle incoming merged map messages
    def map_callback(self, msg):
        self.merged_map = msg
        
        # Check if this is the first merged map received and if the global TF tree is ready
        if not self.initial_trigger_done:
            if self.tf_buffer.can_transform('world', f'{self.robots[0]}/base_footprint', rclpy.time.Time()):
                self.initial_trigger_done = True

                # Save the start time of the exploration
                self.start_time = self.get_clock().now()
                self.get_logger().info(f"\033[94mFirst merged map received and TF tree is ready! Starting goal assignment...\033[0m")
                self.coordinator()

            else:
                self.get_logger().info(f"\033[93mMap received, but the TF tree is not yet fully connected. Waiting...\033[0m",
                                       throttle_duration_sec=2.0)
 

    # Function to track the distance traveled by each robot
    def track_distances(self):
        if not self.initial_trigger_done:
            return

        for robot in self.robots:
            current_x, current_y = self.get_robot_pose(robot)
            if current_x is None or current_y is None:
                continue
            
            last_pose = self.tracking_last_poses[robot]
            if last_pose is not None:
                dist = self.euclidean_distance((current_x, current_y), last_pose)

                # Exclude very small movements
                if dist > 0.01: 
                    self.total_distances[robot] += dist
                    
            self.tracking_last_poses[robot] = (current_x, current_y)
 

    # Function to convert Euler angles to quaternion
    def get_quaternion_from_euler(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]
       

    # Function to calculate the Euclidean distance between two points
    def euclidean_distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
 

    # Function to get the robot's current pose in the global world frame
    def get_robot_pose(self, robot_name):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'world', 
                f'{robot_name}/base_footprint', 
                now
            )
            global_x = trans.transform.translation.x
            global_y = trans.transform.translation.y
            return global_x, global_y
            
        except TransformException as e:
            self.get_logger().error(f"\033[91mPose not available for {robot_name}: {str(e)}\033[0m", throttle_duration_sec=5.0)
            return None, None
 

    # Function to compute the path cost from a starting point to a goal point considering the other robots positions 
    def compute_path_cost(self, start_w, goal_w, other_robots_positions):
        if self.merged_map is None: return float('inf')

        # Extract map dimensions and resolution from the merged map
        res = self.merged_map.info.resolution
        ox = self.merged_map.info.origin.position.x
        oy = self.merged_map.info.origin.position.y
        width = self.merged_map.info.width
        height = self.merged_map.info.height
       
        # Compute the pixels referred to the start and goal coordinates
        sx, sy = int((start_w[0] - ox) / res), int((start_w[1] - oy) / res)
        gx, gy = int((goal_w[0] - ox) / res), int((goal_w[1] - oy) / res)
       
        if not (0 <= sx < width and 0 <= sy < height and 0 <= gx < width and 0 <= gy < height):
            return float('inf')
 
        # Reshape the 1D map data array into a 2D grid 
        grid = np.array(self.merged_map.data, dtype=np.int8).reshape((height, width))

        # Compute the pixels referred to the other robots coordinates
        other_pixels = [(int((rx - ox)/res), int((ry - oy)/res)) for rx, ry in other_robots_positions]
        
        # Define a social radius converted to pixels to keep robots apart
        social_radius_px = 1.0 / res
 
        # Initialize the frontier set for the A* algorithm and set the starting cost
        frontier_set = []
        heapq.heappush(frontier_set, (0, (sx, sy)))
        g_score = {(sx, sy): 0}
        directions = [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]
       
        while frontier_set:
            _, current = heapq.heappop(frontier_set)

            # If the goal is reached, return the accumulated path cost converted back to meters
            if current == (gx, gy): return g_score[current] * res
           
            for dx, dy in directions:
                nx, ny = current[0] + dx, current[1] + dy

                if 0 <= nx < width and 0 <= ny < height:
                    cell_val = grid[ny, nx]
                    
                    # Skip the cell if it is an obstacle
                    if cell_val == 100:
                        continue
                   
                    # Calculate the base step cost
                    step_cost = math.sqrt(dx**2 + dy**2)
                   
                    # Heavily penalize moving through unknown space to prioritize known clear paths
                    if cell_val == -1:
                        step_cost *= 20.0
                   
                    # Add a social cost penalty if the cell is within the social radius of other robots
                    for orx, ory in other_pixels:
                        if math.sqrt((nx - orx)**2 + (ny - ory)**2) < social_radius_px:
                            step_cost += 10.0
                            break
                           
                    tentative_g = g_score[current] + step_cost
                    neighbor = (nx, ny)
                    
                    # Update the path cost if a cheaper path is found, and push to the frontier set
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        g_score[neighbor] = tentative_g

                        # Compute the Euclidean heuristic from the neighbor to the goal
                        h = self.euclidean_distance([nx, ny], [gx, gy])
                        heapq.heappush(frontier_set, (tentative_g + h, neighbor))

        return float('inf')
 

    # Function to extract the frontiers points in the merged map
    def extract_frontiers(self):
        if self.merged_map is None: return []
    
        # Extract map dimensions and resolution from the merged map
        width  = self.merged_map.info.width
        height = self.merged_map.info.height
        res = self.merged_map.info.resolution
        ox  = self.merged_map.info.origin.position.x
        oy  = self.merged_map.info.origin.position.y

        # Set the minimum size of accettable frontier
        min_frontier_size = 12

        # Reshape the 1D map data array into a 2D grid
        map_grid = np.array(self.merged_map.data, dtype=np.int8).reshape((height, width))

        # Define a 3x3 structuring element for morphological operations (8-connectivity)
        struct = ndimage.generate_binary_structure(2, 2)

        # Identify raw frontiers (free space adjacent to unknown space)
        frontier_mask = (map_grid == 0) & ndimage.binary_dilation((map_grid == -1), structure=struct)

        # Increase the thickness of the obstacles to avoid extracting frontiers too close to walls
        dilated_obstacles = ndimage.binary_dilation((map_grid == 100), iterations=7)

        # Remove dilated_obstacles from the frontier_mask 
        frontier_mask = frontier_mask & (~dilated_obstacles)
    
        # Group contiguous frontier pixels into separate labeled clusters
        labeled_array, num_features = ndimage.label(frontier_mask, structure=struct)
        centroids = []
    
        for i in range(1, num_features + 1):
            y_idx, x_idx = np.where(labeled_array == i)
            cluster_size = len(x_idx)

            # Discard clusters that are too small to be considered real frontiers
            if cluster_size < min_frontier_size:
                continue
    
            # Compute a proportional number of target points to assign based on the cluster length
            num_points = max(1, min(30, cluster_size // 20))
    
            if num_points == 1:
                # Compute the centroid of the cluster
                mx, my = x_idx.mean(), y_idx.mean()

                # Find the actual frontier pixel closest to the mathematical centroid to avoid targeting walls
                j = np.argmin((x_idx - mx)**2 + (y_idx - my)**2)
                reps = [(int(x_idx[j]), int(y_idx[j]))]
            else:
                # If the frontier is long, split it along its major axis
                span_x = x_idx.max() - x_idx.min()
                span_y = y_idx.max() - y_idx.min()

                # Determine the longest axis to sort the pixels
                key = x_idx if span_x >= span_y else y_idx
                order = np.argsort(key)
    
                reps = []
                for chunk in np.array_split(order, num_points):
                    cxs, cys = x_idx[chunk], y_idx[chunk]

                    # Compute the centroid of each chunk
                    mx, my = cxs.mean(), cys.mean()

                    # Find the actual frontier pixel closest to the chunk's centroid
                    j = np.argmin((cxs - mx)**2 + (cys - my)**2)  # pixel reale, mai in mezzo a un muro
                    reps.append((int(cxs[j]), int(cys[j])))

            # Convert the selected pixel coordinates back to the global map reference frame
            for cx, cy in reps:
                centroids.append((ox + cx * res, oy + cy * res))
    
        return centroids
 

    # Function to coordinate the assignment of frontiers
    def coordinator(self):
        # 
        repulsion_radius = 1.0

        if not self.initial_trigger_done or self.merged_map is None:
            return
 
        idle_robots = [r for r in self.robots if self.robot_status[r] == 'IDLE']
        if not idle_robots: return
 
        raw_frontiers = self.extract_frontiers()
        if not raw_frontiers:
            self.get_logger().info("🔍 Nessuna frontiera fisica estratta (Mappa chiusa).", throttle_duration_sec=5.0)
            return
 
        frontier_centroids = []
        for fx, fy in raw_frontiers:
            too_close = False
            for r in self.robots:
                pose = self.get_robot_pose(r)
                # Il robot pulirà le frontiere a più di 40 cm da sé per evitare stalli infiniti
                if pose[0] is not None and self.euclidean_distance((pose[0], pose[1]), (fx, fy)) < 0.4:
                    too_close = True
                    break
            if not too_close:
                frontier_centroids.append((fx, fy))
 
        if not frontier_centroids:
            self.get_logger().info(f"🔍 Trovate {len(raw_frontiers)} frontiere, ma tutte a meno di 40cm dai robot. Attendo espansione...", throttle_duration_sec=5.0)
            return
 
        ready_to_dispatch = []
        all_poses = []
        for r in self.robots:
            p = self.get_robot_pose(r)
            if p[0] is not None: all_poses.append((p[0], p[1]))
 
        for robot in idle_robots:
            rx, ry = self.get_robot_pose(robot)
            if rx is None: continue
 
            best_frontier = None
            min_cost = float('inf')
 
            active_goals = [goal for r, goal in self.assigned_goals.items() if goal is not None and r != robot]
            active_goals.extend([(gx, gy) for _, gx, gy in ready_to_dispatch])
 
            for fx, fy in frontier_centroids:
                other_poses = [p for p in all_poses if p != (rx, ry)]
                path_cost = self.compute_path_cost((rx, ry), (fx, fy), other_poses)
               
                if path_cost == float('inf'): continue
               
                cost = path_cost
                for ax, ay in active_goals:
                    if self.euclidean_distance((fx, fy), (ax, ay)) < repulsion_radius:
                        cost += 5000.0  
 
                if cost < min_cost:
                    min_cost = cost
                    best_frontier = (fx, fy)
 
            if best_frontier is not None and min_cost < 5000.0:
                self.assigned_goals[robot] = best_frontier
                ready_to_dispatch.append((robot, best_frontier[0], best_frontier[1]))
                frontier_centroids.remove(best_frontier)
            else:
                self.get_logger().info(f"🚧 [{robot}] Frontiere trovate, ma TUTTE irraggiungibili (A* bloccato dai muri).", throttle_duration_sec=3.0)
 
        for robot_name, gx, gy in ready_to_dispatch:
            self.dispatch_robot(robot_name, gx, gy)
 

    # Function to dispatch a robot to a specific goal
    def dispatch_robot(self, robot_name, x, y):
        # Check if the action server for the robot is ready
        if not self.action_clients[robot_name].wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(f"\033[93m[{robot_name}] Action server not ready. Cannot dispatch goal.\033[0m")
            return
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'world'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        last_pose = self.last_robot_poses.get(robot_name)
        rx = last_pose[0] if last_pose is not None else 0.0
        ry = last_pose[1] if last_pose is not None else 0.0
        yaw = math.atan2(y - ry, x - rx)

        # Assign a random orientation to the robot for exploration purposes
        q = self.get_quaternion_from_euler(0.0, 0.0, yaw)
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        self.robot_status[robot_name] = 'NAVIGATING'
        self.assigned_goals[robot_name] = (x, y)
        self.get_logger().info(f"\033[94m[{robot_name}] Dispatching to goal: ({x:.2f}, {y:.2f})\033[0m")
        
        # Send the goal asynchronously and set up a callback for the response
        future = self.action_clients[robot_name].send_goal_async(goal_msg)
        future.add_done_callback(lambda fut, r=robot_name: self.goal_response_callback(fut, r))

 
    # Function to check if any robot is stuck and force a goal change if necessary
    def check_robots_stall(self):
        for robot in self.robots:
            if self.robot_status[robot] == 'NAVIGATING':
                current_x, current_y = self.get_robot_pose(robot)
                
                if current_x is None or current_y is None:
                    continue
                
                last_pose = self.last_robot_poses[robot]
                
                if last_pose is not None:
                    # Calculate the distance moved since the last check
                    dist_moved = self.euclidean_distance((current_x, current_y), last_pose)
                    
                    # If the robot has moved less than 0.15 meters, consider it stuck and force a goal change
                    if dist_moved < 0.15:
                        self.get_logger().error(f"\033[91m[{robot}] appears to be stuck. Forcing a new goal assignment.\033[0m")
                        self.reset_stuck_robot(robot)
                        continue 
                
                self.last_robot_poses[robot] = (current_x, current_y)
            else:
                self.last_robot_poses[robot] = None
    
    # Function to reset the state of a stuck robot and request a new global goal
    def reset_stuck_robot(self, robot_name):
        # Reset the robot's state to IDLE and clear its assigned goal
        self.robot_status[robot_name] = 'IDLE'
        self.assigned_goals[robot_name] = None
        self.stuck_counters[robot_name] = 0
        self.last_robot_poses[robot_name] = None
        # Request a new global goal for the robot by calling the coordinator
        self.coordinator()
 
    # Callback function to handle the response from the action server when a goal is sent
    def goal_response_callback(self, future, robot_name):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"\033[93m[{robot_name}] Goal rejected by the action server. Applying cooldown...\033[0m")
            timer = []
            timer.append(self.create_timer(2.0, lambda: self.cooldown_wrapper(robot_name, timer)))
            return

        # If the goal is accepted, set up a callback to handle the result when the robot reaches its destination or fails
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut, r=robot_name: self.get_result_callback(fut, r))

    # Callback function to handle the result of the navigation goal
    def get_result_callback(self, future, robot_name):
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"\033[92m[{robot_name}] Navigation succeeded! Ready for a new frontier.\033[0m")
            self.robot_status[robot_name] = 'IDLE'
            self.assigned_goals[robot_name] = None
            self.coordinator()
        else:
            self.get_logger().warn(f"\033[91m[{robot_name}] Navigation failed. Applying cooldown...\033[0m")
            self.assigned_goals[robot_name] = None
            timer = []
            timer.append(self.create_timer(0.2, lambda: self.cooldown_wrapper(robot_name, timer)))
 

    # Function to handle the cooldown period after a robot fails to reach its goal
    def cooldown_wrapper(self, robot_name, timer_list):
        if timer_list and timer_list[0]:
            timer_list[0].destroy() 
        
        self.robot_status[robot_name] = 'IDLE'
        self.get_logger().info(f"\033[94m[{robot_name}] Cooldown complete. Ready for a new frontier.\033[0m")
        self.coordinator()
 
    # Function to monitor the exploration process and determine if it should be terminated
    def exploration_watchdog(self):
        if self.merged_map is None:
            return 
            
        # Check if all robots are idle
        all_robots_idle = all(status != 'NAVIGATING' for status in self.robot_status.values())

        # Check if all robots are idle and there are no frontiers available
        if all_robots_idle and not len(self.extract_frontiers()) == 0:
            # Increment the counter for ticks with no frontiers
            self.no_frontiers_ticks += 1
            self.get_logger().info(f"\033[94mNo frontiers available. Shutting down... ({self.no_frontiers_ticks}/{self.max_ticks_to_stop})\033[0m")
            
            # If the number of ticks with no frontiers exceeds the maximum allowed, terminate the exploration
            if self.no_frontiers_ticks >= self.max_ticks_to_stop:
                self.terminate_exploration()
        else:
            # Reset the counter for ticks with no frontiers if there are frontiers available or robots are navigating
            self.no_frontiers_ticks = 0
            
            # If there are frontiers available and any robot is idle, call the coordinator to assign new goals
            if len(self.extract_frontiers()) == 0 and any(status == 'IDLE' for status in self.robot_status.values()):
                self.coordinator()
 
    # Function to terminate the exploration process, save the merged map, print metrics, and shut down the node
    def terminate_exploration(self):
        if self.start_time is not None:
            end_time = self.get_clock().now()
            # Calculate the total duration of the exploration
            total_duration = end_time - self.start_time 
            
            duration_secs = total_duration.nanoseconds / 1e9
            mins = int(duration_secs // 60)
            secs = int(duration_secs % 60)
            
            # Calculate the total distance traveled by the swarm of robots
            total_swarm_distance = sum(self.total_distances.values())
            
            self.get_logger().info(f"\033[92m=====================================================\033[0m")
            self.get_logger().info(f"\033[92mTotal duration: {mins} min and {secs} sec ({duration_secs:.2f} s)\033[0m")
            self.get_logger().info(f"\033[92mDistance traveled:\033[0m")
            for robot in self.robots:
                self.get_logger().info(f"\033[92m   - {robot}: {self.total_distances[robot]:.2f} meters\033[0m")
            self.get_logger().info(f"\033[92mTotal swarm distance: {total_swarm_distance:.2f} meters\033[0m")
            self.get_logger().info(f"\033[92m=====================================================\033[0m")

        self.get_logger().info(f"\033[92mExploration Complete. All accessible areas have been mapped.\033[0m")
        self.get_logger().info(f"\033[94mInitiating automatic map saving...\033[0m")
        
        # Use subprocess to call the ROS 2 map_saver_cli command to save the merged map
        try:
            subprocess.run(
                [
                    "ros2", "run", "nav2_map_server", "map_saver_cli", 
                    "-f", "deterministic_swarm_map", 
                    "--ros-args", 
                    "-p", "use_sim_time:=true",
                    "-p", "map_subscribe_transient_local:=true"
                ],
                check=True
            )
            self.get_logger().info(f"\033[92mMap 'deterministic_swarm_map.yaml' and '.pgm' saved successfully!\033[0m")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"\033[91mCritical error occurred while saving the map: {e}\033[0m")
            
        self.get_logger().info(f"\033[94mClosing Swarm Explorer module to free up simulation resources.\033[0m")
        
        # Shutdown the ROS 2 node and exit the program
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = DeterministicExplorer()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally: 
        node.destroy_node(); 
        rclpy.shutdown()
 
if __name__ == '__main__':
    main()