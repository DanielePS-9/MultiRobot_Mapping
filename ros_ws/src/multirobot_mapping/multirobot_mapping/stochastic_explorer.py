# This code implements a ROS 2 node that coordinates the stochastic exploration of a swarm of three robots in a Gazebo simulation. 
# The node subscribes to the merged global map, identifies unexplored frontiers, and assigns them to idle robots.
# It also monitors the robots's progress, detects if they are stuck, and reassigns goals as necessary. 

import rclpy
import subprocess
import sys
import random
import math

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import GoalStatus
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener, TransformException


class StochasticExplorer(Node):
    def __init__(self):
        super().__init__('stochastic_explorer')
        
        self.merged_map_sub = self.create_subscription(OccupancyGrid,'/map',self.map_callback,10)
        
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
        
        # Initialize a timer to check for robot stalling every 4 seconds
        self.stall_checker_timer = self.create_timer(4.0, self.check_robots_stall)
        self.get_logger().info(f"\033[94mMultiRobotExplorer node initialized. Waiting for the first merged map and TF tree to be ready...\033[0m")

        # Initialize variables for the exploration watchdog
        self.no_frontiers_ticks = 0
        self.max_ticks_to_stop = 6  

        # Initialize a timer to monitor exploration progress every 5 seconds
        self.watchdog_timer = self.create_timer(5.0, self.exploration_watchdog)


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


    # Function to coordinate the assignment of frontiers to idle robots
    def coordinator(self):
        if self.merged_map is None:
            return

        idle_robots = [r for r in self.robots if self.robot_status[r] == 'IDLE']
        if not idle_robots:
            return 
        
        robot_poses = {}
        for r in self.robots:
            rx, ry = self.get_robot_pose(r)
            robot_poses[r] = (rx, ry)

        # Set the maximum number of attempts to find a frontier before giving up
        max_coordinator_attempts = 150 
        attempts = 0
        ready_to_dispatch = []

        
        while idle_robots and attempts < max_coordinator_attempts:
            attempts += 1

            # Find a collaborative frontier
            goal_x, goal_y = self.find_collaborative_frontier(robot_poses=robot_poses)
            
            if goal_x is None or goal_y is None:
                continue 

            closest_robot = None
            min_dist = float('inf')

            for robot in self.robots:
                rx, ry = self.get_robot_pose(robot)
                if rx is not None and ry is not None:
                    d = self.euclidean_distance((rx, ry), (goal_x, goal_y))

                    # Check if the robot is the closest one to the frontier
                    if d < min_dist:
                        min_dist = d
                        closest_robot = robot

            if closest_robot is None:
                continue
            
            # If the closest robot is idle, assign the goal to it and prepare for dispatch
            if closest_robot in idle_robots:
                idle_robots.remove(closest_robot)
                self.assigned_goals[closest_robot] = (goal_x, goal_y)
                ready_to_dispatch.append((closest_robot, goal_x, goal_y))
            else:
                continue
                
        # If we have robots ready to dispatch, send them their goals, otherwise, log a warning if we've exceeded the maximum attempts        
        if ready_to_dispatch:
            self.get_logger().info(f"\033[94mDispatching {len(ready_to_dispatch)} robots to their assigned frontiers.\033[0m")
            for robot_name, gx, gy in ready_to_dispatch:
                self.dispatch_robot(robot_name, gx, gy)
        elif not ready_to_dispatch and attempts >= max_coordinator_attempts:
            self.get_logger().warn(f"\033[93mNo frontiers found.\033[0m")


    # Function to find a collaborative frontier in the merged map
    def find_collaborative_frontier(self, robot_poses=None):
        # Extract map dimensions and resolution from the merged map
        width = self.merged_map.info.width
        height = self.merged_map.info.height
        resolution = self.merged_map.info.resolution
        origin_x = self.merged_map.info.origin.position.x
        origin_y = self.merged_map.info.origin.position.y

        max_attempts = 1000 
        # Define the search radius for free space and the obstacle radius to avoid 
        search_radius = 2   
        obstacle_radius = 6 

        active_targets = [pos for pos in self.assigned_goals.values() if pos is not None]

        for _ in range(max_attempts):
            grid_x = random.randint(0, width - 1)
            grid_y = random.randint(0, height - 1)
            
            # Calculate the index in the occupancy grid data array (1D representation)
            index = grid_x + grid_y * width
            cost = self.merged_map.data[index]

            if cost == -1: 
                free_space_nearby = False
                too_close_to_obstacle = False
                
                # Define the bounds for the search area around the randomly selected cell in the occupancy grid
                min_x = max(0, grid_x - max(search_radius, obstacle_radius))
                max_x = min(width - 1, grid_x + max(search_radius, obstacle_radius))
                min_y = max(0, grid_y - max(search_radius, obstacle_radius))
                max_y = min(height - 1, grid_y + max(search_radius, obstacle_radius))

                for nx in range(min_x, max_x + 1):
                    for ny in range(min_y, max_y + 1):
                        n_index = nx + ny * width
                        n_cost = self.merged_map.data[n_index]
                        
                        cell_dist = math.sqrt((nx - grid_x)**2 + (ny - grid_y)**2)

                        # Check if in the obstacle radius there is a cell with cost 100
                        if n_cost == 100 and cell_dist <= obstacle_radius:
                            too_close_to_obstacle = True
                            # Early exit if we find an obstacle too close
                            break

                        # Check if in the search radius there is a cell with cost 0        
                        if n_cost == 0 and cell_dist <= search_radius: 
                            free_space_nearby = True
                    
                    if too_close_to_obstacle:
                        break 
                
                if free_space_nearby and not too_close_to_obstacle:
                    target_x = origin_x + (grid_x * resolution)
                    target_y = origin_y + (grid_y * resolution)
                    
                    too_close_to_robot = False
                    if robot_poses:
                        for rx, ry in robot_poses.values():
                            # Check if the robot's position is too close to the target (less than 0.4 meters)
                            if rx is not None and self.euclidean_distance((target_x, target_y), (rx, ry)) < 0.4:
                                too_close_to_robot = True
                                # Early exit if we find a robot too close to the target
                                break
                            
                    if too_close_to_robot:
                        continue

                    too_close_to_others = False
                    for active_target in active_targets:
                        # Check if the target is too close to any other active target (less than 1.5 meters)
                        if self.euclidean_distance((target_x, target_y), active_target) < 1.5:
                            too_close_to_others = True
                            # Early exit if we find an active target too close to the new target
                            break
                    
                    # If all the conditions are satisfied, return the target coordinates
                    if not too_close_to_others:
                        return target_x, target_y

        return None, None

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

        # Assign a random orientation to the robot for exploration purposes
        q = self.get_quaternion_from_euler(0.0, 0.0, random.uniform(-math.pi, math.pi))
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
                    
                    # If the robot has moved less than 0.05 meters, consider it stuck and force a goal change
                    if dist_moved < 0.05:
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
            timer.append(self.create_timer(2.5, lambda: self.cooldown_wrapper(robot_name, timer)))

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

        # Check if there are any frontiers available for exploration
        target_x, target_y = self.find_collaborative_frontier()
        has_frontiers = (target_x is not None and target_y is not None)
        
        # Check if all robots are idle and there are no frontiers available
        if all_robots_idle and not has_frontiers:
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
            if has_frontiers and any(status == 'IDLE' for status in self.robot_status.values()):
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
                    "-f", "stochastic_swarm_map", 
                    "--ros-args", 
                    "-p", "use_sim_time:=true",
                    "-p", "map_subscribe_transient_local:=true"
                ],
                check=True
            )
            self.get_logger().info(f"\033[92mMap 'stochastic_swarm_map.yaml' and '.pgm' saved successfully!\033[0m")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"\033[91mCritical error occurred while saving the map: {e}\033[0m")
            
        self.get_logger().info(f"\033[94mClosing Swarm Explorer module to free up simulation resources.\033[0m")
        
        # Shutdown the ROS 2 node and exit the program
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = StochasticExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()