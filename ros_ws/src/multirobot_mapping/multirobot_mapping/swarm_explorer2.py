import rclpy
import subprocess
import sys
import math
import numpy as np
import scipy.ndimage as ndimage
import heapq  
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import GoalStatus
 
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
 
import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
from rclpy.time import Time

from visualization_msgs.msg import Marker, MarkerArray
 
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
 
class SmartSwarmExplorer(Node):
    def __init__(self):
        super().__init__('smart_swarm_explorer')
       
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
           
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.merged_map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_profile)
        self.frontier_marker_pub = self.create_publisher(MarkerArray, '/frontier_markers', 10)
        self.viz_timer = self.create_timer(1.0, self.visualization_tick)
        self.tf_buffer = Buffer(rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
       
        self.merged_map = None
        self.robots = ['robot1', 'robot2', 'robot3']
        self.initial_trigger_done = False
       
        # Variabili per le METRICHE
        self.start_time = None
        self.robot_distances = {robot: 0.0 for robot in self.robots}
        self.previous_poses_for_dist = {robot: None for robot in self.robots}
       
        self.action_clients = {}
        self.robot_status = {}  
        self.assigned_goals = {}
        self.last_robot_poses = {}
        self.stuck_counters = {}    
       
        self.active_goal_handles = {robot: None for robot in self.robots}
       
        self.REPULSION_RADIUS = 1.0  
        # [MODIFICA] Impostato a 12: un ottimo compromesso tra filtraggio del rumore e sensibilità agli spazi.
        self.MIN_FRONTIER_SIZE = 12
       
        for robot_name in self.robots:
            action_name = f'/{robot_name}/navigate_to_pose'
            self.action_clients[robot_name] = ActionClient(self, NavigateToPose, action_name)
            self.robot_status[robot_name] = 'IDLE'
            self.assigned_goals[robot_name] = None
            self.last_robot_poses[robot_name] = None
            self.stuck_counters[robot_name] = 0
 
        self.get_logger().info("Smart Swarm Explorer: Smart Targeting per le frontiere attivato.")
       
        self.startup_timer = self.create_timer(2.0, self.startup_monitor)
        self.stall_checker_timer = self.create_timer(4.0, self.check_robots_stall)
        self.watchdog_timer = self.create_timer(5.0, self.exploration_watchdog)
       
        # Timer dedicato per calcolare accuratamente la distanza percorsa ogni secondo
        self.distance_timer = self.create_timer(1.0, self.update_distances)
       
        self.no_frontiers_ticks = 0
        self.max_ticks_to_stop = 6
 
    def visualization_tick(self):
        # Pubblica SEMPRE i candidati, anche quando nessun robot è IDLE:
        # così vedi il bug (puntini raggruppati lontano) indipendentemente dal dispatch.
        if self.merged_map is None:
            return
        self.publish_frontier_markers(self.extract_frontiers())
 
    def publish_frontier_markers(self, centroids):
        marker_array = MarkerArray()
    
        # --- Pulizia dei marker del ciclo precedente ---
        clear = Marker()
        clear.header.frame_id = 'world'
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)
    
        now = self.get_clock().now().to_msg()
    
        # --- Candidati di frontiera: sfere CIANO ---
        for idx, (fx, fy) in enumerate(centroids):
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
            m.scale.x = m.scale.y = m.scale.z = 0.2
            m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 1.0, 0.9
            marker_array.markers.append(m)
    
        # --- Goal assegnati: sfere ROSSE grandi + etichetta col nome robot ---
        gid = 0
        for robot, goal in self.assigned_goals.items():
            if goal is None:
                continue
    
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
            m.scale.x = m.scale.y = m.scale.z = 0.35
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 1.0
            marker_array.markers.append(m)
    
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
            text.scale.z = 0.3   # per il testo conta solo scale.z
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = robot
            marker_array.markers.append(text)
    
        self.frontier_marker_pub.publish(marker_array)

    def startup_monitor(self):
        if self.merged_map is None:
            self.get_logger().warn("⏳ In attesa della mappa globale sul topic '/map'...", throttle_duration_sec=4.0)
            return
        if not self.initial_trigger_done:
            try:
                if self.tf_buffer.can_transform('world', f'{self.robots[0]}/base_footprint', rclpy.time.Time()):
                    self.initial_trigger_done = True
                    # Registriamo l'inizio del tempo di simulazione (Clock di Gazebo)
                    self.start_time = self.get_clock().now()
                    self.get_logger().info("✅ Mappa e TF ('world' -> base_footprint) ricevuti! Inizio orchestrazione...")
                    self.startup_timer.destroy()
                    self.orchestrator_loop()
                else:
                    self.get_logger().warn("⏳ Mappa ricevuta, ma in attesa dell'albero TF da 'world' ai robot...", throttle_duration_sec=4.0)
            except Exception:
                pass
 
    def map_callback(self, msg):
        self.merged_map = msg
        if self.initial_trigger_done:
            self.orchestrator_loop()
 
    def update_distances(self):
        """Calcola e accumula la distanza percorsa da ciascun robot."""
        if not self.initial_trigger_done:
            return
           
        for robot in self.robots:
            pose = self.get_robot_pose(robot)
            if pose[0] is not None:
                curr_pose = (pose[0], pose[1])
                if self.previous_poses_for_dist[robot] is not None:
                    dist = self.euclidean_distance(curr_pose, self.previous_poses_for_dist[robot])
                    # Filtro anti-rumore: consideriamo solo movimenti maggiori di 5 millimetri
                    if dist > 0.005:
                        self.robot_distances[robot] += dist
                self.previous_poses_for_dist[robot] = curr_pose
 
    def get_quaternion_from_euler(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]
       
    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
 
    def euclidean_distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
 
    def get_robot_pose(self, robot_name):
        try:
            trans = self.tf_buffer.lookup_transform('world', f'{robot_name}/base_footprint', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            yaw = self.get_yaw_from_quaternion(trans.transform.rotation)
            return x, y, yaw
        except TransformException:
            return None, None, None
 
    def compute_path_cost(self, start_w, goal_w, other_robots_poses):
        if self.merged_map is None: return float('inf')
 
        res = self.merged_map.info.resolution
        ox = self.merged_map.info.origin.position.x
        oy = self.merged_map.info.origin.position.y
        width = self.merged_map.info.width
        height = self.merged_map.info.height
       
        sx, sy = int((start_w[0] - ox) / res), int((start_w[1] - oy) / res)
        gx, gy = int((goal_w[0] - ox) / res), int((goal_w[1] - oy) / res)
       
        if not (0 <= sx < width and 0 <= sy < height and 0 <= gx < width and 0 <= gy < height):
            return float('inf')
 
        grid = np.array(self.merged_map.data, dtype=np.int8).reshape((height, width))
        other_pixels = [(int((rx - ox)/res), int((ry - oy)/res)) for rx, ry in other_robots_poses]
        social_radius_px = 1.0 / res
 
        open_set = []
        heapq.heappush(open_set, (0, (sx, sy)))
        g_score = {(sx, sy): 0}
        directions = [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]
       
        while open_set:
            _, current = heapq.heappop(open_set)
            if current == (gx, gy): return g_score[current] * res
           
            for dx, dy in directions:
                nx, ny = current[0] + dx, current[1] + dy
                if 0 <= nx < width and 0 <= ny < height:
                    cell_val = grid[ny, nx]
                   
                    if cell_val == 100:
                        continue
                   
                    step_cost = math.sqrt(dx**2 + dy**2)
                   
                    if cell_val == -1:
                        step_cost *= 20.0
                   
                    for orx, ory in other_pixels:
                        if math.sqrt((nx - orx)**2 + (ny - ory)**2) < social_radius_px:
                            step_cost += 10.0
                            break
                           
                    tentative_g = g_score[current] + step_cost
                    neighbor = (nx, ny)
                   
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        g_score[neighbor] = tentative_g
                        h = math.sqrt((nx - gx)**2 + (ny - gy)**2)
                        heapq.heappush(open_set, (tentative_g + h, neighbor))
        return float('inf')
 
    def extract_frontiers(self):
        if self.merged_map is None: return []
    
        width  = self.merged_map.info.width
        height = self.merged_map.info.height
        res = self.merged_map.info.resolution
        ox  = self.merged_map.info.origin.position.x
        oy  = self.merged_map.info.origin.position.y
    
        map_data = np.array(self.merged_map.data, dtype=np.int8).reshape((height, width))
        struct = ndimage.generate_binary_structure(2, 2)
        frontier_mask = (map_data == 0) & ndimage.binary_dilation((map_data == -1), structure=struct)
        dilated_obstacles = ndimage.binary_dilation((map_data == 100), iterations=7)
        frontier_mask = frontier_mask & (~dilated_obstacles)
    
        labeled_array, num_features = ndimage.label(frontier_mask, structure=struct)
        centroids = []
    
        for i in range(1, num_features + 1):
            y_idx, x_idx = np.where(labeled_array == i)
            cluster_size = len(x_idx)
            if cluster_size < self.MIN_FRONTIER_SIZE:
                continue
    
            # numero di punti proporzionale alla lunghezza della frontiera (non più cap fisso a 4)
            num_points = max(1, min(30, cluster_size // 20))
    
            if num_points == 1:
                # centroide vero, agganciato al pixel di frontiera più vicino
                mx, my = x_idx.mean(), y_idx.mean()
                j = np.argmin((x_idx - mx)**2 + (y_idx - my)**2)
                reps = [(int(x_idx[j]), int(y_idx[j]))]
            else:
                # split GEOMETRICO: ordino lungo l'asse più lungo del cluster e taglio in chunk
                span_x = x_idx.max() - x_idx.min()
                span_y = y_idx.max() - y_idx.min()
                key = x_idx if span_x >= span_y else y_idx
                order = np.argsort(key)
    
                reps = []
                for chunk in np.array_split(order, num_points):
                    cxs, cys = x_idx[chunk], y_idx[chunk]
                    mx, my = cxs.mean(), cys.mean()
                    j = np.argmin((cxs - mx)**2 + (cys - my)**2)  # pixel reale, mai in mezzo a un muro
                    reps.append((int(cxs[j]), int(cys[j])))
    
            for cx, cy in reps:
                centroids.append((ox + cx * res, oy + cy * res))
    
        return centroids
 
    def orchestrator_loop(self):
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
            rx, ry, _ = self.get_robot_pose(robot)
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
                    if self.euclidean_distance((fx, fy), (ax, ay)) < self.REPULSION_RADIUS:
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
 
    def dispatch_robot(self, robot_name, x, y):
        if not self.action_clients[robot_name].wait_for_server(timeout_sec=1.0): return
       
        pose_world = PoseStamped()
        pose_world.header.frame_id = 'world'
        pose_world.header.stamp = Time().to_msg()
        pose_world.pose.position.x = float(x)
        pose_world.pose.position.y = float(y)
       
        last_pose = self.last_robot_poses.get(robot_name)
        rx = last_pose[0] if last_pose is not None else 0.0
        ry = last_pose[1] if last_pose is not None else 0.0
        yaw = math.atan2(y - ry, x - rx)
       
        q = self.get_quaternion_from_euler(0.0, 0.0, yaw)
        pose_world.pose.orientation.x = q[0]
        pose_world.pose.orientation.y = q[1]
        pose_world.pose.orientation.z = q[2]
        pose_world.pose.orientation.w = q[3]
 
        try:
            transform = self.tf_buffer.lookup_transform(
                f'{robot_name}/map',
                'world',
                Time(),
                rclpy.duration.Duration(seconds=1.0))
           
            pose_local = tf2_geometry_msgs.do_transform_pose(pose_world.pose, transform)
           
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = f'{robot_name}/map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose = pose_local
 
            self.robot_status[robot_name] = 'NAVIGATING'
            self.get_logger().info(f"🎯 [{robot_name}] PARTITO! Goal assegnato: X={pose_local.position.x:.2f}, Y={pose_local.position.y:.2f}")
           
            future = self.action_clients[robot_name].send_goal_async(goal_msg)
            future.add_done_callback(lambda fut, r=robot_name: self.goal_response_callback(fut, r))
           
        except TransformException as ex:
            self.get_logger().warn(f"[{robot_name}] Attesa TF per trasformazione: {ex}")
            return
 
    def check_robots_stall(self):
        for robot in self.robots:
            if self.robot_status[robot] == 'NAVIGATING':
                current_x, current_y, current_yaw = self.get_robot_pose(robot)
                if current_x is None: continue
               
                last_pose = self.last_robot_poses.get(robot)
                if last_pose is not None:
                    last_x, last_y, last_yaw = last_pose
                    dist_moved = self.euclidean_distance((current_x, current_y), (last_x, last_y))
                   
                    yaw_diff = abs(math.atan2(math.sin(current_yaw - last_yaw), math.cos(current_yaw - last_yaw)))
                   
                    if dist_moved < 0.10 and yaw_diff < 0.25:
                        self.stuck_counters[robot] += 1
                        if self.stuck_counters[robot] >= 2:
                            self.get_logger().warn(f"🚨 [{robot}] FERMO! Tronco le manovre di recovery del Nav2 e avvio il cooldown...")
                            self.stuck_counters[robot] = 0
                           
                            handle = self.active_goal_handles.get(robot)
                            if handle is not None:
                                try:
                                    handle.cancel_goal_async()
                                except Exception:
                                    pass
                            else:
                                self.robot_status[robot] = 'IDLE'
                                self.assigned_goals[robot] = None
                                self.orchestrator_loop()
                            continue
                    else:
                        self.stuck_counters[robot] = 0
                self.last_robot_poses[robot] = (current_x, current_y, current_yaw)
 
    def goal_response_callback(self, future, robot_name):
        goal_handle = future.result()
        if not goal_handle.accepted:
            timer = []
            # Riduciamo l'attesa per i goal rifiutati
            timer.append(self.create_timer(0.2, lambda: self.cooldown_wrapper(robot_name, timer)))
            return
           
        self.active_goal_handles[robot_name] = goal_handle
       
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut, r=robot_name: self.get_result_callback(fut, r))
 
    def get_result_callback(self, future, robot_name):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.robot_status[robot_name] = 'IDLE'
            self.assigned_goals[robot_name] = None
            self.orchestrator_loop()
        else:
            self.assigned_goals[robot_name] = None
            timer = []
            # [FIX LENTEZZA] Riduciamo l'attesa di riassegnazione da 2.0s a 0.2s!
            timer.append(self.create_timer(0.2, lambda: self.cooldown_wrapper(robot_name, timer)))
 
    def cooldown_wrapper(self, robot_name, timer_list):
        if timer_list and timer_list[0]: timer_list[0].destroy()
        self.robot_status[robot_name] = 'IDLE'
        self.orchestrator_loop()
 
    def exploration_watchdog(self):
        if self.merged_map is None: return
        if all(status != 'NAVIGATING' for status in self.robot_status.values()):
            if len(self.extract_frontiers()) == 0:
                self.no_frontiers_ticks += 1
                if self.no_frontiers_ticks >= self.max_ticks_to_stop:
                    self.terminate_exploration()
            else:
                self.no_frontiers_ticks = 0
                self.orchestrator_loop()
 
    def terminate_exploration(self):
        # --- STAMPA DELLE METRICHE FINALI ---
        if self.start_time is not None:
            # Calcolo del tempo trascorso e della distanza totale
            elapsed_duration = self.get_clock().now() - self.start_time
            elapsed_seconds = elapsed_duration.nanoseconds / 1e9
            total_dist = sum(self.robot_distances.values())
           
            self.get_logger().info("==================================================")
            self.get_logger().info("📊 REPORT METRICHE ESPLORAZIONE:")
            self.get_logger().info(f"⏱️  Tempo di simulazione (Clock): {elapsed_seconds:.2f} secondi")
            for robot in self.robots:
                self.get_logger().info(f"📏 Distanza percorsa da [{robot}]: {self.robot_distances[robot]:.2f} metri")
            self.get_logger().info(f"📈 Distanza TOTALE dello sciame:  {total_dist:.2f} metri")
            self.get_logger().info("==================================================")
 
        self.get_logger().info("✅ ESPLORAZIONE COMPLETATA! Salvataggio mappa in corso...")
        try:
            subprocess.run(["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", "smart_mappa",
                            "--ros-args", "-p", "use_sim_time:=true", "-p", "map_subscribe_transient_local:=true"], check=True)
        except subprocess.CalledProcessError: pass
        sys.exit(0)
 
def main(args=None):
    rclpy.init(args=args)
    node = SmartSwarmExplorer()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()
 
if __name__ == '__main__':
    main()