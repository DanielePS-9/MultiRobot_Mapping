import rclpy
import subprocess
import sys
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import GoalStatus

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

# Importiamo TF2 per localizzare i robot in tempo reale
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

import random
import math

class MarketBasedExplorer(Node):
    def __init__(self):
        super().__init__('market_based_explorer')
        
        # Sottoscrizione alla Mappa Globale Fusa dal map_merger
        self.merged_map_sub = self.create_subscription(
            OccupancyGrid,
            '/map', 
            self.map_callback,
            10)
        
        # Inizializziamo il listener per i TF (Transform)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.merged_map = None
        self.robots = ['robot1', 'robot2', 'robot3']
        self.initial_trigger_done = False 
        self.start_time = None
        
        # --- METRICHE DI CONFRONTO (DISTANZA) ---
        self.total_distances = {robot: 0.0 for robot in self.robots}
        self.tracking_last_poses = {robot: None for robot in self.robots}
        self.distance_tracker_timer = self.create_timer(1.0, self.track_distances)
        
        # Strutture dati per gestire lo sciame
        self.action_clients = {}
        self.robot_status = {}   # 'IDLE' o 'NAVIGATING'
        self.assigned_goals = {} # Coordinate assegnate (x, y)
        self.last_robot_poses = {} 
        self.stuck_counters = {}    
        
        for robot_name in self.robots:
            action_name = f'/{robot_name}/navigate_to_pose'
            self.action_clients[robot_name] = ActionClient(self, NavigateToPose, action_name)
            self.robot_status[robot_name] = 'IDLE'
            self.assigned_goals[robot_name] = None
            self.last_robot_poses[robot_name] = None
            self.stuck_counters[robot_name] = 0
            self.get_logger().info(f"Connessione ad Action Server configurata per {robot_name}")

        self.get_logger().info("Market-Based Explorer inizializzato.")
        
        # Timer che controlla lo stallo ogni 4.0 secondi
        self.stall_checker_timer = self.create_timer(4.0, self.check_robots_stall)

        # --- WATCHDOG DI FINE ESPLORAZIONE ---
        self.no_frontiers_ticks = 0
        self.max_ticks_to_stop = 6  
        self.watchdog_timer = self.create_timer(5.0, self.exploration_watchdog)

    def map_callback(self, msg):
        """Aggiorna la mappa e innesca la primissima assegnazione all'avvio."""
        self.merged_map = msg
        
        if not self.initial_trigger_done:
            if self.tf_buffer.can_transform('world', f'{self.robots[0]}/base_footprint', rclpy.time.Time()):
                self.initial_trigger_done = True
                self.start_time = self.get_clock().now()
                self.get_logger().info("Prima mappa fusa ricevuta e albero TF pronto! Inizio l'asta Market-Based...")
                self.orchestrator_loop()
            else:
                self.get_logger().info(
                    "Mappa ricevuta, ma l'albero TF non è ancora completamente connesso. Attendo...", 
                    throttle_duration_sec=2.0
                )

    def track_distances(self):
        """Calcola e accumula la distanza percorsa da ciascun robot per le metriche."""
        if not self.initial_trigger_done:
            return

        for robot in self.robots:
            current_x, current_y = self.get_robot_pose(robot)
            if current_x is None or current_y is None:
                continue
            
            last_pose = self.tracking_last_poses[robot]
            if last_pose is not None:
                dist = self.distance((current_x, current_y), last_pose)
                if dist > 0.01: 
                    self.total_distances[robot] += dist
                    
            self.tracking_last_poses[robot] = (current_x, current_y)

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def distance(self, p1, p2):
        """Calcola la distanza Euclidea tra due punti 2D."""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def get_robot_pose(self, robot_name):
        """Ottiene la posizione del robot rispetto al frame globale 'world'."""
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('world', f'{robot_name}/base_footprint', now)
            return trans.transform.translation.x, trans.transform.translation.y
        except TransformException:
            return None, None

    def generate_frontier_pool(self, required_frontiers):
        """Genera un pool di frontiere (lotti d'asta) distinte e sicure."""
        width = self.merged_map.info.width
        height = self.merged_map.info.height
        resolution = self.merged_map.info.resolution
        origin_x = self.merged_map.info.origin.position.x
        origin_y = self.merged_map.info.origin.position.y

        max_attempts = 5000  
        search_radius = 2   
        obstacle_radius = 6 

        frontier_pool = []
        active_targets = [pos for pos in self.assigned_goals.values() if pos is not None]

        for _ in range(max_attempts):
            if len(frontier_pool) >= required_frontiers:
                break

            grid_x = random.randint(0, width - 1)
            grid_y = random.randint(0, height - 1)
            index = grid_x + grid_y * width
            cost = self.merged_map.data[index]

            if cost == -1: 
                free_space_nearby = False
                too_close_to_obstacle = False
                
                min_x = max(0, grid_x - max(search_radius, obstacle_radius))
                max_x = min(width - 1, grid_x + max(search_radius, obstacle_radius))
                min_y = max(0, grid_y - max(search_radius, obstacle_radius))
                max_y = min(height - 1, grid_y + max(search_radius, obstacle_radius))

                for nx in range(min_x, max_x + 1):
                    for ny in range(min_y, max_y + 1):
                        n_index = nx + ny * width
                        n_cost = self.merged_map.data[n_index]
                        cell_dist = math.sqrt((nx - grid_x)**2 + (ny - grid_y)**2)

                        if n_cost > 50 and cell_dist <= obstacle_radius:
                            too_close_to_obstacle = True
                            break

                        if 0 <= n_cost < 20 and cell_dist <= search_radius: 
                            free_space_nearby = True
                    
                    if too_close_to_obstacle:
                        break 
                
                if free_space_nearby and not too_close_to_obstacle:
                    target_x = origin_x + (grid_x * resolution)
                    target_y = origin_y + (grid_y * resolution)
                    
                    # Verifica che non sia vicino a frontiere di altri robot
                    too_close_to_others = any(self.distance((target_x, target_y), active) < 1.5 for active in active_targets)
                    # Verifica che non sia vicino a un'altra frontiera già nel pool d'asta
                    too_close_in_pool = any(self.distance((target_x, target_y), pool_f) < 2.0 for pool_f in frontier_pool)
                    
                    if not too_close_to_others and not too_close_in_pool:
                        frontier_pool.append((target_x, target_y))

        return frontier_pool

    def orchestrator_loop(self):
        """Logica Market-Based: Raccoglie le offerte dai robot liberi e le assegna al minor costo globale."""
        if self.merged_map is None:
            return

        idle_robots = [r for r in self.robots if self.robot_status[r] == 'IDLE']
        if not idle_robots:
            return 

        # 1. Creazione Lotti d'Asta (Chiediamo frontiere = robot liberi + 2 per avere scelte migliori)
        lotti_frontiere = self.generate_frontier_pool(required_frontiers=len(idle_robots) + 2)
        
        if not lotti_frontiere:
            return

        # 2. Raccolta delle offerte (Bidding Phase)
        bids = []
        for robot in idle_robots:
            rx, ry = self.get_robot_pose(robot)
            if rx is None or ry is None:
                continue
            
            for index, (fx, fy) in enumerate(lotti_frontiere):
                costo_distanza = self.distance((rx, ry), (fx, fy))
                bids.append({
                    'robot': robot,
                    'frontier_idx': index,
                    'frontier_pos': (fx, fy),
                    'cost': costo_distanza
                })

        # 3. Chiusura dell'Asta (Clearing Phase - Ordinamento globale)
        bids.sort(key=lambda x: x['cost'])

        assigned_robots = set()
        assigned_frontiers = set()
        ready_to_dispatch = []

        for bid in bids:
            r_name = bid['robot']
            f_idx = bid['frontier_idx']
            f_pos = bid['frontier_pos']

            # Assegna solo se né il robot né la frontiera sono già stati presi
            if r_name not in assigned_robots and f_idx not in assigned_frontiers:
                assigned_robots.add(r_name)
                assigned_frontiers.add(f_idx)
                
                self.assigned_goals[r_name] = f_pos
                ready_to_dispatch.append((r_name, f_pos[0], f_pos[1]))
                
                if len(assigned_robots) == len(idle_robots):
                    break

        # 4. Lancio simultaneo dei target vinti
        if ready_to_dispatch:
            self.get_logger().info(f"🔨 Asta Market-Based conclusa! Assegnati {len(ready_to_dispatch)} target simultaneamente.")
            for robot_name, gx, gy in ready_to_dispatch:
                self.dispatch_robot(robot_name, gx, gy)

    def dispatch_robot(self, robot_name, x, y):
        """Invia il comando di navigazione al robot specifico tramite il suo Action Server."""
        if not self.action_clients[robot_name].wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(f"Server Nav2 non pronto per {robot_name}")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'world' 
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        q = self.get_quaternion_from_euler(0.0, 0.0, random.uniform(-math.pi, math.pi))
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        self.robot_status[robot_name] = 'NAVIGATING'
        self.assigned_goals[robot_name] = (x, y)
        self.get_logger().info(f"[{robot_name}] Target vinto all'asta: X={x:.2f}, Y={y:.2f}")
        
        future = self.action_clients[robot_name].send_goal_async(goal_msg)
        future.add_done_callback(lambda fut, r=robot_name: self.goal_response_callback(fut, r))

    def check_robots_stall(self):
        """Controlla lo stallo e cambia IMMEDIATAMENTE goal se il robot è fermo."""
        for robot in self.robots:
            if self.robot_status[robot] == 'NAVIGATING':
                current_x, current_y = self.get_robot_pose(robot)
                if current_x is None or current_y is None:
                    continue
                
                last_pose = self.last_robot_poses[robot]
                if last_pose is not None:
                    dist_moved = self.distance((current_x, current_y), last_pose)
                    if dist_moved < 0.15:
                        self.get_logger().error(f"🚨 [{robot}] FERMO. Forzo rientro in asta!")
                        self.reset_stuck_robot(robot)
                        continue 
                
                self.last_robot_poses[robot] = (current_x, current_y)
            else:
                self.last_robot_poses[robot] = None
    
    def reset_stuck_robot(self, robot_name):
        self.robot_status[robot_name] = 'IDLE'
        self.assigned_goals[robot_name] = None
        self.stuck_counters[robot_name] = 0
        self.last_robot_poses[robot_name] = None
        self.orchestrator_loop()

    def goal_response_callback(self, future, robot_name):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"[{robot_name}] Goal rifiutato dal Nav2. Cooling down...")
            timer = []
            timer.append(self.create_timer(2.0, lambda: self.cooldown_wrapper(robot_name, timer)))
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut, r=robot_name: self.get_result_callback(fut, r))

    def get_result_callback(self, future, robot_name):
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"🟢 [{robot_name}] Arrivato a destinazione con successo!")
            self.robot_status[robot_name] = 'IDLE'
            self.assigned_goals[robot_name] = None
            self.orchestrator_loop()
        else:
            self.get_logger().warn(f"🔴 [{robot_name}] Navigazione fallita. Applico Cooldown...")
            self.assigned_goals[robot_name] = None
            timer = []
            timer.append(self.create_timer(2.5, lambda: self.cooldown_wrapper(robot_name, timer)))

    def cooldown_wrapper(self, robot_name, timer_list):
        if timer_list and timer_list[0]:
            timer_list[0].destroy() 
        
        if self.robot_status[robot_name] != 'NAVIGATING':
            self.robot_status[robot_name] = 'IDLE'
            self.get_logger().info(f"🔄 [{robot_name}] Cooldown terminato. Pronto per nuova asta.")
            self.orchestrator_loop()

    def exploration_watchdog(self):
        """Controlla se l'esplorazione è terminata."""
        if self.merged_map is None:
            return 
            
        all_robots_idle = all(status != 'NAVIGATING' for status in self.robot_status.values())
        
        # Chiediamo al generatore di lotti se esiste almeno 1 frontiera rimanente
        test_pool = self.generate_frontier_pool(required_frontiers=1)
        has_frontiers = len(test_pool) > 0
        
        if all_robots_idle and not has_frontiers:
            self.no_frontiers_ticks += 1
            self.get_logger().info(
                f"Nessuna frontiera rilevata e robot fermi. Spegnimento in corso... "
                f"({self.no_frontiers_ticks}/{self.max_ticks_to_stop})"
            )
            
            if self.no_frontiers_ticks >= self.max_ticks_to_stop:
                self.terminate_exploration()
        else:
            self.no_frontiers_ticks = 0
            if has_frontiers and any(status == 'IDLE' for status in self.robot_status.values()):
                self.orchestrator_loop()

    def terminate_exploration(self):
        """Salva mappa, stampa metriche d'asta e spegne."""
        if self.start_time is not None:
            end_time = self.get_clock().now()
            elapsed_duration = end_time - self.start_time 
            
            elapsed_secs = elapsed_duration.nanoseconds / 1e9
            mins = int(elapsed_secs // 60)
            secs = int(elapsed_secs % 60)
            
            total_swarm_distance = sum(self.total_distances.values())
            
            self.get_logger().info("=====================================================")
            self.get_logger().info(f"⏱️ TEMPO TOTALE (MARKET-BASED): {mins} min e {secs} sec ({elapsed_secs:.2f} s)")
            self.get_logger().info("📏 DISTANZA PERCORSA:")
            for robot in self.robots:
                self.get_logger().info(f"   - {robot}: {self.total_distances[robot]:.2f} metri")
            self.get_logger().info(f"🏆 DISTANZA TOTALE SCIAME: {total_swarm_distance:.2f} metri")
            self.get_logger().info("=====================================================")

        self.get_logger().info("✅ ESPLORAZIONE COMPLETATA! Tutte le aree accessibili sono state mappate.")
        self.get_logger().info("💾 Avvio il salvataggio automatico della mappa fusa...")
        
        try:
            subprocess.run(
                [
                    "ros2", "run", "nav2_map_server", "map_saver_cli", 
                    "-f", "mappa_sciame_completata_mk", 
                    "--ros-args", 
                    "-p", "use_sim_time:=true",
                    "-p", "map_subscribe_transient_local:=true"
                ],
                check=True
            )
            self.get_logger().info("🎉 Mappa salvata con successo come 'mappa_sciame_completata_mk.yaml' e '.pgm'!")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"❌ Errore critico durante il salvataggio: {e}")
            
        self.get_logger().info("🛑 Chiusura del modulo Market-Based Explorer.")
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = MarketBasedExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()