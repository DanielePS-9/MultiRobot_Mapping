import rclpy
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

class MultiRobotExplorer(Node):
    def __init__(self):
        super().__init__('multi_robot_explorer')
        
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
        self.initial_trigger_done = False # Sostituisce il timer per il primissimo avvio
        
        # Definiamo gli offset di spawn esatti per aggirare il bug dei relay sui tf_static
        self.spawn_offsets = {
            'robot1': {'x': -4.0, 'y': 0.0},
            'robot2': {'x': -4.0, 'y': -1.0},
            'robot3': {'x': -4.0, 'y': -2.0}
        }
        
        # Strutture dati per gestire lo sciame
        self.action_clients = {}
        self.robot_status = {}   # Può essere 'IDLE' o 'NAVIGATING'
        self.assigned_goals = {} # Coordinate assegnate (x, y)
        
        for robot_name in self.robots:
            action_name = f'/{robot_name}/navigate_to_pose'
            self.action_clients[robot_name] = ActionClient(self, NavigateToPose, action_name)
            self.robot_status[robot_name] = 'IDLE'
            self.assigned_goals[robot_name] = None
            self.get_logger().info(f"Connessione ad Action Server configurata per {robot_name}")

        self.get_logger().info("Swarm Explorer inizializzato in modalità EVENT-DRIVEN (No Timer).")

    def map_callback(self, msg):
        """Aggiorna la mappa e innesca la primissima assegnazione all'avvio."""
        self.merged_map = msg
        
        # Innesco iniziale: appena abbiamo la mappa, scateniamo l'orchestratore per farli partire
        if not self.initial_trigger_done:
            self.initial_trigger_done = True
            self.get_logger().info("Prima mappa fusa ricevuta! Inizio l'assegnazione dei target...")
            self.orchestrator_loop()

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
        """Ottiene la posizione aggirando il TF statico difettoso dei relay"""
        try:
            # Chiediamo il TF DINAMICO locale del robot (che funziona sempre a 30Hz)
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(f'{robot_name}/map', f'{robot_name}/base_footprint', now)
            
            local_x = trans.transform.translation.x
            local_y = trans.transform.translation.y
            
            # Calcoliamo la posizione globale sommando l'offset di spawn
            global_x = local_x + self.spawn_offsets[robot_name]['x']
            global_y = local_y + self.spawn_offsets[robot_name]['y']
            
            return global_x, global_y
            
        except TransformException:
            return None, None

    def orchestrator_loop(self):
        """Il cuore del sistema: si attiva SOLO su richiesta (event-driven)."""
        if self.merged_map is None:
            return

        idle_robots = [r for r in self.robots if self.robot_status[r] == 'IDLE']
        if not idle_robots:
            return 

        max_orchestrator_attempts = 50
        attempts = 0

        while idle_robots and attempts < max_orchestrator_attempts:
            attempts += 1
            goal_x, goal_y = self.find_collaborative_frontier()
            
            if goal_x is None or goal_y is None:
                self.get_logger().warn("Nessuna nuova frontiera trovata in questo momento.")
                break 

            closest_robot = None
            min_dist = float('inf')

            for robot in self.robots:
                rx, ry = self.get_robot_pose(robot)
                if rx is not None and ry is not None:
                    d = self.distance((rx, ry), (goal_x, goal_y))
                    if d < min_dist:
                        min_dist = d
                        closest_robot = robot

            if closest_robot is None:
                continue

            if self.robot_status[closest_robot] == 'IDLE' and closest_robot in idle_robots:
                self.dispatch_robot(closest_robot, goal_x, goal_y)
                idle_robots.remove(closest_robot) 
            else:
                continue

    def find_collaborative_frontier(self):
        """Trova un punto inesplorato che NON sia già stato assegnato a un altro robot."""
        width = self.merged_map.info.width
        height = self.merged_map.info.height
        resolution = self.merged_map.info.resolution
        origin_x = self.merged_map.info.origin.position.x
        origin_y = self.merged_map.info.origin.position.y

        max_attempts = 300
        search_radius = 2 

        active_targets = [pos for pos in self.assigned_goals.values() if pos is not None]

        for _ in range(max_attempts):
            grid_x = random.randint(0, width - 1)
            grid_y = random.randint(0, height - 1)
            
            index = grid_x + grid_y * width
            cost = self.merged_map.data[index]

            if cost == -1: 
                free_space_nearby = False
                
                min_x = max(0, grid_x - search_radius)
                max_x = min(width - 1, grid_x + search_radius)
                min_y = max(0, grid_y - search_radius)
                max_y = min(height - 1, grid_y + search_radius)

                for nx in range(min_x, max_x + 1):
                    for ny in range(min_y, max_y + 1):
                        n_index = nx + ny * width
                        n_cost = self.merged_map.data[n_index]
                        
                        if 0 <= n_cost < 20: 
                            free_space_nearby = True
                            break
                    if free_space_nearby:
                        break
                
                if free_space_nearby:
                    target_x = origin_x + (grid_x * resolution)
                    target_y = origin_y + (grid_y * resolution)
                    
                    too_close_to_others = False
                    for active_target in active_targets:
                        if self.distance((target_x, target_y), active_target) < 1.5:
                            too_close_to_others = True
                            break
                    
                    if not too_close_to_others:
                        return target_x, target_y

        return None, None

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
        self.get_logger().info(f"[{robot_name}] Assegnata frontiera TERRITORIALE: X={x:.2f}, Y={y:.2f}")
        
        future = self.action_clients[robot_name].send_goal_async(goal_msg)
        future.add_done_callback(lambda fut, r=robot_name: self.goal_response_callback(fut, r))

    def goal_response_callback(self, future, robot_name):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"[{robot_name}] Goal rifiutato dal Nav2. Torna IDLE.")
            self.robot_status[robot_name] = 'IDLE'
            self.assigned_goals[robot_name] = None
            
            # EVENTO 1: Goal rifiutato, inneschiamo sùbito una nuova ricerca!
            self.orchestrator_loop()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut, r=robot_name: self.get_result_callback(fut, r))

    def get_result_callback(self, future, robot_name):
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"🟢 [{robot_name}] Arrivato a destinazione con successo!")
        else:
            self.get_logger().warn(f"🔴 [{robot_name}] Navigazione fallita o interrotta (Ostacolo imprevisto?).")

        self.robot_status[robot_name] = 'IDLE'
        self.assigned_goals[robot_name] = None
        
        # EVENTO 2: Qualsiasi sia l'esito, il robot ha finito. Inneschiamo la nuova ricerca!
        self.orchestrator_loop()

def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()