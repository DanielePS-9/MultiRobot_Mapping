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

        self.assigned_goals = {} # Coordinate assegnate (x, y)
        
        # --- NUOVO PER IL CONTROLLO STALLO ---
        self.last_robot_poses = {} # Salva l'ultima posizione nota: {'robot1': (x, y), ...}
        self.stuck_counters = {}    # Conta quanti controlli consecutivi il robot è rimasto fermo
        
        for robot_name in self.robots:
            action_name = f'/{robot_name}/navigate_to_pose'
            self.action_clients[robot_name] = ActionClient(self, NavigateToPose, action_name)
            self.robot_status[robot_name] = 'IDLE'
            self.assigned_goals[robot_name] = None
            # Inizializziamo le nuove strutture
            self.last_robot_poses[robot_name] = None
            self.stuck_counters[robot_name] = 0
            
        # Timer che controlla lo stallo ogni 4.0 secondi
        self.stall_checker_timer = self.create_timer(4.0, self.check_robots_stall)
        
        self.get_logger().info("Swarm Explorer inizializzato con controllo anti-stallo attivo.")

        # --- WATCHDOG DI FINE ESPLORAZIONE ---
        self.no_frontiers_ticks = 0
        
        self.max_ticks_to_stop = 6  
        
        # Timer indipendente che controlla lo stato globale ogni 5 secondi
        self.watchdog_timer = self.create_timer(5.0, self.exploration_watchdog)

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
        """Ottiene direttamente la posizione del robot rispetto al frame globale 'world'."""
        try:
            now = rclpy.time.Time()
            
            # Chiediamo il TF direttamente dal mondo al robot, ignorando il frame 'map' locale interrotto
            trans = self.tf_buffer.lookup_transform(
                'world', 
                f'{robot_name}/base_footprint', 
                now
            )
            
            # Poiché chiediamo la trasformazione direttamente rispetto a 'world',
            # Gazebo/TF2 ha già calcolato la posizione globale reale includendo gli offset!
            global_x = trans.transform.translation.x
            global_y = trans.transform.translation.y
            
            return global_x, global_y
            
        except TransformException as e:
            # Usiamo un log throttled per evitare di intasare lo schermo se l'errore persiste all'avvio
            self.get_logger().error(f"Errore TF globale per {robot_name}: {str(e)}", throttle_duration_sec=5.0)
            return None, None

    def orchestrator_loop(self):
        """Il cuore del sistema: si attiva SOLO su richiesta o tramite Watchdog."""
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

            # TORNATO ALLA LOGICA ORIGINALE: Valutiamo la distanza di TUTTI i robot
            # per mantenere i territori separati ed evitare attraversamenti della mappa.
            for robot in self.robots:
                rx, ry = self.get_robot_pose(robot)
                if rx is not None and ry is not None:
                    d = self.distance((rx, ry), (goal_x, goal_y))
                    if d < min_dist:
                        min_dist = d
                        closest_robot = robot

            if closest_robot is None:
                continue

            # Se il robot più vicino alla frontiera è libero, ci va lui.
            # Se è già occupato, IGNORIAMO questa frontiera e passiamo alla prossima,
            # così i robot lontani non invaderanno la zona di competenza degli altri!
            if self.robot_status[closest_robot] == 'IDLE' and closest_robot in idle_robots:
                self.dispatch_robot(closest_robot, goal_x, goal_y)
                idle_robots.remove(closest_robot) 
            else:
                continue

    def find_collaborative_frontier(self):
        """Trova un punto inesplorato (-1) adiacente a spazio libero (bianco) e LONTANO da ostacoli."""
        width = self.merged_map.info.width
        height = self.merged_map.info.height
        resolution = self.merged_map.info.resolution
        origin_x = self.merged_map.info.origin.position.x
        origin_y = self.merged_map.info.origin.position.y

        max_attempts = 1000  # Aumentato leggermente visto il filtro più severo
        search_radius = 2   # Raggio per cercare spazio libero
        obstacle_radius = 8 # Raggio di sicurezza dagli ostacoli (evita punti neri)

        active_targets = [pos for pos in self.assigned_goals.values() if pos is not None]

        for _ in range(max_attempts):
            grid_x = random.randint(0, width - 1)
            grid_y = random.randint(0, height - 1)
            
            index = grid_x + grid_y * width
            cost = self.merged_map.data[index]

            # 1. Deve essere un punto INESPLORATO (Grigio)
            if cost == -1: 
                free_space_nearby = False
                too_close_to_obstacle = False
                
                # Definiamo i confini della sotto-griglia di analisi
                min_x = max(0, grid_x - max(search_radius, obstacle_radius))
                max_x = min(width - 1, grid_x + max(search_radius, obstacle_radius))
                min_y = max(0, grid_y - max(search_radius, obstacle_radius))
                max_y = min(height - 1, grid_y + max(search_radius, obstacle_radius))

                for nx in range(min_x, max_x + 1):
                    for ny in range(min_y, max_y + 1):
                        n_index = nx + ny * width
                        n_cost = self.merged_map.data[n_index]
                        
                        # Calcoliamo la distanza in celle dal punto centrale
                        cell_dist = math.sqrt((nx - grid_x)**2 + (ny - grid_y)**2)

                        # CONTROLLO OSTACOLI: Se c'è un ostacolo (punto nero, es. > 50) troppo vicino, scartiamo il punto
                        if n_cost > 50 and cell_dist <= obstacle_radius:
                            too_close_to_obstacle = True
                            break

                        # CONTROLLO SPAZIO LIBERO: C'è spazio bianco nei paraggi?
                        if 0 <= n_cost < 20 and cell_dist <= search_radius: 
                            free_space_nearby = True
                    
                    if too_close_to_obstacle:
                        break # Inutile continuare a controllare questa cella
                
                # Accettiamo il punto SOLO SE c'è spazio libero vicino E NON ci sono ostacoli attorno
                if free_space_nearby and not too_close_to_obstacle:
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
                    
                    # Se in 4 secondi si è mosso meno di 15 cm, lo sblocchiamo ISTANTANEAMENTE
                    if dist_moved < 0.15:
                        self.get_logger().error(
                            f"🚨 [{robot}] FERMO (Spostamento: {dist_moved:.3f}m). Forzo cambio goal IMMEDIATO!"
                        )
                        self.reset_stuck_robot(robot)
                        continue # Passa al prossimo robot, saltando l'aggiornamento della posa
                
                # Aggiorniamo la posa solo se il robot si sta muovendo correttamente
                self.last_robot_poses[robot] = (current_x, current_y)
            else:
                self.last_robot_poses[robot] = None
    
    def reset_stuck_robot(self, robot_name):
        """Ripristina lo stato del robot bloccato e richiede un nuovo goal globale."""
        # Rimettiamo il robot in IDLE sùbito per permettere all'orchestratore di riusarlo
        self.robot_status[robot_name] = 'IDLE'
        self.assigned_goals[robot_name] = None
        self.stuck_counters[robot_name] = 0
        self.last_robot_poses[robot_name] = None
        
        # Inneschiamo l'orchestratore per dare immediatamente una nuova frontiera 
        # (se ci sono altri robot liberi o se lui stesso può ripartire altrove)
        self.orchestrator_loop()

    def goal_response_callback(self, future, robot_name):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"[{robot_name}] Goal rifiutato dal Nav2. Cooling down...")
            
            # Creiamo il timer periodico temporaneo
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
            
            # Usiamo un wrapper con una lista per passare il riferimento del timer stesso e distruggerlo
            timer = []
            timer.append(self.create_timer(2.5, lambda: self.cooldown_wrapper(robot_name, timer)))

    def cooldown_wrapper(self, robot_name, timer_list):
        """Spegne il timer appena scatta (one-shot) e sblocca il robot."""
        if timer_list and timer_list[0]:
            timer_list[0].destroy() # Distrugge il timer per evitare che si ripeta!
        
        # Ora eseguiamo lo sblocco in sicurezza
        if self.robot_status[robot_name] != 'NAVIGATING':
            self.robot_status[robot_name] = 'IDLE'
            self.get_logger().info(f"🔄 [{robot_name}] Cooldown terminato. Pronto per una nuova frontiera.")
            self.orchestrator_loop()

    # ==========================================================
    # --- LOGICA WATCHDOG E SALVATAGGIO MAPPA AGGIUNTA QUI ---
    # ==========================================================
    def exploration_watchdog(self):
        """Controlla periodicamente se l'esplorazione è terminata o se serve svegliare i robot."""
        if self.merged_map is None:
            return 
            
        all_robots_idle = all(status != 'NAVIGATING' for status in self.robot_status.values())
        target_x, target_y = self.find_collaborative_frontier()
        has_frontiers = (target_x is not None and target_y is not None)
        
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
            
            # ERRORE CORRETTO: Se ci sono frontiere e almeno un robot è fermo, forziamo l'assegnazione!
            if has_frontiers and any(status == 'IDLE' for status in self.robot_status.values()):
                self.get_logger().info("Watchdog: Frontiera rilevata con robot inattivi. Forzo l'assegnazione!")
                self.orchestrator_loop()

    def terminate_exploration(self):
        """Salva la mappa fusa tramite processo di sistema e spegne il nodo."""
        self.get_logger().info("✅ ESPLORAZIONE COMPLETATA! Tutte le aree accessibili sono state mappate.")
        self.get_logger().info("💾 Avvio il salvataggio automatico della mappa fusa...")
        
        try:
            # Aggiunto use_sim_time in modo che il map_saver accetti i messaggi di Gazebo
            subprocess.run(
                [
                    "ros2", "run", "nav2_map_server", "map_saver_cli", 
                    "-f", "mappa_sciame_completata", 
                    "--ros-args", "-p", "use_sim_time:=true"
                ],
                check=True
            )
            self.get_logger().info("🎉 Mappa salvata con successo come 'mappa_sciame_completata.yaml' e '.pgm'!")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"❌ Errore critico durante il salvataggio della mappa: {e}")
            
        self.get_logger().info("🛑 Chiusura del modulo Swarm Explorer per liberare le risorse della simulazione.")
        
        # Usciamo semplicemente dal sistema. L'eccezione verrà catturata 
        # dal blocco 'finally' del main() che spegnerà ROS 2 in modo pulito.
        sys.exit(0)


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