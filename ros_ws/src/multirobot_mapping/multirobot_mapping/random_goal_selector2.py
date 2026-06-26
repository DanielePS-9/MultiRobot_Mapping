import rclpy            
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import GoalStatus

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

import random
import math

class RandomGoalSelector(Node):
    def __init__(self):
        super().__init__('random_goal_selector')
        
        # Sottoscrizione alla costmap globale
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            'global_costmap/costmap',
            self.costmap_callback,
            10)
        
        # Invece di un Publisher, creiamo un Action Client verso Nav2
        # Il nome 'navigate_to_pose' prenderà automaticamente il namespace del robot (es. /robot1/navigate_to_pose)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.costmap = None
        self.initial_goal_sent = False
        
        self.get_logger().info(f"Random Goal Selector avviato nel namespace: {self.get_namespace()}")

    def costmap_callback(self, msg):
        """Aggiorna la costmap e invia il PRIMO goal appena la mappa è disponibile."""
        self.costmap = msg
        if not self.initial_goal_sent:
            self.initial_goal_sent = True
            self.get_logger().info("Prima costmap ricevuta. Calcolo il primo goal...")
            self.send_random_goal()

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def send_random_goal(self):
        if self.costmap is None:
            self.get_logger().warn("Costmap non ancora disponibile.")
            self.schedule_next_goal(2.0)
            return

        width = self.costmap.info.width
        height = self.costmap.info.height
        resolution = self.costmap.info.resolution
        origin_x = self.costmap.info.origin.position.x
        origin_y = self.costmap.info.origin.position.y

        max_attempts = 200
        valid_goal_found = False
        target_x = 0.0
        target_y = 0.0

        # Raggio (in celle) per cercare spazio noto attorno al punto inesplorato (es. 6 celle = 30cm)
        search_radius = 2 

        # Tenta di trovare un punto inesplorato sul bordo dell'area nota
        for _ in range(max_attempts):
            grid_x = random.randint(0, width - 1)
            grid_y = random.randint(0, height - 1)
            
            index = grid_x + grid_y * width
            cost = self.costmap.data[index]

            # 1. Vogliamo specificamente una cella INESPLORATA (-1)
            if cost == -1:
                # 2. Controlliamo che ci sia almeno una cella libera e sicura nelle vicinanze.
                # Questo garantisce che il punto sia "attaccato" al rettangolo esplorato e non sperduto nel nulla.
                free_space_nearby = False
                
                min_x = max(0, grid_x - search_radius)
                max_x = min(width - 1, grid_x + search_radius)
                min_y = max(0, grid_y - search_radius)
                max_y = min(height - 1, grid_y + search_radius)

                for nx in range(min_x, max_x + 1):
                    for ny in range(min_y, max_y + 1):
                        n_index = nx + ny * width
                        n_cost = self.costmap.data[n_index]
                        
                        # Trovato spazio libero (0-19) vicino all'inesplorato
                        if 0 <= n_cost < 20: 
                            free_space_nearby = True
                            break
                    if free_space_nearby:
                        break
                
                # Se è inesplorato E connesso allo spazio noto, l'abbiamo trovato!
                if free_space_nearby:
                    target_x = origin_x + (grid_x * resolution)
                    target_y = origin_y + (grid_y * resolution)
                    valid_goal_found = True
                    break

        if valid_goal_found:
            # Attendiamo che il server di navigazione sia pronto
            if not self.action_client.wait_for_server(timeout_sec=3.0):
                self.get_logger().error("Il server Action 'navigate_to_pose' non è disponibile!")
                self.schedule_next_goal(2.0)
                return

            # Creiamo il messaggio Goal per l'Action
            goal_msg = NavigateToPose.Goal()
            
            ns = self.get_namespace()
            frame_id = 'map' if ns == '/' else f'{ns[1:]}/map'
            goal_msg.pose.header.frame_id = frame_id
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

            goal_msg.pose.pose.position.x = target_x
            goal_msg.pose.pose.position.y = target_y
            goal_msg.pose.pose.position.z = 0.0

            random_yaw = random.uniform(-math.pi, math.pi)
            q = self.get_quaternion_from_euler(0.0, 0.0, random_yaw)
            goal_msg.pose.pose.orientation.x = q[0]
            goal_msg.pose.pose.orientation.y = q[1]
            goal_msg.pose.pose.orientation.z = q[2]
            goal_msg.pose.pose.orientation.w = q[3]

            self.get_logger().info(f"Target INESPLORATO trovato: X={target_x:.2f}, Y={target_y:.2f} (Attendo l'esito...)")
            
            # Invia il goal in modo asincrono
            self._send_goal_future = self.action_client.send_goal_async(goal_msg)
            self._send_goal_future.add_done_callback(self.goal_response_callback)
        else:
            self.get_logger().warn("Nessun confine inesplorato trovato. Ritento a breve...")
            self.schedule_next_goal(2.0)

    def goal_response_callback(self, future):
        """Callback chiamata quando Nav2 accetta o rifiuta il nostro goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal RIFIUTATO da Nav2. Ne calcolo un altro...")
            self.schedule_next_goal(2.0)
            return

        self.get_logger().info("Goal ACCETTATO! In navigazione...")
        # Attendiamo asincronamente il risultato finale del percorso
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Callback chiamata quando il robot HA RAGGIUNTO il goal o HA FALLITO."""
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("GOAL RAGGIUNTO CON SUCCESSO! Cerco il prossimo...")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn("GOAL FALLITO (Abortito). Cerco un'alternativa...")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("GOAL CANCELLATO. Ne genero uno nuovo...")
        else:
            self.get_logger().info(f"Goal terminato con stato ignoto: {status}")

        # Una volta ottenuto l'esito (positivo o negativo), aspettiamo 2 secondi e ripartiamo
        self.schedule_next_goal(1.0)

    def schedule_next_goal(self, delay_sec):
        """Usa un timer 'one-shot' per ritardare leggermente il prossimo goal."""
        self.timer = self.create_timer(delay_sec, self.timer_callback)

    def timer_callback(self):
        self.timer.cancel() # Ferma il timer per non farlo scattare all'infinito
        self.send_random_goal()

def main(args=None):
    rclpy.init(args=args)
    node = RandomGoalSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()