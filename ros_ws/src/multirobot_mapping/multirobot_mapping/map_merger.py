#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
 
class CustomMapMerger(Node):
    def __init__(self):
        super().__init__('custom_map_merger')
       
        # Dichiarazione parametri
        self.declare_parameter('r1_x', -4.0)
        self.declare_parameter('r1_y', 0.0)
        self.declare_parameter('r2_x', -4.0)
        self.declare_parameter('r2_y', -1.0)
        self.declare_parameter('r3_x', -4.0)
        self.declare_parameter('r3_y', -2.0)
 
        # Conversione forzata a float
        self.poses = {
            'robot1': (float(self.get_parameter('r1_x').value), float(self.get_parameter('r1_y').value)),
            'robot2': (float(self.get_parameter('r2_x').value), float(self.get_parameter('r2_y').value)),
            'robot3': (float(self.get_parameter('r3_x').value), float(self.get_parameter('r3_y').value)),
        }
 
        self.local_maps = {'robot1': None, 'robot2': None, 'robot3': None}
 
        # Sottoscrizioni ai topic
        self.create_subscription(OccupancyGrid, '/robot1/map', lambda msg: self.map_callback(msg, 'robot1'), 10)
        self.create_subscription(OccupancyGrid, '/robot2/map', lambda msg: self.map_callback(msg, 'robot2'), 10)
        self.create_subscription(OccupancyGrid, '/robot3/map', lambda msg: self.map_callback(msg, 'robot3'), 10)
 
        self.merged_map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
       
        # Timer per la fusione (1 volta al secondo)
        self.create_timer(1.0, self.merge_and_publish)
        self.get_logger().info("Custom Dynamic Map Merger avviato!")
 
    def map_callback(self, msg, robot_name):
        self.local_maps[robot_name] = msg
 
    def merge_and_publish(self):
        if not any(self.local_maps.values()):
            return
 
        resolution = 0.05  # Risoluzione della mappa globale
       
        # Liste per accumulare tutti i punti validi scoperti nel mondo
        all_x_world = []
        all_y_world = []
        all_vals = []
 
        # --- FASE 1: RACCOLTA E TRASFORMAZIONE DI TUTTI I PUNTI NOTI ---
        for r_name, l_map in self.local_maps.items():
            if l_map is None:
                continue
           
            spawn_x, spawn_y = self.poses[r_name]
            l_res = l_map.info.resolution
            l_w = l_map.info.width
            l_h = l_map.info.height
            l_orig_x = l_map.info.origin.position.x
            l_orig_y = l_map.info.origin.position.y
           
            local_grid = np.array(l_map.data, dtype=np.int8).reshape((l_h, l_w))
 
            # Trova gli indici delle celle note (esclude il -1 che è l'inesplorato locale)
            j_indices, i_indices = np.where(local_grid != -1)
            vals = local_grid[j_indices, i_indices]
 
            if len(vals) == 0:
                continue
 
            # Calcola coordinate metriche relative al frame di origine del robot
            x_local = l_orig_x + i_indices * l_res
            y_local = l_orig_y + j_indices * l_res
 
            # Trasla nel frame globale 'world' usando lo spawn
            x_world = spawn_x + x_local
            y_world = spawn_y + y_local
 
            all_x_world.append(x_world)
            all_y_world.append(y_world)
            all_vals.append(vals)
 
        # Se nessun robot ha ancora inviato celle note, ci fermiamo
        if not all_x_world:
            return
 
        # Uniamo tutti i vettori dei robot in macro-array NumPy
        all_x = np.concatenate(all_x_world)
        all_y = np.concatenate(all_y_world)
        all_v = np.concatenate(all_vals)
 
        # --- FASE 2: CALCOLO DINAMICO DEI CONFINI DELLA MAPPA ---
        # L'origine diventa il punto minimo assoluto esplorato
        g_origin_x = float(np.min(all_x))
        g_origin_y = float(np.min(all_y))
       
        # Convertiamo temporaneamente le coordinate in indici pixel grezzi per trovare il massimo
        g_i_raw = ((all_x - g_origin_x) / resolution).astype(int)
        g_j_raw = ((all_y - g_origin_y) / resolution).astype(int)
 
        # La dimensione della mappa si adatta perfettamente al pixel massimo trovato
        g_width = int(np.max(g_i_raw) + 1)
        g_height = int(np.max(g_j_raw) + 1)
 
        # --- FASE 3: COSTRUZIONE DELLA GRIGLIA GLOBALE ---
        # Creiamo una griglia dimensionata al millimetro, inizializzata a -1 (inesplorato)
        global_grid = np.full((g_height, g_width), -1, dtype=np.int8)
 
        # Scriviamo i dati: prima lo spazio libero (0), poi gli ostacoli (100) per dargli priorità
        free_mask = (all_v == 0)
        global_grid[g_j_raw[free_mask], g_i_raw[free_mask]] = 0
       
        obs_mask = (all_v == 100)
        global_grid[g_j_raw[obs_mask], g_i_raw[obs_mask]] = 100
 
        # --- FASE 4: PUBBLICAZIONE MESSAGGIO ---
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