#!/usr/bin/env python3

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.time import Time as RclTime
from PIL import Image
import rclpy
import numpy as np
import yaml
import os
import tf2_ros
import math


class ObstacleMapPublisher(Node):
    def __init__(self):
        super().__init__('submap_publisher')
        self.get_logger().info("Iniciando submap publisher")

        # --- Parâmetros do nó ---
        self.declare_parameter('map_yaml_path', '/home/robot/maps/GASI_submap.yaml')
        self.declare_parameter('frame_id', 'obstacle_map')     # frame do mapa publicado
        self.declare_parameter('parent_frame_id', 'map')       # frame de referência (pai do anterior)
        self.declare_parameter('topic_name', '/obstacle_layer_map')
        self.declare_parameter('publish_rate', 1.0)

        self.map_yaml_path = self.get_parameter('map_yaml_path').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.parent_frame_id = self.get_parameter('parent_frame_id').get_parameter_value().string_value
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # --- Carrega e prepara o mapa ---
        self.map_msg = self.load_map(self.map_yaml_path)

        # --- Publisher do mapa ---
        self.publisher = self.create_publisher(OccupancyGrid, self.topic_name, qos_profile)

        # --- Publicação periódica ---
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_map)

        # --- Publicador de TF estático ---
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Publica a transformação uma vez
        self.publish_static_tf()

        self.get_logger().info(f'Publicando mapa em "{self.topic_name}" com frame_id "{self.frame_id}"')

    def load_map(self, yaml_path):
        with open(yaml_path, 'r') as f:
            map_data = yaml.safe_load(f)

        pgm_path = os.path.join(os.path.dirname(yaml_path), map_data['image'])
        img = Image.open(pgm_path).convert('L')
        img_data = np.array(img)

        if map_data.get('negate', 0):
            img_data = 255 - img_data

        occupancy = []
        occupied_thresh_val = map_data['occupied_thresh'] * 255
        free_thresh_val = map_data['free_thresh'] * 255

        for pixel in img_data.flatten():
            # Se o pixel for mais escuro que o limiar "ocupado" (preto -> ocupado)
            if pixel <= free_thresh_val: # Nota: Usamos 'free_thresh_val' aqui para determinar o limite mais escuro/preto (ocupado)
                occupancy.append(100)    # Preto (baixo valor) = Ocupado (100)
            
            # Se o pixel for mais claro que o limiar "livre" (branco -> desocupado)
            elif pixel >= occupied_thresh_val: # Nota: Usamos 'occupied_thresh_val' aqui para determinar o limite mais claro/branco (livre)
                occupancy.append(0)      # Branco (alto valor) = Desocupado (0)
            
            # Caso contrário, é desconhecido
            else:
                occupancy.append(-1)     # Cinza = Desconhecido (-1)

        msg = OccupancyGrid()
        msg.header.frame_id = self.frame_id
        msg.info.resolution = float(map_data['resolution'])
        msg.info.width = img_data.shape[1]
        msg.info.height = img_data.shape[0]

        origin = map_data['origin']
        msg.info.origin.position.x = origin[0]
        msg.info.origin.position.y = origin[1]
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0  # sem rotação

        msg.data = occupancy
        return msg

    def publish_map(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.map_msg)

    def publish_static_tf(self):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.parent_frame_id  # ex: map
        tf.child_frame_id = self.frame_id          # ex: obstacle_map

        # Transformação identidade
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(tf)
        self.get_logger().info(f'Transformação estática publicada: {self.parent_frame_id} -> {self.frame_id}')


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
