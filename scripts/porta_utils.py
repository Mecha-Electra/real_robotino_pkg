#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math
import sys

class PortaDetectorNode(Node):
    def __init__(self):
        super().__init__('porta_detector_node')
        self.publisher = self.create_publisher(Bool, '/porta_aberta', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar',
            self.lidar_callback,
            10
        )
        self.porta_ja_aberta = False

        # Configurações do setor de varredura
        self.angulo_min = -2.0  # graus
        self.angulo_max = 2.0   # graus
        self.distancia_limite = 0.5  # metros

        self.get_logger().info("Node de detecção da porta iniciado.")

    def lidar_callback(self, msg):
        # Converte ângulos para radianos
        ang_min_rad = math.radians(self.angulo_min)
        ang_max_rad = math.radians(self.angulo_max)

        start_idx = int((ang_min_rad - msg.angle_min) / msg.angle_increment)
        end_idx = int((ang_max_rad - msg.angle_min) / msg.angle_increment)

        start_idx = max(0, start_idx)
        end_idx = min(len(msg.ranges) - 1, end_idx)

        setor = msg.ranges[start_idx:end_idx + 1]
        setor_filtrado = [d for d in setor if not math.isinf(d) and not math.isnan(d)]

        if not setor_filtrado:
            return

        distancia_media = sum(setor_filtrado) / len(setor_filtrado)
        self.get_logger().info(f"Distância média na porta: {distancia_media:.2f} m")

        porta_aberta = distancia_media >= self.distancia_limite

        if porta_aberta:
            self.get_logger().info("Porta aberta detectada!")
            self.publisher.publish(Bool(data=True))
            self.porta_detectada = True

            # Aguarda um pequeno tempo para garantir que a mensagem foi publicada
            self.create_timer(0.2, self.encerrar_node)

    def encerrar_node(self):
        self.get_logger().info("Destruindo node PortaDetector.")
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)  # Encerra o processo

def main(args=None):
    rclpy.init(args=args)
    node = PortaDetectorNode()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
