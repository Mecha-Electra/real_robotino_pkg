#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class MissionMainNode(Node):
    def __init__(self):
        super().__init__('mission_one')

        self.subscription = self.create_subscription(Bool, '/porta_aberta', self.porta_callback, 10)
        self.porta_aberta = False
        self.operacao_iniciada = False

        self.get_logger().info("Aguardando abertura da porta...")

    def porta_callback(self, msg):
        if msg.data and not self.operacao_iniciada:
            self.porta_aberta = True
            self.operacao_iniciada = True
            self.iniciar_missao()

    def iniciar_missao(self):
        self.get_logger().info("orta aberta! Iniciando operação da missão")
        
        print("Tarefa Iniciada")

def main(args=None):
    rclpy.init(args=args)
    node = MissionMainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
