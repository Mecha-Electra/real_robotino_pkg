#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

from std_msgs.msg import Bool
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion

class MissionSpeechRecognition(Node):
    def __init__(self):
        super().__init__('speech_recognition')
        self.get_logger().info("Iniciando Speech Recognition")

        self.porta_aberta = False
        
        self.talker_publisher = self.create_publisher(String, 'say_text', 10)

        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        #----------OPERADOR PERGUNTA----------

        #----------RESPONDER----------

        #----------GERAR LOG----------
        self.falar("Creating Log")

    # ========= LÓGICA DE NAVEGAÇÃO =========

    def ir_para_waypoint(self, x, y, yaw):

        self.get_logger().info(f"Enviando objetivo: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad")
        self.nav_action_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Posição
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # Orientação
        q = self.quaternion_from_yaw(yaw)
        goal_msg.pose.pose.orientation = q

        # Enviar objetivo
        send_goal_future = self.nav_action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Objetivo rejeitado pelo servidor de navegação.')
            return

        self.get_logger().info('Objetivo aceito, aguardando resultado...')

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result().result
        status = get_result_future.result().status

        if status == 4:
            self.get_logger().warn("Objetivo alcançado com sucesso! / Navegação Abortada")

    def quaternion_from_yaw(self, yaw):
        """Converte ângulo yaw (em rad) para quaternion."""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q
    
    def falar(self, texto):
        msg = String()
        msg.data = texto
        self.get_logger().info(f'Publicando texto')
        self.talker_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = MissionSpeechRecognition()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
