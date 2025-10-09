#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

from std_msgs.msg import Bool
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion

class MissionNavigationFollow(Node):
    def __init__(self):
        super().__init__('navigation_follow_me')
        self.get_logger().info("Iniciando Navigation and Follow Me")

        self.porta_aberta = False

        self.door_state_subscriber = self.create_subscription(Bool, '/porta_aberta', self.porta_callback, 10)

        self.talker_publisher = self.create_publisher(String, 'say_text', 10)

        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        #----------ESPERAR ABERTURA DA PORTA----------
        self.get_logger().info("Esperando abertura da porta...")
        while not self.verificar_porta():
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Porta aberta!")

        #----------CONFIRMAÇÃO DE ABERTURA----------
        self.falar("Door Open!!")

        #----------IR PARA WAYPOINT 1----------
        self.falar("Going to Waypoint 1!")
        self.ir_para_waypoint(1.0, 0.0, 0) 

        #Confirmar
        self.falar("Waypoint 1 Reached!")

        #----------IR PARA WAYPOINT 2----------
        self.falar("Going to Waypoint 2!")
        self.ir_para_waypoint(1.0, 0.5, 0)
        self.falar("Waypoint 2 Reached!")

        #Memorizar Operador

        #----------SINALIZAR----------
        self.falar("Operator Memorized!!")

        #Seguir
        self.falar("Following")

        #----------IR PARA WAYPOINT 2----------
        self.falar("Going to Waypoint 2!")
        self.ir_para_waypoint(1.0, 0.5, 0)
        self.falar("Waypoint 2 Reached!")

        #----------VOLTAR PARA A PORTA----------
        self.falar("Returning to the Door!")
        self.ir_para_waypoint(0.0, 0.0, 0)

    def verificar_porta(self):
        return self.porta_aberta

    def porta_callback(self, msg):
        self.get_logger().info(f"Recebido estado da porta: {'ABERTA' if msg.data else 'FECHADA'}")
        self.porta_aberta = msg.data

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

    node = MissionNavigationFollow()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
