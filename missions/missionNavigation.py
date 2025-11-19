#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_interfaces.action import Speak 
from action_msgs.msg import GoalStatus

from std_msgs.msg import Bool, Empty, String
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped

class MissionNavigationAndFollow(Node):

    def __init__(self):
        super().__init__('navigation_and_follow')
        self.get_logger().info("Iniciando Navigation and Follow Me")

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.speak_client = ActionClient(self, Speak, 'speak_action')

        self.create_subscription(Bool, '/porta_aberta', self.door_callback, 10)

        #Variaveis de Controle
        self.stage = 0
        self.is_navigating = False
        self.is_speaking = False
        self.porta_aberta = False

        self.create_timer(1.0, self.executar_missao)
    
    # ====================== UTILS ======================
    #Avança o estagio da Missão
    def advance_stage(self):
        self.stage += 1
        self.get_logger().info(f"*** Missão avançou para o Stage {self.stage} ***")

    # ====================== MISSÃO MAIN ======================
    def executar_missao(self):

        if self.is_navigating:
            return
        elif self.is_speaking:
            return
        elif self.stage == 0:
            if(self.porta_aberta):
                self.advance_stage()
        
        elif self.stage == 1:
            #----------WAYPOINT 1----------
            self.get_logger().info(f"Stage {self.stage}")
            self.ir_para_waypoint(6.0, 7.73, -1.57) #3.5, 2.5
            self.advance_stage()
        
        elif self.stage == 2:
            #----------WAYPOINT 2----------
            self.get_logger().info(f"Stage {self.stage}")
            self.ir_para_waypoint(2.45, 1.33, 0.0) #2.5, 1.5
            self.advance_stage()

        elif self.stage == 3:
            #----------PORTA----------
            pass
            #self.get_logger().info(f"Stage {self.stage}")
            #self.ir_para_waypoint(0.0, 0.0, 0.0)
            #self.advance_stage()

    # ========= LÓGICA DE NAVEGAÇÃO =========
    def ir_para_waypoint(self, x, y, yaw):

        """Envia objetivo de navegação para o Nav2."""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Servidor de navegação não disponível!")
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation = self.quaternion_from_yaw(yaw)

        self.is_navigating = True

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):

        """Chamado quando o servidor responde ao goal."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Objetivo rejeitado pelo servidor.")
            return

        self.get_logger().info("Objetivo aceito. Aguardando resultado...")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_done_callback)

    def navigation_done_callback(self, future):

        """Chamado quando o robô termina a navegação."""
        self.is_navigating = False

        status = future.result().status

        if status == 4:
            self.get_logger().info("Objetivo alcançado!")
        else:
            self.get_logger().info("Navegação abortada.")

    # ====================== FALA ======================
    def falar(self, texto):

        """Envia objetivo de navegação para o Nav2."""
        if not self.speak_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Servidor de fala não disponível! Avançando Stage.")

            return

        self.get_logger().info(f"[ACTION FALA] Enviando: {texto}")

        goal = Speak.Goal()
        goal.text_to_speak = texto

        self.is_speaking = True

        future = self.speak_client.send_goal_async(goal)
        future.add_done_callback(self.speak_response_callback)

    def speak_response_callback(self, future):

        """Chamado quando o servidor responde ao goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal de fala rejeitado pelo servidor.")
            return
        
        self.get_logger().info("Goal de fala aceito. Aguardando resultado...")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.speak_done_callback)

    def speak_done_callback(self, future):

        """Chamado quando o robô termina a navegação."""
        self.is_speaking = False
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Fala concluída com sucesso.")
        else:
            self.get_logger().error(f"Fala falhou/abortou. Status: {status}") 

    # ====================== CALLBACKS E UTILITÁRIOS ======================

    def quaternion_from_yaw(self, yaw):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q
    
    def door_callback(self, msg: Bool):
        self.porta_aberta = msg.data
    
def main(args=None):
    rclpy.init(args=args)

    node = MissionNavigationAndFollow()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
