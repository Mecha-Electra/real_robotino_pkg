#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_interfaces.action import Speak 
from action_msgs.msg import GoalStatus


from std_msgs.msg import Empty, String, Bool
from geometry_msgs.msg import PoseStamped, Quaternion
from vision_msgs.msg import Detection3DArray


class MissionInspecao(Node):
    def __init__(self):
        super().__init__('inspection')
        self.get_logger().info("Iniciando Inspection!")

        # Publishers e Subscribers
        self.trigger_pub = self.create_publisher(Empty, 'object_detector/trigger', 10)

        self.create_subscription(Detection3DArray, '/objects/detections', self.object_detections_callback, 10)
        self.create_subscription(Bool, '/porta_aberta', self.door_callback, 10)

        # Action Client de navegação
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.speak_client = ActionClient(self, Speak, 'speak_action')

        # Estado interno
        self.detections = None
        self.stage = 0  # Controla o estado da missão
        self.is_navigating = False
        self.is_speaking = False # NOVO FLAG para travar o timer durante uma fala
        self.speak_queue = [] # Fila de frases para o Stage 2
        self.porta_aberta = False
        self.operator_name = "operator"

        # LIMPAR REGISTROS REALSENSE_ID

        # Começa a missão após 1s
        self.create_timer(1.0, self.executar_missao)

    # ====================== UTILS ======================
    
    def advance_stage(self):
        """Avança o estágio da missão e loga."""
        self.stage += 1
        self.get_logger().info(f"*** Missão avançou para o Stage {self.stage} ***")

    # ====================== MISSÃO ======================

    def executar_missao(self):

        if self.is_navigating:
            return
        if self.is_speaking:
            return
        
        if self.stage == 0:
            if(self.porta_aberta):
                self.advance_stage()

        elif self.stage == 1:
            self.get_logger().info("Stage 1")
            self.falar("Door open, going inside")
            self.advance_stage()

        elif self.stage == 2:
            self.ir_para_waypoint(4.8, 3.0, 0.0)
            self.advance_stage()    

        elif self.stage == 3:
            self.falar("I have arrived at my destination")
            self.advance_stage()


        elif self.stage == 4:
            self.falar("Hello Robocup @Home 2025")
            self.advance_stage()

        
        elif self.stage == 5:
            self.ir_para_waypoint(0.0, 8.0, 0.0)
            self.advance_stage()


    # ====================== NAVEGAÇÃO ======================

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

    def object_detections_callback(self, msg: Detection3DArray):
        """Recebe resultados do detector de objetos."""
        self.get_logger().info(f"Recebido {len(msg.detections)} objetos detectados.")
        self.detections = msg
    
    def door_callback(self, msg: Bool):
        self.porta_aberta = msg.data


    def quaternion_from_yaw(self, yaw):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

def main(args=None):
    rclpy.init(args=args)
    node = MissionInspecao()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
