#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_interfaces.action import Speak 
from action_msgs.msg import GoalStatus

from std_msgs.msg import Empty, String
from geometry_msgs.msg import PoseStamped, Quaternion
from vision_msgs.msg import Detection3DArray

class MissionObjectRecognition(Node):

    def __init__(self):

        super().__init__('object_recognition')
        self.get_logger().info("Iniciando Object Recognition")

        # Publishers e Subscribers
        self.trigger_pub = self.create_publisher(Empty, 'object_detector/trigger', 10)
        self.create_subscription(Detection3DArray, '/objects/detections', self.object_detections_callback, 10)

        # Action Client de navegação
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.speak_client = ActionClient(self, Speak, 'speak_action')


        # Estado interno
        self.detections = None
        self.stage = 0 # Controla o estado da missão
        self.is_navigating = False
        self.is_speaking = False
        self.speak_queue = [] 

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
            self.falar("Going to Waypoint 1!")
            self.ir_para_waypoint(0.7, 0.7, 0.0)
            self.advance_stage()

        elif self.stage == 1:
            self.get_logger().info("Stage 1")
            self.falar("Waypoint 1 reached! Triggering object detection.")
            self.trigger_pub.publish(Empty())
            self.advance_stage()

        elif self.stage == 2:
            self.get_logger().info("Stage 2")
            # Esperando detections chegarem

            self.get_logger().info(f"detections: {self.detections}")
            self.get_logger().info(f"speak_queue: {self.speak_queue}")


            if self.detections is not None and not self.speak_queue:
                self.speak_queue.clear()
                num_objs = len(self.detections.detections)
                self.speak_queue.append(f"{num_objs} objects detected!")

                if num_objs > 0:
                    for det in self.detections.detections:
                        if det.results:
                            label = det.results[0].hypothesis.class_id
                            self.speak_queue.append(f"I detected a {label}.")
                        else:
                            self.speak_queue.append("No objects detected.")
                
                self.detections = None
                self.start_queued_speaking() 
                self.advance_stage() # -> Stage 3 (Avança para o estágio de ESPERA)
        
        elif self.stage == 3:
            pass

        elif self.stage == 4:
            self.falar("Going to Delivery Area.", target_stage=5)
            self.advance_stage()

        elif self.stage == 5:
            self.ir_para_waypoint(0.0, 0.0, 1.57, target_stage=6)
            self.advance_stage()

        elif self.stage == 6:
            self.falar("Delivery Area reached! Mission complete.", target_stage=99)
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

    # ====================== FALAR ======================

    def start_queued_speaking(self):
        """Inicia o processamento da fila de falas."""
        if self.speak_queue:
            next_phrase = self.speak_queue.pop(0) 
            self.falar(next_phrase) 
        
        # Se a fila estiver vazia E estamos no Stage 2 (o estágio do relatório)
        elif self.stage == 2: # <--- CORRIGIDO: Avança se Stage 2 terminou o relatório
            self.advance_stage() # -> Stage 4 (Início da Navegação 2)

    def falar(self, texto, target_stage=None):
        """Envia um Goal de fala. Usa target_stage para Actions únicas."""
        if not self.speak_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Servidor de fala não disponível! Avançando Stage.")
            if target_stage is not None:
                self.stage = target_stage # Avança o stage se falhar
            elif self.stage == 3: # Se falhou a fala de fila, tenta a próxima
                self.start_queued_speaking()
            return

        self.get_logger().info(f"[ACTION FALA] Enviando: {texto}")
        goal = Speak.Goal()
        goal.text_to_speak = texto

        self.is_speaking = True
        future = self.speak_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self.speak_response_callback(f, target_stage))

    def speak_response_callback(self, future, target_stage):
        """Chamado quando o servidor responde ao goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal de fala rejeitado pelo servidor.")
            self.is_speaking = False 
            if target_stage is not None:
                self.stage = target_stage
            elif self.stage == 3:
                self.start_queued_speaking()
            return

        self.get_logger().info("Goal de fala aceito. Aguardando resultado...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.speak_done_callback(f, target_stage))


    def speak_done_callback(self, future, target_stage):
        """Chamado quando o robô termina a fala."""
        self.is_speaking = False
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Fala concluída com sucesso.")
        else:
            self.get_logger().error(f"Fala falhou/abortou. Status: {status}")

        # Lógica de Avanço:
        if target_stage is not None:
            # Ação Única: Avança para o próximo estágio principal.
            self.stage = target_stage
        elif self.stage == 3:
            # Relatório Sequencial: Tenta a próxima frase da fila ou avança o Stage.
            self.start_queued_speaking() 



    # ====================== CALLBACKS E UTILITÁRIOS ======================

    def object_detections_callback(self, msg: Detection3DArray):

        """Recebe resultados do detector de objetos."""
        self.get_logger().info(f"Recebido {len(msg.detections)} objetos detectados.")
        self.detections = msg

    def quaternion_from_yaw(self, yaw):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main(args=None):

    rclpy.init(args=args)

    node = MissionObjectRecognition()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

    main() 