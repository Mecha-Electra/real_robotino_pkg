#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math
import numpy as np

class GoalSenderNode(Node):
    def __init__(self):
        super().__init__('goal_sender_node')
        self.get_logger().info('Nó Enviador de Metas de Navegação iniciado.')

        self.reference_frame = 'map'
        self.target_frame = 'human_nose'
        self.robot_base_frame = 'base_link'
        self.safety_distance = 0.7

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # O timer é o coração da atualização contínua.
        # Ele executa a cada 1.0 segundo.
        self.timer = self.create_timer(0.5, self.timer_callback)
         
        # Não precisamos mais do flag `self.goal_sent`.
        # O nó agora sempre tentará enviar uma meta no timer.
        
        self.get_logger().info('Aguardando pelo servidor da ação "navigate_to_pose"...')
        self._action_client.wait_for_server()
        self.get_logger().info('Servidor da ação encontrado.')

    def calculate_safety_goal_pose(self, human_transform, robot_transform):
        """
        Calcula a pose da meta com distância de segurança e orientação correta.
        Retorna um objeto PoseStamped ou None se não for possível calcular.
        """
        human_position = np.array([
            human_transform.transform.translation.x,
            human_transform.transform.translation.y
        ])
        
        robot_position = np.array([
            robot_transform.transform.translation.x,
            robot_transform.transform.translation.y
        ])

        vector_to_robot = robot_position - human_position
        current_distance = np.linalg.norm(vector_to_robot)

        if current_distance < 0.1:
            self.get_logger().warn("Distância entre robô e humano é muito pequena para calcular uma meta segura.")
            return None

        direction_to_robot = vector_to_robot / current_distance
        goal_position = human_position + direction_to_robot * self.safety_distance

        vector_to_human = human_position - goal_position
        yaw = math.atan2(vector_to_human[1], vector_to_human[0])
        
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        
        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = self.reference_frame
        
        goal_pose.pose.position.x = 0.0 #goal_position[0]
        goal_pose.pose.position.y = 0.0 #goal_position[1]
        
        goal_pose.pose.orientation.z = sy
        goal_pose.pose.orientation.w = cy
        return goal_pose

    def timer_callback(self):
        """
        A cada chamada do timer, tenta obter a posição do humano e envia uma nova meta.
        """
        try:
            # 1. Obter transformações mais recentes
            human_transform = self.tf_buffer.lookup_transform(
                self.reference_frame, self.target_frame, rclpy.time.Time())
            
            robot_transform = self.tf_buffer.lookup_transform(
                self.reference_frame, self.robot_base_frame, rclpy.time.Time())
                
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Não foi possível obter uma das transformações necessárias: {e}')
            return

        # 2. Calcular a nova meta
        goal_pose_msg = self.calculate_safety_goal_pose(human_transform, robot_transform)

        if goal_pose_msg is None:
            self.get_logger().info("Não foi possível calcular uma meta válida nesta iteração. Tentando novamente na próxima.")
            return

        # 3. Enviar a nova meta para o Nav2
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose_msg
        
        self.get_logger().info(f'Atualizando a meta para ({goal_pose_msg.pose.position.x:.2f}, {goal_pose_msg.pose.position.y:.2f}) para manter {self.safety_distance}m do humano.')
        self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GoalSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()