#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math

class Navegador(Node):
    def __init__(self):
        super().__init__('navegador_node')

        # Cria o cliente da action
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Envia o objetivo
        self.enviar_objetivo(2.0, 1.0, 1.57)  # x, y, yaw (rad)

    def enviar_objetivo(self, x, y, yaw=0.0):
        self.get_logger().info('Esperando servidor da ação...')
        self._action_client.wait_for_server()

        # Cria a pose de destino
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # Envia o objetivo com feedback
        self.get_logger().info(f'Enviando objetivo: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Objetivo rejeitado pelo servidor!')
            return

        self.get_logger().info('Objetivo aceito! Esperando resultado...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.resultado_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'[Feedback] Distância restante: {feedback.distance_remaining:.2f} m')

    def resultado_callback(self, future):
        result = future.result().result

        if future.result().status == 4:  # STATUS_ABORTED
            self.get_logger().warn('Navegação abortada!')
        elif future.result().status == 5:  # STATUS_REJECTED
            self.get_logger().warn('Navegação rejeitada!')
        elif future.result().status == 3:  # STATUS_SUCCEEDED
            self.get_logger().info('Objetivo alcançado com sucesso! ✅')
        else:
            self.get_logger().warn(f'Finalizado com status desconhecido: {future.result().status}')

        # Encerra o nó após a conclusão
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = Navegador()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
