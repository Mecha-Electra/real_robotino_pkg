#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_interfaces.action import Speak 
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import Image
import tf2_ros
from rclpy.duration import Duration
from rsid_ros.srv import Authenticate, Enroll
from cv_bridge import CvBridge

from std_msgs.msg import Bool, Empty, String
from geometry_msgs.msg import PoseStamped, Quaternion, PoseArray, PointStamped, Pose
import cv2

class MissionPersonalRecognition(Node):

    def __init__(self):
        super().__init__('personal_recognition')
        self.get_logger().info("Iniciando Personal Recognition")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.crowd_trigger_pub = self.create_publisher(Empty, '/crowd_detector/trigger', 10)
        self.create_subscription(PoseArray, '/crowd/all_people_poses', self.all_people_crowd_callback, 10)
        self.create_subscription(Image, '/realsense_id/preview/image_raw', self.realsense_id_callback, 10)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.speak_client = ActionClient(self, Speak, 'speak_action')

        self.auth_service = self.create_client(Authenticate, 'realsense_id/authenticate')
        self.enroll_service = self.create_client(Enroll, 'realsense_id/enroll')

        self.bridge = CvBridge()

        self.stage = 0  # Controla o estado da missão
        self.is_navigating = False
        self.is_speaking = False
        self.crowd_poses = None
        self.crowd_poses_map = None
        self.operator_pose_map = None # PointStamped no frame 'map'
        self.operator_name = None

        self.CAMERA_X_OFFSET = 0.15 # Ex: A câmera está 20cm à frente do centro do robô
        self.CAMERA_Y_OFFSET = 0.0 # Ex: A câmera está centrada
        self.CAMERA_YAW_OFFSET = 0.0

        self.sucess_enroll = False
        self.client_future = None
        self.auth_client_future = None
        self.sucess_auth = False
        self.pose_index = 0

        self.create_timer(1.0, self.executar_missao)
        self._wait_timer = None
    
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
        if self._wait_timer is not None and not self._wait_timer.is_canceled():
            return
        
        if self.stage == 0:
            #----------OPERADOR SE APRESENTA----------
            self.get_logger().info(f"Stage {self.stage}")
            self.operator_name = "Heitor"
            self.advance_stage()

        """
        if self.stage == 1:
            self.get_logger().info(f"Stage {self.stage}")
            
            if self.client_future is None and self.sucess_enroll == False:     
                self.get_logger().info("Iniciando chamada do serviço Enroll...")
                request = Enroll.Request() 
                request.user_id = self.operator_name
                # Inicia a chamada assíncrona e armazena o Future
                self.client_future = self.enroll_service.call_async(request)

            elif self.client_future is not None and self.client_future.done():
                try:
                    response = self.client_future.result()
                    self.sucess_enroll = response.success
                    
                    if self.sucess_enroll:
                        self.get_logger().info(f"Registro bem-sucedido. Mensagem: {response.message}")
                        self.advance_stage()
                    else:
                        self.get_logger().warn(f"Registro falhou. Mensagem: {response.message}")
                        # Lógica de re-tentativa ou erro pode vir aqui
                        
                except Exception as e:
                    self.get_logger().error(f"Erro ao obter resultado do serviço Enroll: {e}")
                
                # Limpa o Future para indicar que a transição foi processada
                self.client_future = None 
                
            # 1.3 Se a chamada JÁ FOI INICIADA e AINDA NÃO CONCLUIU (apenas loga)
            elif self.client_future is not None and not self.client_future.done():
                 self.get_logger().info("Aguardando resposta do serviço Enroll...")

            # 1.4 Se já houve sucesso, avança
            elif self.sucess_enroll == True:
                self.advance_stage()
        """     

        #----------PEGA POSIÇÃO ATUAL DO ROBO----------
        if self.stage == 1:
            self.get_logger().info(f"Stage {self.stage}")
            self.get_robot_pose()
            self.wait_for(10)

        #----------ESPERAR UM MINUTO----------
        if self.stage == 2:
            self.get_logger().info(f"Stage {self.stage}")
            if self.robot_map_pose is None:
                self.get_logger().error("Pose do robô não disponível. Retornando ao Stage 2.")
                self.stage = 1
                return
            self.falar("Operator can go to the crowd now")
            self.get_logger().info("Esperando 1 minuto")
            self.wait_for(60)

        #----------GIRAR 180----------
        if self.stage == 3:
            self.get_logger().info(f"Stage {self.stage}")
            self.falar("Turning 180 Degrees")
            self.advance_stage()

        if self.stage == 4:
            self.get_logger().info(f"Stage {self.stage}")
            x = self.robot_map_pose.translation.x
            y = self.robot_map_pose.translation.y
            self.get_logger().info(f"Pos: {x}, {y}")
            self.ir_para_waypoint(x, y, 0)
            self.advance_stage()

        if self.stage == 5:
            self.get_logger().info(f"Stage {self.stage}")
            self.wait_for(7)

        if self.stage == 6:
            self.get_logger().info(f"Stage {self.stage}")
            self.falar("Triggering crowd detection.")
            self.crowd_trigger_pub.publish(Empty())
            self.advance_stage()

        if self.stage == 7:
            self.get_logger().info(f"Stage {self.stage}")
            #----------ESPERAR DETECÇÃO E CALCULAR POSE----------
            if self.crowd_poses_map is None:
                self.get_logger().warn("Aguardando a pose transformada do operador...")
                #self.stage = 6
                return
            
            self.operator_pose_map = self.crowd_poses_map.poses[self.pose_index]
            if self.operator_pose_map is None:
                self.get_logger().error("Pose do operador é None após indexação! Pulando estágio.")
                self.stage = 6
                return

            # Navega para a pose calculada manualmente
            self.get_logger().info("Pose do operador no mapa capturada. Pronto para navegar.")
            x = self.operator_pose_map.position.z
            y = self.operator_pose_map.position.x
            
            self.get_logger().info(f"Operator pose on map: {x}, {y}")
            self.advance_stage()

        if self.stage == 8:
            #----------SE MOVER ATÉ O OPERADOR----------
            self.get_logger().info(f"Stage {self.stage}")
            if self.crowd_poses_map is None or not (0 <= self.pose_index < len(self.crowd_poses_map.poses)):
                self.pose_index = 0
                self.stage = 7
                return
            
            self.operator_pose_map = self.crowd_poses_map.poses[self.pose_index]
    
            # Verifica se a pose resultante não é None (embora com a verificação acima, isso seja menos provável)
            if self.operator_pose_map is None:
                self.get_logger().error("Pose do operador é None após indexação! Pulando estágio.")
                self.stage = 7
                return

            # Navega para a pose calculada manualmente
            x = self.operator_pose_map.position.z
            y = self.operator_pose_map.position.x
            yaw = 0.0 # Orientação final
            
            self.falar("Going to the Operator")
            self.ir_para_waypoint(x, y, yaw)
            self.advance_stage()

        """
        if self.stage == 9:
            #---------VERIFICAR OPERADOR----------
            self.get_logger().info(f"Stage {self.stage}")
            
            if self.auth_client_future is None and self.sucess_auth == False:     
                self.get_logger().info("Iniciando chamada do serviço Authenticate...")
                request = Authenticate.Request() 
                self.auth_client_future = self.auth_service.call_async(request)

            elif self.auth_client_future is not None and self.auth_client_future.done():
                try:
                    response = self.auth_client_future.result()
                    self.sucess_auth = response.success
                    
                    if self.sucess_auth:
                        self.image_x = response.x
                        self.image_y = response.y
                        self.image_width = response.width
                        self.image_height = response.height
                        
                        self.get_logger().info(f"Registro bem-sucedido. Nome Operador: {response.user_id}")
                        self.advance_stage()
                    else:
                        self.get_logger().warn(f"Registro falhou.")
                        self.stage = 8
                        self.pose_index += 1
                        
                except Exception as e:
                    self.get_logger().error(f"Erro ao obter resultado do serviço Authenticate: {e}")
                
                # Limpa o Future para indicar que a transição foi processada
                self.auth_client_future = None 
                
            # 1.3 Se a chamada JÁ FOI INICIADA e AINDA NÃO CONCLUIU (apenas loga)
            elif self.auth_client_future is not None and not self.auth_client_future.done():
                 self.get_logger().info("Aguardando resposta do serviço Authenticate...")

            # 1.4 Se já houve sucesso, avança
            elif self.sucess_auth == True:
                self.advance_stage()
        """
                
        if self.stage == 9:
            #----------SAUDAR----------
            self.get_logger().info(f"Stage {self.stage}")
            self.falar(f"Hello Operator{self.operator_name}")
            self.advance_stage()

        if self.stage == 10:
            #----------INFORMAR TAMANHO DA MULTIDÃO----------
            self.get_logger().info(f"Stage {self.stage}")
            self.falar(f"The Crowd Has {len(self.crowd_poses_map.poses)} People")
            self.advance_stage()

        if self.stage == 11:
            #----------EXIBIR LOG----------
            self.get_logger().info(f"Stage {self.stage}")
            self.log_realsense()
            self.advance_stage()

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

    def all_people_crowd_callback(self, msg: PoseArray):
        if not msg.poses:
            self.crowd_poses_map = PoseArray() # Limpa se não houver poses
            return

        target_frame = 'map'
        source_frame = 'base_link'

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, 
                source_frame, 
                rclpy.time.Time()
            )
            self.robot_map_pose = transform.transform
            
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"Não foi possível obter a pose do robô ({source_frame} -> {target_frame}) para o crowd: {ex}")
            self.crowd_poses_map = PoseArray()
            return

        q = self.robot_map_pose.rotation
        
        robot_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        # O yaw total é a orientação do robô + o offset estático da câmera
        yaw_total = robot_yaw + self.CAMERA_YAW_OFFSET # Assumindo que self.CAMERA_YAW_OFFSET existe
        
        # Posição do robô no mapa (para translação final)
        robot_x_map = self.robot_map_pose.translation.x
        robot_y_map = self.robot_map_pose.translation.y
        
        crowd_poses_map = PoseArray()
        crowd_poses_map.header.frame_id = 'map'
        crowd_poses_map.header.stamp = self.get_clock().now().to_msg()
        
        
        for pose_cam in msg.poses:
            # A posição no PoseArray é pose.position (x, y, z)
            x_cam = pose_cam.position.x
            y_cam = pose_cam.position.y
            z_cam = pose_cam.position.z

            # --- Sub-passo A: Rotação (Câmera -> Base do Robô) ---
            # Rotacionar o ponto (x_cam, y_cam) pelo yaw_total (orientação do robô + offset da câmera).
            x_rotated = x_cam * math.cos(yaw_total) - y_cam * math.sin(yaw_total)
            y_rotated = x_cam * math.sin(yaw_total) + y_cam * math.cos(yaw_total)
            
            # --- Sub-passo B: Offset Estático da Câmera ---
            # Adicionar o offset estático da câmera (X e Y da câmera em relação à base do robô)
            x_base_offsetted = x_rotated + self.CAMERA_X_OFFSET # Assumindo que self.CAMERA_X_OFFSET existe
            y_base_offsetted = y_rotated + self.CAMERA_Y_OFFSET # Assumindo que self.CAMERA_Y_OFFSET existe
            
            # --- Sub-passo C: Translação Final (Base do Robô -> Mapa) ---
            # Transladar o ponto rotacionado e offsetado pela posição do robô no mapa
            x_map = robot_x_map + x_base_offsetted
            y_map = robot_y_map + y_base_offsetted

            new_pose_map = Pose()
            new_pose_map.position.x = x_map
            new_pose_map.position.y = y_map
            new_pose_map.position.z = z_cam  # Z geralmente não muda (assumindo plano horizontal)
            
            new_pose_map.orientation = pose_cam.orientation # Mantém a orientação original (ou defina como neutra)

            crowd_poses_map.poses.append(new_pose_map)

        # Atualiza a variável de instância com o novo PoseArray transformado
        self.crowd_poses_map = crowd_poses_map

        self.get_logger().info(f"Transformado {len(self.crowd_poses_map.poses)} poses para o frame 'map'.")

    def get_robot_pose(self):
        target_frame = 'map'
        source_frame = 'base_link'

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, 
                source_frame, 
                rclpy.time.Time(), # Procura o transform mais recente
                timeout=Duration(seconds=0.1) # Espera até 100ms
            )
            self.robot_map_pose = transform.transform
            
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(
                f"Não foi possível obter a pose do robô ({source_frame} -> {target_frame}): {ex}"
            )
            self.robot_map_pose = None
            return
    
    def wait_for(self, duration_sec):
        if self._wait_timer is not None and not self._wait_timer.is_canceled():
            return

        self.get_logger().info(f"Esperando por {duration_sec} segundos...")
        
        # Cria um novo timer ou reconfigura o existente
        if self._wait_timer is None:
            self._wait_timer = self.create_timer(duration_sec, self.wait_done)
        else:
            self._wait_timer.cancel()
            self._wait_timer = self.create_timer(duration_sec, self.wait_done)

    def wait_done(self):
        """Chamado quando o timer de espera termina."""
        if self._wait_timer:
            self._wait_timer.cancel()
            self.destroy_timer(self._wait_timer)
            self._wait_timer = None
        self.advance_stage()

    def quaternion_from_yaw(self, yaw):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q
    
    def realsense_id_callback(self, msg: Image):
        self.image_id = msg

    def log_realsense(self):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.image_id, desired_encoding='bgr8')
            self.draw_and_save_box(cv_image)
        except Exception as e:
            self.get_logger().error(f'Erro na conversão da imagem: {e}')

    def draw_and_save_box(self, cv_image):
        try:
            x = int(self.image_x)
            y = int(self.image_y)
            width = int(self.image_width)
            height = int(self.image_height)
            operator_name_text = str(self.operator_name) if self.operator_name else "UNKNOWN"
        except (TypeError, ValueError):
            self.get_logger().error(
                "Erro: Coordenadas da imagem (x, y, w, h) não são números válidos."
            )
            return # Sai da função se a conversão falhar

        output_directory = "/home/robot/ros2_ws/crowd_detections/log_realsense_id.png"

        # 2. Desenhar a Bounding Box
        # Verifica se as coordenadas são válidas antes de chamar a função
        if width > 0 and height > 0:
            # cv2.rectangle usa o canto superior esquerdo (x, y) e o inferior direito (x+w, y+h)
            cv2.rectangle(cv_image, (x, y), (x + width, y + height), (0, 0, 255), 2)
            self.get_logger().info(f'Bounding Box desenhada em x={x}, y={y}, w={width}, h={height}')

            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.7
            font_thickness = 2
            text_color = (0, 255, 0)  # Verde
            
            # Posição do texto (canto superior esquerdo da caixa, ligeiramente acima)
            text_x = x
            text_y = y - 10 if y - 10 > 10 else y + 20 # Coloca 10px acima, a menos que esteja perto do topo da imagem
            
            cv2.putText(
                cv_image,
                operator_name_text,
                (text_x, text_y),
                font,
                font_scale,
                text_color,
                font_thickness,
                cv2.LINE_AA
            )

            # 3. Salvar a imagem no diretório
            cv2.imwrite(output_directory, cv_image)
            self.get_logger().info(f'Imagem salva em: {output_directory}')
        else:
            self.get_logger().warn("Não foi possível desenhar a caixa: Dimensões (width/height) são zero ou negativas.")
    

def main(args=None):
    rclpy.init(args=args)

    node = MissionPersonalRecognition()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
