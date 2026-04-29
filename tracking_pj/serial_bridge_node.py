import rclpy
from rclpy.node import Node

import socket

import select

import yaml

import os

import sys

from geometry_msgs.msg import Twist, Pose2D

from std_msgs.msg import Float64, String



class SerialBridgeNode(Node):

    def __init__(self):

        super().__init__('serial_bridge_node_1') # 0번차 노드와 충돌 방지



        # 1. 파라미터 설정 (기본값 robot_1)

        self.declare_parameter('robot_name', 'robot_1')

        self.declare_parameter('port', 12345)



        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        self.default_port = self.get_parameter('port').get_parameter_value().integer_value



        # 2. IP 로드

        self.load_config_from_yaml()



        # 3. UDP 소켓

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.sock.setblocking(False)



        # [핵심 수정] 절대 경로 토픽 설정

        target_cmd_vel = f'/{self.robot_name}/cmd_vel'

        target_action = f'/{self.robot_name}/action'



        self.get_logger().info(f'[{self.robot_name}] 시작! IP: {self.esp32_ip} | Topic: {target_cmd_vel}')



        # 4. Subscriber 설정 (수정된 토픽명 사용)

        self.sub_manual = self.create_subscription(Twist, target_cmd_vel, self.cmd_vel_callback, 10)

        self.sub_auto = self.create_subscription(Pose2D, target_action, self.auto_action_callback, 10)



        # 5. Publisher 설정

        self.echo_pub = self.create_publisher(Float64, f'/{self.robot_name}/latency_echo', 10)

        self.status_pub = self.create_publisher(String, f'/{self.robot_name}/status', 10)



        self.create_timer(0.01, self.receive_feedback_callback)



    def load_config_from_yaml(self):

        config_path = '/home/hwibuk/ros2_ws/src/tracking_pj/config/esp_ip.yaml'

        if not os.path.exists(config_path):

            self.get_logger().error(f'설정 파일 없음: {config_path}')

            sys.exit(1)

        try:

            with open(config_path, 'r') as f:

                config_data = yaml.safe_load(f)

                robot_info = config_data.get('robots', {}).get(self.robot_name)

                if robot_info:

                    self.esp32_ip = robot_info['ip']

                    self.port = robot_info.get('port', self.default_port)

                else:

                    self.get_logger().error(f'YAML에 {self.robot_name} 정보가 없습니다.')

                    sys.exit(1)

        except Exception as e:

            self.get_logger().error(f'Config Load Error: {e}')

            sys.exit(1)



    def cmd_vel_callback(self, msg):

        linear_x = msg.linear.x

        angular_z = msg.angular.z



        # [임계값 수정] 0.1보다 클 때만 움직임으로 인식

        if linear_x > 0.1: command = 'w'

        elif linear_x < -0.1: command = 's'

        elif angular_z > 0.1: command = 'a'

        elif angular_z < -0.1: command = 'd'

        else: command = 'x'



        try:

            self.sock.sendto(command.encode(), (self.esp32_ip, self.port))

            if command != 'x':

                self.get_logger().info(f'==> {self.robot_name} 전송: {command}')

        except Exception as e:

            self.get_logger().error(f'Send Error: {e}')



    def receive_feedback_callback(self):

        try:

            ready = select.select([self.sock], [], [], 0)

            if ready[0]:

                data, addr = self.sock.recvfrom(1024)

                if data:

                    message = data.decode('utf-8').strip()

                    if "STATUS:" in message:

                        status_msg = String()

                        status_msg.data = message.replace("STATUS:", "")

                        self.status_pub.publish(status_msg)

        except Exception:

            pass



    def auto_action_callback(self, msg):

        try:

            if msg.x == 0.0: command = "x"

            else: command = f"D{msg.x:.2f},A{msg.theta:.2f}\n"

            self.sock.sendto(command.encode(), (self.esp32_ip, self.port))

        except Exception as e:

            self.get_logger().error(f'Auto Send Error: {e}')



    def destroy_node(self):

        if hasattr(self, 'sock'): self.sock.close()

        super().destroy_node()



def main(args=None):

    rclpy.init(args=args)

    node = SerialBridgeNode()

    try:

        rclpy.spin(node)

    except (KeyboardInterrupt, SystemExit):

        pass

    finally:

        node.destroy_node()

        if rclpy.ok(): rclpy.shutdown()



if __name__ == '__main__':

    main()
