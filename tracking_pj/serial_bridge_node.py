import rclpy
from rclpy.node import Node
import socket
import select
import yaml
import os
import sys
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import Float64, String
from ament_index_python.packages import get_package_share_directory  # 경로 탐색을 위한 패키지 추가

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node_1') # 0번차 노드와 충돌 방지

        # 1. 파라미터 설정 (기본값 robot_1)
        self.declare_parameter('robot_name', 'robot_1')
        self.declare_parameter('port', 12345)

        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.default_port = self.get_parameter('port').get_parameter_value().integer_value

        # 2. IP 로드 (동적 경로 사용)
        self.load_config_from_yaml()

        # 3. UDP 소켓 설정
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

        # 절대 경로 토픽 설정
        target_cmd_vel = f'/{self.robot_name}/cmd_vel'
        target_action = f'/{self.robot_name}/action'

        self.get_logger().info(f'[{self.robot_name}] 시작! IP: {self.esp32_ip} | Topic: {target_cmd_vel}')

        # 4. Subscriber 설정
        self.sub_manual = self.create_subscription(Twist, target_cmd_vel, self.cmd_vel_callback, 10)
        self.sub_auto = self.create_subscription(Pose2D, target_action, self.auto_action_callback, 10)

        # 5. Publisher 설정
        self.echo_pub = self.create_publisher(Float64, f'/{self.robot_name}/latency_echo', 10)
        self.status_pub = self.create_publisher(String, f'/{self.robot_name}/status', 10)

        # 6. 피드백 수신을 위한 타이머 (100Hz)
        self.create_timer(0.01, self.receive_feedback_callback)

    def load_config_from_yaml(self):
        """
        패키지의 share 디렉토리에서 설정 파일을 동적으로 로드합니다.
        """
        try:
            # 'tracking_pj' 패키지가 설치된 share 경로를 가져옵니다.
            # 예: /home/user/ros2_ws/install/tracking_pj/share/tracking_pj
            package_share_dir = get_package_share_directory('tracking_pj')

            # share 디렉토리 내의 config/esp_ip.yaml 경로를 생성
            config_path = os.path.join(package_share_dir, 'config', 'esp_ip.yaml')

            if not os.path.exists(config_path):
                self.get_logger().error(f'설정 파일을 찾을 수 없습니다: {config_path}')
                sys.exit(1)

            with open(config_path, 'r') as f:
                config_data = yaml.safe_load(f)
                # YAML 구조: robots -> robot_1 -> ip
                robot_info = config_data.get('robots', {}).get(self.robot_name)

                if robot_info:
                    self.esp32_ip = robot_info['ip']
                    self.port = robot_info.get('port', self.default_port)
                    self.get_logger().info(f'설정 로드 완료: {self.esp32_ip}:{self.port}')
                else:
                    self.get_logger().error(f'YAML 파일 내에 [{self.robot_name}] 정보가 없습니다.')
                    sys.exit(1)

        except Exception as e:
            self.get_logger().error(f'Config Load Error: {str(e)}')
            sys.exit(1)

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        if linear_x > 0.1: command = 'w\n'
        elif linear_x < -0.1: command = 's\n'
        elif angular_z > 0.1: command = 'a\n'
        elif angular_z < -0.1: command = 'd\n'
        else: command = 'x\n'

        try:
            self.sock.sendto(command.encode(), (self.esp32_ip, self.port))
            if 'x' not in command:
                self.get_logger().info(f'==> {self.robot_name} 전송: {command.strip()}')
        except Exception as e:
            self.get_logger().error(f'Send Error: {e}')

    def receive_feedback_callback(self):
        try:
            ready = select.select([self.sock], [], [], 0)
            if ready[0]:
                data, addr = self.sock.recvfrom(1024)
                if data:
                    message = data.decode('utf-8').strip()

                    # 1. 상태 정보 처리
                    if "STATUS:" in message:
                        status_msg = String()
                        status_msg.data = message.replace("STATUS:", "")
                        self.status_pub.publish(status_msg)

                    # 2. 레이턴시 데이터 처리
                    else:
                        try:
                            timestamp_val = float(message)
                            echo_msg = Float64()
                            echo_msg.data = timestamp_val
                            self.echo_pub.publish(echo_msg)
                        except ValueError:
                            pass
        except Exception:
            pass

    def auto_action_callback(self, msg):
        try:
            if msg.x == 0.0:
                command = "x\n"
            else:
                # 현재 시간 밀리초 타임스탬프 생성
                timestamp = int(self.get_clock().now().nanoseconds / 1e6)
                # 아두이노 프로토콜 형식에 맞춰 전송
                command = f"D{msg.x:.2f},A{msg.theta:.2f},T{timestamp}\n"

            self.sock.sendto(command.encode(), (self.esp32_ip, self.port))
        except Exception as e:
            self.get_logger().error(f'Auto Send Error: {e}')

    def destroy_node(self):
        if hasattr(self, 'sock'):
            self.sock.close()
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
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
