import rclpy
from rclpy.node import Node
import socket
import select
import yaml
import os
import sys
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import Float64, String
from ament_index_python.packages import get_package_share_directory # 경로 탐색 라이브러리 추가

class SerialBridgeNode(Node):
    def __init__(self):
        # 노드 이름을 명확히 지정
        super().__init__('target_serial_bridge_node')

        # 1. 파라미터 기본값을 'robot_0'으로 고정
        self.declare_parameter('robot_name', 'robot_0')
        self.declare_parameter('port', 12345)

        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.default_port = self.get_parameter('port').get_parameter_value().integer_value

        # 2. 외부 YAML 파일에서 IP 불러오기 (동적 경로 사용)
        self.load_config_from_yaml()

        # 3. UDP 소켓 설정 (비차단 모드)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

        # ---------------------------------------------------------
        # 실행 인자가 없어도 무조건 설정된 robot_name 토픽을 보게 설정
        # ---------------------------------------------------------
        target_topic = f'/{self.robot_name}/cmd_vel'
        target_action = f'/{self.robot_name}/action'

        self.get_logger().info(f'!!! 고정 타겟 모드 실행 !!!')
        self.get_logger().info(f'대상 로봇: {self.robot_name} (IP: {self.esp32_ip})')
        self.get_logger().info(f'구독 토픽: {target_topic}')

        # 4. Subscriber 설정 (경로를 절대 경로로 고정)
        self.sub_manual = self.create_subscription(Twist, target_topic, self.cmd_vel_callback, 10)
        self.sub_auto = self.create_subscription(Pose2D, target_action, self.auto_action_callback, 10)
        # ---------------------------------------------------------

        # 5. Publisher 설정
        self.echo_pub = self.create_publisher(Float64, f'/{self.robot_name}/latency_echo', 10)
        self.status_pub = self.create_publisher(String, f'/{self.robot_name}/status', 10)

        # 6. 로봇 피드백 수신 타이머 (100Hz)
        self.create_timer(0.01, self.receive_feedback_callback)

    def load_config_from_yaml(self):
        """
        패키지의 share 디렉토리 내 config 폴더에서 설정을 로드합니다.
        """
        try:
            # 패키지 설치 경로를 자동으로 찾음
            package_share_dir = get_package_share_directory('tracking_pj')
            config_path = os.path.join(package_share_dir, 'config', 'esp_ip.yaml')

            if not os.path.exists(config_path):
                self.get_logger().error(f'설정 파일 없음: {config_path}')
                sys.exit(1)

            with open(config_path, 'r') as f:
                config_data = yaml.safe_load(f)
                # robot_0(기본값) 정보를 가져옴
                robot_info = config_data.get('robots', {}).get(self.robot_name)

                if robot_info:
                    self.esp32_ip = robot_info['ip']
                    self.port = robot_info.get('port', self.default_port)
                    self.get_logger().info(f'설정 로드 성공: {self.esp32_ip}:{self.port}')
                else:
                    self.get_logger().error(f'YAML에 {self.robot_name} 정보가 없습니다.')
                    sys.exit(1)

        except Exception as e:
            self.get_logger().error(f'파일 로드 에러: {str(e)}')
            sys.exit(1)

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # 조작 감도 설정 (소수점 값 대응)
        if linear_x > 0.1: command = 'w\n'
        elif linear_x < -0.1: command = 's\n'
        elif angular_z > 0.1: command = 'a\n'
        elif angular_z < -0.1: command = 'd\n'
        else: command = 'x\n'

        try:
            # 명령을 보내고 터미널에 출력 (확인용)
            self.sock.sendto(command.encode(), (self.esp32_ip, self.port))
            if 'x' not in command: # 정지 상태가 아닐 때만 로그 출력
                self.get_logger().info(f'==> [{self.robot_name}] 전송: {command.strip()}')
        except Exception as e:
            self.get_logger().error(f'전송 실패: {e}')

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
            if msg.x == 0.0:
                command = "x\n"
            else:
                command = f"D{msg.x:.2f},A{msg.theta:.2f}\n"

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
