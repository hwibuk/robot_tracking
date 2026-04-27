import rclpy
from rclpy.node import Node
import socket
import select  # 소켓 읽기 가능 여부 확인용
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import Float64, String

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        # 1. 파라미터 설정
        self.declare_parameter('esp32_ip', '192.168.0.29')
        self.declare_parameter('port', 12345)

        self.esp32_ip = self.get_parameter('esp32_ip').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value

        # 2. UDP 소켓 설정 (비차단 모드)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.get_logger().info(f'UDP Bridge Started. Target ESP32: {self.esp32_ip}:{self.port}')

        # 3. Subscriber 설정
        self.sub_manual = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.sub_auto = self.create_subscription(Pose2D, '/robot_1/action', self.auto_action_callback, 10)

        # 4. Publisher 설정
        self.echo_pub = self.create_publisher(Float64, 'latency_echo', 10)
        self.status_pub = self.create_publisher(String, '/robot_1/status', 10)

        # 5. 피드백 수신을 위한 타이머 (100Hz)
        self.create_timer(0.01, self.receive_feedback_callback)

    def receive_feedback_callback(self):
        """ ESP32로부터 들어오는 상태 패킷을 읽어 ROS 토픽으로 발행 """
        try:
            ready = select.select([self.sock], [], [], 0)
            if ready[0]:
                data, addr = self.sock.recvfrom(1024)
                if data:
                    message = data.decode('utf-8').strip()

                    # 아두이노 STATUS 메시지 처리
                    if "STATUS:" in message:
                        status_value = message.replace("STATUS:", "")
                        status_msg = String()
                        status_msg.data = status_value
                        self.status_pub.publish(status_msg)
        except Exception:
            pass

    def auto_action_callback(self, msg):
        """
        트래커 노드로부터 목표 거리/각도를 수신
        x가 0.0이면 정지 거리 내 진입으로 판단하여 'x' 명령 전송
        """
        try:
            if msg.x == 0.0:
                # [수정] 정지 거리 안으로 들어오면 즉시 강제 정지 명령 전송
                command = "x"
                self.get_logger().info("Target Reached: Sending Stop Signal 'x'")
            else:
                # 정상 추적 명령 (Distance, Angle)
                command = f"D{msg.x:.2f},A{msg.theta:.2f}\n"

            self.sock.sendto(command.encode(), (self.esp32_ip, self.port))
        except Exception as e:
            self.get_logger().error(f'Auto Send Error: {e}')

    def cmd_vel_callback(self, msg):
        """ 웹 대시보드 등의 수동 조작 처리 """
        web_sent_time = msg.linear.y
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        if linear_x > 0: command = 'w'
        elif linear_x < 0: command = 's'
        elif angular_z > 0: command = 'a'
        elif angular_z < 0: command = 'd'
        else: command = 'x'

        try:
            self.sock.sendto(command.encode(), (self.esp32_ip, self.port))
            if web_sent_time > 0:
                echo_msg = Float64()
                echo_msg.data = web_sent_time
                self.echo_pub.publish(echo_msg)
        except Exception as e:
            self.get_logger().error(f'Manual Send Error: {e}')

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
