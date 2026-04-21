import rclpy
from rclpy.node import Node
import socket
import select
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        # 1. 파라미터 설정 (필요 시 외부에서 변경 가능)
        self.declare_parameter('esp32_ip', '192.168.0.2') # 실제 ESP32 IP로 수정
        self.declare_parameter('port', 12345)
        
        self.esp32_ip = self.get_parameter('esp32_ip').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value

        # 2. UDP 소켓 설정
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False) # Non-blocking 모드로 설정하여 수신 대기 시 멈춤 방지
        self.get_logger().info(f'UDP Bridge Started. Target ESP32: {self.esp32_ip}:{self.port}')

        # 3. Publisher & Subscriber
        # /cmd_vel을 구독하여 로봇에 명령 전달
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        # 로봇으로부터 받은 데이터를 /sensor_data로 발행
        self.publisher_ = self.create_publisher(String, 'sensor_data', 10)

        # 4. 수신 타이머 (아두이노 데이터 체크 - 50ms 주기)
        self.timer = self.create_timer(0.05, self.receive_udp_data)

    def cmd_vel_callback(self, msg):
        """ /cmd_vel 메시지를 받아 UDP 패킷으로 변환 전송 """
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # 로봇 제어 규약 (w, s, a, d, x) 변환 로직
        if linear_x > 0:
            command = 'w'
        elif linear_x < 0:
            command = 's'
        elif angular_z > 0:
            command = 'a'
        elif angular_z < 0:
            command = 'd'
        else:
            command = 'x'

        try:
            self.sock.sendto(command.encode(), (self.esp32_ip, self.port))
            # self.get_logger().info(f'Sent Command: {command}') # 디버깅용
        except Exception as e:
            self.get_logger().error(f'Send Error: {e}')

    def receive_udp_data(self):
        """ ESP32로부터 들어오는 센서 데이터를 받아 ROS 토픽으로 발행 """
        try:
            # 읽을 데이터가 있는지 확인 (timeout 0)
            ready = select.select([self.sock], [], [], 0)
            if ready[0]:
                data, addr = self.sock.recvfrom(1024)
                raw_string = data.decode().strip()
                
                # 수신된 데이터가 "DATA,"로 시작하는지 확인 (규약 준수)
                if raw_string.startswith("DATA,"):
                    msg = String()
                    msg.data = raw_string
                    self.publisher_.publish(msg)
                    # self.get_logger().info(f'Published: {raw_string}') # 디버깅용
        except Exception as e:
            # 수신 데이터가 없을 때 발생하는 에러는 무시
            pass

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
