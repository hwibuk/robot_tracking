import rclpy
from rclpy.node import Node
import socket
import select
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64 # RTT 계산용 타임스탬프 반환

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        # 1. 파라미터 설정 (IP 192.168.0.19로 수정)
        self.declare_parameter('esp32_ip', '192.168.0.19')
        self.declare_parameter('port', 12345)

        self.esp32_ip = self.get_parameter('esp32_ip').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value

        # 2. UDP 소켓 설정
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.get_logger().info(f'UDP Bridge Started. Target ESP32: {self.esp32_ip}:{self.port}')

        # 3. Publisher & Subscriber
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # [RTT용] 웹으로 타임스탬프를 다시 돌려줄 토픽
        self.echo_pub = self.create_publisher(Float64, 'latency_echo', 10)

    def cmd_vel_callback(self, msg):
        """ 메시지를 받자마자 ESP32로 전송하고 웹으로 타임스탬프 반환 """
        web_sent_time = msg.linear.y # 웹에서 보낸 T1

        # 통신 규약 변환 (w, s, a, d, x)
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        if linear_x > 0: command = 'w'
        elif linear_x < 0: command = 's'
        elif angular_z > 0: command = 'a'
        elif angular_z < 0: command = 'd'
        else: command = 'x'

        try:
            # 1. ESP32로 제어 명령 전송
            self.sock.sendto(command.encode(), (self.esp32_ip, self.port))

            # 2. 웹으로 에코 메시지 전송 (받았던 T1을 그대로 돌려줌)
            if web_sent_time > 0:
                echo_msg = Float64()
                echo_msg.data = web_sent_time
                self.echo_pub.publish(echo_msg)

        except Exception as e:
            self.get_logger().error(f'Send Error: {e}')

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
