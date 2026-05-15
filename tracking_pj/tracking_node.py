import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # --- [설정값] ---
        self.STOP_DISTANCE = 40.0
        self.REVERSE_STEER = True
        self.SEND_INTERVAL = 0.1     # 일반 액션 명령 전송 주기 (10Hz)
        self.SAFETY_TIMEOUT = 0.1    # 아루코 인식 상실 타임아웃 (0.1초)

        # --- [상태 변수] ---
        self.last_pose_time = time.time()  # 마지막 아루코 인식 시간
        self.last_sent_time = 0            # 마지막 액션 명령 전송 시간
        self.is_tracking = False           # 현재 주행 중인지 여부

        # --- [통신 설정] ---
        self.sub_pose = self.create_subscription(Pose2D, '/target/relative_pose', self.pose_callback, 10)
        self.pub_action = self.create_publisher(Pose2D, '/robot_1/action', 10)

        # 워치독 타이머: 0.1초 인식을 놓쳤는지 아주 빈번하게(0.02초마다) 감시
        self.create_timer(0.02, self.safety_watchdog)

        self.get_logger().info("Robot Controller: 0.1s Action Interval & 0.1s Watchdog Active")

    def pose_callback(self, msg):
        """
        아루코 노드로부터 데이터 수신 시 실행
        """
        now = time.time()
        self.last_pose_time = now  # 아루코가 인식될 때마다 시간 업데이트

        dist = msg.x
        angle = msg.theta

        # [핵심 로직] 일반 주행 명령은 0.1초 간격(SEND_INTERVAL)으로만 전송
        if now - self.last_sent_time >= self.SEND_INTERVAL:
            cmd = Pose2D()

            if dist < self.STOP_DISTANCE:
                # 목표 거리에 도달했을 때 정지
                self.stop_robot("Target Reached")
            else:
                # 일반 주행 명령 생성 및 전송
                self.is_tracking = True
                cmd.x = dist
                cmd.theta = -angle if self.REVERSE_STEER else angle
                self.pub_action.publish(cmd)
                self.last_sent_time = now
                # self.get_logger().info(f"Command Sent (0.1s interval): Dist {dist:.1f}")

    def safety_watchdog(self):
        """
        주행 명령 주기와 상관없이, 아루코 인식이 끊기면 즉시 개입
        """
        # 주행 중일 때만 감시
        if self.is_tracking:
            elapsed_time = time.time() - self.last_pose_time

            # 아루코 인식을 놓친 지 0.1초가 넘었는가?
            if elapsed_time > self.SAFETY_TIMEOUT:
                # 주행 주기(0.1초)를 기다리지 않고 즉시 정지 명령 전송
                self.stop_robot("MARKER LOST (Immediate Interrupt)")

    def stop_robot(self, reason):
        """
        정지 명령을 즉시 발행하고 상태를 초기화
        """
        stop_cmd = Pose2D(x=0.0, y=0.0, theta=0.0)
        self.pub_action.publish(stop_cmd)

        # 상태 업데이트를 통해 중복 정지 명령 방지
        self.is_tracking = False
        self.last_sent_time = time.time() # 정지 명령도 전송 시간에 포함
        self.get_logger().warn(f"!!! STOP: {reason} !!!")

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
