import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # 설정값
        self.STOP_DISTANCE = 35.0
        self.REVERSE_STEER = True
        self.MIN_INTERVAL = 0.1

        # 상태 변수
        self.robot_status = "IDLE"
        self.last_sent_time = 0
        self.sent_stop = False

        # 통신 설정
        self.sub_pose = self.create_subscription(Pose2D, '/target/relative_pose', self.pose_callback, 10)
        self.sub_status = self.create_subscription(String, '/robot_1/status', self.status_callback, 10)
        self.pub_action = self.create_publisher(Pose2D, '/robot_1/action', 10)

        self.get_logger().info("Robot Controller Node Started")

    def status_callback(self, msg):
        self.robot_status = msg.data.upper()

    def pose_callback(self, msg):
        dist = msg.x
        angle = msg.theta
        now = time.time()

        # 정지 거리 이내면 멈춤
        if dist < self.STOP_DISTANCE:
            if not self.sent_stop:
                self.pub_action.publish(Pose2D(x=0.0, y=0.0, theta=0.0))
                self.sent_stop = True
                self.get_logger().info("Target Reached. Stop.")
        else:
            self.sent_stop = False
            # 아두이노가 IDLE 상태이거나 전송 간격이 지났을 때만 전송
            if self.robot_status == "IDLE" or (now - self.last_sent_time > self.MIN_INTERVAL):
                cmd = Pose2D()
                cmd.x = dist
                cmd.theta = -angle if self.REVERSE_STEER else angle

                self.pub_action.publish(cmd)
                self.last_sent_time = now
                self.get_logger().info(f"Command Sent: Dist {dist:.1f}, Angle {angle:.1f}")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(RobotController())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
