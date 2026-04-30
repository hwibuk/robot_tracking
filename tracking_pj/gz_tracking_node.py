import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Pose, Twist  # Pose 수신, Twist(속도) 발행
from std_msgs.msg import String

class TrackingController(Node):
    def __init__(self):
        super().__init__('tracking_node')
        
        # Gazebo 설정 (m 단위)
        self.STOP_DISTANCE = 0.35  # 35cm -> 0.35m
        
        # 구독 및 발행
        self.sub_pose = self.create_subscription(Pose, '/target/relative_pose', self.pose_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10) # Gazebo 로봇 제어 토픽
        
        self.get_logger().info("Gazebo Tracking Controller Started")

    def quaternion_to_euler_yaw(self, q):
        """쿼터니언에서 Yaw(Z축 회전) 각도만 추출"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def pose_callback(self, msg):
        # 거리(m) 추출
        dist = msg.position.x
        # 쿼터니언 -> 라디안 각도 변환
        yaw_rad = self.quaternion_to_euler_yaw(msg.orientation)
        
        twist = Twist()

        if dist < self.STOP_DISTANCE:
            # 정지
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            # 간단한 P 제어 (가제보 로봇 특성에 맞춰 게인값 조정 필요)
            twist.linear.x = 0.5 * (dist - self.STOP_DISTANCE) # 거리 비례 속도
            twist.angular.z = 1.0 * yaw_rad # 각도 비례 회전
            
            # 최대 속도 제한
            twist.linear.x = min(twist.linear.x, 0.5)
            twist.angular.z = min(max(twist.angular.z, -1.0), 1.0)

        self.pub_cmd.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TrackingController())
    rclpy.shutdown()
