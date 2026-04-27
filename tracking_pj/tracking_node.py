import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
import math
import time

class MultiArucoFollower(Node):
    def __init__(self):
        super().__init__('multi_aruco_follower')

        # --- 제어 설정 ---
        self.REVERSE_DRIVE = False
        self.REVERSE_STEER = True
        self.STOP_DISTANCE = 30.0  # 정지 거리 (cm)

        # 상태 관리 변수
        self.robot_status = "IDLE"
        self.last_command_time = 0
        self.MIN_COMMAND_INTERVAL = 0.5  # 명령 전송 최소 간격 (초)
        self.sent_stop = False           # 정지 명령 중복 방지 플래그

        # Publisher & Subscriber
        # SerialBridgeNode가 이 토픽을 구독하여 ESP32로 전달함
        self.pub_action = self.create_publisher(Pose2D, '/robot_1/action', 10)
        self.sub_status = self.create_subscription(String, '/robot_1/status', self.status_callback, 10)

        # 타이머 (실시간 처리 30fps)
        self.timer = self.create_timer(0.033, self.timer_callback)

        # 카메라 캘리브레이션 파라미터 (기존 데이터 유지)
        self.mtx = np.array([[1459.88, 0.0, 326.34], [0.0, 1600.67, 244.17], [0.0, 0.0, 1.0]])
        self.dist = np.array([0.00226, 9.375, -0.0115, 0.00222, -104.07])

        # 카메라 설정 (장치 번호는 상황에 따라 0 또는 2 등으로 변경 필요)
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

    def status_callback(self, msg):
        """로봇(아두이노)으로부터 현재 주행 상태 수신"""
        self.robot_status = msg.data

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret: return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 5.0, self.mtx, self.dist)
            id_list = ids.flatten().tolist()

            # 0번(타겟)과 1번(로봇 본체) 마커가 모두 보일 때만 실행
            if 0 in id_list and 1 in id_list:
                try:
                    i0, i1 = id_list.index(0), id_list.index(1)

                    # 시각화를 위한 투영 좌표 계산
                    img_c1, _ = cv2.projectPoints(np.array([[0, 0, 0]], dtype=np.float32), rvecs[i1], tvecs[i1], self.mtx, self.dist)
                    img_h1, _ = cv2.projectPoints(np.array([[0, 15.0, 0]], dtype=np.float32), rvecs[i1], tvecs[i1], self.mtx, self.dist)
                    img_t0, _ = cv2.projectPoints(np.array([[0, 6.0, 0]], dtype=np.float32), rvecs[i0], tvecs[i0], self.mtx, self.dist)

                    pt_c1 = tuple(img_c1.astype(int).ravel())
                    pt_h1 = tuple(img_h1.astype(int).ravel())
                    pt_t0 = tuple(img_t0.astype(int).ravel())

                    # 상대 각도 및 거리 계산
                    vec_robot = np.array([pt_h1[0] - pt_c1[0], pt_h1[1] - pt_c1[1]])
                    vec_target = np.array([pt_t0[0] - pt_c1[0], pt_t0[1] - pt_c1[1]])
                    rel_angle = math.degrees(math.atan2(vec_target[1], vec_target[0]) - math.atan2(vec_robot[1], vec_robot[0]))
                    
                    while rel_angle > 180: rel_angle -= 360
                    while rel_angle < -180: rel_angle += 360
                    
                    real_dist = np.linalg.norm(tvecs[i0] - tvecs[i1])

                    current_time = time.time()

                    # --- [지능형 정지 및 추적 로직] ---
                    if real_dist < self.STOP_DISTANCE:
                        # 정지 거리 안으로 들어오면 즉시 x(정지) 명령 유도
                        if not self.sent_stop:
                            stop_action = Pose2D()
                            stop_action.x = 0.0  # 거리 0 신호 전달
                            stop_action.theta = 0.0
                            self.pub_action.publish(stop_action)
                            self.sent_stop = True
                            self.get_logger().info("Target Reached: Sent Stop Command.")
                    else:
                        # 타겟이 정지 거리 밖으로 벗어나면 다시 추적 시작
                        self.sent_stop = False
                        should_send = False

                        # 상태 기반 명령 전송 판단
                        if self.robot_status == "IDLE":
                            should_send = True
                        elif self.robot_status == "DRIVING" and (current_time - self.last_command_time > self.MIN_COMMAND_INTERVAL):
                            # 주행 중이라도 타겟이 이동 중이면 0.5초마다 목표 갱신
                            should_send = True

                        if should_send:
                            action = Pose2D()
                            action.x = -real_dist if self.REVERSE_DRIVE else real_dist
                            action.theta = -rel_angle if self.REVERSE_STEER else rel_angle
                            self.pub_action.publish(action)
                            self.last_command_time = current_time

                    # 화면 시각화 (정지 시 빨간색 점, 추적 시 초록색 점)
                    status_color = (0, 0, 255) if self.sent_stop else (0, 255, 0)
                    cv2.circle(frame, pt_h1, 10, status_color, -1)
                    cv2.circle(frame, pt_t0, 10, (0, 0, 255), -1)
                    cv2.line(frame, pt_h1, pt_t0, (255, 0, 0), 3)
                    
                    cv2.putText(frame, "STATUS: " + ("STOPPED" if self.sent_stop else self.robot_status), (10, 60), 1, 1.5, (0, 255, 255), 2)
                    cv2.putText(frame, f"D:{real_dist:.1f} A:{rel_angle:.1f}", (10, 30), 1, 1.5, (255, 255, 255), 2)

                except Exception as e:
                    self.get_logger().error(f"Logic Error: {e}")

            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        cv2.imshow('Smart Tracking View', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = MultiArucoFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
