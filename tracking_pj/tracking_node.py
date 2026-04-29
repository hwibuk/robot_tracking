import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import math
import time
import os
import sys # 시스템 종료를 위해 추가

from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class MultiArucoFollower(Node):
    def __init__(self):
        super().__init__('multi_aruco_follower')

        # --- 1. .npz 파일 로드 (실패 시 노드 종료) ---
        self.load_camera_params_npz()

        # --- 2. ROS 통신 설정 ---
        self.bridge = CvBridge()
        self.pub_image = self.create_publisher(Image, '/usb_camera/image_raw', 10)
        self.pub_action = self.create_publisher(Pose2D, '/robot_1/action', 10)
        self.sub_status = self.create_subscription(String, '/robot_1/status', self.status_callback, 10)

        # --- 3. 제어 및 상태 변수 ---
        self.REVERSE_DRIVE = False
        self.REVERSE_STEER = True
        self.STOP_DISTANCE = 30.0
        self.robot_status = "IDLE"
        self.last_command_time = 0
        self.MIN_COMMAND_INTERVAL = 0.1
        self.sent_stop = False

        # --- 4. 카메라 하드웨어 설정 ---
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.timer = self.create_timer(0.033, self.timer_callback)
        self.get_logger().info("Node Started. Camera params loaded successfully.")

    def load_camera_params_npz(self):
        """지정된 절대 경로에서 캘리브레이션 파일을 로드합니다. 실패 시 프로세스를 종료합니다."""
        file_path = '/home/hwibuk/ros2_ws/src/tracking_pj/tracking_pj/calibration_data.npz'

        # 파일 존재 여부 확인
        if not os.path.exists(file_path):
            self.get_logger().error(f"CRITICAL ERROR: 파일을 찾을 수 없습니다 -> {file_path}")
            self.get_logger().error("캘리브레이션 파일 없이 노드를 실행할 수 없어 종료합니다.")
            sys.exit(1) # 프로세스 강제 종료

        try:
            data = np.load(file_path)
            # 파일 안에 mtx와 dist 키가 있는지 확인
            if 'mtx' not in data or 'dist' not in data:
                raise KeyError("npz 파일 내에 'mtx' 또는 'dist' 데이터가 포함되어 있지 않습니다.")

            self.mtx = data['mtx']
            self.dist = data['dist']
            self.get_logger().info(f"성공적으로 캘리브레이션 데이터를 로드했습니다: {file_path}")

        except Exception as e:
            self.get_logger().error(f"CRITICAL ERROR: 데이터 로드 중 오류 발생: {e}")
            sys.exit(1) # 프로세스 강제 종료

    def status_callback(self, msg):
        self.robot_status = msg.data

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret: return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 5.0, self.mtx, self.dist)
            id_list = ids.flatten().tolist()

            if 0 in id_list and 1 in id_list:
                try:
                    i0, i1 = id_list.index(0), id_list.index(1)

                    img_c1, _ = cv2.projectPoints(np.array([[0, 0, 0]], dtype=np.float32), rvecs[i1], tvecs[i1], self.mtx, self.dist)
                    img_h1, _ = cv2.projectPoints(np.array([[0, 15.0, 0]], dtype=np.float32), rvecs[i1], tvecs[i1], self.mtx, self.dist)
                    img_t0, _ = cv2.projectPoints(np.array([[0, 6.0, 0]], dtype=np.float32), rvecs[i0], tvecs[i0], self.mtx, self.dist)

                    pt_c1 = tuple(img_c1.astype(int).ravel())
                    pt_h1 = tuple(img_h1.astype(int).ravel())
                    pt_t0 = tuple(img_t0.astype(int).ravel())

                    vec_robot = np.array([pt_h1[0] - pt_c1[0], pt_h1[1] - pt_c1[1]])
                    vec_target = np.array([pt_t0[0] - pt_c1[0], pt_t0[1] - pt_c1[1]])
                    rel_angle = math.degrees(math.atan2(vec_target[1], vec_target[0]) - math.atan2(vec_robot[1], vec_robot[0]))

                    while rel_angle > 180: rel_angle -= 360
                    while rel_angle < -180: rel_angle += 360

                    real_dist = np.linalg.norm(tvecs[i0] - tvecs[i1])
                    current_time = time.time()

                    if real_dist < self.STOP_DISTANCE:
                        if not self.sent_stop:
                            self.pub_action.publish(Pose2D(x=0.0, y=0.0, theta=0.0))
                            self.sent_stop = True
                    else:
                        self.sent_stop = False
                        if self.robot_status == "IDLE" or (current_time - self.last_command_time > self.MIN_COMMAND_INTERVAL):
                            action = Pose2D()
                            action.x = -real_dist if self.REVERSE_DRIVE else real_dist
                            action.theta = -rel_angle if self.REVERSE_STEER else rel_angle
                            self.pub_action.publish(action)
                            self.last_command_time = current_time

                    color = (0, 0, 255) if self.sent_stop else (0, 255, 0)
                    cv2.line(frame, pt_h1, pt_t0, (255, 0, 0), 2)
                    cv2.circle(frame, pt_h1, 5, color, -1)
                    cv2.putText(frame, f"{real_dist:.1f}cm", (10, 30), 1, 1.5, (255, 255, 255), 2)

                except Exception:
                    pass

            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.pub_image.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"Publish Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MultiArucoFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
