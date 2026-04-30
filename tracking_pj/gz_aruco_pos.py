import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import math
import os
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose  # Pose2D에서 Pose로 변경
from cv_bridge import CvBridge

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_pos_node')
        self.bridge = CvBridge()
        self.load_params()
        
        # Gazebo 카메라 토픽명 확인 필요 (보통 /camera/image_raw 또는 /usb_camera/image_raw)
        self.sub_image = self.create_subscription(Image, '/usb_camera/image_raw', self.image_callback, 10)
        self.pub_pose = self.create_publisher(Pose, '/target/relative_pose', 10)
        
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.get_logger().info("Gazebo Aruco Pose Node Started (Standard Pose)")

    def load_params(self):
        # 가제보 가상 카메라의 경우 실제 캘리브레이션 mtx가 필요함
        path = '/home/hwibuk/ros2_ws/src/tracking_pj/tracking_pj/calibration_data.npz'
        if os.path.exists(path):
            data = np.load(path)
            self.mtx, self.dist = data['mtx'], data['dist']
        else:
            # 파일이 없을 경우 기본값 (가상 카메라는 보통 왜곡이 없음)
            self.mtx = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=np.float32)
            self.dist = np.zeros(5)

    def euler_to_quaternion(self, yaw_deg):
        """오일러 각도(Degree)를 쿼터니언으로 변환 (Z축 회전만 고려)"""
        yaw_rad = math.radians(yaw_deg)
        return {
            'x': 0.0,
            'y': 0.0,
            'z': math.sin(yaw_rad / 2.0),
            'w': math.cos(yaw_rad / 2.0)
        }

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            # 5.0은 마커의 실제 크기(cm) -> 가제보 마커 크기에 맞춰 수정 필요
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 5.0, self.mtx, self.dist)
            id_list = ids.flatten().tolist()
            
            if 0 in id_list and 1 in id_list:
                i0, i1 = id_list.index(0), id_list.index(1)
                
                # 이미지 좌표 투영 (각도 계산용)
                img_c1, _ = cv2.projectPoints(np.array([[0,0,0]], dtype=np.float32), rvecs[i1], tvecs[i1], self.mtx, self.dist)
                img_h1, _ = cv2.projectPoints(np.array([[0,15,0]], dtype=np.float32), rvecs[i1], tvecs[i1], self.mtx, self.dist)
                img_t0, _ = cv2.projectPoints(np.array([[0,6,0]], dtype=np.float32), rvecs[i0], tvecs[i0], self.mtx, self.dist)
                
                p1, p2, p3 = img_c1[0][0], img_h1[0][0], img_t0[0][0]
                angle = math.degrees(math.atan2(p3[1]-p1[1], p3[0]-p1[0]) - math.atan2(p2[1]-p1[1], p2[0]-p1[0]))
                while angle > 180: angle -= 360
                while angle < -180: angle += 360

                # 실제 거리 계산 (cm를 m로 변환)
                dist_m = np.linalg.norm(tvecs[i0] - tvecs[i1]) / 100.0

                # Pose 메시지 생성 및 발행
                pose_msg = Pose()
                pose_msg.position.x = dist_m  # 거리(m)
                pose_msg.position.y = 0.0
                pose_msg.position.z = 0.0
                
                q = self.euler_to_quaternion(angle)
                pose_msg.orientation.x = q['x']
                pose_msg.orientation.y = q['y']
                pose_msg.orientation.z = q['z']
                pose_msg.orientation.w = q['w']
                
                self.pub_pose.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ArucoDetector())
    rclpy.shutdown()
