import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import math
import os
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()

        # 1. 캘리브레이션 데이터 로드
        self.load_params()

        # 2. 통신 설정
        self.sub_image = self.create_subscription(Image, '/usb_camera/image_raw', self.image_callback, 10)
        self.pub_pose = self.create_publisher(Pose2D, '/target/relative_pose', 10)
        self.pub_debug_img = self.create_publisher(Image, '/camera/debug_image', 10)

        # 3. 아루코 설정
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.get_logger().info("Aruco Detector Node Started")

    def load_params(self):
        path = '/home/hwibuk/ros2_ws/src/tracking_pj/tracking_pj/calibration_data.npz'
        if not os.path.exists(path):
            self.get_logger().error(f"Calibration file missing: {path}")
            return
        data = np.load(path)
        self.mtx, self.dist = data['mtx'], data['dist']

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 5.0, self.mtx, self.dist)
            id_list = ids.flatten().tolist()

            if 0 in id_list and 1 in id_list:
                i0, i1 = id_list.index(0), id_list.index(1)

                # 좌표 계산 및 시각화용 포인트 추출
                img_c1, _ = cv2.projectPoints(np.array([[0,0,0]], dtype=np.float32), rvecs[i1], tvecs[i1], self.mtx, self.dist)
                img_h1, _ = cv2.projectPoints(np.array([[0,15,0]], dtype=np.float32), rvecs[i1], tvecs[i1], self.mtx, self.dist)
                img_t0, _ = cv2.projectPoints(np.array([[0,6,0]], dtype=np.float32), rvecs[i0], tvecs[i0], self.mtx, self.dist)

                p1, p2, p3 = img_c1[0][0], img_h1[0][0], img_t0[0][0]

                # 각도 및 거리 계산
                angle = math.degrees(math.atan2(p3[1]-p1[1], p3[0]-p1[0]) - math.atan2(p2[1]-p1[1], p2[0]-p1[0]))
                while angle > 180: angle -= 360
                while angle < -180: angle += 360
                dist = np.linalg.norm(tvecs[i0] - tvecs[i1])

                # 결과 발행 (x에 거리, theta에 각도 저장)
                self.pub_pose.publish(Pose2D(x=float(dist), y=0.0, theta=float(angle)))

                # 디버그 영상 생성
                cv2.line(frame, tuple(p2.astype(int)), tuple(p3.astype(int)), (255,0,0), 2)
                cv2.putText(frame, f"{dist:.1f}cm", (10,30), 1, 1.5, (0,255,0), 2)

        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        self.pub_debug_img.publish(self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ArucoDetector())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
