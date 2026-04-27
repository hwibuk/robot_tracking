import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge

class MultiArucoTracker(Node):
    def __init__(self):
        super().__init__('multi_aruco_tracker')

        # ID 0, 1별 퍼블리셔
        self.pub_0 = self.create_publisher(Pose2D, '/robot_0/pos', 10)
        self.pub_1 = self.create_publisher(Pose2D, '/robot_1/pos', 10)

        self.timer = self.create_timer(0.033, self.timer_callback)

        # --- 업데이트된 캘리브레이션 데이터 ---
        self.mtx = np.array([[1459.88, 0.0, 326.34],
                             [0.0, 1600.67, 244.17],
                             [0.0, 0.0, 1.0]])
        # 왜곡 계수 업데이트
        self.dist = np.array([0.00226, 9.375, -0.0115, 0.00222, -104.07])

        self.marker_length_cm = 5.0

        # 카메라 설정
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret: return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length_cm, self.mtx, self.dist
            )

            for i in range(len(ids)):
                marker_id = ids[i][0]

                if marker_id in [0, 1]:
                    # --- 각도(Theta) 0~359 변환 로직 ---
                    c = corners[i][0]
                    # 마커의 윗변 방향 벡터 기반 각도 계산
                    radians = np.arctan2(c[1][1] - c[0][1], c[1][0] - c[0][0])
                    degrees = np.degrees(radians)
                    theta_360 = float((degrees + 360) % 360)

                    msg = Pose2D()
                    # tvecs 결과값 (cm) 적용
                    msg.x = float(tvecs[i][0][0])
                    msg.y = float(tvecs[i][0][1])
                    msg.theta = theta_360

                    if marker_id == 0:
                        self.pub_0.publish(msg)
                    elif marker_id == 1:
                        self.pub_1.publish(msg)

                    # 디버깅 로그
                    self.get_logger().info(f"ID:{marker_id} | X:{msg.x:.1f}cm Y:{msg.y:.1f}cm Angle:{int(theta_360)}°")

                    # 시각화 (좌표축 그리기)
                    cv2.drawFrameAxes(frame, self.mtx, self.dist, rvecs[i], tvecs[i], 2.0)

            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        cv2.imshow('Aruco Tracker Optimized', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = MultiArucoTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
