import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np

class ArucoCameraNode(Node):
    def __init__(self):
        super().__init__('aruco_webcam_node')
        # 큐 사이즈를 1로 줄여 최신 데이터 우선 처리 (지연 감소)
        self.publisher_ = self.create_publisher(Image, 'image_raw', 1)
        self.timer = self.create_timer(0.033, self.timer_callback) # 약 30fps

        # 1. V4L2 드라이버 및 백엔드 설정
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

        # 2. MJPG 압축 및 해상도 설정 (WSL2 필수)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # 중요: 버퍼 사이즈 최소화 (지연 시간 방지)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.bridge = CvBridge()

        # 3. ArUco 설정 (버전에 상관없이 호환되도록 구버전 API 기반 작성)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

    def timer_callback(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn('카메라 프레임을 읽을 수 없습니다.')
            return

        # --- ArUco 인식 로직 추가 ---
        # 그레이스케일 변환 (인식 속도 향상)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 마커 탐지
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        # 마커가 발견되면 화면에 그리기
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            # 로그에 발견된 ID 출력 (너무 자주 찍히지 않게 주의)
            # self.get_logger().info(f'Detected IDs: {ids.flatten()}')

        # --- 이미지 발행 ---
        try:
            # 병목을 줄이기 위해 인코딩을 빠르게 수행
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f'이미지 발행 실패: {e}')

        # WSL GUI 확인용 (필요 없으면 주석 처리하여 성능 향상 가능)
        cv2.imshow('WSL ArUco Detection', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info('사용자 요청으로 종료합니다.')
            # 여기서 rclpy.shutdown()을 직접 부르지 않고 루프를 빠져나가게 유도

def main(args=None):
    rclpy.init(args=args)
    node = ArucoCameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT) 감지')
    finally:
        # 종료 절차 강화: 카메라 해제 후 노드 파괴
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
