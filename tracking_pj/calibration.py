import cv2
import numpy as np
import time  # 시간 측정을 위해 추가

# --- 설정 ---
CHECKERBOARD = (6, 9)  # 체커보드 내부 코너 개수 (가로, 세로)
SQUARE_SIZE = 22.0    # 체커보드 한 칸의 실제 크기 (mm 단위)
CAPTURE_INTERVAL = 1.0 # 자동 캡쳐 간격 (초)

# 체커보드의 실제 3D 좌표 정의
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints = [] # 실제 세계의 3D 점들
imgpoints = [] # 이미지 상의 2D 점들

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("자동 캡쳐 모드 시작...")
print("체커보드를 화면에 비추면 0.5초마다 자동으로 저장됩니다.")
print("최소 20장 이상 다양한 각도에서 촬영하세요. 종료는 'q'.")

count = 0
last_capture_time = time.time()  # 마지막 캡쳐 시간 초기화

while True:
    ret, frame = cap.read()
    if not ret: break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # 체커보드 코너 찾기
    ret_found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    display_frame = frame.copy()
    current_time = time.time()

    if ret_found:
        # 화면에 코너 그려주기
        cv2.drawChessboardCorners(display_frame, CHECKERBOARD, corners, ret_found)

        # --- 자동 캡쳐 로직 ---
        # 1. 체커보드가 발견됨 (ret_found == True)
        # 2. 마지막 캡쳐로부터 설정한 간격(0.5초)이 지남
        if current_time - last_capture_time >= CAPTURE_INTERVAL:
            objpoints.append(objp)
            imgpoints.append(corners)
            count += 1
            last_capture_time = current_time  # 캡쳐 시간 업데이트
            print(f"자동 캡쳐 성공! (데이터 수: {count})")

            # 캡쳐 시 화면에 피드백 (잠시 하얗게 반전)
            display_frame = cv2.bitwise_not(display_frame)

    # 안내 문구 출력
    cv2.putText(display_frame, f"Count: {count}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow('Auto Calibration', display_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# 캘리브레이션 계산
if count >= 15: # 자동 캡쳐이므로 조금 더 넉넉하게 수집 권장
    print("\n계산 중... 잠시만 기다려주세요.")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # 결과 저장
    np.savez("calibration_data.npz", mtx=mtx, dist=dist)
    print("캘리브레이션 완료! 'calibration_data.npz' 파일이 생성되었습니다.")
    print(f"최종 재투영 에러(RMS Error): {ret}") # 정확도 확인용
else:
    print(f"\n데이터 부족(현재 {count}장). 최소 15장 이상 촬영해야 계산이 시작됩니다.")
