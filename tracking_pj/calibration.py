import cv2
import numpy as np
import os

# --- 설정 ---
CHECKERBOARD = (6, 9)  # 체커보드 내부 코너 개수 (가로, 세로)
SQUARE_SIZE = 22.0    # 체커보드 한 칸의 실제 크기 (mm 단위)

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

print("준비되면 's'를 눌러 캡처하세요. 최소 10장 이상 필요합니다. 종료는 'q'.")

count = 0
while True:
    ret, frame = cap.read()
    if not ret: break
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # 체커보드 코너 찾기
    ret_found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    display_frame = frame.copy()
    if ret_found:
        # 화면에 코너 그려주기
        cv2.drawChessboardCorners(display_frame, CHECKERBOARD, corners, ret_found)

    cv2.imshow('Calibration', display_frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('s') and ret_found:
        objpoints.append(objp)
        imgpoints.append(corners)
        count += 1
        print(f"캡처 성공! 현재 데이터 수: {count}")

    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# 캘리브레이션 계산
if count > 10:
    print("계산 중... 잠시만 기다려주세요.")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    
    # 결과 저장 (이 파일이 나중에 ArUco 노드에서 사용됨)
    np.savez("calibration_data.npz", mtx=mtx, dist=dist)
    print("캘리브레이션 완료! 'calibration_data.npz' 파일이 생성되었습니다.")
    print("카메라 행렬(mtx):\n", mtx)
    print("왜곡 계수(dist):\n", dist)
else:
    print("데이터가 부족하여 계산을 취소합니다.")
