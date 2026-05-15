import numpy as np

# 파일 로드
data = np.load('calibration_data.npz')

# 저장된 키(Key) 확인 (보통 mtx, dist 등이 있음)
print("저장된 항목들:", data.files)

# 카메라 행렬 (Intrinsic Matrix)
mtx = data['mtx']
# 왜곡 계수
dist = data['dist']

print("\n[카메라 행렬 (mtx)]")
print(mtx)

print("\n[왜곡 계수 (dist)]")
print(dist)
