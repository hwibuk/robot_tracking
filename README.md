ardu.ino : 아두이노 코드  
esp.ino : esp32 코드  
app.py : 웹 연결을 위한 flask서버  
aruco_pos.py : 아르코 마커 인식, 상대 거리와 각도 발행  
calch.py : calibration_data.npz 계수 확인  
calibration : 체커보드 캡쳐로 변환계수 추출 후 calibration_data.npz에 저장  
serial_bridge_node.py : 추적차 아두이노에 명령 전달  
target_serial_bridge_node.py : 피추적차 아두이노에 명령 전달  
tracking_node : 트래킹 알고리즘에 따라 serial_bridge_node.py에 명령 전달  
webcam_node : usb 카메라 영상을 토픽으로 송출  
  



이미지 다운로드
docker pull ghcr.io/hwibuk/robot_tracking:latest

도커런
docker run -it \
  --rm \
  --device=/dev/video0:/dev/video0 \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ghcr.io/hwibuk/robot_tracking:latest
