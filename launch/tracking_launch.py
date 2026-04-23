import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지의 설치된 share 디렉토리 경로를 가져옵니다.
    # /home/hwibuk/ros2_ws/install/tracking_pj/share/tracking_pj
    pkg_share_dir = get_package_share_directory('tracking_pj')

    # 실제 app.py가 위치한 소스 경로 (가장 확실한 방법으로 경로 지정)
    # 위에서 확인하신 실제 경로인 /home/hwibuk/ros2_ws/src/tracking_pj/tracking_pj/app.py 를 가리킵니다.
    app_path = os.path.join('/home/hwibuk/ros2_ws/src/tracking_pj/tracking_pj', 'app.py')

    return LaunchDescription([
        # 1. 웹캠 노드
        Node(
            package='tracking_pj',
            executable='webcam_node',
            name='webcam_node'
        ),
        # 2. 시리얼 브릿지 노드
        Node(
            package='tracking_pj',
            executable='serial_bridge_node',
            name='serial_bridge_node'
        ),
        # 3. Rosbridge WebSocket
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_server'
        ),
        # 4. Web Video Server
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server'
        ),
        # 5. Flask 서버 실행 (경로 수정됨)
        ExecuteProcess(
            cmd=['python3', app_path],
            output='screen'
        )
    ])
