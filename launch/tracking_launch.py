import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 공유 디렉토리 경로 (설정 파일 로드용)
    pkg_share_dir = get_package_share_directory('tracking_pj')

    # Flask 앱 경로 (사용자 소스 경로 유지)
    app_path = os.path.join('/home/hwibuk/ros2_ws/src/tracking_pj/tracking_pj', 'app.py')

    return LaunchDescription([
        # 1. 피추적차 브릿지 (robot_0)
        # 웹 키보드 명령(/robot_0/cmd_vel)을 받아 ID 0번 ESP32로 전달
        Node(
            package='tracking_pj',
            executable='target_serial_bridge_node',
            name='serial_bridge_node_0',
            namespace='robot_0',
            parameters=[{'robot_name': 'robot_0'}],
            output='screen'
        ),

        # 2. 추적차 브릿지 (robot_1)
        # 트래킹 노드 명령(/robot_1/action)을 받아 ID 1번 ESP32로 전달
        Node(
            package='tracking_pj',
            executable='serial_bridge_node',
            name='serial_bridge_node_1',
            namespace='robot_1',
            parameters=[{'robot_name': 'robot_1'}],
            output='screen'
        ),

        # 3. 트래킹 노드 (ArUco 추적 및 명령 계산)
        # /robot_1/action 토픽으로 추적 명령 발행
        Node(
            package='tracking_pj',
            executable='tracking_node',
            name='multi_aruco_follower',
            output='screen'
        ),

        # 4. Rosbridge WebSocket (웹과 ROS 통신용)
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_server',
            output='screen'
        ),

        # 5. Web Video Server (웹에 카메라 화면 송출용)
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen'
        ),

        # 6. Flask 웹 서버 실행
        ExecuteProcess(
            cmd=['python3', app_path],
            output='screen'
        )
    ])
