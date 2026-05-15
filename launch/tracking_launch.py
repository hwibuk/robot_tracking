import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 공유 디렉토리 경로
    pkg_share_dir = get_package_share_directory('tracking_pj')

    # Flask 앱 경로
    app_path = os.path.join(pkg_share_dir, 'tracking_pj', 'app.py')
    return LaunchDescription([
        # 1. 피추적차 브릿지 (robot_0)
        Node(
            package='tracking_pj',
            executable='target_serial_bridge_node',
            name='serial_bridge_node_0',
            namespace='robot_0',
            parameters=[{'robot_name': 'robot_0'}],
            output='screen'
        ),

        # 2. 추적차 브릿지 (robot_1)
        Node(
            package='tracking_pj',
            executable='serial_bridge_node',
            name='serial_bridge_node_1',
            namespace='robot_1',
            parameters=[{'robot_name': 'robot_1'}],
            output='screen'
        ),

        # --- 분리된 트래킹 시스템 3개 노드 ---

        # 3-1. 웹캠 노드 (영상 획득 및 송출)
        Node(
            package='tracking_pj',
            executable='webcam_node',
            name='webcam_node',
            output='screen'
        ),

        # 3-2. ArUco 포지션 노드 (마커 인식 및 거리/각도 계산)
        Node(
            package='tracking_pj',
            executable='aruco_pos_node',
            name='aruco_pos_node',
            output='screen'
        ),

        # 3-3. 트래킹 컨트롤 노드 (제어 알고리즘 및 명령 발행)
        Node(
            package='tracking_pj',
            executable='tracking_node',
            name='tracking_controller',
            output='screen'
        ),

        # ----------------------------------

        # 4. Rosbridge WebSocket
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_server',
            output='screen'
        ),

        # 5. Web Video Server
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
