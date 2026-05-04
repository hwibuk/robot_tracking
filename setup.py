from setuptools import setup
import os
from glob import glob

package_name = 'tracking_pj'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 런치 파일 설치
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # 웹 템플릿 및 정적 파일
        (os.path.join('share', package_name, 'templates'), glob('templates/*.html')),
        (os.path.join('share', package_name, 'static'), glob('static/*')),
        # YAML 설정 파일 및 기타 데이터(npz 등) 설치 경로 등록
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # 만약 npz 파일을 패키지 안에 두었다면 아래와 같이 추가할 수 있습니다.
        # (os.path.join('share', package_name), glob('tracking_pj/*.npz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hwibuk',
    maintainer_email='hwibuk@todo.todo',
    description='ROS 2 Robot Tracking Project - Separated Nodes',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 분리된 3개의 핵심 노드
            'webcam_node = tracking_pj.webcam_node:main',
            'aruco_pos_node = tracking_pj.aruco_pos:main',
            'tracking_node = tracking_pj.tracking_node:main',

            # 기존 시리얼 브릿지 및 웹 서버 노드
            'serial_bridge_node = tracking_pj.serial_bridge_node:main',
            'target_serial_bridge_node = tracking_pj.target_serial_bridge_node:main',
            'flask_app = tracking_pj.app:main',
        ],
    },
)
