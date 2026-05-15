from setuptools import setup
import os
from glob import glob

# ROS 2 시스템이 인식할 패키지 이름
package_name = 'tracking_pj'

setup(
    name=package_name,
    version='0.0.0',
    # 실제 파이썬 소스 코드가 들어있는 폴더 이름은 그대로 유지
    packages=['tracking_pj'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # 런치 파일 및 데이터 설치 경로
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'templates'), glob('templates/*.html')),
        (os.path.join('share', package_name, 'static'), glob('static/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name), glob('tracking_pj/calibration_data.npz')),
        (os.path.join('share', package_name, 'tracking_pj'), glob('tracking_pj/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hwibuk',
    maintainer_email='hwibuk@todo.todo',
    description='ROS 2 Robot Tracking Project - Backup Version (Source naming kept)',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 핵심: 실행 경로는 [실제폴더명].[파일명]:main 구조를 따라야 합니다.
            'webcam_node = tracking_pj.webcam_node:main',
            'aruco_pos_node = tracking_pj.aruco_pos:main',
            'tracking_node = tracking_pj.tracking_node:main',

            'serial_bridge_node = tracking_pj.serial_bridge_node:main',
            'target_serial_bridge_node = tracking_pj.target_serial_bridge_node:main',
            'flask_app = tracking_pj.app:main',
        ],
    },
)
