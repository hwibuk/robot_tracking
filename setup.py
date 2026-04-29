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
        # [추가] YAML 설정 파일 설치 경로 등록
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hwibuk',
    maintainer_email='hwibuk@todo.todo',
    description='ROS 2 Robot Tracking Project',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webcam_node = tracking_pj.webcam_node:main',
            'serial_bridge_node = tracking_pj.serial_bridge_node:main', # 추적차(ID 1)
            'target_serial_bridge_node = tracking_pj.target_serial_bridge_node:main', # [추가] 피추적차(ID 0)
            'flask_app = tracking_pj.app:main',
            'tracking_node = tracking_pj.tracking_node:main',
        ],
    },
)
