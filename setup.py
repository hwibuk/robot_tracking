from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tracking_pj'

setup(
    name=package_name,
    version='0.0.0',
    # packages=find_packages(exclude=['test']) 대신 명확하게 패키지 지정
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 만약 launch 폴더를 만드실 계획이라면 아래 주석을 해제하세요
        # (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hwibuk',
    maintainer_email='hwibuk@todo.todo',
    description='RC Car Tracking Project with ROS 2, ESP32, and Arduino',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # '실행할_이름 = 패키지명.파일명:메인함수'
            'webcam_node = tracking_pj.webcam_node:main',
            'serial_bridge_node = tracking_pj.serial_bridge_node:main',
        ],
    },
)
