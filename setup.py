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
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # 파일이 설치 경로(install/)로 제대로 복사되도록 설정됨
        (os.path.join('share', package_name, 'templates'), glob('templates/*.html')),
        (os.path.join('share', package_name, 'static'), glob('static/*')),
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
            'serial_bridge_node = tracking_pj.serial_bridge_node:main',
            'flask_app = tracking_pj.app:main', # Flask 노드 추가
        ],
    },
)
