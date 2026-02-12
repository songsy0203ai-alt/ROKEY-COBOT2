from setuptools import setup
import os
from glob import glob

package_name = 'gemini_robot_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch 파일 등록
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Config 및 데이터 폴더 등록
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'data/calibration'), glob('data/calibration/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Seoyoung Song',
    maintainer_email='songsy0203ai@gmail.com', 
    description='Gemini Robotics ER 2.5 based Collaborative Robot Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # scripts.calib_node 가 아니라 패키지명.파일명 입니다.
            'calib_node = gemini_robot_pkg.calib_node:main',
            'main_node = gemini_robot_pkg.main_node:main',
            'eye = gemini_robot_pkg.eye:main',
            'brain = gemini_robot_pkg.brain:main',
            'nerve = gemini_robot_pkg.nerve:main',
        ],
    },
)