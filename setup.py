from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'touch_detection_evaluation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
   data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
             # Include all launch files
            (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
            (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dimitar Dimeski',
    maintainer_email='dimitar.dimeski23@gmail.com',
    description='Package for evaluating touch detection on a table surface using 2 Orbbec RGBD Cameras.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'aruco_display_node = touch_detection_evaluation.show_aruco_display_node:main',
        'screen_calibration_node = touch_detection_evaluation.screen_calibration_node:main',
        'dot_grid_display_node = touch_detection_evaluation.show_dot_grid_display_node:main',
        
        ],
    },
)
