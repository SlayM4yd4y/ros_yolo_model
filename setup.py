from setuptools import setup

package_name = 'ros_yolo_model'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'detector'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/yolo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='M4yd4y',
    maintainer_email='attilaotvos8@g.com',
    description='YOLO object detection using ROS2',
    license='GNU GENERAL PUBLIC LICENSE 3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector = detector:main',
        ],
    },
)
