from setuptools import find_packages, setup

package_name = 'my_robot_yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zeus',
    maintainer_email='bruno_mello@edu.univali.br',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "camera_no = my_robot_yolo.camera:main",
            "yolov8_no = my_robot_yolo.yolov8_no:main",
            "yolov8_apont = my_robot_yolo.yolov8_apont:main",
            "debug_no = my_robot_yolo.debug_no:main"
        ],
    },
)
