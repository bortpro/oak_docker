from setuptools import setup

package_name = 'oak_camera_capture'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dipro',
    maintainer_email='diproc@stanford.edu',
    description='OAK Camera Image Capture',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros_grab = oak_camera_capture.ros_grab_node:main',
            'ros_video = oak_camera_capture.ros_video_node:main',
            'device_discovery = oak_camera_capture.device_discovery:main',
        ],
    },
)
