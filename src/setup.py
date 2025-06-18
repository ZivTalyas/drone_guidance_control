from setuptools import find_packages, setup # Use find_packages

package_name = 'drone_control_pkg'

setup(
    name=package_name,
    version='0.0.1', # Slightly incremented version
    packages=find_packages(exclude=['test']), # Use find_packages
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # If you have launch files, add them here
        # (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools', 'rclpy', 'numpy', 'mavsdk'], # Added dependencies
    zip_safe=True,
    maintainer='galsiton', # Keep original maintainer
    maintainer_email='galsit@post.bgu.ac.il', # Keep original email
    description='ROS2 package for drone control using MAVSDK.', # Updated description
    license='Apache License 2.0', # Changed to a common open-source license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "trajectory_maker_node = drone_control_pkg.trajectory_maker:main",
            "acceleration_frd_node = drone_control_pkg.acceleration_frd:main",
            "close_loop_control_node = drone_control_pkg.close_loop_control:main",
            "attitude_quaternion_node = drone_control_pkg.attitude_quaternion:main",
            "acceleration_tune_node = drone_control_pkg.acceleration_tune_node:main",
        ],
    },
)
