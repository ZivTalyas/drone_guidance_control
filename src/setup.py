from setuptools import setup

package_name = 'my_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='galsiton',
    maintainer_email='galsit@post.bgu.ac.il',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 
            "trajectory_maker = my_py_pkg.trajectory_maker:main"
            "acceleration_frd = my_py_pkg.acceleration_frd:main"
            "close_loop_control = my_py_pkg.close_loop_control:main"
            "attitude_quaternion = my_py_pkg.attitude_quaternion:main"
        ], 
    },
)
