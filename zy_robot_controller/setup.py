from setuptools import setup

package_name = 'zy_robot_controller'

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
    maintainer='zytech',
    maintainer_email='zytech@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = zy_robot_controller.my_first_node:main",
            "draw_circle = zy_robot_controller.draw_circle:main",
            "pose_subscriber = zy_robot_controller.pose_subscriber:main",
            "turtle_controller = zy_robot_controller.turtle_controller:main",
            "zy_motor_controller = zy_robot_controller.zy_motor_controller:main"
        ],
    },
)