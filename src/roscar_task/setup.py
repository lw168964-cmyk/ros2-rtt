from setuptools import find_packages, setup

package_name = 'roscar_task'

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
    maintainer='sc',
    maintainer_email='2580329627@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'init_robot_pose = roscar_task.init_robot_pose:main',
            'get_robot_pose = roscar_task.get_robot_pose:main',
            'nav_to_pose = roscar_task.nav_to_pose:main',
            'waypoint_follow = roscar_task.waypoint_follow:main',
        ],
    },
)
