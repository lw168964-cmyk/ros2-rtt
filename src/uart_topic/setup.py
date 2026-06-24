from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'uart_topic'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
        glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'pyserial',
        'PyQt5'
        ],
    zip_safe=True,
    maintainer='lpy',
    maintainer_email='3100065913@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'uart_send_receive = uart_topic.uart_send_receive:main',
            'uart_qt = uart_topic.uart_qt:main',
            'uart_keyboard_control = uart_topic.keyboard_control:main',
        ],
    },
)
