from setuptools import setup, find_packages

package_name = 'dsr_cumotion_goal_interface'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/start_motion_command.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Minsoo Song',
    maintainer_email='minsoo.song@doosan.com',
    author='Gijung Nam',
    author_email='rlwnd0122@gmail.com',
    description='cuMotion goal interface (pose/joint/named/relative) with clean reset per command',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'move_command_node = dsr_cumotion_goal_interface.move_command_node:main',
        ],
    },
)
