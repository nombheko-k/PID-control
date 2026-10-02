from setuptools import setup

package_name = 'pid_tank_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='PID-controlled drone altitude simulator with a Tkinter/Matplotlib GUI for ROS 2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sim_node = pid_tank_sim.sim_node:main',
        ],
    },
)