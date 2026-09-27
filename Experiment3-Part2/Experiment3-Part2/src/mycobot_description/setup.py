from glob import glob
from setuptools import setup

package_name = 'mycobot_description'

setup(
    name=package_name,
    version='0.4.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'LICENSE']),
        ('share/' + package_name + '/urdf/mecharm_270_pi', glob('urdf/mecharm_270_pi/*')),
        ('share/' + package_name + '/urdf/adaptive_gripper', glob('urdf/adaptive_gripper/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Elephant Robotics / Group 5 coursework snapshot',
    maintainer_email='noreply@example.com',
    description='Minimal mechArm270 Pi and adaptive-gripper description assets required by Experiment 3 Part 2.',
    license='BSD-2-Clause',
)
