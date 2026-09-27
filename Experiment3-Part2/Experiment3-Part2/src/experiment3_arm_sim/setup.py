import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'experiment3_arm_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
        (
            os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')
        ),
        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')
        ),
        (
            os.path.join('share', package_name, 'models',
                         'tape_ring', 'meshes'),
            glob('models/tape_ring/meshes/*')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Experiment 3 automatic object sorting simulation for mechArm 270',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pick_and_place_server = experiment3_arm_sim.pick_and_place_server:main',
            'auto_square_sort = experiment3_arm_sim.auto_square_sort:main',
            'auto_circle_sort = experiment3_arm_sim.auto_circle_sort:main',
            'auto_full_sort = experiment3_arm_sim.auto_full_sort:main',
        ],
    },
)
