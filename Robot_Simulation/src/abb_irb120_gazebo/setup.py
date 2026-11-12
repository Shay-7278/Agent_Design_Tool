from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'abb_irb120_gazebo'

def data_files_from_dir(source_dir, install_dir):
    data_files = []
    for path, _, files in os.walk(source_dir):
        if files:
            install_path = os.path.join(install_dir, os.path.relpath(path, source_dir))
            file_list = [os.path.join(path, f) for f in files]
            data_files.append((install_path, file_list))
    return data_files

data_files = [
    ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
    (f'share/{package_name}', ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    (os.path.join('share', package_name, 'abb_irb120_gazebo'), glob('abb_irb120_gazebo/*')),
]

data_files += data_files_from_dir('models', os.path.join('share', package_name, 'models'))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Ruan',
    maintainer_email='daniel.ruan@princeton.edu',
    description='Package for simulating the ABB IRB120 robot in Gazebo',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'simulation_reset_node = abb_irb120_gazebo.simulation_reset_node:main',
        ],
    },
)