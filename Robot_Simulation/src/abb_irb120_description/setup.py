from setuptools import setup
from glob import glob
import os

package_name = 'abb_irb120_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # URDF/Xacro
        (os.path.join('share', package_name, 'urdf'),
         glob('urdf/*.xacro') + glob('urdf/*.urdf')),

        # Meshes
        (os.path.join('share', package_name, 'meshes', 'collision'),
         glob('meshes/collision/*.stl')),
        (os.path.join('share', package_name, 'meshes', 'visual'),
         glob('meshes/visual/*.dae')),
         (os.path.join('share', package_name, 'meshes', 'visual'),
         glob('meshes/visual/*.stl')),

        # Launch + RViz configs (remove these two blocks if the folders don't exist yet)
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py') + glob('launch/*.xml') + glob('launch/*.launch.py')),
        # (os.path.join('share', package_name, 'rviz'),
        #  glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Ruan',
    maintainer_email='daniel.ruan@princeton.edu',
    description='URDF/Xacro and meshes for ABB IRB 120.',
    license='Apache-2.0',
)
