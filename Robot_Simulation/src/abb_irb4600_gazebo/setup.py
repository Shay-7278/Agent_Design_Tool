import os
from glob import glob

from setuptools import find_packages, setup


package_name = "abb_irb4600_gazebo"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config"), glob("config/*.json")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.xacro")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.sdf")),
        (os.path.join("share", package_name, "models", "brick"), glob("models/brick/*")),
    ],
    install_requires=["setuptools"],
    entry_points={"console_scripts": [
        "spawn_pickup_grid = abb_irb4600_gazebo.startup_brick_spawner:main",
        "execute_one_brick = abb_irb4600_gazebo.execute_one_brick:main",
        "execute_wall = abb_irb4600_gazebo.execute_one_brick:wall_main",
    ]},
    zip_safe=True,
    maintainer="Shay-7278",
    maintainer_email="an7278@princeton.edu",
    description="Gazebo control integration for the dual ABB IRB4600 workcell.",
    license="Apache-2.0",
)
