import os
from glob import glob

from setuptools import setup


package_name = "abb_irb4600_gazebo"


setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.xacro")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Shay-7278",
    maintainer_email="an7278@princeton.edu",
    description="Gazebo control integration for the dual ABB IRB4600 workcell.",
    license="Apache-2.0",
)
