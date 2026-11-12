import os
from glob import glob

from setuptools import setup

package_name = "abb_irb4600_description"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # URDF/Xacro
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.xacro") + glob("urdf/*.urdf")),
        # Meshes
        (os.path.join("share", package_name, "meshes", "collision"), glob("meshes/collision/*")),
        (os.path.join("share", package_name, "meshes", "visual"), glob("meshes/visual/*")),
        # Launch files
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        # # RViz config
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Daniel Ruan",
    maintainer_email="daniel.ruan@princeton.edu",
    description="URDF/Xacro and meshes for the ABB IRB4600-40/2.55 industrial robot.",
    license="Apache-2.0",
)
