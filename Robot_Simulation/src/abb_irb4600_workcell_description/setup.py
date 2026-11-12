from setuptools import setup
from glob import glob
import os

package_name = "abb_irb4600_workcell_description"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Config (tools.yaml and any future package-level config)
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        # URDF/Xacro
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.xacro")),
        (os.path.join("share", package_name, "urdf", "fixtures"), glob("urdf/fixtures/*.xacro")),
        (os.path.join("share", package_name, "urdf", "end_effectors"), glob("urdf/end_effectors/*.xacro")),
        # Meshes — workcell fixtures
        (os.path.join("share", package_name, "meshes", "collision", "fixtures"), glob("meshes/collision/fixtures/*")),
        (os.path.join("share", package_name, "meshes", "visual", "fixtures"), glob("meshes/visual/fixtures/*")),
        # Meshes — end-effectors
        (os.path.join("share", package_name, "meshes", "collision", "end_effectors"), glob("meshes/collision/end_effectors/*")),
        (os.path.join("share", package_name, "meshes", "visual", "end_effectors"), glob("meshes/visual/end_effectors/*")),
        # Launch files
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        # # RViz config
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Daniel Ruan",
    maintainer_email="daniel.ruan@princeton.edu",
    description="URDF/Xacro and meshes for ABB IRB4600 workcell at Princeton University.",
    license="Apache-2.0",
)
