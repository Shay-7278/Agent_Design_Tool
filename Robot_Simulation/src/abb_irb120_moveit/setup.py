from setuptools import setup
import os
from glob import glob

package_name = "abb_irb120_moveit"

def glob_dir(d):
    return glob(os.path.join(d, "**"), recursive=True)

data_files = [
    ("share/ament_index/resource_index/packages", [os.path.join("resource", package_name)]),
    (os.path.join("share", package_name), ["package.xml"]),
]

# Install launch/ and config/ (recursively) if they exist.
for folder in ["launch", "config"]:
    if os.path.isdir(folder):
        files = [f for f in glob_dir(folder) if os.path.isfile(f)]
        # Preserve subfolders under share/<pkg>/<folder>/...
        for f in files:
            rel_dir = os.path.dirname(f)
            data_files.append((os.path.join("share", package_name, rel_dir), [f]))

# Install the marker file from MoveIt Setup Assistant if present.
if os.path.isfile(".setup_assistant"):
    data_files.append((os.path.join("share", package_name), [".setup_assistant"]))

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Daniel Ruan",
    maintainer_email="daniel.ruan@princeton.edu",
    description="ABB IRB120 MoveIt config package (pure ament_python for cross-platform installs).",
    license="Apache-2.0",
    entry_points={},
)