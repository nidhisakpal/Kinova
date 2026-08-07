import os
from glob import glob
from setuptools import find_packages, setup

package_name = "kinova_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nidhi Sakpal",
    maintainer_email="nidhi@example.com",
    description="Safe hand-gesture teleoperation for Kinova Gen3 robots",
    license="Apache-2.0",
    entry_points={"console_scripts": ["teleop_bridge = kinova_teleop.teleop_bridge:main"]},
)

