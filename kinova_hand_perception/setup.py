from setuptools import find_packages, setup

package_name = "kinova_hand_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nidhi Sakpal",
    maintainer_email="nidhi@example.com",
    description="MediaPipe RGB-D hand tracking for Kinova teleoperation",
    license="Apache-2.0",
    entry_points={"console_scripts": ["hand_tracker = kinova_hand_perception.hand_tracker:main"]},
)

