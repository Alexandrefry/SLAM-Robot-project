import os
from glob import glob
from setuptools import find_packages, setup

package_name = "ld06_slam"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alexandre Foray",
    maintainer_email="foray_alexandre@yahoo.fr",
    description="ROS 2 driver and SLAM bringup package for LD06 LiDAR",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ld06_node = ld06_slam.ld06_node:main",
        ],
    },
)
