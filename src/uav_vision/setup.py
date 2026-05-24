from glob import glob
from setuptools import find_packages, setup

package_name = "uav_vision"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            "share/" + package_name + "/config",
            glob("config/*"),
        ),
        (
            "share/" + package_name + "/launch",
            glob("launch/*.py"),
        ),
        (
            "share/" + package_name + "/calibration",
            glob("calibration/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="orangepi",
    maintainer_email="orangepi@todo.todo",
    description="Vision package for DATN UAV",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "aruco_detector_node = uav_vision.aruco_detector_node:main",
        ],
    },
)