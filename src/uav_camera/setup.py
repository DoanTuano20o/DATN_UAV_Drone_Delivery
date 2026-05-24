from setuptools import find_packages, setup

package_name = "uav_camera"

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
            ["config/camera.yaml"],
        ),
        (
            "share/" + package_name + "/launch",
            ["launch/camera.launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="orangepi",
    maintainer_email="orangepi@todo.todo",
    description="Camera package for DATN UAV",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_node = uav_camera.camera_node:main",
        ],
    },
)