from setuptools import find_packages, setup

package_name = "uav_flight"

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
            ["config/flight.yaml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="orangepi",
    maintainer_email="orangepi@todo.todo",
    description="Flight bridge package for DATN UAV",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "flight_bridge_node = uav_flight.flight_bridge_node:main",
        ],
    },
)