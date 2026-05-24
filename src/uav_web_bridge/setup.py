from setuptools import find_packages, setup

package_name = "uav_web_bridge"

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
            "lib/" + package_name,
            ["scripts/web_server_node"],
        ),
    ],
    package_data={
        "uav_web_bridge": [
            "app/templates/*.html",
            "app/static/css/*",
            "app/static/js/*",
            "app/static/img/*",
            "app/static/assets/images/*",
            "app/static/assets/videos/*",
            "app/static/assets/icons/*",
        ],
    },
    include_package_data=True,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="orangepi",
    maintainer_email="orangepi@todo.todo",
    description="Web bridge node for DATN UAV",
    license="MIT",
    tests_require=["pytest"],
)
