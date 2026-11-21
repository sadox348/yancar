from glob import glob
from setuptools import find_packages, setup


package_name = "stm32_base_bridge"

setup(
    name=package_name,
    version="0.9.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="yancar",
    maintainer_email="yancar@todo.todo",
    description="ROS 2 serial bridge and odometry for an STM32F407 AGV base.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "stm32_base_bridge = stm32_base_bridge.bridge_node:main",
        ],
    },
)

