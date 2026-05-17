"""触觉抓取控制器包的安装配置。

定义 ROS 2 包的 setuptools 打包信息，包括节点入口点和数据文件。
"""

from setuptools import find_packages, setup

package_name = "tactile_grasp_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["README.md"]),
        (
            "share/" + package_name + "/config",
            [
                "config/tactile_grasp_controller.yaml",
                "config/tactile_grasp_controller.hardware.yaml",
                "config/hardware_bringup_coordinator.yaml",
            ],
        ),
        (
            "share/" + package_name + "/launch",
            [
                "launch/tactile_grasp_controller.launch.py",
                "launch/tactile_grasp_bringup.launch.py",
                "launch/tactile_grasp_hardware_bringup.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Repository User",
    maintainer_email="user@example.com",
    description="ROS 2 tactile grasp controller package for the Robotiq 2F-85 stack.",
    license="Proprietary",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "tactile_grasp_controller_node = tactile_grasp_controller.tactile_grasp_controller_node:main",
            "hardware_bringup_coordinator = tactile_grasp_controller.hardware_bringup_coordinator:main",
        ],
    },
)
