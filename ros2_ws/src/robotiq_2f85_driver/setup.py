"""Robotiq 2F-85 夹爪 ROS 2 驱动包的安装配置文件。

定义包的元数据、依赖、数据文件映射以及控制台入口点。
"""

from setuptools import find_packages, setup

package_name = "robotiq_2f85_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),    # 排除测试目录
    data_files=[
        # 资源索引（供 ament 工具发现）
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        # 包描述文件
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["README.md"]),
        # 配置文件（YAML）
        (
            "share/" + package_name + "/config",
            [
                "config/robotiq_2f85_driver.yaml",
                "config/robotiq_2f85_driver.hardware.yaml",
            ],
        ),
        # 启动文件
        ("share/" + package_name + "/launch", ["launch/robotiq_2f85_driver.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Repository User",
    maintainer_email="user@example.com",
    description="ROS 2 dry-run-first driver package for the Robotiq 2F-85 gripper.",
    license="Proprietary",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            # 注册 ROS 2 可执行入口：ros2 run robotiq_2f85_driver robotiq_driver_node
            "robotiq_driver_node = robotiq_2f85_driver.robotiq_driver_node:main",
        ],
    },
)
