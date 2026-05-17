"""Contactile PTS 触觉传感器可视化工具 — 包安装配置。

定义该 ROS2 包的 setuptools 安装配置，包括依赖声明、数据文件映射和
命令行入口点。
"""

# ===== 包构建工具导入 =====
from setuptools import find_packages, setup

# ===== 包名 =====
package_name = "contactile_visualizer"

# ===== 安装配置 =====
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # ROS2 ament 资源索引
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        # 包清单与启动文件
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/visualizer.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xiaodalaing",
    maintainer_email="xiaodalaing@example.com",
    description="Realtime Contactile PTS tactile sensor visualizer.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # 注册 tactile_gui 命令行工具
            "tactile_gui = contactile_visualizer.app:cli",
        ],
    },
)
