"""Sphinx configuration for tactile_grasp_lab."""

from __future__ import annotations

import sys
from pathlib import Path

DOCS_SOURCE = Path(__file__).resolve().parent
REPO_ROOT = DOCS_SOURCE.parents[1]

for pkg in (
    "tactile_grasp_controller",
    "robotiq_2f85_driver",
    "contactile_visualizer",
):
    sys.path.insert(0, str(REPO_ROOT / "ros2_ws" / "src" / pkg))

# -- Project information -----------------------------------------------------
project = "tactile_grasp_lab"
author = "langxin11"
copyright = "2026, langxin11"
version = "0.1.0"
release = "0.1.0"

# -- General configuration ---------------------------------------------------
extensions = [
    "myst_parser",
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
    "sphinx.ext.intersphinx",
    "sphinx_copybutton",
]

templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

source_suffix = {
    ".rst": "restructuredtext",
    ".md": "markdown",
}

language = "zh_CN"

# -- MyST --------------------------------------------------------------------
myst_enable_extensions = [
    "colon_fence",
    "deflist",
    "linkify",
    "substitution",
]
myst_heading_anchors = 3

# -- Autodoc / Napoleon ------------------------------------------------------
autodoc_default_options = {
    "members": True,
    "undoc-members": False,
    "show-inheritance": True,
    "member-order": "bysource",
}
autodoc_mock_imports = [
    "rclpy",
    "rcl_interfaces",
    "std_msgs",
    "sensor_msgs",
    "control_msgs",
    "action_msgs",
    "builtin_interfaces",
    "sensor_interfaces",
    "pymodbus",
    "serial",
]

napoleon_google_docstring = True
napoleon_numpy_docstring = False
napoleon_include_init_with_doc = True
napoleon_use_ivar = True  # 把 Google `Attributes:` 渲染为 ivar，避免与 dataclass 字段重复登记

# -- Warnings ----------------------------------------------------------------
# HARDWARE_BRINGUP.md / historical/codex_plan.md 内残留旧仓库名（robotiq2f85_control）
# 的绝对路径链接，MyST 会把它们当成失败的 xref 报警。这里统一抑制，
# 由源文件后续清理时一并修掉。
suppress_warnings = ["myst.xref_missing"]

# -- Intersphinx -------------------------------------------------------------
intersphinx_mapping = {
    "python": ("https://docs.python.org/3", None),
}

# -- HTML --------------------------------------------------------------------
html_theme = "furo"
html_static_path = ["_static"]
html_title = "tactile_grasp_lab 文档"
