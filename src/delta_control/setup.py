import os
from glob import glob

from setuptools import find_packages, setup


package_name = "delta_control"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tuan",
    maintainer_email="tuan@todo.todo",
    description="Delta robot Cartesian jog and trajectory GUI",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "delta_gui = delta_control.gui:main",
            "workspace_benchmark = delta_control.workspace_benchmark:main",
        ],
    },
)
