from glob import glob
from setuptools import find_packages, setup


package_name = "rm_safe_demo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="maintainer",
    maintainer_email="maintainer@example.com",
    description="Safety-oriented bringup and small-motion tools for RM65/RM75.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "state_monitor = rm_safe_demo.state_monitor:main",
            "small_joint_move = rm_safe_demo.small_joint_move:main",
            "stop_motion = rm_safe_demo.stop_motion:main",
        ],
    },
)
