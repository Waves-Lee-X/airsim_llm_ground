from setuptools import setup

package_name = "aeromind_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AeroMind Team",
    maintainer_email="team@aeromind.local",
    description="键盘遥控节点",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "teleop_node = aeromind_teleop.teleop_node:main",
            "console_node = aeromind_teleop.console_node:main",
        ],
    },
)
