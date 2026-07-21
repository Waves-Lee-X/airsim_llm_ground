from setuptools import setup

package_name = "aeromind_bridge"

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
    description="ROS 2 与 AirSim 桥接节点",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "airsim_bridge_node = aeromind_bridge.airsim_bridge_node:main",
            "slam_pose_node = aeromind_bridge.slam_pose_node:main",
        ],
    },
)
