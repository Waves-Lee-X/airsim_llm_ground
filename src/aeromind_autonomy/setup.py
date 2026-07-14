from setuptools import setup

package_name = "aeromind_autonomy"

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
    description="Real-time local autonomy and obstacle avoidance",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "autonomy_node = aeromind_autonomy.autonomy_node:main",
        ],
    },
)

