import os
from glob import glob

from setuptools import setup

package_name = "aeromind_web"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "static"), glob("aeromind_web/static/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AeroMind Team",
    maintainer_email="team@aeromind.local",
    description="AeroMind Web 控制台",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "web_console_node = aeromind_web.web_console_node:main",
        ],
    },
)
