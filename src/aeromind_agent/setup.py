from setuptools import find_packages, setup

package_name = "aeromind_agent"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AeroMind Team",
    maintainer_email="team@aeromind.local",
    description="LLM Agent 任务调度节点",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "agent_node = aeromind_agent.agent_node:main",
        ],
    },
)
