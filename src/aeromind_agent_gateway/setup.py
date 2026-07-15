from setuptools import find_packages, setup


package_name = "aeromind_agent_gateway"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    package_data={package_name: ["skill_docs/*/SKILL.md", "skill_docs/*/skill.yaml"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
        "claude-agent-sdk>=0.1.62,<0.2",
        "fastapi>=0.115,<1",
        "httpx>=0.27,<1",
        "uvicorn[standard]>=0.34,<1",
        "lark-oapi>=1.4,<2",
        "Pillow>=9,<13",
        "PyYAML>=5.4,<7",
    ],
    zip_safe=True,
    maintainer="AeroMind Team",
    maintainer_email="team@aeromind.local",
    description="Claude Agent SDK session gateway for Web and messaging channels",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "agent_gateway = aeromind_agent_gateway.main:main",
        ],
    },
)
