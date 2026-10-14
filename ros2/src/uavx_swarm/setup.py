from glob import glob

from setuptools import setup

package_name = "uavx_swarm"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name, f"{package_name}.nodes"],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="UAV-X team",
    maintainer_email="uavx@users.noreply.github.com",
    description="UAV-X Stage 2 swarm autonomy on ArduPilot SITL + Gazebo + ROS 2",
    license="MIT",
    entry_points={
        "console_scripts": [
            "bench_check = uavx_swarm.nodes.bench_check:main",
            "companion = uavx_swarm.nodes.companion:main",
            "poi_detector = uavx_swarm.nodes.poi_detector:main",
            "radio_net = uavx_swarm.nodes.radio_net:main",
            "gcs = uavx_swarm.nodes.gcs:main",
            "scenario_manager = uavx_swarm.nodes.scenario_manager:main",
            "mission_monitor = uavx_swarm.nodes.mission_monitor:main",
            "inject = uavx_swarm.nodes.inject:main",
        ],
    },
)
