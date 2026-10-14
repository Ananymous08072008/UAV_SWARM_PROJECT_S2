"""UAV-X Stage 2 - one launch file for every level.

    ros2 launch uavx_swarm uavx.launch.py scenario:=level4 [gui:=true] [run_dir:=...]

Reads the scenario (uavx_swarm/config/<scenario>.yaml) and starts:
  * Gazebo Harmonic server (+ GUI if gui:=true) with the uavx_disaster world
  * /clock from Gazebo (throttled to 100 Hz) - ArduPilot and every node run on sim time
  * per UAV: ArduPilot SITL (lock-step JSON), MAVProxy, the uavx_quad model, its
    ros_gz bridge (camera, rangefinder, GPS, baro, mag, ground truth) - via
    ardupilot_gz_bringup robots/robot.launch.py; one shared micro-ROS agent
  * the uavx_swarm nodes listed under `components:` in the scenario
The run ends when the referee (mission_monitor) or bench_check exits.
"""
import os
import re
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from uavx_swarm.common import load_scenario, resolve_scenario
from uavx_swarm.world_map import WorldMap

PER_UAV = {"bench_check", "companion", "poi_detector"}
SINGLETONS = ["radio_net", "gcs", "scenario_manager", "mission_monitor"]
ENDS_RUN = {"bench_check", "mission_monitor"}


def _prepare_run_dir(run_dir: str, scenario: dict) -> str:
    if not run_dir:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_dir = os.path.join(os.path.expanduser("~/uavx_runs"), f"level{scenario['level']}_{stamp}")
    run_dir = os.path.abspath(os.path.expanduser(run_dir))
    os.makedirs(os.path.join(run_dir, "models"), exist_ok=True)
    return run_dir


def _vehicle_sdf(uav: dict, run_dir: str) -> str:
    src = os.path.join(get_package_share_directory("uavx_gazebo"), "models", "uavx_quad", "model.sdf")
    with open(src, encoding="utf-8") as fh:
        sdf = fh.read()
    rgba = " ".join(f"{c:.3f}" for c in list(uav["color"])[:3]) + " 1"
    sdf = sdf.replace("0.1 0.6 1.0 1", rgba)
    if not uav.get("camera", True):
        sdf = re.sub(r'<sensor name="survey_camera".*?</sensor>', "", sdf, flags=re.S)
    if not uav.get("rangefinder", True):
        sdf = re.sub(r'<sensor name="rangefinder".*?</sensor>', "", sdf, flags=re.S)
    path = os.path.join(run_dir, "models", f"{uav['id']}.sdf")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(sdf)
    return path


def _world_file(scenario: dict, run_dir: str) -> str:
    """The world to load: the base world, plus the scenario's 3-D map, plus a physics override."""
    world = os.path.join(get_package_share_directory("uavx_gazebo"), "worlds", f"{scenario['world']}.sdf")
    step = scenario.get("physics_step_s")
    if not step and not scenario.get("map"):
        return world
    path = os.path.join(run_dir, f"{scenario['world']}.sdf")
    if scenario.get("map"):
        WorldMap.load(scenario["map"]).write_world(world, path)
    else:
        with open(world, encoding="utf-8") as fh, open(path, "w", encoding="utf-8") as out:
            out.write(fh.read())
    if step:
        with open(path, encoding="utf-8") as fh:
            text = fh.read()
        text = re.sub(r"<max_step_size>[^<]*</max_step_size>", f"<max_step_size>{step}</max_step_size>", text)
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(text)
    return path


def _launch(context):
    scenario_arg = LaunchConfiguration("scenario").perform(context)
    scenario_path = resolve_scenario(scenario_arg)
    scenario = load_scenario(scenario_path)
    run_dir = _prepare_run_dir(LaunchConfiguration("run_dir").perform(context), scenario)
    gui = LaunchConfiguration("gui").perform(context).lower() in ("1", "true", "yes")
    headless = LaunchConfiguration("headless_rendering").perform(context).lower() in ("1", "true", "yes")
    # SITL writes eeprom.bin / logs into its working directory: keep each run separate,
    # so parameters from an earlier run never leak into this one.
    os.chdir(run_dir)

    pkg_bringup = get_package_share_directory("ardupilot_gz_bringup")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_sitl = get_package_share_directory("ardupilot_sitl")
    pkg_gazebo = get_package_share_directory("uavx_gazebo")

    world_path = _world_file(scenario, run_dir)
    actions = [
        LogInfo(msg=f"UAV-X level {scenario['level']} '{scenario['name']}' -> run dir {run_dir}"),
        SetEnvironmentVariable("UAVX_RUN_DIR", run_dir),
    ]

    gz_args = f"-s -r -v2 {world_path}" + (" --headless-rendering" if headless else "")
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items()))
    if gui:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
            launch_arguments={"gz_args": "-g -v1"}.items()))

    # Sim clock: Gazebo publishes every 1 ms step; 100 Hz is plenty for ROS timers.
    actions.append(Node(package="ros_gz_bridge", executable="parameter_bridge", name="clock_bridge",
                        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
                        remappings=[("/clock", "/gz_clock")], output="log"))
    actions.append(Node(package="topic_tools", executable="throttle", name="clock_throttle",
                        arguments=["messages", "/gz_clock", "100", "/clock"], output="log"))

    defaults = ",".join([
        os.path.join(pkg_sitl, "config", "default_params", "copter.parm"),
        os.path.join(pkg_sitl, "config", "default_params", "dds_udp.parm"),
        os.path.join(pkg_sitl, "config", "default_params", "dds_use_ns.parm"),
        os.path.join(pkg_gazebo, "config", "uavx_quad.parm"),
    ])
    bridge_cfg = os.path.join(pkg_gazebo, "config", "uavx_quad_bridge.yaml")
    for uav in scenario["uavs"]:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_bringup, "launch", "robots", "robot.launch.py")),
            launch_arguments={
                "namespace": uav["id"],
                "robot_name": uav["id"],
                "world_name": scenario["world"],
                "sdf_file": _vehicle_sdf(uav, run_dir),
                "bridge_config_file": bridge_cfg,
                "defaults": defaults,
                "instance": str(uav["instance"]),
                "sysid": str(uav["sysid"]),
                "use_instance_dir": "True",
                "use_dds_agent": "True" if uav["instance"] == 0 else "False",
                "use_gz_tf": "false",
                "x": str(uav["pad_m"][0]),
                "y": str(uav["pad_m"][1]),
                "z": "0.195",
                "Y": "0.0",
            }.items()))

    common = {"use_sim_time": True, "scenario": scenario_path, "run_dir": run_dir,
              "seed": int(LaunchConfiguration("seed").perform(context))}
    components = scenario["components"]
    enders = []
    for comp in components:
        if comp in PER_UAV:
            for uav in scenario["uavs"]:
                node = Node(package="uavx_swarm", executable=comp, name=comp, namespace=uav["id"],
                            parameters=[dict(common, uav_id=uav["id"])], output="screen",
                            emulate_tty=True)
                actions.append(node)
                if comp in ENDS_RUN:
                    enders.append(node)
    for comp in SINGLETONS:
        if comp in components:
            node = Node(package="uavx_swarm", executable=comp, name=comp,
                        parameters=[common], output="screen", emulate_tty=True)
            actions.append(node)
            if comp in ENDS_RUN:
                enders.append(node)
    for node in enders:
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=node,
            on_exit=[LogInfo(msg=f"UAV-X run finished - results in {run_dir}"),
                     EmitEvent(event=Shutdown(reason="level finished"))])))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="level0",
                              description="Scenario name in uavx_swarm/config (level0..level4) or a YAML path"),
        DeclareLaunchArgument("gui", default_value="false", description="Open the Gazebo GUI"),
        DeclareLaunchArgument("headless_rendering", default_value="false",
                              description="EGL rendering without a display (Docker / CI)"),
        DeclareLaunchArgument("run_dir", default_value="",
                              description="Where logs go (default ~/uavx_runs/level<N>_<time>)"),
        DeclareLaunchArgument("seed", default_value="-1",
                              description="Seed for hidden disturbances / radio fading (-1 = scenario's seed, "
                                          "or a fresh random one if it says 'random')"),
        OpaqueFunction(function=_launch),
    ])
