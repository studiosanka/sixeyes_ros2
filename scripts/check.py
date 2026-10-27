#!/usr/bin/env python3
"""Static checks that need no ROS: names and files must agree across this repository.

    python scripts/check.py

Catches the mistakes that would otherwise only show up as a launch failure: a joint spelled differently in the
controllers than in the model, a plugin name that does not match the class, a source file the build forgot,
or vendored code that is out of date. It does NOT replace building and launching with ROS 2 (see CI).
Needs PyYAML.
"""
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
DESCRIPTION = ROOT.parent / "sixeyes_description"
errors, notes = [], []


def err(msg):
    errors.append(msg)


def description_joints_and_links():
    """The arm's joints and links, read from the sibling sixeyes_description repository (or None if absent)."""
    urdf = DESCRIPTION / "urdf" / "sixeyes.urdf.xacro"
    if not urdf.exists():
        notes.append("sixeyes_description not found next to this repository: model names were not cross-checked")
        return None
    root = ET.parse(urdf).getroot()
    joints = {j.get("name"): j.get("type") for j in root.findall("joint")}
    links = {l.get("name") for l in root.findall("link")}
    return joints, links


def check_packages():
    for pkg in ("sixeyes_hardware", "sixeyes_bringup", "sixeyes_moveit_config"):
        xml = ROOT / pkg / "package.xml"
        try:
            name = ET.parse(xml).getroot().findtext("name")
        except ET.ParseError as e:
            err(f"{pkg}/package.xml: not valid XML ({e})")
            continue
        if name != pkg:
            err(f"{pkg}/package.xml: name is {name!r}")
        cmake = (ROOT / pkg / "CMakeLists.txt").read_text(encoding="utf-8")
        if f"project({pkg}" not in cmake:
            err(f"{pkg}/CMakeLists.txt: project name does not match")


def check_hardware():
    cmake = (ROOT / "sixeyes_hardware" / "CMakeLists.txt").read_text(encoding="utf-8")
    for src in re.findall(r"^\s+(src/\S+\.cpp)\s*$", cmake, re.M):
        if not (ROOT / "sixeyes_hardware" / src).exists():
            err(f"sixeyes_hardware/CMakeLists.txt lists {src}, which does not exist")
    plugin = ET.parse(ROOT / "sixeyes_hardware" / "sixeyes_hardware.xml").getroot()
    cls = plugin.find("class")
    source = (ROOT / "sixeyes_hardware" / "src" / "sixeyes_system.cpp").read_text(encoding="utf-8")
    if cls is None:
        err("sixeyes_hardware.xml has no <class>")
    else:
        cpp_type = cls.get("type")
        if f"PLUGINLIB_EXPORT_CLASS({cpp_type}," not in source:
            err(f"sixeyes_hardware.xml says the class is {cpp_type}, but sixeyes_system.cpp exports something else")
        if cls.get("name") != "sixeyes_hardware/SixEyesSystem":
            err(f"plugin name {cls.get('name')!r} differs from the one the model uses")
    xacro = (ROOT / "sixeyes_bringup" / "urdf" / "sixeyes_system.urdf.xacro").read_text(encoding="utf-8")
    if "<plugin>sixeyes_hardware/SixEyesSystem</plugin>" not in xacro:
        err("the model does not use the plugin name sixeyes_hardware/SixEyesSystem")
    # The joint names the hardware code looks for must be the ones in the host protocol.
    names = re.findall(r'"(\w+)"', re.search(r"kJointNames\[[^\]]*\]\s*=\s*\{(.*?)\};", source, re.S).group(1))
    host_yaml = ROOT.parent / "sixeyes-host" / "protocol" / "messages.yaml"
    if host_yaml.exists():
        protocol = yaml.safe_load(host_yaml.read_text(encoding="utf-8"))["joints"]
        if names != protocol:
            err(f"sixeyes_system.cpp joint names {names} differ from the host protocol {protocol}")
    return names


def check_names(arm_joints):
    model = description_joints_and_links()
    xacro = ET.parse(ROOT / "sixeyes_bringup" / "urdf" / "sixeyes_system.urdf.xacro").getroot()
    ns = "{http://www.ros.org/wiki/xacro}"
    used = [e.get("name") for e in xacro.iter(f"{ns}position_joint")]
    if used != arm_joints:
        err(f"ros2_control joints in the model {used} differ from the arm's joints {arm_joints}")

    controllers = yaml.safe_load((ROOT / "sixeyes_bringup" / "config" / "controllers.yaml").read_text(encoding="utf-8"))
    arm = controllers["arm_controller"]["ros__parameters"]["joints"]
    grip = controllers["gripper_controller"]["ros__parameters"]["joints"]
    if arm + grip != arm_joints:
        err(f"controllers.yaml joints {arm + grip} differ from the arm's joints {arm_joints}")
    managed = controllers["controller_manager"]["ros__parameters"]
    for name in ("joint_state_broadcaster", "arm_controller", "gripper_controller"):
        if name not in managed:
            err(f"controllers.yaml: controller_manager does not define {name}")

    moveit = yaml.safe_load((ROOT / "sixeyes_moveit_config" / "config" / "moveit_controllers.yaml").read_text(encoding="utf-8"))
    simple = moveit["moveit_simple_controller_manager"]
    for name in simple["controller_names"]:
        if name not in controllers:
            err(f"moveit_controllers.yaml uses controller {name}, which controllers.yaml does not configure")
        elif simple[name]["joints"] != controllers[name]["ros__parameters"]["joints"]:
            err(f"moveit_controllers.yaml joints for {name} differ from controllers.yaml")

    srdf = ET.parse(ROOT / "sixeyes_moveit_config" / "config" / "sixeyes.srdf").getroot()
    kin = yaml.safe_load((ROOT / "sixeyes_moveit_config" / "config" / "kinematics.yaml").read_text(encoding="utf-8"))
    groups = {g.get("name") for g in srdf.findall("group")}
    for group in kin:
        if group not in groups:
            err(f"kinematics.yaml has group {group}, which the SRDF does not define")
    if model:
        joints, links = model
        for g in srdf.findall("group"):
            for j in g.findall("joint"):
                if j.get("name") not in joints:
                    err(f"SRDF group {g.get('name')}: no joint {j.get('name')} in the arm model")
            for c in g.findall("chain"):
                for key in ("base_link", "tip_link"):
                    if c.get(key) not in links:
                        err(f"SRDF chain {key} {c.get(key)} is not a link in the arm model")
        for state in srdf.findall("group_state"):
            for j in state.findall("joint"):
                if j.get("name") not in joints:
                    err(f"SRDF state {state.get('name')}: no joint {j.get('name')} in the arm model")
        for d in srdf.findall("disable_collisions"):
            for key in ("link1", "link2"):
                if d.get(key) not in links:
                    err(f"SRDF disable_collisions: no link {d.get(key)} in the arm model")
        for j in arm_joints:
            if j not in joints:
                err(f"the arm model has no joint {j}")


def check_launch_files():
    for launch in ROOT.rglob("launch/*.py"):
        try:
            compile(launch.read_text(encoding="utf-8"), str(launch), "exec")  # syntax only; nothing is written
        except SyntaxError as e:
            err(f"{launch.relative_to(ROOT)}: does not compile ({e})")


def check_yaml_and_xml():
    for path in list(ROOT.rglob("*.yaml")) + list(ROOT.rglob("*.yml")):
        if ".git" in path.parts:
            continue
        try:
            yaml.safe_load(path.read_text(encoding="utf-8"))
        except yaml.YAMLError as e:
            err(f"{path.relative_to(ROOT)}: invalid YAML ({e})")
    for path in list(ROOT.rglob("*.xacro")) + list(ROOT.rglob("*.srdf")) + list(ROOT.rglob("*.xml")):
        try:
            ET.parse(path)
        except ET.ParseError as e:
            err(f"{path.relative_to(ROOT)}: invalid XML ({e})")


def check_vendored():
    r = subprocess.run([sys.executable, str(ROOT / "scripts" / "sync-host-core.py"), "--check"], capture_output=True, text=True)
    if r.returncode == 2:
        notes.append("sixeyes-host not found next to this repository: vendored C++ was not checked")
    elif r.returncode != 0:
        err("vendored sixeyes-host code is out of date: run python scripts/sync-host-core.py\n" + r.stdout.strip())


def main():
    check_packages()
    names = check_hardware()
    check_names(names)
    check_launch_files()
    check_yaml_and_xml()
    check_vendored()
    for n in notes:
        print("NOTE:", n)
    for e in errors:
        print("ERROR:", e)
    print("OK" if not errors else f"{len(errors)} error(s)")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
