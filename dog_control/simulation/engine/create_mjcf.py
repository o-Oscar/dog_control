import xml.etree.ElementTree as ET
from pathlib import Path
from xml.dom import minidom

import numpy as np


def add_commons(root, config):
    commons = ["skybox", "visual", "materials"]
    for common in commons:
        common_path = "./common/{}.xml".format(common)
        ET.SubElement(root, "include", {"file": common_path})


def add_defaults(mujoco, config):
    root = ET.SubElement(mujoco, "default")

    idefX_default = ET.SubElement(root, "default", {"class": "IdefX"})
    ET.SubElement(
        idefX_default,
        "joint",
        {
            # "limited": "true",
            "damping": ".01",
            "armature": ".1",
            "stiffness": "8",
            "type": "hinge",
            "frictionloss": "0",
        },
    )
    ET.SubElement(
        idefX_default,
        "geom",
        {
            "contype": "1",
            "conaffinity": "1",
            "condim": "3",
            "friction": ".4 .1 .1",
            "material": "self",
        },
    )

    free_default = ET.SubElement(root, "default", {"class": "free"})
    ET.SubElement(
        free_default,
        "joint",
        {
            "limited": "false",
            "damping": "0",
            "armature": "0",
            "stiffness": "0",
            "frictionloss": "0",
        },
    )

    # ET.SubElement(root, "motor", {"ctrllimited": "true", "ctrlrange": "-1 1"})
    # ET.SubElement(root, "motor", {"forcelimited": "true", "forcerange": "-1 1"})


def create_header(mujoco, config):
    add_commons(mujoco, config)
    ET.SubElement(mujoco, "compiler", {"settotalmass": "17"})
    add_defaults(mujoco, config)
    ET.SubElement(mujoco, "statistic", {"center": "0 0 .7", "extent": "2"})
    ET.SubElement(mujoco, "option", {"timestep": config["timestep"]})


def create_ground(worldbody, config):
    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": "ground",
            "type": "plane",
            "conaffinity": "1",
            "pos": "0 0 0",
            "size": "100 100 1",
            "material": "grid",
        },
    )


def create_misc(torso, config):
    ET.SubElement(
        torso,
        "light",
        {
            "name": "light",
            "pos": "0 0 2",
            "mode": "trackcom",
        },
    )
    ET.SubElement(
        torso,
        "camera",
        {
            "name": "side",
            "pos": "0 -3 0",
            "quat": "0.707 0.707 0 0",
            "mode": "trackcom",
        },
    )
    ET.SubElement(
        torso,
        "camera",
        {
            "name": "back",
            "pos": "-1.8 -1.3 0.8",
            "xyaxes": "0.45 -0.9 0 0.3 0.15 0.94",
            "mode": "trackcom",
        },
    )


def create_lower_leg(thigh, leg_data, config):

    leg_pos_x = 0.0 * leg_data["x_invert"]
    leg_pos_y = 0.0 * leg_data["y_invert"]
    leg_pos_str = "{} {} -.2".format(leg_pos_x, leg_pos_y)

    lower_leg = ET.SubElement(
        thigh,
        "body",
        {
            "name": "lower_leg_" + leg_data["abbrev"],
            "pos": leg_pos_str,
        },
    )

    ET.SubElement(
        lower_leg,
        "geom",
        {
            "name": "lower_leg_" + leg_data["abbrev"],
            "type": "capsule",
            "fromto": "0 0 0 0 0 -.2",
            "size": ".02",
        },
    )

    ET.SubElement(
        lower_leg,
        "joint",
        {
            "name": "lower_leg_" + leg_data["abbrev"],
            # "range": "-135 0",
            "stiffness": config["joint_stiffness"],
            "damping": config["joint_damping"],
            "axis": "0 1 0",
            # "springref": "-90",
        },
    )
    return lower_leg


def create_thigh(shoulder, leg_data, config):

    leg_pos_y = config["thigh_dy"] * leg_data["y_invert"]
    leg_pos_str = "0 {} 0".format(leg_pos_y)

    thigh = ET.SubElement(
        shoulder,
        "body",
        {
            "name": "thigh_" + leg_data["abbrev"],
            "pos": leg_pos_str,
        },
    )

    ET.SubElement(
        thigh,
        "geom",
        {
            "name": "thigh_" + leg_data["abbrev"],
            "type": "capsule",
            "fromto": "0 0 0 0 0 -.2",
            "size": ".02",
        },
    )

    motor_shoulder_dy = 0.04 * leg_data["y_invert"]
    motor_pos_str = "0 {} 0".format(motor_shoulder_dy)
    ET.SubElement(
        thigh,
        "geom",
        {
            "name": "shoulder_motor_" + leg_data["abbrev"],
            "type": "cylinder",
            "size": "0.05 0.02",
            "pos": motor_pos_str,
            "quat": ".707 .707 0 0",
        },
    )

    ET.SubElement(
        thigh,
        "joint",
        {
            "name": "thigh_" + leg_data["abbrev"],
            # "range": "0 90",
            "stiffness": config["joint_stiffness"],
            "damping": config["joint_damping"],
            "axis": "0 1 0",
            # "springref": "45",
        },
    )

    return thigh


def create_shoulder(torso, leg_data, config):

    leg_pos_x = config["shoulder_dx"] * leg_data["x_invert"]
    leg_pos_y = config["shoulder_dy"] * leg_data["y_invert"]
    leg_pos_str = "{} {} 0".format(leg_pos_x, leg_pos_y)

    shoulder = ET.SubElement(
        torso,
        "body",
        {
            "name": "shoulder_" + leg_data["abbrev"],
            "pos": leg_pos_str,
        },
    )

    motor_pos_x = (config["shoulder_dx"] - 0.08) * leg_data["x_invert"]
    motor_pos_str = "{} {} 0".format(motor_pos_x, leg_pos_y)
    ET.SubElement(
        torso,
        "geom",
        {
            "name": "shoulder_motor_1_" + leg_data["abbrev"],
            "type": "cylinder",
            "size": "0.05 0.02",
            "pos": motor_pos_str,
            "quat": ".707 0 .707 0",
        },
    )

    motor_shoulder_dy = (config["thigh_dy"] - 0.04) * leg_data["y_invert"]
    motor_pos_str = "0 {} 0".format(motor_shoulder_dy)
    ET.SubElement(
        shoulder,
        "geom",
        {
            "name": "shoulder_motor_2_" + leg_data["abbrev"],
            "type": "cylinder",
            "size": "0.05 0.02",
            "pos": motor_pos_str,
            "quat": ".707 .707 0 0",
        },
    )

    ET.SubElement(
        shoulder,
        "joint",
        {
            "name": "shoulder_" + leg_data["abbrev"],
            # "range": "-20 20",
            "stiffness": config["joint_stiffness"],
            "damping": config["joint_damping"],
            "axis": "1 0 0",
        },
    )

    return shoulder


def create_robot(worldbody, config):
    torso = ET.SubElement(
        worldbody, "body", {"name": "torso", "pos": "0 0 .7", "childclass": "IdefX"}
    )
    create_misc(torso, config)

    ET.SubElement(
        torso,
        "geom",
        {
            "name": "torso",
            "type": "box",
            "size": config["torso_size"],
        },
    )

    # ET.SubElement(
    #     torso,
    #     "geom",
    #     {
    #         "name": "batteries",
    #         "type": "capsule",
    #         "fromto": "-.1 0 .04 .1 0 .04",
    #         "size": "0.08",
    #     },
    # )

    ET.SubElement(
        torso,
        "joint",
        {
            "name": "torso",
            "type": "free",
            "limited": "false",
            "stiffness": config["root_stiffness"],
            "damping": config["root_damping"],
        },
    )

    leg_datas = [
        {
            "abbrev": "fl",
            "x_invert": 1,
            "y_invert": 1,
        },
        {
            "abbrev": "fr",
            "x_invert": 1,
            "y_invert": -1,
        },
        {
            "abbrev": "bl",
            "x_invert": -1,
            "y_invert": 1,
        },
        {
            "abbrev": "br",
            "x_invert": -1,
            "y_invert": -1,
        },
    ]

    for leg_data in leg_datas:
        shoulder = create_shoulder(torso, leg_data, config)
        thigh = create_thigh(shoulder, leg_data, config)
        lower_leg = create_lower_leg(thigh, leg_data, config)


def create_actuators(mujoco, config):
    actuators = ET.SubElement(mujoco, "actuator")

    abbrevs = ["fl", "fr", "bl", "br"]

    for abbrev in abbrevs:
        name = "shoulder_" + abbrev
        ET.SubElement(
            actuators,
            "motor",
            {"name": name, "joint": name, "gear": config["joint_stiffness"]},
        )

        name = "thigh_" + abbrev
        ET.SubElement(
            actuators,
            "motor",
            {"name": name, "joint": name, "gear": config["joint_stiffness"]},
        )

        name = "lower_leg_" + abbrev
        ET.SubElement(
            actuators,
            "motor",
            {"name": name, "joint": name, "gear": config["joint_stiffness"]},
        )


def create_file_tree(config):
    mujoco = ET.Element("mujoco", {"model": "IdefX"})
    create_header(mujoco, config)

    worldbody = ET.SubElement(mujoco, "worldbody")
    create_ground(worldbody, config)
    create_robot(worldbody, config)

    create_actuators(mujoco, config)

    # worldbody = ET.SubElement(mujoco, 'worldbody')

    return mujoco


DEFAULT_SRC_PATH = Path(__file__).parent / "src" / "IdefX.xml"


def write_robot_to_file(fix_root, substeps, base_motor_kp, base_motor_kd):
    config = {}

    step_dt = 1/30
    substep_dt = step_dt / substeps
    config["timestep"] = str(substep_dt)

    torso_size = [str(x / 2) for x in [0.578, 0.135, 0.100]]
    # torso_size = [str(x / 8) for x in [0.578, 0.135, 0.100]]
    config["torso_size"] = " ".join(torso_size)

    motor_backplate_size = 0.055
    shoulder_backplate_to_backplate = 0.505
    shoulder_y_hole_to_hole = 0.105

    config["shoulder_dx"] = (shoulder_backplate_to_backplate - motor_backplate_size) / 2
    config["shoulder_dy"] = shoulder_y_hole_to_hole / 2

    leg_to_leg_outer = 0.285
    leg_to_leg_inner = 0.200
    thigh_y = (leg_to_leg_outer + leg_to_leg_inner) / 4
    config["thigh_dy"] = thigh_y - config["shoulder_dy"]

    motor_kp = base_motor_kp * 1
    motor_kd = base_motor_kd * 1
    stiffness = motor_kp * motor_kd * 1
    joint_damping = motor_kd * 0.75
    config["joint_stiffness"] = str(stiffness)
    config["joint_damping"] = str(joint_damping)

    if fix_root:
        config["root_stiffness"] = "10000"
        config["root_damping"] = "1000"
    else:
        config["root_stiffness"] = "0"
        config["root_damping"] = "0"

    robot = create_file_tree(config)

    with open(DEFAULT_SRC_PATH, "w") as f:
        xmlstr = minidom.parseString(ET.tostring(robot)).toprettyxml(indent="\t")
        f.write(xmlstr)


if __name__ == "__main__":
    write_robot_to_file(False)
