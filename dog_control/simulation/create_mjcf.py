import xml.etree.ElementTree as ET
from xml.dom import minidom
from pathlib import Path
import numpy as np

def add_commons(root):
    commons = ["skybox", "visual", "materials"]
    for common in commons:
        common_path = "./common/{}.mjcf".format(common)
        ET.SubElement(root, "include", {"file": common_path})


def add_defaults(mujoco):
    root = ET.SubElement(mujoco, "default")

    idefX_default = ET.SubElement(root, "default", {"class": "IdefX"})
    ET.SubElement(
        idefX_default,
        "joint",
        {
            "limited": "true",
            "damping": ".01",
            "armature": ".1",
            "stiffness": "8",
            "type": "hinge",
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
        {"limited": "false", "damping": "0", "armature": "0", "stiffness": "0"},
    )

    ET.SubElement(root, "motor", {"ctrllimited": "true", "ctrlrange": "-1 1"})


def create_header(mujoco):
    add_commons(mujoco)
    ET.SubElement(mujoco, "compiler", {"settotalmass": "17"})
    add_defaults(mujoco)
    ET.SubElement(mujoco, "statistic", {"center": "0 0 .7", "extent": "2"})
    ET.SubElement(mujoco, "option", {"timestep": "0.01"})


def create_ground(worldbody):
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


def create_misc(torso):
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


def create_lower_leg(thigh, leg_data):

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
            "size": ".03",
        },
    )

    ET.SubElement(
        lower_leg,
        "joint",
        {
            "name": "lower_leg_" + leg_data["abbrev"],
            "range": "-135 0",
            "stiffness": "240",
            "damping": "6",
            "axis": "0 1 0",
            "springref": "-90",
        },
    )
    return lower_leg


def create_thigh(shoulder, leg_data):

    leg_pos_x = 0.0 * leg_data["x_invert"]
    leg_pos_y = 0.0 * leg_data["y_invert"]
    leg_pos_str = "{} {} 0".format(leg_pos_x, leg_pos_y)

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
            "size": ".03",
        },
    )

    ET.SubElement(
        shoulder,
        "joint",
        {
            "name": "thigh_" + leg_data["abbrev"],
            "range": "0 90",
            "stiffness": "240",
            "damping": "6",
            "axis": "0 1 0",
            "springref": "45",
        },
    )

    return thigh


def create_shoulder(torso, leg_data):

    leg_pos_x = 0.3 * leg_data["x_invert"]
    leg_pos_y = 0.2 * leg_data["y_invert"]
    leg_pos_str = "{} {} 0".format(leg_pos_x, leg_pos_y)

    shoulder = ET.SubElement(
        torso,
        "body",
        {
            "name": "shoulder_" + leg_data["abbrev"],
            "pos": leg_pos_str,
        },
    )

    ET.SubElement(
        shoulder,
        "geom",
        {
            "name": "shoulder_" + leg_data["abbrev"],
            "type": "cylinder",
            "size": "0.1 0.03",
            "quat": ".707 .707 0 0",
        },
    )

    ET.SubElement(
        shoulder,
        "joint",
        {
            "name": "shoulder_" + leg_data["abbrev"],
            "range": "-20 20",
            "stiffness": "240",
            "damping": "6",
            "axis": "1 0 0",
        },
    )

    return shoulder


def create_robot(worldbody):
    torso = ET.SubElement(
        worldbody, "body", {"name": "torso", "pos": "0 0 .7", "childclass": "IdefX"}
    )
    create_misc(torso)

    ET.SubElement(
        torso,
        "geom",
        {
            "name": "torso",
            "type": "capsule",
            "fromto": "-.3 0 0 .3 0 0",
            "size": "0.1",
        },
    )

    ET.SubElement(
        torso,
        "geom",
        {
            "name": "batteries",
            "type": "capsule",
            "fromto": "-.1 0 .04 .1 0 .04",
            "size": "0.08",
        },
    )
    
    ET.SubElement(
        torso,
        "joint",
        {
            "name": "torso",
            "type": "free",
            "limited": "false",
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
        shoulder = create_shoulder(torso, leg_data)
        thigh = create_thigh(shoulder, leg_data)
        lower_leg = create_lower_leg(thigh, leg_data)


def create_file_tree():
    mujoco = ET.Element("mujoco", {"model": "IdefX"})
    create_header(mujoco)

    worldbody = ET.SubElement(mujoco, "worldbody")
    create_ground(worldbody)
    create_robot(worldbody)

    # worldbody = ET.SubElement(mujoco, 'worldbody')

    return mujoco


robot = create_file_tree()

with open(Path(__file__).parent / "src/test.mjcf", "w") as f:
    xmlstr = minidom.parseString(ET.tostring(robot)).toprettyxml(indent="\t")
    f.write(xmlstr)
