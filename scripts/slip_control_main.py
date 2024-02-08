#! /usr/bin/env python
import rospy
import yaml
import sys, os
from pathlib import Path
base_path = Path(__file__).parent.resolve()
sys.path.append(os.path.abspath(base_path))

from slip_controller import SlipController

if __name__ == '__main__':
    obj_name = 'coke_bottle_pivot' # coke_bottle / paper_cup / plastic_cup
    with open(f"{base_path}/grasp_config/" + obj_name + ".yaml", 'r') as stream:
        print("Loading config for " + obj_name + "...")
        config = yaml.safe_load(stream)

    controller = SlipController(config)
    if controller.init_grasp():
        controller.detect_slip()


