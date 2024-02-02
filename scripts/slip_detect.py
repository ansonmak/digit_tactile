#! /usr/bin/env python
import rospy
import time
import yaml
import sys, os
from pathlib import Path
base_path = Path(__file__).parent.resolve()
sys.path.append(os.path.abspath(base_path))

# actionlib for Robotiq gripper
import actionlib
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq

# digit contact msg
from digit_tactile.msg import Contact
from digit_tactile.srv import ResetDepth

rospy.init_node('Robotiq_client')

action_name = rospy.get_param('~action_name', 'command_robotiq_action')
robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)
robotiq_client.wait_for_server()
MAX_WIDTH = 0.085 # 85mm
MIN_WIDTH = 0.001 # 1mm
SPEED = 0.01 #m/s
FORCE  = 0 # percentage

left_depth = None
def leftContact_callback(data):
    global left_depth
    left_depth = data.depth

right_depth = None
def rightContact_callback(data):
    global right_depth
    right_depth = data.depth

def get_avg_depth():
    return (left_depth + right_depth)/2

def get_max_depth():
    return max(left_depth, right_depth)

rospy.Subscriber("/digit/left/contact/", Contact, leftContact_callback)
rospy.Subscriber("/digit/right/contact/", Contact, rightContact_callback)

def reset_digit():
    rospy.wait_for_service('/digit/reset_depth')
    try:
        reset = rospy.ServiceProxy('/digit/reset_depth', ResetDepth)
        return reset()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def control_depth(target_depth, current_width):
    if left_depth is None or right_depth is None: # return false if sensors fail
        rospy.logwarn("Cannot receive digit feedback")
        return False 

    Kp = 8.0
    Kd = 1.0
    prev_error = 0
    depth_setpoint_range = 0.05
    width = current_width
    while True: # PD controller
        current_depth = get_max_depth() # use max depth of two sensors
        error = target_depth - current_depth
        if abs(error) < depth_setpoint_range: # return success if depth within setpoint range
            print(f"Depth={target_depth}mm achieved!")
            return True 
        derivative = error - prev_error
        width_control = Kp*error + Kd*derivative # width change in mm
        previous_error = error
        width -= width_control/1000
        if width > MAX_WIDTH or width < MIN_WIDTH: # return false if width out of gripper's range
            rospy.logwarn("No object detected!")
            return False 
        
        print("-------")
        print(f"Target depth={target_depth}mm")
        print(f"Left={left_depth:.2f}mm, Right={right_depth:.2f}mm")
        print(f"Width change={width_control:.2f}mm, Gripper go to width={width*1000:.2f}mm")

        # For testing
        # ans = input("Enter y to execute, any key to abort: ")
        # if ans != 'y': return False

        Robotiq.goto(robotiq_client, width, SPEED, FORCE, block=True)
        rospy.sleep(0.5)


def grasp(width, depth):
    print("Resetting Digit...")
    reset_digit()
    rospy.sleep(3)

    print(f"Gripping width: {width*1000:.2f} mm")
    Robotiq.goto(robotiq_client, width, block=False)
    input("Press ENTER to start gripping")
    rospy.sleep(1)

    return control_depth(depth, width) # return if grasp success of not

# def detect_slip(init_depth, max_depth):
#     if get_max_depth(depth, ) < init_depth: break

    # while True:

def release():
    print("Opening gripper")
    Robotiq.open(robotiq_client, block=False)
    print("Resetting Digit...")
    reset_digit()

if __name__ == '__main__':
    obj_name = 'paper_cup'
    with open(f"{base_path}/grasp_config/" + obj_name + ".yaml", 'r') as stream:
        config = yaml.safe_load(stream)
    obj_size = config['object_size']
    obj_size /= 1000 # convert to meter
    grip_offset = 0 # mm
    grip_offset /= 1000 # convert to meter
    grip_width = obj_size + grip_offset
    init_grip_depth = config['init_grip_depth']
    max_grip_depth = config['max_grip_depth']

    print(grasp(grip_width, init_grip_depth))

    # while not rospy.is_shutdown(): 
    #     if grasp(grip_width, init_grip_depth): 
    #         detect_slip(init_grip_depth, max_grip_depth)
    #     release()
