#! /usr/bin/env python
import rospy
import time
import yaml
from pynput import keyboard
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
left_pos = (None, None)
def leftContact_callback(data):
    global left_depth
    global left_pos
    left_depth = data.depth
    left_pos = (data.x, data.y)

right_depth = None
right_pos = (None, None)
def rightContact_callback(data):
    global right_depth
    global right_pos
    right_depth = data.depth
    right_pos = (data.x, data.y)

def get_avg_depth():
    return (left_depth + right_depth)/2

def get_max_depth():
    return max(left_depth, right_depth)

rospy.Subscriber("/digit/left/contact/", Contact, leftContact_callback)
rospy.Subscriber("/digit/right/contact/", Contact, rightContact_callback)

disable_slip_control = False
def on_press(key):
    try:
        global disable_slip_control
        disable_slip_control = True
    except AttributeError: pass

def on_release(key):
    global disable_slip_control
    disable_slip_control = False

listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()

def reset_digit():
    rospy.wait_for_service('/digit/reset_depth')
    try:
        reset = rospy.ServiceProxy('/digit/reset_depth', ResetDepth)
        return reset()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def control_depth(target_depth, current_width):
    if left_depth is None or right_depth is None: # return false if sensors fail
        rospy.logwarn("Cannot receive digit feedback!")
        return False, None

    Kp = 5.0
    Kd = 0.5
    prev_error = 0
    depth_setpoint_range = 0.05
    width = current_width
    while True: # PD controller
        current_depth = get_max_depth() # use max depth of two sensors
        error = target_depth - current_depth
        if abs(error) < depth_setpoint_range: # return success if depth within setpoint range
            print(f"Depth={target_depth}mm achieved!")
            return True, width
        derivative = error - prev_error
        width_control = Kp*error + Kd*derivative # width change in mm
        previous_error = error
        width -= width_control/1000
        if width > MAX_WIDTH or width < MIN_WIDTH: # return false if width out of gripper's range
            rospy.logwarn("No object detected!")
            return False, None 
        
        print("--------Controlling Depth--------")
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

def detect_slip(width, config):
    if left_pos[0] is None or right_pos[0] is None: 
        rospy.logwarn("Cannot receive digit feedback!")
        return

    global disable_slip_control
    init_depth = config['init_grip_depth']
    max_depth = config['max_grip_depth']
    slip_threshold = config['slip_detect_threshold']
    max_slip = config['max_slip_dist']

    left_init_pos = left_pos
    right_init_pos = right_pos
    print("Start slip detection...")
    slip_control = False
    prev_grip_depth = 0.0
    grip_depth = 0.0 
    while True:
        if left_pos[0] == -1 or right_pos[0] == -1:
            rospy.logwarn("Contacts not detected!")
            return
        
        left_pos_error = ((left_pos[0] - left_init_pos[0])**2 + (left_pos[1] - left_init_pos[1])**2)**0.5
        right_pos_error = ((right_pos[0] - right_init_pos[0])**2 + (right_pos[1] - right_init_pos[1])**2)**0.5

        if slip_control:
            error = max(left_pos_error, right_pos_error)
            prev_grip_depth = grip_depth
            grip_depth = error/max_slip * max_depth # grip_depth proportional to slip dist
            grip_depth = min(max(grip_depth, init_depth), max_depth) # bound grip depth within init and max depth
            rospy.logwarn(f"Controlling gripping depth at {grip_depth:.2f}mm")
            status, width = control_depth(grip_depth, width)
            if not status: return
            
            # disable slip control if keyboard hit
            if disable_slip_control:
                rospy.logwarn("Disabling slip controling")
                status, width = control_depth(init_depth, width)
                if not status: return
                left_init_pos = left_pos
                right_init_pos = right_pos
                prev_grip_depth = 0.0
                slip_control = False
                disable_slip_control = False
            rospy.sleep(1)
        else:
            if left_pos_error > slip_threshold or right_pos_error > slip_threshold:
                rospy.logwarn("Slip detected!")
                slip_control = True

def release():
    print("Opening gripper")
    Robotiq.open(robotiq_client, block=False)
    print("Resetting Digit...")
    reset_digit()

if __name__ == '__main__':
    obj_name = 'plastic_cup' # coke_bottle / paper_cup / plastic_cup
    with open(f"{base_path}/grasp_config/" + obj_name + ".yaml", 'r') as stream:
        print("Loading config for" + obj_name + "...")
        config = yaml.safe_load(stream)
    
    obj_size = config['object_size'] / 1000 # convert mm to meter
    grip_offset = config['grip_offset'] / 1000 # convert mm to meter
    MIN_WIDTH = config['min_width'] / 1000 # convert mm to meter
    init_grip_width = obj_size + grip_offset
    init_grip_depth = config['init_grip_depth']

    grasp_status, width = grasp(init_grip_width, init_grip_depth)
    if grasp_status: 
        detect_slip(width, config)
    # release()
    
    # while not rospy.is_shutdown(): 
    #     if grasp(grip_width, init_grip_depth): 
    #         detect_slip(init_grip_depth, max_grip_depth)
    #     release()
