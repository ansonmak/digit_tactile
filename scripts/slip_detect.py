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

def detect_slip(width, init_depth, max_depth, slip_threshold):
    if left_pos[0] is None or right_pos[0] is None: 
        rospy.logwarn("Cannot receive digit feedback!")
        return
    elif left_pos[0] == -1 or right_pos[0] == -1:
        rospy.logwarn("Position feedback not detected!")
        return

    left_init_pos = left_pos
    right_init_pos = right_pos
    print("Start slip detection...")
    while True:
        left_pos_error = ((left_pos[0] - left_init_pos[0])**2 + (left_pos[1] - left_init_pos[1])**2)**0.5
        right_pos_error = ((right_pos[0] - right_init_pos[0])**2 + (right_pos[1] - right_init_pos[1])**2)**0.5
        if left_pos_error > slip_threshold or right_pos_error > slip_threshold:
            rospy.logwarn("Slip detected!")
            rospy.logwarn("Increasing gripping force")
            status, width = control_depth(max_depth, width)
            if not status: return
            #TODO: target depth base on distance change
            #larger distance change -> larger target depth
            # new para: max dist change
            input("Press ENTER to relax gripping force")
            status, width = control_depth(init_depth, width)
            if not status: return
            left_init_pos = left_pos
            right_init_pos = right_pos

def release():
    print("Opening gripper")
    Robotiq.open(robotiq_client, block=False)
    print("Resetting Digit...")
    reset_digit()

if __name__ == '__main__':
    obj_name = 'coke_bottle'
    with open(f"{base_path}/grasp_config/" + obj_name + ".yaml", 'r') as stream:
        config = yaml.safe_load(stream)
    obj_size = config['object_size']
    obj_size /= 1000 # convert to meter
    grip_offset = 0 # mm
    grip_offset /= 1000 # convert to meter
    init_grip_width = obj_size + grip_offset
    init_grip_depth = config['init_grip_depth']
    max_grip_depth = config['max_grip_depth']
    slip_detect_threshold = config['slip_detect_threshold']

    grasp_status, width = grasp(init_grip_width, init_grip_depth)
    if grasp_status: 
        detect_slip(width, init_grip_depth, max_grip_depth, slip_detect_threshold)
    # release()

    # while not rospy.is_shutdown(): 
    #     if grasp(grip_width, init_grip_depth): 
    #         detect_slip(init_grip_depth, max_grip_depth)
    #     release()
