#! /usr/bin/env python
import rospy
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

rospy.init_node('slip_controller')

action_name = rospy.get_param('~action_name', 'command_robotiq_action')
robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)
robotiq_client.wait_for_server()

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

class SlipController:
    def __init__(self, config):
        # gripper const variables
        self.MAX_WIDTH = 0.085 # 85mm
        self.MIN_WIDTH = 0.001 # 1mm
        self.SPEED = 0.01 # m/s
        self.FORCE  = 0 # percentage

        # controller setting
        self.obj_size = config['object_size'] / 1000 # convert mm to meter
        self.grip_offset = config['grip_offset'] / 1000 # convert mm to meter
        self.MIN_WIDTH = config['min_width'] / 1000 # convert mm to meter
        self.init_width = self.obj_size + self.grip_offset
        self.init_depth = config['init_grip_depth']
        self.max_depth = config['max_grip_depth']
        self.slip_threshold = config['slip_detect_threshold']
        self.max_slip = config['max_slip_dist']
        self.control_time = 0.5 # time to sleep at each control loop
        self.Kp = 4
        self.Kd = 0.1

        self.width = self.init_width
        self.target_depth = self.init_depth

    def control_depth(self):
        if left_depth is None or right_depth is None: # return false if sensors fail
            rospy.logwarn("Cannot receive digit feedback!")
            return False

        prev_error = 0
        depth_setpoint_range = 0.05 #TODO: Maybe add this to obj config
        while True: # PD controller
            current_depth = get_max_depth() # use max depth of two sensors
            error = self.target_depth - current_depth
            if abs(error) < depth_setpoint_range: # return success if depth within setpoint range
                print(f"Depth={self.target_depth}mm achieved!")
                return True
            derivative = error - prev_error
            width_control = self.Kp*error + self.Kd*derivative # width change in mm
            previous_error = error
            self.width -= width_control/1000
            if self.width > self.MAX_WIDTH or self.width < self.MIN_WIDTH: # return false if width out of gripper's range
                rospy.logwarn("No object detected!")
                return False
            
            print("--------Controlling Depth--------")
            print(f"Target depth={self.target_depth}mm")
            print(f"Left={left_depth:.2f}mm, Right={right_depth:.2f}mm")
            print(f"Width change={width_control:.2f}mm, Gripper go to width={self.width*1000:.2f}mm")

            # For testing
            # ans = input("Enter y to execute, any key to abort: ")
            # if ans != 'y': return False

            Robotiq.goto(robotiq_client, self.width, self.SPEED, self.FORCE, block=True)
            rospy.sleep(0.5)


    def init_grasp(self):
        print("Resetting Digit...")
        reset_digit()
        rospy.sleep(3)

        print(f"Gripping width: {self.init_width*1000:.2f} mm")
        Robotiq.goto(robotiq_client, self.init_width, block=False)
        input("Press ENTER to start gripping")
        rospy.sleep(1)

        self.width = self.init_width
        self.target_depth = self.init_depth
        return self.control_depth() # return if grasp success of not

    def detect_slip(self):
        if left_pos[0] is None or right_pos[0] is None: 
            rospy.logwarn("Cannot receive digit feedback!")
            return

        global disable_slip_control
        left_init_pos = left_pos
        right_init_pos = right_pos
        print("Start slip detection...")
        slip_control = False
        prev_depth = 0.0
        while True:
            if left_pos[0] == -1 or right_pos[0] == -1:
                rospy.logwarn("Contacts not detected!")
                return
            
            left_pos_error = ((left_pos[0] - left_init_pos[0])**2 + (left_pos[1] - left_init_pos[1])**2)**0.5
            right_pos_error = ((right_pos[0] - right_init_pos[0])**2 + (right_pos[1] - right_init_pos[1])**2)**0.5

            if slip_control:
                error = max(left_pos_error, right_pos_error)
                prev_depth = self.target_depth
                self.target_depth = error/self.max_slip * self.max_depth # target_depth proportional to slip dist
                self.target_depth = min(max(self.target_depth, self.init_depth), self.max_depth) # bound grip depth within init and max depth
                rospy.logwarn(f"Controlling gripping depth at {self.target_depth:.2f}mm")
                if not self.control_depth(): return
                
                # disable slip control if keyboard hit
                if disable_slip_control:
                    rospy.logwarn("Disabling slip controling")
                    self.width = self.init_width
                    self.target_depth = self.init_depth
                    if not self.control_depth(): return 
                    left_init_pos = left_pos
                    right_init_pos = right_pos
                    prev_depth = 0.0
                    slip_control = False
                    disable_slip_control = False

                rospy.sleep(self.control_time)
            else:
                if left_pos_error > self.slip_threshold or right_pos_error > self.slip_threshold:
                    rospy.logwarn("Slip detected!")
                    slip_control = True


    def release(self):
        print("Opening gripper")
        Robotiq.open(robotiq_client, block=False)
        print("Resetting Digit...")
        reset_digit()
