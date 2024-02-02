#! /usr/bin/env python
import rospy
import time

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

# close gripper in small step, stop closing when detected
def grasp_steps():
    grip_depth = 0.2
    step = float(1.0 / 1000.0)  # meter
    speed = 0.2
    force = 0.0

    start_width = 0.07 #0.085
    # Robotiq.open(robotiq_client, block=False)
    Robotiq.goto(robotiq_client, start_width, block=False)

    while not rospy.is_shutdown(): 
        # print(left_depth, right_depth)
        input("Press ENTER to close gripper")
        width = start_width
        while (get_max_depth() < grip_depth and width > 0.0):
            width -= step
            print(f"Closing at: {width*1000 :.2f}mm")
            Robotiq.goto(robotiq_client, width, speed, force, block=True)
            rospy.sleep(0.1)
        if (width > 0.0001):
            print("Object detected! Stopped gripper action")
            print(f"Left {left_depth :.2f}mm", f"Right {right_depth :.2f}mm")

        rospy.sleep(1)
        print("After stopping")
        print(f"Left {left_depth :.2f}mm", f"Right {right_depth :.2f}mm")
        input("Press ENTER to open gripper")
        Robotiq.open(robotiq_client, speed, force, block=True)
        print("Reset Digit")
        reset_digit()

# close gripper in one command and stop when detect
def grasp_oneshot():
    grip_depth = 0.1
    speed = 0.001
    force = 0.00
    Robotiq.open(robotiq_client, speed, force, block=False)

    while not rospy.is_shutdown(): 
        # print(left_depth, right_depth)
        input("Press ENTER to close gripper")
        Robotiq.close(robotiq_client, speed, force, block=False)
        close_time = time.time()
        while True:
            if (get_max_depth() > grip_depth):
                print("Object detected! Stopped gripper action")
                Robotiq.stop(robotiq_client, block=True)
                print(f"Left {left_depth :.2f}mm", f"Right {right_depth :.2f}mm")
                break
            if (time.time() - close_time) > 3:
                print("Gripper closed")
                break
        
        rospy.sleep(1)
        print("After stopping")
        print(f"Left {left_depth :.2f}mm", f"Right {right_depth :.2f}mm")
        input("Press ENTER to open gripper")
        Robotiq.open(robotiq_client, speed, force, block=True)
        print("Reset Digit")
        reset_digit()

if __name__ == '__main__':
    # grasp_oneshot()
    grasp_steps()