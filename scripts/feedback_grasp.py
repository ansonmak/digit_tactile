#! /usr/bin/env python
import rospy
import time

# actionlib for Robotiq gripper
import actionlib
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq

# digit contact msg
from digit_tactile.msg import Contact

rospy.init_node('Robotiq_client')

action_name = rospy.get_param('~action_name', 'command_robotiq_action')
robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)
robotiq_client.wait_for_server()


left_depth = 0.0
def leftContact_callback(data):
    global left_depth
    left_depth = data.depth

right_depth = 0.0
def rightContact_callback(data):
    global right_depth
    right_depth = data.depth

def get_avg_depth():
    return (left_depth + right_depth)/2

rospy.Subscriber("/digit/left/contact/", Contact, leftContact_callback)
rospy.Subscriber("/digit/right/contact/", Contact, rightContact_callback)


def grasp():
    grip_depth = 0.2
    open = 0.085
    close = 0.0
    speed = 0.01
    force = 0.1
    Robotiq.goto(robotiq_client, open, speed, force, block=False)

    while not rospy.is_shutdown(): 
        # print(left_depth, right_depth)
        input("Press ENTER to close gripper")
        Robotiq.goto(robotiq_client, close, speed, force, block=False)
        close_time = time.time()
        while True:
            if (get_avg_depth() > grip_depth):
                print("Object detected! Stopped gripper action")
                Robotiq.stop(robotiq_client, block=False)
                print(f"Contact depth: {get_avg_depth() :.2f}mm")
                break
            if (time.time() - close_time) > 3:
                print("Gripper closed")
                break
        

        input("Press ENTER to open gripper")
        Robotiq.goto(robotiq_client, open, speed, force, block=False)

        

if __name__ == '__main__':
    grasp()