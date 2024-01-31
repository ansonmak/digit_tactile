#!/usr/bin/env python
import sys, os
from pathlib import Path
base_path = Path(__file__).parent.parent.resolve()
sys.path.append(os.path.abspath(base_path))

import cv2
import yaml
import rospy
import numpy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32
from digit_tactile.msg import Contact
from digit_tactile.srv import ResetDepth
from cv_bridge import CvBridge
br = CvBridge()

from digit_depth.third_party import geom_utils
from digit_depth.digit import DigitSensor
from digit_depth.train.prepost_mlp import *
from digit_depth.handlers import find_recent_model
from digit_depth.third_party.vis_utils import ContactArea
seed = 42
torch.seed = seed
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

class DigitConfig:
    def __init__(self, file_name):
        with open(f"{base_path}/config/"+file_name, 'r') as stream:
            self.config = yaml.safe_load(stream)
        self.fps = self.config['sensor']['fps']
        self.res = self.config['sensor']['resolution']
        self.ID = self.config['sensor']['serial_num']
        self.gel_width = self.config['sensor']['gel_width']
        self.gel_height = self.config['sensor']['gel_height']
        self.max_depth = self.config['max_depth']
        
class DigitContact:
    def __init__(self, config, publish, model_name, depth_scale=1.0):
        self.config = config
        self._digit = DigitSensor(self.config.fps, self.config.res, self.config.ID)
        self.call = self._digit()
        self.dm_zero = 0
        self.prev_deformation = 0.0
        self.actual_deformation = 0.0
        self.zero_depth_sample = 50
        self.pub = publish
        model_path = find_recent_model(f"{base_path}/"+model_name)
        self.model = torch.load(model_path).to(device)
        self.model.eval()
        self.init_counter = 0
        self.reset_depth = True
        self.depth_scale = depth_scale

    def get_depth_img(self):
        # get camera frame from digit sensor
        frame = self.call.get_frame()
        # get normal mapping img with trained model
        img_np = preproc_mlp(frame)
        img_np = self.model(img_np).detach().cpu().numpy()
        img_np, _ = post_proc_mlp(img_np)
        # get gradx and grady
        gradx_img, grady_img = geom_utils._normal_to_grad_depth(img_normal=img_np, gel_width=self.config.gel_width,
                                                                gel_height=self.config.gel_height,bg_mask=None)
        # reconstruct depth
        self.img_depth = geom_utils._integrate_grad_depth(gradx_img, grady_img, boundary=None, bg_mask=None,max_depth=self.config.max_depth)
        self.img_depth = self.img_depth.detach().cpu().numpy() # final depth image for current image

        max_deformation = np.min(self.img_depth.flatten())
        self.prev_deformation = self.actual_deformation
        self.actual_deformation = float(np.abs((max_deformation - np.min(self.dm_zero))) * 1000) # convert to mm

        # Get the first 50 frames and average them to get the zero depth
        if self.reset_depth:
            if self.init_counter < self.zero_depth_sample:
                self.dm_zero += self.img_depth
                self.init_counter += 1
                return None
            elif self.init_counter == self.zero_depth_sample:
                self.dm_zero = self.dm_zero/self.zero_depth_sample
                self.reset_depth = False
                self.init_counter = 0
                rospy.loginfo("Digit sensor {} initialized.". format(self.config.ID))
        
        # remove the zero depth
        self.diff = self.img_depth - self.dm_zero
        # convert pixels into 0-255 range
        self.diff = self.diff*255
        self.diff = self.diff*-1
        return self.diff

    def get_result_img(self):
        ret, self.result_img = cv2.threshold(self.diff, 0, 255, cv2.THRESH_TOZERO)
        return self.result_img

    def publish_contact(self, start_time):
        pt = ContactArea()
        contact_msg = Contact()
        contact_msg.header.stamp = rospy.Time.now() -start_time
        contact_msg.theta, contact_msg.x, contact_msg.y = pt.__call__(target=self.result_img)
        if abs(self.actual_deformation-self.prev_deformation) > 1.0:
            print("Error reading...Skip publishing.")
            return False
        else: contact_msg.depth = self.actual_deformation * self.depth_scale
        self.pub.publish(contact_msg)
        return True

    def reset(self):
        self.dm_zero = 0
        self.reset_depth = True
        rospy.loginfo("Resetting Digit sensor {}...".format(self.config.ID))

reset_sensor = False
def reset_depth_handler(req):
    global reset_sensor
    reset_sensor = True
    return True

def publish_contacts():
    rospy.init_node('dual_digit', anonymous=True)
    rate = rospy.Rate(50)
    depth_pub = rospy.Publisher("/digit/depth_image/", Image, queue_size=10)
    left_contact_pub = rospy.Publisher("/digit/left/contact/", Contact, queue_size=10)
    right_contact_pub = rospy.Publisher("/digit/right/contact/", Contact, queue_size=10)
    reset_depth_srv = rospy.Service('/digit/reset_depth', ResetDepth, reset_depth_handler)

    left_conf = DigitConfig('digit_left.yaml')
    right_conf = DigitConfig('digit_right.yaml')

    leftDigit = DigitContact(left_conf, left_contact_pub, "models/model_left")
    rightDigit = DigitContact(right_conf, right_contact_pub, "models/model_right")
    
    global reset_sensor
    print_time = rospy.Time(0)
    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        if reset_sensor:
            leftDigit.reset()
            rightDigit.reset()
            reset_sensor = False

        left_depth_img = leftDigit.get_depth_img()
        right_depth_img = rightDigit.get_depth_img()

        if left_depth_img is None or right_depth_img is None:
            continue # skip the loop during sampling intial frames

        left_result_img = leftDigit.get_result_img()
        if not leftDigit.publish_contact(start_time): continue

        right_result_img = rightDigit.get_result_img()
        if not rightDigit.publish_contact(start_time): continue

        # add boarder between two image
        left_bordered_image = cv2.copyMakeBorder(
                 left_result_img, 
                 top=0,
                 bottom=0, 
                 left=0, 
                 right=5, 
                 borderType=cv2.BORDER_CONSTANT, 
                 value=[255,255,255]
              )
        left_right_combine_img = np.concatenate((left_bordered_image, right_result_img), axis=1)
        img_msg = br.cv2_to_imgmsg(left_right_combine_img, encoding="passthrough")
        img_msg.header.stamp = rospy.Time.now()
        depth_pub.publish(img_msg)

        now = rospy.Time.now()
        if (now - print_time >= rospy.Duration(1)):
            rospy.loginfo("Contact msg published at t={}s".format(now.secs-start_time.secs)) # show on rqt_image_view
            print_time = rospy.Time.now()
        rate.sleep()


if __name__ == "__main__":
    rospy.loginfo("starting...")
    publish_contacts()
