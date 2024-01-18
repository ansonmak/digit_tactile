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
from digit_tactile.msg import Contact
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

class Config:
    def __init__(self, file_name):
        with open(f"{base_path}/config/"+file_name, 'r') as stream:
            self.config = yaml.safe_load(stream)
        self.fps = self.config['sensor']['fps']
        self.res = self.config['sensor']['resolution']
        self.ID = self.config['sensor']['serial_num']
        self.gel_width = self.config['sensor']['gel_width']
        self.gel_height = self.config['sensor']['gel_height']
        self.max_depth = self.config['max_depth']
        
class Digit:
    def __init__(self, config, publish, model):
        self.config = config
        self._digit = DigitSensor(self.config.fps, self.config.res, self.config.ID)
        self.call = self._digit()
        self.dm_zero = 0
        self.zero_depth_sample = 50
        self.pub = publish
        self.model = model

    def get_depth_img(self, counter):
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
        img_depth = geom_utils._integrate_grad_depth(gradx_img, grady_img, boundary=None, bg_mask=None,max_depth=self.config.max_depth)
        img_depth = img_depth.detach().cpu().numpy() # final depth image for current image

        # Get the first 50 frames and average them to get the zero depth
        if counter < self.zero_depth_sample:
            self.dm_zero += img_depth
            return None
        elif counter == self.zero_depth_sample:
            self.dm_zero = self.dm_zero/self.zero_depth_sample
        
        # remove the zero depth
        self.diff = img_depth - self.dm_zero
        # convert pixels into 0-255 range
        self.diff = self.diff*255
        self.diff = self.diff*-1
        return self.diff

    def get_result_img(self):
        ret, self.result_img = cv2.threshold(self.diff, 0, 255, cv2.THRESH_TOZERO)
        return self.result_img

    def publish_contact(self):
        pt = ContactArea()
        contact_msg = Contact()
        contact_msg.theta, contact_msg.x, contact_msg.y = pt.__call__(target=self.result_img)
        self.pub.publish(contact_msg)


def publish_contacts():
    rospy.init_node('dual_digit', anonymous=True)
    depth_pub = rospy.Publisher("/digit/depth/image_raw/", Image, queue_size=10)
    left_contact_pub = rospy.Publisher("/digit/left/contact/", Contact, queue_size=10)
    right_contact_pub = rospy.Publisher("/digit/right/contact/", Contact, queue_size=10)
    rate = rospy.Rate(50)

    left_conf = Config('digit_left.yaml')
    right_conf = Config('digit_right.yaml')

    model_path = find_recent_model(f"{base_path}/models")
    model = torch.load(model_path).to(device)
    model.eval()

    leftD = Digit(left_conf, left_contact_pub, model)
    rightD = Digit(right_conf, right_contact_pub, model)
    counter = 0
    while not rospy.is_shutdown():
        left_depth_img = leftD.get_depth_img(counter)
        right_depth_img = rightD.get_depth_img(counter)

        counter += 1
        if left_depth_img is None or right_depth_img is None:
            continue # skip the loop during sampling intial frames

        left_result_img = leftD.get_result_img()
        leftD.publish_contact()

        right_result_img = rightD.get_result_img()
        rightD.publish_contact()

        left_right_combine_img = np.concatenate((left_result_img, right_result_img), axis=1)
        img_msg = br.cv2_to_imgmsg(left_right_combine_img, encoding="passthrough")
        img_msg.header.stamp = rospy.Time.now()
        depth_pub.publish(img_msg)

        now = rospy.get_rostime()
        rospy.loginfo("published depth image at {}".format(now)) # show on rqt_image_view
        rate.sleep()


if __name__ == "__main__":
    rospy.loginfo("starting...")
    publish_contacts()
