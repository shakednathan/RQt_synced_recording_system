#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology (base, in github)
# Modified by Avi Tzahi and Shaked Nathan, 2023:
## Move to python3
## Create target directory if does not exist

"""Extract images from a rosbag.
"""

import os
import argparse

import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    """Extract a folder of images from a rosbag.
    """
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("output_dir", help="Output directory.")
    parser.add_argument("image_topic", help="Image topic.")

    args = parser.parse_args()

    print("Extract images from %s on topic %s into %s" % (args.bag_file, args.image_topic, args.output_dir))

    bag = rosbag.Bag(args.bag_file, "r")
    bridge = CvBridge()
    count = 0
    if not os.path.exists(args.output_dir):
        os.mkdir(args.output_dir)
    print("Converting...")
    for topic, msg, t in bag.read_messages(topics=[args.image_topic]):
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        cv2.imwrite(os.path.join(args.output_dir, "frame%06i.jpg" % count), cv_img)
        # print("Wrote image %i" % count)

        count += 1

    bag.close()
    print(f"Finished conversion. Your output is in ./{args.output_dir}")

    return

if __name__ == '__main__':
    main()
    
# ffmpeg -framerate 30 -i frame%06d.jpg -c:v libx264 -profile:v high -crf 20 -pix_fmt yuv420p output.mp4
