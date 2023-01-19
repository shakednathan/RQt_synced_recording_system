#!/usr/bin/python3

from re import I, T
import rospy
import sys
import message_filters
from rospy.topics import Topic
from sensor_msgs.msg import CompressedImage, Image, CameraInfo, PointCloud2
from inertiallabs_msgs.msg import ins_data, sensor_data, gps_data



arg2topics_dic = {
  "lidar": ["/invz_reflection_0"],
  #"mux_radar": ["/as_tx/radar_markers"], # the marker msg type is not good for this kind of syncing
  "flir": ["/flir_boson/image_raw"],
  "vel_lidar": ["/velodyne_points"],
  "camera": ["/image_raw"],
  "camera_comp": ["/image_raw/compressed"],
  "ins": ["/Inertial_Labs/gps_data","/Inertial_Labs/ins_data","/Inertial_Labs/sensor_data"],
}

MAX_ARGS = len(arg2topics_dic) + 1
sync_topics_list = []

args = rospy.myargv(argv = sys.argv)
if (len(args) > MAX_ARGS):
    print("ERROR: too many arguments for " + args[0])
    sys.exit(1)

print(args)
print(args[1:])

if len(args) != 1:
    topics_list = args[1:]
    rospy.init_node("sensors_synchronizer")
else:
    print("No sensors in arguments")
    sys.exit(0)

i = 0
msgs2indx = {}

if "vel_lidar" in topics_list:
    vel_lidar_pub = rospy.Publisher('sensor_sync/velodyne_points', PointCloud2, queue_size=5)
    vel_lidar_sub =  message_filters.Subscriber('velodyne_points', PointCloud2)

    msgs2indx["vel_lidar"] = i
    i += 1
    sync_topics_list.append(vel_lidar_sub)
else:
    vel_lidar_pub = None

if "lidar" in topics_list:
    lidar_pub = rospy.Publisher('sensor_sync/invz_reflection_0', PointCloud2, queue_size=5)
    lidar_sub =  message_filters.Subscriber('invz_reflection_0', PointCloud2)

    msgs2indx["lidar"] = i
    i += 1
    sync_topics_list.append(lidar_sub)
else:
    lidar_pub = None

if "camera" in topics_list:
    image_pub = rospy.Publisher('sensor_sync/image_raw', Image, queue_size=5)
    image_sub = message_filters.Subscriber('image_raw', Image)
    
    msgs2indx["camera"] = i
    i += 1
    sync_topics_list.append(image_sub)
else:
    image_pub = None

if "flir" in topics_list:
    flir_pub = rospy.Publisher('sensor_sync/flir_boson/image_raw', Image, queue_size=5)
    flir_sub = message_filters.Subscriber('flir_boson/image_raw', Image)
    
    msgs2indx["flir"] = i
    i += 1
    sync_topics_list.append(flir_sub)
else:
    flir_pub = None

if "camera_comp" in topics_list:
    image_comp_pub = rospy.Publisher('sensor_sync/image_raw/compressed', CompressedImage, queue_size=5)
    image_comp_sub = message_filters.Subscriber('image_raw/compressed', CompressedImage)
    
    msgs2indx["camera_comp"] = i
    i += 1
    sync_topics_list.append(image_comp_sub)
else:
    image_comp_pub = None

if "ins" in topics_list:
    ins_ins_pub = rospy.Publisher('sensor_sync/Inertial_Labs/ins_data', ins_data, queue_size=5)
    ins_sensor_pub = rospy.Publisher('sensor_sync/Inertial_Labs/sensor_data', sensor_data, queue_size=5)
    ins_gps_pub = rospy.Publisher('sensor_sync/Inertial_Labs/gps_data', gps_data, queue_size=5)

    ins_ins_sub = message_filters.Subscriber('Inertial_Labs/ins_data', ins_data)
    ins_sensor_sub = message_filters.Subscriber('Inertial_Labs/sensor_data', sensor_data)
    ins_gps_sub = message_filters.Subscriber('Inertial_Labs/gps_data', gps_data)
    msgs2indx["ins"] = i
    i += 1

    sync_topics_list.append(ins_ins_sub)
    sync_topics_list.append(ins_sensor_sub)
    sync_topics_list.append(ins_gps_sub)
else:
    ins_ins_pub = None


def callback(*callback_msgs):
    if image_pub != None:
        #print(msgs2indx)
        image_pub.publish(callback_msgs[msgs2indx["camera"]])
    if image_comp_pub != None:
        image_comp_pub.publish(callback_msgs[msgs2indx["camera_comp"]])
    if flir_pub != None:
        flir_pub.publish(callback_msgs[msgs2indx["flir"]])
    if lidar_pub != None:
        lidar_pub.publish(callback_msgs[msgs2indx["lidar"]])
    if vel_lidar_pub != None:
        vel_lidar_pub.publish(callback_msgs[msgs2indx["vel_lidar"]])
    if ins_ins_pub != None:
        ins_indx = msgs2indx["ins"]
        ins_ins_pub.publish(callback_msgs[ins_indx])
        ins_sensor_pub.publish(callback_msgs[ins_indx+1])
        ins_gps_pub.publish(callback_msgs[ins_indx+2])

ts = message_filters.ApproximateTimeSynchronizer(sync_topics_list, 5 ,0.05)
ts.registerCallback(callback)
print("sync node is running")
rospy.spin()
