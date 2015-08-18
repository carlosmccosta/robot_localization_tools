#!/usr/bin/env python
# coding=UTF-8

import rospy
import rosbag
import argparse
import yaml
import sensor_msgs.msg


def read_camera_info_yaml(camera_info_yaml_filename):
    stream = file(camera_info_yaml_filename, 'r')
    calib_data = yaml.load(stream)
    camera_info = sensor_msgs.msg.CameraInfo()
    camera_info.width = calib_data['image_width']
    camera_info.height = calib_data['image_height']
    camera_info.K = calib_data['camera_matrix']['data']
    camera_info.D = calib_data['distortion_coefficients']['data']
    camera_info.R = calib_data['rectification_matrix']['data']
    camera_info.P = calib_data['projection_matrix']['data']
    camera_info.distortion_model = calib_data['distortion_model']
    return camera_info



def change_camera_info(inbag_filename, outbag_filename, new_camera_info, camera_info_topic):
    print ' Processing input bagfile: %s' % (inbag_filename)
    print 'Writing to output bagfile: %s' % (outbag_filename)

    inbag = rosbag.Bag(inbag_filename, 'r')
    outbag = rosbag.Bag(outbag_filename, 'w', rosbag.bag.Compression.BZ2)

    for topic, msg, t in inbag.read_messages():
        if topic == camera_info_topic:
            new_camera_info.header = msg.header
            outbag.write(topic, new_camera_info, t)
        else:
            outbag.write(topic, msg, t)

    print 'Closing output bagfile %s' % (outbag_filename)
    inbag.close()
    outbag.close()



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Script to change the sensor_msgs/CameraInfo calibration data')
    parser.add_argument('-i', metavar='INPUT_BAGFILE', required=True, help='Input bag filename')
    parser.add_argument('-o', metavar='OUTPUT_BAGFILE', required=True, help='Output bag filename')
    parser.add_argument('-c', metavar='NEW_CAMERA_INFO_YAML', required=True, help='New camera info yaml calibration')
    parser.add_argument('-t', metavar='CAMERA_INFO_TOPIC', required=False, default='/camera/depth/camera_info', help='Camera info topic in bag file')
    args = parser.parse_args()

    try:
        new_camera_info = read_camera_info_yaml(args.c)
        change_camera_info(args.i, args.o, new_camera_info, args.t)
        exit(0)
    except Exception, e:
        import traceback
        traceback.print_exc()
