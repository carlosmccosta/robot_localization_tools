#!/usr/bin/env python
# coding=UTF-8


import rospy
import rosbag
import argparse
import tf2_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
from tf import transformations


def str2bool(v):
    return v.lower() in ("yes", "true", "t", "1")


def reset_quaternion(quaternion, reset_config):
    print ' Reset configuration: %s' % (reset_config)
    
    reset_yaw = 0
    reset_pitch = 0
    reset_roll = 0
    yaw, pitch, roll = transformations.euler_from_quaternion(quaternion, axes='szyx')
    
    if 'y' in reset_config:
        print ' Resetting yaw'
        reset_yaw = yaw
    
    if 'p' in reset_config:
        print ' Resetting pitch'
        reset_pitch = pitch
        
    if 'r' in reset_config:
        print ' Resetting roll'
        reset_roll = roll
    
    return transformations.quaternion_from_euler(reset_yaw, reset_pitch, reset_roll, axes='szyx')


def print_reset_quaternion(quaternion, message_header):
    reset_yaw, reset_pitch, reset_roll = transformations.euler_from_quaternion(quaternion, axes='szyx')
    print '%s | quaternion [qx: %f, qy: %f, qz: %f, qw: %f] and fixed euler angles [yaw: %f, pitch: %f, roll: %f]' % (message_header, quaternion[0], quaternion[1], quaternion[2], quaternion[3], reset_yaw, reset_pitch, reset_roll)


def print_reset_coversion(original_quaternion, new_quaternion):
    original_yaw, original_pitch, original_roll = transformations.euler_from_quaternion(original_quaternion, axes='szyx')
    new_yaw, new_pitch, new_roll = transformations.euler_from_quaternion(new_quaternion, axes='szyx')
    print ' Reset orientation [qx: %f, qy: %f, qz: %f, qw: %f] -> [qx: %f, qy: %f, qz: %f, qw: %f] | [y: %f, p: %f, r: %f] -> [y: %f, p: %f, r: %f]' % (original_quaternion[0], original_quaternion[1], original_quaternion[2], original_quaternion[3], new_quaternion[0], new_quaternion[1], new_quaternion[2], new_quaternion[3], original_yaw, original_pitch, original_roll, new_yaw, new_pitch, new_roll)


def reset_imu_rotation(inbag_filename, outbag_filename, imu_topic, pose_topic, reset_config, print_verbose_info):
    print ' Resetting IMU orientation in topic %s within bag %s' % (imu_topic, inbag_filename)
    print ' Resetting PoseStamped orientation in topic %s within bag %s' % (pose_topic, inbag_filename)

    inbag = rosbag.Bag(inbag_filename,'r')
    outbag = rosbag.Bag(outbag_filename, 'w', rosbag.bag.Compression.BZ2)

    reset_quaternion_imu_valid = False
    reset_quaternion_pose_valid = False

    for topic, msg, t in inbag.read_messages():
        if topic == imu_topic:
            current_quaternion = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
            if not reset_quaternion_imu_valid:
                reset_quaternion_imu = transformations.quaternion_inverse(current_quaternion)
                if reset_config:
                    reset_quaternion_imu = reset_quaternion(reset_quaternion_imu, reset_config)
                reset_quaternion_imu_valid = True
                print_reset_quaternion(reset_quaternion_imu, ' Resetting IMU messages\t\t')


            new_quaternion = transformations.quaternion_multiply(reset_quaternion_imu, current_quaternion)
            msg.orientation.x = new_quaternion[0]
            msg.orientation.y = new_quaternion[1]
            msg.orientation.z = new_quaternion[2]
            msg.orientation.w = new_quaternion[3]

            if print_verbose_info:
                print_reset_coversion(current_quaternion, new_quaternion)

        if topic == pose_topic:
            current_quaternion = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
            if not reset_quaternion_pose_valid:
                reset_quaternion_pose = transformations.quaternion_inverse(current_quaternion)
                if reset_config:
                    reset_quaternion_pose = reset_quaternion(reset_quaternion_pose, reset_config)
                reset_quaternion_pose_valid = True
                print_reset_quaternion(reset_quaternion_pose, ' Resetting PoseStamped messages\t')

            new_quaternion = transformations.quaternion_multiply(reset_quaternion_pose, current_quaternion)
            msg.pose.orientation.x = new_quaternion[0]
            msg.pose.orientation.y = new_quaternion[1]
            msg.pose.orientation.z = new_quaternion[2]
            msg.pose.orientation.w = new_quaternion[3]

            if print_verbose_info:
                print_reset_coversion(current_quaternion, new_quaternion)

        outbag.write(topic, msg, t)

    print 'Closing output bagfile %s and exit...' % (outbag_filename)
    inbag.close()
    outbag.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Script to reset IMU and PoseStamped orientation')
    parser.register('type', 'bool', str2bool)
    parser.add_argument('-i', metavar='INPUT_BAGFILE', required=True, help='Input bagfile')
    parser.add_argument('-o', metavar='OUTPUT_BAGFILE', required=True, help='Output bagfile')
    parser.add_argument('-t', metavar='TOPIC_IMU', required=True, help='Topic to reset IMU rotation')
    parser.add_argument('-p', metavar='TOPIC_POSE', required=True, help='Topic to reset PoseStamped rotation')
    parser.add_argument('-r', metavar='RESET_CONFIG', required=False, default='', help='Reset config (string with rpy to reset each / combined fixed euler angles)')
    parser.add_argument('-v', metavar='PRINT_VERBOSE_INFO', type='bool', required=False, default=False, help='Print verbose convertion info')
    args = parser.parse_args()

    try:
      reset_imu_rotation(args.i, args.o, args.t, args.p, args.r, args.v)
      exit(0)
    except Exception, e:
      import traceback
      traceback.print_exc()
