#!/usr/bin/python
# coding=UTF-8


import rospy
import rosbag
import argparse
import tf2_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
from tf import transformations


def reset_imu_rotation(inbag_filename, outbag_filename, imu_topic, pose_topic):
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
                reset_quaternion_imu_valid = True

            new_quaternion = transformations.quaternion_multiply(reset_quaternion_imu, current_quaternion)
            msg.orientation.x = new_quaternion[0]
            msg.orientation.y = new_quaternion[1]
            msg.orientation.z = new_quaternion[2]
            msg.orientation.w = new_quaternion[3]

        if topic == pose_topic:
            current_quaternion = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
            if not reset_quaternion_pose_valid:
                reset_quaternion_pose = transformations.quaternion_inverse(current_quaternion)
                reset_quaternion_pose_valid = True

            new_quaternion = transformations.quaternion_multiply(reset_quaternion_pose, current_quaternion)
            msg.pose.orientation.x = new_quaternion[0]
            msg.pose.orientation.y = new_quaternion[1]
            msg.pose.orientation.z = new_quaternion[2]
            msg.pose.orientation.w = new_quaternion[3]

        outbag.write(topic, msg, t)

    print 'Closing output bagfile %s and exit...' % (outbag_filename)
    inbag.close()
    outbag.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Changes tf frame_ids with the option to invert the tf matrix')
    parser.add_argument('-i', metavar='INPUT_BAGFILE', required=True, help='Input bagfile')
    parser.add_argument('-o', metavar='OUTPUT_BAGFILE', required=True, help='Output bagfile')
    parser.add_argument('-t', metavar='TOPIC_IMU', required=True, help='Topic to reset IMU rotation')
    parser.add_argument('-p', metavar='TOPIC_POSE', required=True, help='Topic to reset PoseStamped rotation')
    args = parser.parse_args()

    try:
      reset_imu_rotation(args.i, args.o, args.t, args.p)
      exit(0)
    except Exception, e:
      import traceback
      traceback.print_exc()
