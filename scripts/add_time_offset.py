#!/usr/bin/env python
# coding=UTF-8

import roslib
import rospy
import rosbag
import geometry_msgs.msg
import argparse



def str2bool(v):
    return v.lower() in ("yes", "true", "t", "1")


def add_time_offset(inbag_filename, outbag_filename, time_offset, topics, use_recorded_time, replace_zero_time):
    print ' Processing input bagfile: %s' % (inbag_filename)
    print 'Writing to output bagfile: %s' % (outbag_filename)
    print '       Adding time offset: %f seconds' % (time_offset)
    print '                   Topics: %s' % (topics)
    print ' Using msg recorded stamp: %s' % (use_recorded_time)

    duration_offset = rospy.Duration.from_sec(time_offset)

    inbag = rosbag.Bag(inbag_filename,'r')
    outbag = rosbag.Bag(outbag_filename, 'w', rosbag.bag.Compression.BZ2)
    number_of_replaced_headers = 0

    for topic, msg, t in inbag.read_messages():
        if topics == ['all'] or topic in topics:
            if (topic == "/tf" or topic == "/tf_static") and msg.transforms:
                for transform in msg.transforms:
                    transform.header.stamp += duration_offset
            elif msg._has_header:
                msg.header.stamp += duration_offset

        if use_recorded_time or (msg._has_header and msg.header.stamp.secs == 0 and msg.header.stamp.nsecs == 0):
            if msg._has_header and replace_zero_time:
                msg.header.stamp = t
                number_of_replaced_headers += 1
            outbag.write(topic, msg, t)
        else:
            outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)

    if replace_zero_time:
        print '%i headers with 0 time were replaced with recorded time' % (number_of_replaced_headers)
        
    print 'Closing output bagfile %s' % (outbag_filename)
    inbag.close()
    outbag.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Script to add offsets to messages headers time and replace recorded time with header time')
    parser.register('type', 'bool', str2bool)
    parser.add_argument('-i', metavar='INPUT_BAGFILE', required=True, help='Input bagfile')
    parser.add_argument('-o', metavar='OUTPUT_BAGFILE', required=True, help='Output bagfile')
    parser.add_argument('-s', metavar='TIME_OFFSET', type=float, required=True, help='Time offset in seconds')
    parser.add_argument('-t', metavar='TOPICS', required=True, help='Topics to add time offset', nargs='+')
    parser.add_argument('-r', type='bool', required=False, default=True, help='Use recorded stamp (false uses header stamp)')
    parser.add_argument('-f', type='bool', required=False, default=True, help='Replace header msgs with 0 time with recorded time')
    args = parser.parse_args()

    try:
      add_time_offset(args.i, args.o, args.s, args.t, args.r, args.f)
      exit(0)
    except Exception, e:
      import traceback
      traceback.print_exc()
