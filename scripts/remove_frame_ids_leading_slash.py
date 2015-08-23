#!/usr/bin/env python
# coding=UTF-8

import roslib
import rospy
import rosbag
import argparse


def frame_id_without_leading_slash(frame_id):
    if frame_id and frame_id[0] == '/':
        if len(frame_id) < 2:
            return ''
        else:
            return frame_id[1:]
    return frame_id


def remove_frame_ids_leading_slash(inbag_filename, outbag_filename, topics):
    print ' Processing input bagfile: %s' % (inbag_filename)
    print 'Writing to output bagfile: %s' % (outbag_filename)
    print '                   Topics: %s' % (topics)


    inbag = rosbag.Bag(inbag_filename,'r')
    outbag = rosbag.Bag(outbag_filename, 'w', rosbag.bag.Compression.BZ2)

    for topic, msg, t in inbag.read_messages():
        if topics == ['all'] or topic in topics:
            if (topic == "/tf" or topic == "/tf_static") and msg.transforms:
                for transform in msg.transforms:
                    transform.header.frame_id = frame_id_without_leading_slash(transform.header.frame_id)
            elif msg._has_header:
                msg.header.frame_id = frame_id_without_leading_slash(msg.header.frame_id)
        outbag.write(topic, msg, t)

    print 'Closing output bagfile %s' % (outbag_filename)
    inbag.close()
    outbag.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Script to add offsets to remove headers frame_id leading slash')
    parser.add_argument('-i', metavar='INPUT_BAGFILE', required=True, help='Input bagfile')
    parser.add_argument('-o', metavar='OUTPUT_BAGFILE', required=True, help='Output bagfile')
    parser.add_argument('-t', metavar='TOPICS', required=True, help='Topics to remove frame_id leading slash', nargs='+')
    args = parser.parse_args()

    try:
      remove_frame_ids_leading_slash(args.i, args.o, args.t)
      exit(0)
    except Exception, e:
      import traceback
      traceback.print_exc()
