#!/usr/bin/python
# coding=UTF-8

import roslib
import rospy
import rosbag
import os
import sys
import argparse

def remove_tf(inbag, outbag, source_frame_ids, target_frame_ids):
    print '   Processing input bagfile: %s' % (inbag)
    print '  Writing to output bagfile: %s' % (outbag)
    print '  Removing source_frame_ids: %s' % (' '.join(source_frame_ids))
    print '  Removing target_frame_ids: %s' % (' '.join(target_frame_ids))
    
    outbag = rosbag.Bag(outbag, 'w', rosbag.bag.Compression.BZ2)
    for topic, msg, t in rosbag.Bag(inbag, 'r').read_messages():
        if topic == "/tf" or topic == '/tf_static':
            new_transforms = []
            for transform in msg.transforms:
                if not (transform.header.frame_id in source_frame_ids and transform.child_frame_id in target_frame_ids):
                    new_transforms.append(transform)
            msg.transforms = new_transforms
        outbag.write(topic, msg, t)
    rospy.loginfo('Closing output bagfile and exit...')
    outbag.close();



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='removes all transforms from the /tf topic that contain one of the given frame_ids in the header as parent or child.')
    parser.add_argument('-i', metavar='INPUT_BAGFILE', required=True, help='input bagfile')
    parser.add_argument('-o', metavar='OUTPUT_BAGFILE', required=True, help='output bagfile')
    parser.add_argument('-s', metavar='SOURCE_FRAME_IDS', required=True, help='source frame_id(s) of the transforms to remove from the /tf topic', nargs='+')
    parser.add_argument('-t', metavar='TARGET_FRAME_IDS', required=True, help='target frame_id(s) of the transforms to remove from the /tf topic', nargs='+')
    args = parser.parse_args()
    
    try:
      remove_tf(args.i, args.o, args.s, args.t)
    except Exception, e:
      import traceback
      traceback.print_exc()
