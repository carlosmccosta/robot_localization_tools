#!/usr/bin/python
# coding=UTF-8

import roslib
import rospy
import rosbag
import argparse



def remove_tf(inbag_filename, outbag_filename, source_frame_ids, target_frame_ids):
    print '   Processing input bagfile: %s' % (inbag_filename)
    print '  Writing to output bagfile: %s' % (outbag_filename)
    print '  Removing source_frame_ids: %s' % (' '.join(source_frame_ids))
    print '  Removing target_frame_ids: %s' % (' '.join(target_frame_ids))

    outbag = rosbag.Bag(outbag_filename, 'w', rosbag.bag.Compression.BZ2)
    for topic, msg, t in rosbag.Bag(inbag_filename, 'r').read_messages():
        if topic == "/tf" or topic == '/tf_static':
            new_transforms = []
            for transform in msg.transforms:
                if not (transform.header.frame_id in source_frame_ids and transform.child_frame_id in target_frame_ids):
                    new_transforms.append(transform)
            msg.transforms = new_transforms
        outbag.write(topic, msg, t)

    print 'Closing output bagfile %s and exit...' % (outbag_filename)
    outbag.close()



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Script to remove tfs that contain one of the given frame_ids in the header as parent or child')
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
