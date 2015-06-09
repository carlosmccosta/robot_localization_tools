#!/usr/bin/python
# coding=UTF-8

import roslib
import rospy
import rosbag
import tf
import geometry_msgs.msg
import argparse
import numpy
from numpy import array



def str2bool(v):
    return v.lower() in ("yes", "true", "t", "1")



def change_tf(inbag_filename, outbag_filename, source_frame, target_frame, new_source_frame, new_target_frame, invert_tf_matrix, change_position, change_orientation, new_position, new_rotation):
    print ' Processing input bagfile: %s' % (inbag_filename)
    print 'Writing to output bagfile: %s' % (outbag_filename)
    print '             Changing tfs: [%s -> %s] [%s -> %s]' % (source_frame, target_frame, new_source_frame, new_target_frame)
    print '      Inverting tf matrix: %s' % (invert_tf_matrix)

    if change_position:
        print '             New position: [x: %f, y: %f, z: %f]' % (new_position[0], new_position[1], new_position[2])

    if change_orientation:
        print '             New rotation: [x: %f, y: %f, z: %f, w: %f]' % (new_rotation[0], new_rotation[1], new_rotation[2], new_rotation[3])

    inbag = rosbag.Bag(inbag_filename,'r')
    outbag = rosbag.Bag(outbag_filename, 'w', rosbag.bag.Compression.BZ2)

    for topic, msg, t in inbag.read_messages():
        if topic == '/tf' or topic == '/tf_static':
            for transform_msg in msg.transforms:
                if transform_msg.header.frame_id == source_frame and transform_msg.child_frame_id == target_frame:
                    transform_msg.header.frame_id = new_source_frame
                    transform_msg.child_frame_id = new_target_frame

                    if change_position:
                        transform_msg.transform.translation.x = new_position[0]
                        transform_msg.transform.translation.y = new_position[1]
                        transform_msg.transform.translation.z = new_position[2]
                    
                    if change_orientation:
                        transform_msg.transform.rotation.x = new_rotation[0]
                        transform_msg.transform.rotation.y = new_rotation[1]
                        transform_msg.transform.rotation.z = new_rotation[2]
                        transform_msg.transform.rotation.w = new_rotation[3]

                    if invert_tf_matrix:
                        matrix_4x4 = tf.transformations.quaternion_matrix(numpy.array((transform_msg.transform.rotation.x, transform_msg.transform.rotation.y, transform_msg.transform.rotation.z, transform_msg.transform.rotation.w), dtype=numpy.float64))
                        matrix_4x4[0,3] = transform_msg.transform.translation.x
                        matrix_4x4[1,3] = transform_msg.transform.translation.y
                        matrix_4x4[2,3] = transform_msg.transform.translation.z
                        
                        matrix_4x4_inverse = tf.transformations.inverse_matrix(matrix_4x4)
                        quaternion_inverse = tf.transformations.quaternion_from_matrix(matrix_4x4_inverse)
                        
                        transform_msg.transform.translation.x = matrix_4x4_inverse[0,3]
                        transform_msg.transform.translation.y = matrix_4x4_inverse[1,3]
                        transform_msg.transform.translation.z = matrix_4x4_inverse[2,3]
                        transform_msg.transform.rotation.x = quaternion_inverse[0]
                        transform_msg.transform.rotation.y = quaternion_inverse[1]
                        transform_msg.transform.rotation.z = quaternion_inverse[2]
                        transform_msg.transform.rotation.w = quaternion_inverse[3]

        outbag.write(topic, msg, t)

    print 'Closing output bagfile %s and exit...' % (outbag_filename)
    inbag.close()
    outbag.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Script to change tf frame_ids with the option to invert the tf matrix')
    parser.register('type', 'bool', str2bool)
    parser.add_argument('-i', metavar='INPUT_BAGFILE', required=True, help='Input bagfile')
    parser.add_argument('-o', metavar='OUTPUT_BAGFILE', required=True, help='Output bagfile')
    parser.add_argument('-s', metavar='SOURCE_FRAME_ID', required=True, help='Header frame_id')
    parser.add_argument('-t', metavar='TARGET_FRAME_ID', required=True, help='Child frame_id')
    parser.add_argument('-b', metavar='NEW_SOURCE_FRAME_ID', required=True, help='New header frame_id')
    parser.add_argument('-e', metavar='NEW_TARGET_FRAME_ID', required=True, help='New child frame_id')
    parser.add_argument('-m', metavar='INVERT_MATRIX', type='bool', required=True, help='Invert transformation matrix')
    parser.add_argument('-p', type='bool', required=False, default=False, help='Use position specified in -X -Y -Z')
    parser.add_argument('-r', type='bool', required=False, default=False, help='Use rotation specified in -x -y -z -w')
    parser.add_argument('-X', type=float, required=False, default=0.0, help='New x position')
    parser.add_argument('-Y', type=float, required=False, default=0.0, help='New y position')
    parser.add_argument('-Z', type=float, required=False, default=0.0, help='New z position')
    parser.add_argument('-x', type=float, required=False, default=0.0, help='New x rotation')
    parser.add_argument('-y', type=float, required=False, default=0.0, help='New y rotation')
    parser.add_argument('-z', type=float, required=False, default=0.0, help='New z rotation')
    parser.add_argument('-w', type=float, required=False, default=0.0, help='New w rotation')
    args = parser.parse_args()

    try:
      change_tf(args.i, args.o, args.s, args.t, args.b, args.e, args.m, args.p, args.r, [args.X, args.Y, args.Z], [args.x, args.y, args.z, args.w])
      exit(0)
    except Exception, e:
      import traceback
      traceback.print_exc()
