#!/usr/bin/env python
# coding=UTF-8

import importlib
import os
import string
import sys

import geometry_msgs.msg
import rospy
import sensor_msgs.msg
import std_msgs.msg


last_message_time = 0


def class_for_name(module_name, class_name):
    m = importlib.import_module(module_name)
    c = getattr(m, class_name)
    return c



def msg_callback(msg):
    global last_message_time
    last_message_time = msg.header.stamp.to_sec()



def topic_supervisor():
    ##############################################################################################################
    # node initialization
    ##############################################################################################################
    rospy.init_node('topic_supervisor', anonymous=True)


    ##############################################################################################################
    # parameters
    ##############################################################################################################
    topic_name = rospy.get_param('~topic_name', '')
    topic_type = rospy.get_param('~topic_type', '')
    topic_type_module = rospy.get_param('~topic_type_module', '')
    recovery_command = rospy.get_param('~recovery_command', '')
    recovery_command = string.replace(recovery_command, "AND", "&&")
    polling_rate = rospy.get_param('~polling_rate', 1.0)
    max_seconds_of_msgs_absence = rospy.get_param('~max_seconds_of_msgs_absence', 10.0)
    minimum_seconds_between_recovery_commands = rospy.get_param('~minimum_seconds_between_recovery_commands', 20.0)


    if not topic_name or not topic_type or not topic_type_module or not recovery_command:
        rospy.logfatal("Must specify the topic name, type, module and recovery command")
        sys.exit(1)


    ##############################################################################################################
    # communication setup
    ##############################################################################################################
    topic_subscriber = rospy.Subscriber(topic_name, class_for_name(topic_type_module, topic_type), msg_callback, queue_size=1)


    ##############################################################################################################
    # topic supervision
    ##############################################################################################################
    rate_wait_for_valid_time = rospy.Rate(10)
    while (rospy.Time.now().to_sec() == 0.0):
        rate_wait_for_valid_time.sleep()

    rate = rospy.Rate(polling_rate)
    global last_message_time
    last_message_time = rospy.Time.now().to_sec()
    last_system_call_time = last_message_time
    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec()
        if (current_time - last_system_call_time > minimum_seconds_between_recovery_commands):
            msg_absence_time = current_time - last_message_time
            if msg_absence_time > max_seconds_of_msgs_absence:
                rospy.loginfo('Running cmd [%s] after stopped receiving [%s] msgs for [%f] seconds in topic [%s]', recovery_command, topic_type, msg_absence_time, topic_name)
                last_system_call_time = current_time
                os.system(recovery_command + ' &')
        rate.sleep()



if __name__ == "__main__":
    try:
        topic_supervisor()
    except rospy.ROSInterruptException:
        rospy.logfatal("Failed to load topic supervisor")
        sys.exit(2)
