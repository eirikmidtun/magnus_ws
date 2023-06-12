#!/usr/bin/env python3

import os
import sys
import rosbag
import rospy
import subprocess
from optparse import OptionParser
from argparse import ArgumentParser
from datetime import datetime


def message_to_csv(stream, msg, flatten=False):
    """
    stream: StringIO
    msg: message
    """
    try:
        for s in type(msg).__slots__:
            val = msg.__getattribute__(s)
            message_to_csv(stream, val, flatten)
    except BaseException:
        msg_str = str(msg)
        if msg_str.find(",") != -1:
            if flatten:
                msg_str = msg_str.strip("(")
                msg_str = msg_str.strip(")")
                msg_str = msg_str.strip(" ")
            else:
                msg_str = "\"" + msg_str + "\""
        stream.write("," + msg_str)


def message_type_to_csv(stream, msg, parent_content_name=""):
    """
    stream: StringIO
    msg: message
    """
    try:
        for s in type(msg).__slots__:
            val = msg.__getattribute__(s)
            message_to_csv(stream, val, flatten)
    except BaseException:
        msg_str = str(msg)
        if msg_str.find(",") != -1:
            if flatten:
                msg_str = msg_str.strip("(")
                msg_str = msg_str.strip(")")
                msg_str = msg_str.strip(" ")
            else:
                msg_str = "\"" + msg_str + "\""
        stream.write("," + msg_str)


def message_type_to_csv(stream, msg, parent_content_name=""):
    """
    stream: StringIO
    msg: message
    """
    try:
        for s in type(msg).__slots__:
            val = msg.__getattribute__(s)
            message_type_to_csv(stream, val, ".".join(
                [parent_content_name, s]))
    except BaseException:
        stream.write("," + parent_content_name)


def format_csv_filename(form, topic_name):
    global seq
    if form is None:
        return "Convertedbag.csv"
    ret = form.replace('%t', topic_name.replace('/', '-'))
    ret = ret[1:]
    return ret


def bag_to_csv(options, fname):
    try:
        bag = rosbag.Bag(fname)
        streamdict = dict()
        stime = None
        if options.start_time:
            stime = rospy.Time(options.start_time)
        etime = None
        if options.end_time:
            etime = rospy.Time(options.end_time)
    except Exception as e:
        rospy.logfatal("failed to load bag file: %s", e)
        exit(1)
    finally:
        rospy.loginfo(f"loaded bag file: {fname}")

    try:
        for topic, msg, time in bag.read_messages(topics=options.topic_names,
                                                  start_time=stime,
                                                  end_time=etime):
            if topic in streamdict:
                stream = streamdict[topic]
            else:
                stream = open(
                    format_csv_filename(
                        options.output_file_format,
                        fname[fname.rfind('/'): -4] + topic),
                    'w')
                streamdict[topic] = stream
                # header
                if options.header:
                    stream.write("time")
                    message_type_to_csv(stream, msg)
                    stream.write('\n')

            stream.write(
                datetime.fromtimestamp(
                    time.to_time()).strftime('%Y/%m/%d/%H:%M:%S.%f'))
            message_to_csv(stream, msg, flatten=not options.header)
            stream.write('\n')
        [s.close for s in streamdict.values()]
    except Exception as e:
        rospy.logwarn("fail: %s", e)
    finally:
        bag.close()


def GetTopicList(path):
    bag = rosbag.Bag(path)
    topics = list(bag.get_type_and_topic_info()[1].keys())
    print(f"{topics=}")
    types = []
    for dict_values in list(bag.get_type_and_topic_info()[1].values()):
        print(dict_values[0])
        types.append(dict_values[0])

    results = []
    for to, ty in zip(topics, types):
        results.append(to)

    return results


def main(options):

    options.topic_names = GetTopicList(options.file)
    options.output_file_format = "%t_" + os.path.splitext(os.path.basename(options.file))[0] + ".csv"

    print("Converting....")
    bag_to_csv(options, options.file)

if __name__ == '__main__':
    print("rosbag_to_csv start!!")
    parser = ArgumentParser()
    parser.add_argument("-f", "--file", dest="file", required=True)
    parser.add_argument("-a", "--all", dest="all_topics",
                      action="store_true",
                      help="exports all topics", default=False)
    parser.add_argument("-t", "--topic", dest="topic_names",
                      action="append",
                      help="white list topic names", metavar="TOPIC_NAME")
    parser.add_argument("-s", "--start-time", dest="start_time",
                      help="start time of bagfile", type=float)
    parser.add_argument("-e", "--end-time", dest="end_time",
                      help="end time of bagfile", type=float)
    parser.add_argument("-n", "--no-header", dest="header",
                      action="store_false", default=True,
                      help="no header / flatten array value")
    (options, args) = parser.parse_args()

    main(options)
