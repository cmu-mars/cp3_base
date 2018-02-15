#!/usr/bin/env python  
from __future__ import print_function, with_statement
import roslib
import rospy
import math
import tf
import transformations as tr
import numpy
from aruco_msgs.msg import MarkerArray,Marker
from std_msgs.msg import UInt32MultiArray

import os
import sys
import json
import argparse
from threading import *

lock = Lock()

global_markers = set()

def process_markers(markers):
    global listener, br, marker_trans_mat

    sum_mat = None
    num = 0
    t_latest = rospy.Time(0)
    for marker in markers.markers:
        print("Seeing marker %s" %marker.id)
        marker_link = '/Marker' + str(marker.id) + "__link"
        marker_id = "/Marker" + str(marker.id)
        tw = listener.getLatestCommonTime(marker_link, 'world')
        to = listener.getLatestCommonTime(marker_id, 'odom')
        t = tw if tw > to else to
        t_latest = t if t > t_latest else t_latest
        (trans_w,rot_w) = listener.lookupTransform(marker_link, 'world', tw)
        (trans_o,rot_o) = listener.lookupTransform(marker_id, 'odom', to)


        w_mat = numpy.dot(tr.translation_matrix(trans_w), tr.quaternion_matrix(rot_w))
        t_o = tr.concatenate_matrices(tr.translation_matrix(trans_o), tr.quaternion_matrix(rot_o))
        # Need to do the transform equivalent to 0.25 0 0 0 0.5 -0.5 -0.5 0.5 on marker_id
        #t_o = numpy.dot(t_o, marker_trans_mat);
        o_mat_i = tr.inverse_matrix(t_o)
        mat3 = numpy.dot(w_mat, o_mat_i)
        mat3 = numpy.inverse_matrix(mat3)
        if sum_mat is None:
            sum_mat = mat3
        else:
            sum_mat = sum_mat + mat3
        num = num + 1

    avg_mat = sum_mat / num

    trans = tr.translation_from_matrix(avg_mat)
    rot = tr.quaternion_from_matrix(avg_mat)
    br.sendTransform(trans, rot, t_latest, "odom", "map")

gazebo_world_pose = tr.identity_matrix()


def getMarkerTFFromMap(m):
    # This doesn't work
    mid = "Marker%s" %m

    marker = marker_dict[mid]
    w_mat = None

    if marker is not None:
        t = [marker["x"], marker["y"], 0.25]
        w = 0
        if marker["wall"] == "north":
            w = -math.pi / 2
        elif marker["wall"] == "south":
            w = math.pi / 2
        elif marker["wall"] == "west":
            w = math.pi

        t = tr.translation_matrix((marker["x"], marker["y"], 0.25))
        r = tr.quaternion_matrix(tr.quaternion_from_euler(0,0,w))
        w_mat = tr.concatenate_matrices(t, r)

    return (w_mat, None)


def getWorldMarkerMatrixFromTF(m):
    global listener
    marker_link = "Marker%s__link" %m
    try:
        tw = listener.getLatestCommonTime(marker_link, 'world')
        (t,r) = listener.lookupTransform(marker_link, 'world', tw)
        w_mat = numpy.dot(tr.translation_matrix(t), tr.quaternion_matrix(r))
    except:
        return (None, None)
    return (w_mat, tw)

def list_markers(markers):
    global listener, br, marker_trans_mat, global_markers
    if len(markers.data) == 0:
        return
    with lock:
        global_markers |= set(markers.data)

def publish_marker_transforms():
    global global_markers
    sum_mat = None
    num = 0.0
    t_latest = rospy.Time(0)
    with lock:

        if len(global_markers) == 0:
            rospy.loginfo("There are no markers to process")
            return

        for marker in global_markers:
            
            marker_id = "/Marker%s" %marker

            to = listener.getLatestCommonTime(marker_id, 'odom')

            t = to
            #(w_mat, tw) = getMarkerTFFromMap(marker)
            (w_mat, tw) = getWorldMarkerMatrixFromTF(marker)

            if w_mat is None:
                continue
            if tw is not None:
                t = tw if tw > to else to

            t_latest = t if t > t_latest else t_latest

            
            (trans_o,rot_o) = listener.lookupTransform(marker_id, 'odom', to)


            t_o = numpy.dot(tr.translation_matrix(trans_o), tr.quaternion_matrix(rot_o))

            o_mat_i = tr.inverse_matrix(t_o) # need odom wrt marker
            mat3 = numpy.dot(o_mat_i, w_mat) # odom wrt world
            mat3 = tr.inverse_matrix(mat3) # world wrt odom
            num = num + 1

            if sum_mat is None:
                sum_mat = mat3
                
            else:
                sum_mat = sum_mat + mat3
        if sum_mat is None:
            return
        avg_mat = sum_mat / num
        #r_o = numpy.dot(tr.translation_matrix((0,0,0)), tr.quaternion_matrix((0,0,0, 1)))

        #avg_mat = numpy.dot(r_o, avg_mat)


        trans = tr.translation_from_matrix(avg_mat)
        rot = tr.quaternion_from_matrix(avg_mat)

        br.sendTransform(trans, rot, t_latest, "odom", "map")
        global_markers.clear()


    #br.sendTransform((0,0,0), (0,0,0,1), t_latest, "odom", "map")

def thread_main():
    rate = rospy.Rate(10)
    while True:
        publish_marker_transforms()
        rate.sleep()

front_transform = {"transform" : None, "time" : None}
back_transform = {"transform" : None, "time" : None}

def list_markers_gen(transform):
    def process_marker_transform(markers):
        sum_mat = None
        num = 0.0
        t_latest = rospy.Time(0)
        for marker in markers.data:
            marker_id = "/Marker%s" %marker

            to = listener.getLatestCommonTime(marker_id, 'odom')

            t = to
            #(w_mat, tw) = getMarkerTFFromMap(marker)
            (w_mat, tw) = getWorldMarkerMatrixFromTF(marker)

            if w_mat is None:
                continue
            if tw is not None:
                t = tw if tw > to else to

            t_latest = t if t > t_latest else t_latest

            
            (trans_o,rot_o) = listener.lookupTransform(marker_id, 'odom', to)


            t_o = numpy.dot(tr.translation_matrix(trans_o), tr.quaternion_matrix(rot_o))

            o_mat_i = tr.inverse_matrix(t_o) # need odom wrt marker
            mat3 = numpy.dot(o_mat_i, w_mat) # odom wrt world
            mat3 = tr.inverse_matrix(mat3) # world wrt odom
            num = num + 1

            if sum_mat is None:
                sum_mat = mat3
                
            else:
                sum_mat = sum_mat + mat3
        if sum_mat is None:
            return
        avg_mat = sum_mat / num

        transform["transform"] = avg_mat;
        transform["time"] = t_latest;
    return process_marker_transform;

published_transform = None

def back_front_thread():
    global published_transform
    rate = rospy.Rate(10)
    while True:
        time = rospy.Time.now()
        num = 0
        sum_mat = None
        #print("Time diff = %s" %(str(time.to_sec() - front_transform["time"].to_sec()) if front_transform["time"] is not None else "??"))
        if front_transform["transform"] is not None and time.to_sec() - front_transform["time"].to_sec() < 1:
            sum_mat = front_transform["transform"]
            num = 1.0
        if back_transform["transform"] is not None and time.to_sec() - back_transform["time"].to_sec() < 1:
            sum_mat = back_transform["transform"] if sum_mat is None else sum_mat + back_transform["transform"]
            num = num + 1.0
        if num > 0:
            transform = sum_mat / num
            published_transform = transform
            trans = tr.translation_from_matrix(transform)
            rot = tr.quaternion_from_matrix(transform)
            br.sendTransform(trans, rot, time, "odom", "map")
        elif published_transform is not None:
            transform = published_transform
            trans = tr.translation_from_matrix(transform)
            rot = tr.quaternion_from_matrix(transform)
            br.sendTransform(trans, rot, time, "odom", "map")
        rate.sleep()

marker_dict = {}

if __name__ == '__main__':

    marker_trans_mat = tr.concatenate_matrices(tr.translation_matrix([0, 0, 0]), tr.quaternion_matrix ([0.5,-0.5,-0.5,0.5])) #0.25,
    
    rospy.init_node('marker_pose_publisher')

    markerfile = None
    try:
        markerfile = rospy.get_param('~marker_file')
    except KeyError:
        parser = argparse.ArgumentParser()
        parser.add_argument("marker_file", type=str, help='The marker file to read in markers')
        args = parser.parse_args()
        markerfile = args.marker_file

    if not os.path.isfile(markerfile):
        print("The marker file '%s' does not exist" %markerfile)
        sys.exit()

    f = open(markerfile)
    s = f.read()
    markers = json.loads(s)

    for m in markers:
        marker_dict[m["id"]] = m

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(60.0)

    #rospy.Subscriber("aruco_marker_publisher/markers", MarkerArray, process_markers)
    #rospy.Subscriber("aruco_marker_publisher_front/markers_list", UInt32MultiArray, list_markers)
    #rospy.Subscriber("aruco_marker_publisher_back/markers_list", UInt32MultiArray, list_markers)
    rospy.Subscriber("aruco_marker_publisher_front/markers_list", UInt32MultiArray, list_markers_gen(front_transform))
    rospy.Subscriber("aruco_marker_publisher_back/markers_list", UInt32MultiArray, list_markers_gen(back_transform))
    rospy.loginfo("Marker Pose Publisher waiting for info")

    #marker_thread = Thread(target=thread_main)
    marker_thread = Thread(target=back_front_thread)
    marker_thread.start()

    rospy.spin()