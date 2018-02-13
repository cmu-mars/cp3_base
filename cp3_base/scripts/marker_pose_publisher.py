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
    except e:
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
            if marker != 658:
                continue
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
            mat3 = numpy.dot(w_mat, o_mat_i) # odom wrt world
            mat3 = tr.inverse_matrix(mat3) # world wrt odom
            num = num + 1

            if sum_mat is None:
                sum_mat = mat3
                break
            else:
                sum_mat = sum_mat + mat3
        if sum_mat is None:
            return
        avg_mat = sum_mat / num
        r_o = numpy.dot(tr.translation_matrix((0,0,0)), tr.quaternion_matrix((0,0,0, 1)))

        #avg_mat = numpy.dot(r_o, avg_mat)


        trans = tr.translation_from_matrix(avg_mat)
        rot = tr.quaternion_from_matrix(avg_mat)

        br.sendTransform(trans, rot, t_latest, "odom", "map")
        global_markers.clear()


    #br.sendTransform((0,0,0), (0,0,0,1), t_latest, "odom", "map")

def thread_main():
    rate = rospy.Rate(60)
    while True:
        publish_marker_transforms()
        rate.sleep()



#         (trans_o,rot_o) = listener.lookupTransform(marker_id, 'camera_temp_link', to)
#         (t_c_o, r_c_o) = listener.lookupTransform('camera_temp_link', 'odom', to)
#         t_c = numpy.dot(tr.translation_matrix(t_c_o), tr.quaternion_matrix(r_c_o))

#         t_r = numpy.dot(tr.translation_matrix((0,0,0)), tr.quaternion_matrix((0,-1,0,1)))


#         w_mat = numpy.dot(tr.translation_matrix(trans_w), tr.quaternion_matrix(rot_w))
#         t_o = numpy.dot(tr.translation_matrix(trans_o), tr.quaternion_matrix(rot_o))
#         #r_o = numpy.dot(tr.translation_matrix((0,0,0)), tr.quaternion_matrix((0.5, -0.5, -0.5, 0.5)))
#         # r_o = numpy.dot(tr.translation_matrix((0,0,0)), tr.quaternion_matrix((0, 0, 0, 1)))

#         t_o = numpy.dot(t_o,t_c)

#         #t_o = numpy.dot(r_o, t_o)
# #        t_o = numpy.dot(tr.quaternion_matrix([0.5, -0.5, -0.5, 0.5]), t_o)
#         # t_o = numpy.dot(t_o, marker_trans_mat)
#         o_mat_i = tr.inverse_matrix(t_o)
#         mat3 = numpy.dot(w_mat, o_mat_i)
#        #mat3 = tr.inverse_matrix(mat3)
#         num = num + 1

#         if sum_mat is None:
#             sum_mat = mat3
#         else:
#             sum_mat = sum_mat + mat3
#         break

#     avg_mat = sum_mat / num
#     r_o = numpy.dot(tr.translation_matrix((0,0,0)), tr.quaternion_matrix((0,0,0,1)))

#     avg_mat = numpy.dot(avg_mat, r_o)


#     trans = tr.translation_from_matrix(avg_mat)
#     rot = tr.quaternion_from_matrix(avg_mat)

#     br.sendTransform(trans, rot, t_latest, "odom", "map")

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
    rospy.Subscriber("aruco_marker_publisher_front/markers_list", UInt32MultiArray, list_markers)
    rospy.Subscriber("aruco_marker_publisher_back/markers_list", UInt32MultiArray, list_markers)
    rospy.loginfo("Marker Pose Publisher waiting for info")

    marker_thread = Thread(target=thread_main)
    marker_thread.start()

    rospy.spin()