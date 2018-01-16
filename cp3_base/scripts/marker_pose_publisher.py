#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import transformations as tr
import numpy
from aruco_msgs.msg import MarkerArray,Marker

def process_markers(markers):
    global listener, br, marker_trans_mat

    sum_mat = None
    num = 0
    t_latest = rospy.Time(0)
    for marker in markers.markers:
        marker_link = '/Marker' + str(marker.id) + "__link"
        marker_id = "/marker" + str(marker.id)
        tw = listener.getLatestCommonTime(marker_link, 'world')
        to = listener.getLatestCommonTime(marker_id, 'odom')
        t = tw if tw > to else to
        t_latest = t if t > t_latest else t_latest
        (trans_w,rot_w) = listener.lookupTransform(marker_link, 'world', tw)
        (trans_o,rot_o) = listener.lookupTransform(marker_id, 'odom', to)


        w_mat = numpy.dot(tr.translation_matrix(trans_w), tr.quaternion_matrix(rot_w))
        t_o = tr.concatenate_matrices(tr.translation_matrix(trans_o), tr.quaternion_matrix(rot_o))
        # Need to do the transform equivalent to 0.25 0 0 0 0.5 -0.5 -0.5 0.5 on marker_id
        t_o = tr.dot(t_o, marker_trans_mat);
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



if __name__ == '__main__':

    marker_trans_mat = numpy.dot(tr.translation_matrix([0.25, 0, 0]), tr.quaternion_matrix([0.5,-0.5,-0.5,0.5]))

    rospy.init_node('marker_pose_publisher')

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(60.0)

    rospy.Subscriber("aruco_marker_publisher/markers", MarkerArray, process_markers)

    rospy.spin()