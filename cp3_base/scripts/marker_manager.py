#! /usr/bin/env python
'''
Because of memory issues with gazebo and the number of markers needed, this
node tracks the location of the robot and adds the markers in a 5m radius of
it and removes markers outside that.
'''
from __future__ import print_function

import argparse
import errno
import os
import sys
import json
import re
import math
import transformations as tf
from gazebo_msgs.srv import *
import rospy
from geometry_msgs.msg import *


MARKER_TEMPLATE = '''

<sdf version='1.4'>
<model name="Marker$ID">
  <static>1</static>
  <link name="link">
    <visual name="visual">
      <geometry>
        <mesh>
          <uri>model:/marker$ID/meshes/Marker$ID.dae</uri>
        </mesh>
      </geometry>
    </visual>
    <self_collide>0</self_collide>
    <kinematic>0</kinematic>
  </link>
</model>
</sdf>'''

class Marker:
    x = None
    y = None
    w = None
    name = None

    def construct(self, x, y, w, name):
        self.x = x
        self.y = y
        self.w = w
        self.name = name
        match = re.search("Marker([0-9]+)", name)
        num = match.group(1)
        self.xml = MARKER_TEMPLATE.replace("$ID", num)
        pose = Pose ()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.25
        
        q = tf.quaternion_from_euler(0, 0, w)
            

        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        self.pose = pose
        # Generate obstacles name (in threadsafe manner)

        # Create gazebo request and call
        self.spawn_request = SpawnModelRequest()
        self.spawn_request.model_name = "Marker%s" %num
        self.spawn_request.model_xml = self.xml
        self.spawn_request.initial_pose = self.pose
    
    def __init__(self, json):
        wall = json["wall"]
        w = None
        if wall == "north":
            w = -math.pi / 2;
        elif wall == "south":
            w = math.pi / 2;
        elif wall == "east":
            w = 0
        elif wall == "west":
            w = math.pi
        self.name = json["id"]
        self.construct(json["x"], json["y"], w, json["id"])

    def __eq__(self, other):
        return isinstance(other, Marker) and self.name == other.name

    def __hash__(self):
        return hash(self.name)


class MarkerManager:
    visible_markers = set()

    all_markers = set()

    def __init__(self, marker_file):

        self.load_markers(marker_file)

        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

        try:
            rospy.wait_for_service("/gazebo/get_model_state", timeout=30)
        except rospy.ROSException as e:
            raise e

    def load_markers(self, file):
        f = open(file)
        s = f.read()
        j = json.loads(s)
        for m in j:
            self.all_markers.add(Marker(m))

    def place_marker(self, marker):
        print ("Adding %s" %marker.name)
        try:
            res = self.spawn_model(marker.spawn_request)
            if res.success:
                self.visible_markers.add(marker)
                return True
            else:
                rospy.logerr("Could not place obstacle. Message: %s" %res.status_message)
                return False
        except rospy.ServiceException as e:
            rospy.logerr("Could not place obstacle. Message %s" %e)
            return False

    def delete_marker(self, marker):
        print("Deleting %s" %marker.name)
        req = DeleteModelRequest()
        req.model_name = marker.name
        res = self.delete_model(req)
        if res.success:
            self.visible_markers.remove(marker)
        else:
            print("Failed to delete the model: %s" %res.status_message)

    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x1-x2)**2 + (y1-y2)**2)

    def markers_in_range(self, rx, ry, radius):
        return set([x for x in self.all_markers if self.distance(rx, ry, x.x, x.y) <= radius])

    def process_markers(self):
        tb = self.get_model_state('mobile_base', '')
        if not tb.success:
            return
        in_range = self.markers_in_range(tb.pose.position.x, tb.pose.position.y, 5.0)

        d = self.visible_markers.difference(in_range) # visbile but no longer in range
        a = in_range.difference(self.visible_markers) # in range but not visible

        # for m in d:
        #     print("Will delete %s" %m.name)
        # for m in a:
        #     print("Will add %s" %m.name)

    #    for m in d:
    #        self.delete_marker(m)
        for m in a:
            self.place_marker(m)

    def loop(self):
        rate = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            self.process_markers()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("marker_memory_manager")
    markerfile = None
    try:
        markerfile = rospy.get_param('~marker_file')
    except KeyError:
        # Not started as part of launch so must be in args
        parser = argparse.ArgumentParser()
        parser.add_argument("marker_file", type=str, help='The marker file to read in markers')
        args = parser.parse_args()
        markerfile = args.marker_file

    mm = MarkerManager (os.path.expandvars(markerfile))
    mm.loop()
