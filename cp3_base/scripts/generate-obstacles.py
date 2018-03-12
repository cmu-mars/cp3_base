#! /usr/bin/env python
from __future__ import print_function

import argparse
from xml.dom.minidom import parse, parseString
import os
import sys
import json
import re


'''
for i in {amcl-kinect,amcl-lidar,mrpt-kinect,mrpt-lidar,aruco}; do python cli.py cover -r -c $i -i -l light3,light63,light62,light2,light10,light88,light11,light18,light72,light17 coverage.dat ../../../all.csv; done
'''

xml = '''
  <model name='%NAME%'>
      <static>1</static>
      <link name='link'>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <!--<uri>file:///home/%USER%/catkin_ws/src/cp3_base/models/media/mate
rials/scripts/cp3.material</uri>-->
              <name>obstacle/box</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <kinematic>0</kinematic>
      </link>
      <!--<pose frame=''>-1.34371 -0.653415 0 0 -0 0</pose>-->
    </model>
'''

def load_map(filename):
    f = open(filename)
    s = f.read()
    return json.loads(s)

def create_obstacle(x, y, id):
   dom = parseString(xml)
   model = dom.getElementsByTagName("model")[0]
   model.setAttribute("name", id)

   pose = dom.createElement("pose")
   pose.appendChild(dom.createTextNode("%s %s 0.25 0 0 0" %(x,y)))
   model.appendChild(pose)

   script = dom.getElementsByTagName("script")[0]
   scriptLoc = "file:///%s/catkin_ws/src/cp3_base/models/media/materials/scripts/cp3.material" %os.path.expanduser('~')
   uri = dom.createElement("uri")
   uri.appendChild(dom.createTextNode(scriptLoc))
   script.appendChild(uri)
   return dom

def process_world(dom, append):
    cur_obs_id = 0

    models = dom.getElementsByTagName("model")
    for model in models:
        name = model.getAttribute("name")
        obs_result = re.match("Obstacle_([0-9]+)", name)
        if obs_result:
            if not append:
                dom.removeChild(model)
            else:
                cur_obs_id = int(obs_result.group(1))
    return cur_obs_id


parser = argparse.ArgumentParser()
parser.add_argument("-o", "--output", type=str, help='The output world file to use')
parser.add_argument("-a", "--append", action='store_true', help="Do not delete existing obstacles")
parser.add_argument("map", help="The map file used to create the obstacles")
parser.add_argument("world", help="The world file to add to")

args = parser.parse_args()

args.map = os.path.expanduser(args.map)
args.world = os.path.expanduser(args.world)

if not hasattr(args, 'output') or args.output is None:
    args.output = args.world
else:
    args.output = os.path.expanduser(args.output)

if not os.path.isfile(args.map):
    print("The provided map file does not exist")
    sys.exit()

if not os.path.isfile(args.world):
    print("The provided world file does not exist")
    sys.exit()

world_dom = parse(args.world)
world = world_dom.getElementsByTagName("world")
cur_obs_id = process_world(world[0], args.append)

map = load_map(args.map)
obstacles = map["worldobstacles"]

for obs in obstacles:
    obs_dom = create_obstacle(obs["x"], obs["y"], "Obstacle_%s" %cur_obs_id)
    if obs_dom is not None:
        cur_obs_id = cur_obs_id + 1
        world[0].appendChild(obs_dom.firstChild)

with open(args.output, 'w') as f:
    world_dom.writexml(f, indent=' ', addindent=' ', newl='\n')
    world_dom.unlink()


