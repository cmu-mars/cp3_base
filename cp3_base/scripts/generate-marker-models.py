#! /usr/bin/env python
'''
Generates Gazebo models for visual markers by manipulating files
in a template directory. Requires png files that are numbered, and 
generated from Aruco image generators.
'''
from __future__ import print_function

import argparse
import xml.etree.ElementTree as ET
import os
import sys
import json
import re
import math
import fnmatch
import shutil
import datetime
import time

def model_config(topath, num):
    # Process model.config
    configT = ET.parse(topath + "/model.config")
    root = configT.getroot()

    name = root.find('name')
    name.text = 'Marker%s' %num
    configT.write(topath + "/model.config")

def models_sdf(topath, num):
    root = ET.parse(topath + "/model.sdf")
    sdf = root.getroot()

    model = sdf.find('model')
    model.set('name', 'Marker%s' %num)
    uri = model.find("./link/visual/geometry/mesh/uri")
    #uri.text="model://marker%s/meshes/Marker%s.dae" %(num,num)
    uri.text="file:///%s/meshes/Marker%s.dae" %(topath,num)

    root.write(topath + "/model.sdf")

    sdf.set('version', '1.4')
    root.write(topath + "/model-1_4.sdf")

    sdf.set('version', '1.5')
    root.write(topath + "/model-1_5.sdf")

def mesh(topath, num):

    # Namespace problems
    with open(topath + "/meshes/Marker0.dae") as f:
        xmls = f.read()

    xmls = re.sub('\\sxmlns="[^"]+"', '', xmls)
    COLLADA = ET.fromstring(xmls)
    #COLLADA = dae.getroot()

    created=COLLADA.find('asset/created')
    created.text = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    modified=COLLADA.find('asset/modified')
    modified.text = created.text

    image = COLLADA.find('library_images/image')
    image.set('id', 'Marker%s_png' %num)
    image.set('name', 'Marker%s_png' %num)
    init_from = image.find('init_from')
    #init_from.text = "marker%s.png" %num
    init_from.text = "file:///%s/materials/textures/marker%s.png" %(topath,num)

    library_effects = COLLADA.find('library_effects')
    effect = library_effects.find('effect')
    effect.set('id', 'Marker%sMat-effect' %num)
    newparam = effect.findall('profile_COMMON/newparam')
    newparam[0].set('sid', 'Marker%s_png-surface' %num)
    init_from = newparam[0].find('surface/init_from')
    init_from.text = "Marker%s_png" %num
    newparam[1].set('sid', 'Marker%s_png-sampler' %num)
    source = newparam[1].find('sampler2D/source')
    source.text = "Marker%s_png-surface" %num

    texture = library_effects.find('effect/profile_COMMON/technique/phong/diffuse/texture')
    texture.set('texture', 'Marker%s_png-sampler' %num)

    polylist = COLLADA.find('library_geometries/geometry/mesh/polylist')
    polylist.set('material', 'Marker%sMat-material' %num)

    material = COLLADA.find('library_materials/material')
    material.set('id', 'Marker%sMat-material' %num)
    material.set('name', 'Marker%sMat' %num)
    instance_effect = material.find('instance_effect')
    instance_effect.set('url', '#Marker%sMat-effect' %num)

    node = COLLADA.find('library_visual_scenes/visual_scene/node')
    node.set('id', 'Marker%s' %num)
    node.set('name', 'Marker%s' %num)

    instance_material = node.find('instance_geometry/bind_material/technique_common/instance_material')
    instance_material.set('symbol', 'Marker%sMat-material' %num)
    instance_material.set('target', '#Marker%sMat-material' %num)

    #Write to Marker<num>.dae
    ET.ElementTree(COLLADA).write(topath + "/meshes/Marker%s.dae" %num)
    #Delete Marker0.dae
    os.remove(topath + "/meshes/Marker0.dae")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('image_dir', type=str, help='The location of all the marker images')
    parser.add_argument('template_dir', type=str, help='The location of all the files for the Gazebo model template')
    parser.add_argument('model_dir', type=str, help='Where to put the finished models')

    args = parser.parse_args()
    args.image_dir = os.path.expandvars(args.image_dir)
    args.template_dir = os.path.expandvars(args.template_dir)
    args.model_dir = os.path.expandvars(args.model_dir)

    if not os.path.isdir(args.image_dir):
        print("Could not find the image directory: " + args.image_dir)
        sys.exit()

    if not os.path.isdir(args.template_dir):
        print("Could not find the model template directory: " + args.image_dir)
        sys.exit()

    if not os.path.isdir(args.model_dir):
        print("Could not find the directory to place the models: " + args.model_dir)
        sys.exit()

    # Get all the numbered PNGs in the image_dir
    pngs = []
    for png in os.listdir(args.image_dir):
        if fnmatch.fnmatch(png, '[0-9]*.png'):
            pngs.append(png)

    for png in pngs:
        # Get the model number from the filename
        num = os.path.splitext(png)[0]

        # Copy the template dir into a directory called marker<num>
        topath = args.model_dir + "/marker" + str(num)
        if os.path.exists(topath):
            shutil.rmtree(topath)
        shutil.copytree(args.template_dir, topath)

        shutil.copy(args.image_dir + "/" + png, topath+"/materials/textures/marker" + str(num) + ".png")

        model_config(topath, num)
        models_sdf(topath, num)
        mesh(topath, num)





