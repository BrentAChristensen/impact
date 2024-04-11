#!/bin/sh
import sys
from urdf_subcriber import Urdf
import yaml
import rospkg
import time


#find the package location
rospack = rospkg.RosPack()
config_path = rospack.get_path(sys.argv[1])

#Pass in the package to find the Yaml file to load
with open(config_path + '/config/components.yaml') as file:
  component_parts = yaml.safe_load(file)
  parts = component_parts['components']
  urdf = Urdf(parts[0])
  if (len(parts)> 1):
    for i in range(1,len(parts)):
      urdf.add(parts[i])
_ = urdf
_.USE_Gazebo=True
print(urdf.robot())



