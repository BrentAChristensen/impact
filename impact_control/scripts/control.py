#!/usr/bin/env python3
import yaml
import rospkg
import roslib
roslib.load_manifest("rosparam")
import rosparam
import rospy
import roslaunch




#Pass in the package to find the Yaml file to load
def LoadYamlFiles():
  #find the package location
  rospack = rospkg.RosPack()
  config_dir = rospack.get_path("impact_urdf")
  with open(config_dir + '/config/components.yaml') as file:
    component_parts = yaml.safe_load(file)
    parts = component_parts['components']
    if (len(parts)> 0):
      for i in range(0,len(parts)):
          try:             
            control_path =  rospack.get_path(parts[i]+str("_control"))
            paramlist=rosparam.load_file(control_path + str("/config/controllers.yaml"),default_namespace="")
            for params, ns in paramlist:
              rosparam.upload_params(ns,params)
          except:
             pass

  controller_list =''
  controllers_dir = rospack.get_path("impact_control")
  with open(controllers_dir + '/config/components.yaml') as file:
    controller_names = yaml.safe_load(file)
    controllers = controller_names['controllers']
    if (len(controllers)>0):
      for x in range(0,len(controllers)):
        controller_list = controller_list + controllers[x] + " "
      
  controller_list = controller_list[0:len(controller_list)-1] 
  return controller_list
    
def StartControllers(controllers):
  uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
  roslaunch.configure_logging(uuid)
  launch = roslaunch.scriptapi.ROSLaunch()
                                                
  launch.start()

# Start another node
  print(controllers)
  node = roslaunch.core.Node(name='controllers_node', package="controller_manager",node_type="spawner", respawn="false", args=controllers)
  launch.launch(node)

  try:
    launch.spin()
  finally:
    # After Ctrl+C, stop all nodes from running
    launch.shutdown()




try:
    Controllers = LoadYamlFiles()
    StartControllers(Controllers)

except rospy.ServiceException as e:
    print("Service call failed: %s" % e)

if __name__ == "__main__":
    print(Controllers)