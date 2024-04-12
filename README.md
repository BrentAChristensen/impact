Create a share folder in your dev folder
clone the following packages
- gazebo-pkgs
- general-messages-pkgs
- odio_urdf
- realsense_gazebo_plugin
- roboticsgroup_upatras_gazebo_plugins
- rviz_visual_tools

- cd <package name>
- create a build folder
- run: $cmake .. -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic/share
- run: $sudo make install

use -DCMAKE_INSTALL_PREFIX=/your/install/path for each package placed in the share folder
- gazebo-pkgs
- general-messages-pkgs
- odio_urdf
- realsense_gazebo_plugin
- roboticsgroup_upatras_gazebo_plugins
Clone find_object_2d into the catkin workspace
 - general-messages-pkgs
create soft links in the catkin_ws for ln -s SOURCE_FILE LINK_NAME
