import rospkg
import sqlite3
import sys
import os
from odio_urdf import *

class robotNamespace(Element): pass

Plugin.allowed_elements += ['robotNamespace']

def connectdb():
    rp = rospkg.RosPack()
    database_path = os.path.join(rp.get_path("impact_urdf"), "scripts", "urdf.db")
    conn = sqlite3.connect(database_path)
    conn.row_factory = sqlite3.Row
    return conn

class Urdf():
    group = Group() 
    USE_Gazebo = True
    USE_Xacro = True
    robot_name = ''

    def __init__(self,name):
        import materials
        import plugins
        self.group = Group()
        self.robot_name = name
        self.name = name
        self.gazebo_ros_control = 'gazebo_ros_control'
        self.gazebo_ros_library = 'libgazebo_ros_control.so'
        self.group(materials.Elements(name))
        self.group(Gazebo(Plugin(filename=self.gazebo_ros_library, name=self.gazebo_ros_control)))

       # self.group(Gazebo(Plugin(robotNamespace(xmltext='ar3'),
        ##                       filename=self.gazebo_ros_library, name=self.gazebo_ros_control)))
        self.group(Link('world'),
                   Joint( 
                   Parent("world"), 
                   Child(name + "_base_link"),
                   Origin(xyz="0 0 0", rpy="0 0 0"),
                   name="world_joint",
                   type="fixed"))

 

        self.add(name)
 
    def add(self,name):
        import gazebo_references
        import plugins
        import transmission 
        import xacro_xml
        import links
        import joints
        import xacro


        self.group(xacro.Xacro(name)) 
                      
        if self.USE_Gazebo == True:      
            self.group(gazebo_references.Elements(name))
            self.group(plugins.Elements(name))
            self.group(xacro_xml.Elements(name))

        self.group(links.Elements(name))
        self.group(joints.Elements(name))
        
        if self.USE_Gazebo ==True:
            self.group(transmission.Elements(name))

        if not name==self.robot_name:
            conn = connectdb()
            sql_statement = "SELECT * FROM Robot_Parent_Link_Names WHERE robot = '" + name +"'" 
            elements = conn.execute(sql_statement)
            for element in elements:
                self.group(Joint(name + '_end_link_joint', 
                Parent(element['link']),
                Child(name+'_base_link'),type='fixed'))
            conn.close()
        return 
    

    def robot(self):
        xml = str(Robot(self.group,self.name)).replace(
        "mechanicalreduction","mechanicalReduction").replace(
        "hardwareinterface","hardwareInterface").replace(
        "updaterate","updateRate").replace(
        "topicname","topicName").replace(
        "gaussiannoise","gaussianNoise").replace(
        "mimicjoint","mimicJoint").replace(
        "mindepth","minDepth").replace(
        'xmltext=""','').replace(
        'providefeedback','provideFeedback').replace(
        "jointname","jointName").replace(
        "haspid","hasPID").replace(
        "robotnamespace","robotNamespace").replace(
        'maxeffort','maxEffort')
        
        return xml




        

 
 
