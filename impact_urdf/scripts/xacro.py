import rospkg
import sqlite3
import sys
import os
import subprocess
from odio_urdf import *



def connectdb(robot):
    rp = rospkg.RosPack()

    database_path = os.path.join(rp.get_path("impact_urdf"), "scripts", "urdf.db")
    conn = sqlite3.connect(database_path)
    conn.row_factory = sqlite3.Row
    return conn



def Xacro(project_name):
    rp = rospkg.RosPack()

    conn = connectdb(project_name)
    sql_statement = "SELECT * FROM v_xacros where name = '" + project_name +"'" 
    elements = conn.execute(sql_statement)
    
    xacro_name= ""
    element_value = ""
    for element in elements:
        xacro_name = element['file_name']
  
    if xacro_name != "":
        my_robot=Group(project_name)
        xacro_project_path = os.path.join(rp.get_path(project_name + "_description") , "urdf", xacro_name)
        res = subprocess.run(['rosrun', 'xacro', 'xacro', xacro_project_path], capture_output=True)
        element_value = Group(xacro_xml = res.stdout)
    else:
        my_robot=Group()

    my_robot = element_value  

    return my_robot

if __name__ == "__main__":
    xml = Robot(Xacro("wheeltec_gripper"))
    print(len(str(xml)))


 