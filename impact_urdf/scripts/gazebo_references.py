import rospkg
import sqlite3
import sys
import os
from odio_urdf import *

class kp(Element): pass

class kd(Element): pass

class minDepth(Element): pass

class provideFeedback(Element): pass
   

# 2: Allow our new element to be added to existing elements
Gazebo.allowed_elements += [ 'kp' ,'kd','minDepth','provideFeedback']

def connectdb(robot):
    rp = rospkg.RosPack()

    database_path = os.path.join(rp.get_path("impact_urdf"), "scripts", "urdf.db")
    conn = sqlite3.connect(database_path)
    conn.row_factory = sqlite3.Row
    return conn

def Elements(robot):
    conn = connectdb(robot)
    sql_statement = "SELECT * FROM v_urdf_gazebo_references WHERE robot_name= '" + robot +"'" 
    elements = conn.execute(sql_statement)
    my_robot=Group(robot)
    for element in elements:
        element_value =Gazebo(
            '' if str(element['kp']) =='None' else kp(xmltext=str(element['kp'])),
            '' if str(element['minDepth']) =='None' else minDepth(xmltext=str(element['minDepth'])),
            '' if str(element['kd']) =='None' else kd(xmltext=str(element['kd'])),
            '' if str(element['provideFeedback']) =='None' else provideFeedback(xmltext=str(element['provideFeedback'])),
            Material('Gazebo/'+ str(element['color_name'])),
                       reference=str(element['reference']))
        my_robot(element_value)
    conn.close
    return my_robot

if __name__ == "__main__":
    print(str(Robot(Elements("wheeltec_gripper"),"wheeltec_gripper")).replace(
        'xmltext=""','').replace(
        'providefeedback','provideFeedback')
    )