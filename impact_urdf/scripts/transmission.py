import rospkg
import sqlite3
import sys
import os
from odio_urdf import *
import subprocess

# 1: Add a new element


class hardwareInterface(Element):
    element_name = "hardwareInterface"

class joint(Element): 
    required_elements = ['hardwareInterface'] 
    required_attributes = ['name']

# 2: Allow our new element to be added to existing elements
Transmission.allowed_elements += [ 'joint' ,'hardwareInterface']
Actuator.allowed_elements += ['hardwareInterface']
Group.allowed_elements += [ 'joint','hardwareInterface' ]
Robot.allowed_elements += [ 'joint','hardwareInterface' ]






def connectdb(robot):
    rp = rospkg.RosPack()

    database_path = os.path.join(rp.get_path("impact_urdf"), "scripts", "urdf.db")
    conn = sqlite3.connect(database_path)
    conn.row_factory = sqlite3.Row
    return conn

def Elements(robot):
    conn = connectdb(robot)
    sql_statement = "SELECT * FROM v_urdf_transmissions WHERE robot_name = '" + robot +"'" 
    elements = conn.execute(sql_statement)
    my_robot=Group(robot)
    for element in elements:
        robot_transmission=Transmission(
        str(element['name']),
        Type(xmltext=str(element['transmission_type_name'])),
        joint(str(element['joint_name']), hardwareInterface(xmltext = str(element['hardwareInterface']))),
        Actuator(str(element['actuator_name']), 
            Mechanicalreduction(xmltext = str(element['mechanicalReduction'])))
             )      
        my_robot(robot_transmission)
    conn.close()
    
    return my_robot  

if __name__ == "__main__":
    print(str(Robot(Elements("ar3"),"ar3")).replace(
        "mechanicalreduction","mechanicalReduction").replace(
        "hardwareinterface","hardwareInterface")
        )

    

    