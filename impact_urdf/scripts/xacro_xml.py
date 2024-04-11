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



def Elements(robot):


    conn = connectdb(robot)
    sql_statement = "SELECT * FROM v_urdf_raw where robot_name = '" + robot +"'" 
    elements = conn.execute(sql_statement)
    my_robot=Group()
    for element in elements:
        element_value = Group(xacro_xml = element['xml'])
        my_robot = element_value   
    conn.close()
    return my_robot

if __name__ == "__main__":
    print(Robot(Elements("openmv_camera"),"openmv_camera"))