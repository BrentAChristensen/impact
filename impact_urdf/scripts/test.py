import rospkg
import sqlite3
import sys
import os
from odio_urdf import *


def connectdb(robot):
    rp = rospkg.RosPack()

    database_path = os.path.join(rp.get_path("impact_urdf"), "scripts", "urdf.db")
    conn = sqlite3.connect(database_path)
    conn.row_factory = sqlite3.Row
    return conn



def Elements(robot):
    rp = rospkg.RosPack()
    conn = connectdb(robot)
    sql_statement = "SELECT * FROM robots"
    elements = conn.execute(sql_statement)
    my_robot=Group(robot)
    #for element in elements:
    #    element_value =Material(str(element['name']),Color(str(element['color'])))
    #    my_robot(element_value)
    conn.close
    xacro_project_path = os.path.join(rp.get_path(robot + "_description") , "urdf", "_d435.urdf.xacro")
    return xacro_project_path
    #return my_robot

if __name__ == "__main__":
    print(Robot(Elements("realsense2"),"realsense2"))
    #print("here")

