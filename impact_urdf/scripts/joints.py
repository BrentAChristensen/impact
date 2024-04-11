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
    conn = connectdb(robot)
    sql_statement = "SELECT * FROM v_urdf_joints where robot_name = '" + robot +"'" 
    elements = conn.execute(sql_statement)
    my_robot=Group()
    for element in elements:
        if element['parent_link'] == None:
            element_value = Joint(
            Parent("world"),
            Child(str(element['child_link_name'])),
            Origin(rpy=str(element['joint_origin_rpy']),
                xyz=str(element['joint_origin_xyz'])),
            name=str(element['name']),
            type=str(element['type']))
            my_robot(element_value)
        else:
            element_value = Joint(
            Axis(str(element['axis'])),
            Limit(effort = str(element['effort']),
                velocity=str(element['velocity']),
                lower=str(element['lower']),
                upper=str(element['upper'])),
        '' if str(element['mimic_joint']) =='None' else Mimic(joint=str(element['mimic_joint']),multiplier=str(element['multiplier'])),     
        Dynamics(damping = str(element['dynamics_damping']),
                    friction = str(element['dynamics_friction'])),
            Origin(rpy=str(element['joint_origin_rpy']),
                xyz=str(element['joint_origin_xyz'])),
            Parent( str(element['parent_link_name'] )),
            Child(str(element['child_link_name'])),
            name=str(element['name']),
            type=str(element['type'])
            )
            my_robot(element_value)
    conn.close()
    return my_robot

if __name__ == "__main__":
    print(Robot(Elements("ar3"),"ar3"))


#        Limit(effort = str(element['effort']),
#              velocity=str(element['velocity']),
#              lower=str(element['lower']),
#              upper=str(element['upper'])),
#        '' if str(element['mimic_joint']) =='None' else Mimic(joint=str(element['mimic_joint']),multiplier=str(element['multiplier'])),     
#        Dynamics(damping = str(element['dynamics_damping']),
#                 friction = str(element['dynamics_friction'])),





  