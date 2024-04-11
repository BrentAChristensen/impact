import rospkg
import sqlite3
import sys
import os
from odio_urdf import *

robot_name = ''

def connectdb(robot):
    rp = rospkg.RosPack()

    database_path = os.path.join(rp.get_path("impact_urdf"), "scripts", "urdf.db")
    conn = sqlite3.connect(database_path)
    conn.row_factory = sqlite3.Row
    return conn

def geometryvalues(geoType,geoValue):
    if geoType == 'Mesh':
        return Mesh(filename = geoValue)
    elif geoType =='Box':
        return Box(size=geoValue)
    elif geoType == 'Capsule':
        return Capsule(size=geoValue)
    elif geoType == 'Cylinder':
        return Cylinder(size=geoValue)
    elif geoType =='Sphere':
        return Sphere(size=geoValue)
    else:
        return Mesh(filename = 'none')
        
def Elements(robot):
    conn = connectdb(robot)
    robot_name = robot
    sql_statement = "SELECT * FROM v_urdf_links WHERE robot_name = '" + robot +"'" 
    elements = conn.execute(sql_statement)
    my_robot=Group()
    for element in elements:
        if element['ixx'] == None:
            element_value = Link(name=element['link_name'])
            my_robot(element_value)
        else:
            element_value=Link(
            Inertial(
            Mass(str(element['link_inertial_mass'])),
            Origin(rpy=str(element['link_inertial_origin_rpy']),
                xyz=str(element['link_inertial_origin_xyz'])),
            Inertia(ixx = str(element['ixx']),
                    ixy = str(element['ixy']),
                    ixz = str(element['ixz']),
                    iyy = str(element['iyy']),
                    iyz = str(element['iyz']),
                    izz = str(element['izz'])),
            ),
            Visual(
            Origin(rpy=str(element['link_visual_origin_rpy']),
                    xyz=str(element['link_visual_origin_xyz'])),
            Geometry(
                geometryvalues(str(element['name']),str(element['visual_geometry_parameter_value'])),
                name= str(element['link_visual_geometry_name'])),
            Material(str(element['visual_material_name'])),
            ),
            Collision(
            Origin(rpy=str(element['link_collision_origin_rpy']),
                    xyz=str(element['link_collision_origin_xyz'])),
            Geometry(geometryvalues(str(element['name']),str(element['collision_geometry_parameter_value']))) 
                ),
            name= str(element['link_name']))
            
            my_robot(element_value)
    conn.close()
    return my_robot

if __name__ == "__main__":
    print(Robot(Elements("ar3"),"ar3"))
