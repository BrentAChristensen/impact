import rospkg
import sqlite3
import sys
import os
from odio_urdf import *

# 1: Add a new elements
class updateRate(Element):
    element_name = "updateRate"

class joint(Element): pass

class gaussianNoise(Element):pass

class jointName(Element):pass

class topicName(Element):pass

class multiplier(Element):pass

class offset(Element): pass

class mimicJoint(Element): pass

class jointName(Element): pass

class hasPID(Element): pass

class robotNamespace(Element): pass

class sensitiveness(Element): pass

class maxEffort(Element): pass

Gazebo.allowed_elements += [ 'joint' ,'hardwareInterface']



# 2: Allow our new element to be added to existing elements
Plugin.allowed_elements += [ 'updateRate' ,'joint','gaussianNoise','jointName','topicName','multiplier','offset','mimicJoint','hasPID','robotNamespace','maxEffort','sensitiveness']


def addCustom(conn,robot):
    sql_statement = "SELECT * FROM v_urdf_custom WHERE name =  '" + robot +"'" 
    gazeboOptions = conn.execute(sql_statement)
    gazebo = Gazebo()
    for gazeboOption in gazeboOptions:
        gazebo = Gazebo(xmltext=gazeboOption['xml'] )
        
    return gazebo

def addPluginOptions(conn,plugin_id,plugin_name,file_name):
    sql_statement = "SELECT * FROM v_urdf_plugins WHERE plugin_id = " + str(plugin_id)

    plugin = Plugin(filename= file_name, name=plugin_name)
    pluginoptions = conn.execute(sql_statement)
    for pluginoption in pluginoptions:
        plugin(pluginOptions(pluginoption['plugin_option'],pluginoption['plugin_option_value']))
    plugin(name=plugin_name,filename=file_name)
    return plugin

def pluginOptions(tag,tagValue):
    if tag == 'updateRate':
        return updateRate(xmltext=tagValue)
    elif tag  =='topicName':
        return topicName(xmltext=tagValue)
    elif tag == 'gaussianNoise':
        return gaussianNoise(xmltext=tagValue)
    elif tag == 'joint':
        return joint(xmltext=tagValue)
    elif tag =='multiplier':
        return multiplier(xmltext=tagValue)
    elif tag == 'offset':
        return offset(xmltext=tagValue)
    elif tag == 'mimicJoint':
        return mimicJoint(xmltext=tagValue)
    elif tag == 'jointName':
        return jointName(xmltext=tagValue)
    elif tag == 'hasPID':
        return hasPID(xmltext=tagValue)
    elif tag == 'robotNamespace':
        return robotNamespace(xmltext=tagValue)
    elif tag == 'sensitiveness':
        return sensitiveness(xmltext=tagValue)
    elif tag == 'maxEffort':
        return maxEffort(xmltext=tagValue)
    else:
        return ''

def connectdb(robot):
    rp = rospkg.RosPack()

    database_path = os.path.join(rp.get_path("impact_urdf"), "scripts", "urdf.db")
    conn = sqlite3.connect(database_path)
    conn.row_factory = sqlite3.Row
    return conn

def Elements(robot):
    conn = connectdb(robot)
    sql_statement = "SELECT plugin_name,filename,plugin_id FROM v_urdf_plugins WHERE robot_name = '" + robot +"' GROUP BY plugin_name, plugin_id, filename" 
    elements = conn.execute(sql_statement)
    my_robot=Group(robot)
    for element in elements:
        element_value =Gazebo(
            addPluginOptions(conn,str(element['plugin_id']),str(element['plugin_name']),str(element['filename'])))
        my_robot(element_value)

    my_robot(addCustom(conn,robot))
    conn.close
    return my_robot

if __name__ == "__main__":
    print(str(Robot(Elements("wheeltec_gripper"),
                    "wheeltec_gripper")).replace(
        "updaterate","updateRate").replace(
        "topicname","topicName").replace(
        "gaussiannoise","gaussianNoise").replace(
        "mimicjoint","mimicJoint").replace(
        "jointname","jointName").replace(
        "haspid","hasPID").replace(
        "robotNamespace","robotNamespace").replace(
        'maxeffort','maxEffort'
        )
        )