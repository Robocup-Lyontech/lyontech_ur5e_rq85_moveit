#!/usr/bin/env python

import geometry_msgs.msg


class MoveGroup_Palbator_UR5e_Env(object):

    def __init__(self):

        pass

    def getBackScene(self):

        name = "Back"

        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = "base_link_inertia"
        p.pose.orientation.x = 0.0
        p.pose.orientation.y = 0.0
        p.pose.orientation.z = 0.0
        p.pose.orientation.w = 1.0
        p.pose.position.x = -0.29
        p.pose.position.y = 0.15
        p.pose.position.z = 0.39        

        size = (0.7, 0.05, 1.0)

        return name, p, size
    


    
    def getBaseScene(self):

        name = "Base"

        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = "base_link_inertia"
        p.pose.orientation.x = 0.0
        p.pose.orientation.y = 0.0
        p.pose.orientation.z = 0.0
        p.pose.orientation.w = 1.0
        p.pose.position.x = 0.0
        p.pose.position.y = 0.1
        p.pose.position.z = -0.2       

        return name, p, 0.4, 0.3    





    def getTopScene(self):

        name = "Top"

        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = "base_link_inertia"
        p.pose.orientation.x = 0.0
        p.pose.orientation.y = 0.0
        p.pose.orientation.z = 0.0
        p.pose.orientation.w = 1.0
        p.pose.position.x = 0.0
        p.pose.position.y = 0.0
        p.pose.position.z = 1.0       

        size = (0.5, 0.45, 0.4)

        return name, p, size
    


    def getEquipScene(self):

        name = "Equip"

        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = "base_link_inertia"
        p.pose.orientation.x = 0.0
        p.pose.orientation.y = 0.0
        p.pose.orientation.z = 0.0
        p.pose.orientation.w = 1.0
        p.pose.position.x = 0.15
        p.pose.position.y = 0.2
        p.pose.position.z = 0.61       

        size = (0.2, 0.2, 0.2)

        return name, p, size    