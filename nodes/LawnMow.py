#! /usr/bin/env python3

import rospy
import actionlib
import time 
import sys
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

from bluerov_sim.msg import WaypointAction, WaypointGoal, WaypointResult




class LawnMower():

    def __init__(self):

        #sub
        #pub
        #self.WayPub = rospy.Publisher("/bluerov/SetWayPoint", Pose, queue_size=1)

        #action 
        

        self.Max_x = 2
        self.Max_y = 2
        self.ConstDepth = - 1

        try:
            result = self.MowTheLawn()
            rospy.loginfo("RESULT: "+str(result.success))
        except rospy.ROSInterruptException:
            rospy.loginfo("Move up task interrupted with error: " + str(sys.stderr))
        

    def MowTheLawn(self):

        Lawnmower_client = actionlib.SimpleActionClient("WayPointActionServer", WaypointAction)

        x = self.Max_x
        y = self.Max_y
        z = self.ConstDepth

        result = WaypointResult()
        pose_pub = Pose()
        Lawnmower_client.wait_for_server()

        goal = WaypointGoal()

        for x in range(-self.Max_x,self.Max_x +1):
            for y in range(-self.Max_y,self.Max_y +1):
                pose_pub.position.x = x
                pose_pub.position.y = y
                pose_pub.position.z = z

                goal.waypoint = pose_pub
                print(goal.waypoint)

                Lawnmower_client.send_goal(goal)

                Lawnmower_client.wait_for_result()

                result = Lawnmower_client.get_result()

                #self.WayPub.publish(pose_pub)
                #print(x,y,z)

        return result
                
            
if __name__ == "__main__":
    rospy.init_node("lawnmower")
    LawnMower()
    rospy.spin()








