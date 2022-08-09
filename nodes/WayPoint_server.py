#! /usr/bin/env python3

from unittest import result
import rospy
import actionlib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from bluerov_sim.msg import WaypointAction, WaypointFeedback, WaypointResult


class WPServer():

    def __init__(self):
        
        self.position_now = Pose()

        self.th = 0.3

        #Pub
        self.waypointPub = rospy.Publisher('/bluerov/SetWayPoint', Pose, queue_size=1)
        #Sub
        rospy.Subscriber('/bluerov/mavros/global_position/local', Odometry, self.set_current_position)


        #Action
        self.waypoint_action_server = actionlib.SimpleActionServer("WayPointActionServer", WaypointAction, self.execute_WP, False)
        self.waypoint_action_server.start()

    
    def set_current_position(self, data):
        self.position_now.position.x = data.pose.pose.position.x
        self.position_now.position.y = data.pose.pose.position.y
        self.position_now.position.z = data.pose.pose.position.z


    
    def get_current_position(self):
        return self.position_now

    def execute_WP(self,goal):
        PubWP = Pose()
        PubWP = goal.waypoint

        

        result = WaypointResult()
        feedback = WaypointFeedback()
        success = True

        errors = [0,0,0]

        current = self.get_current_position()

        if current == None:
            rospy.logerr("Current position missing.")
            success = False
        else:
            errors[0] = abs(PubWP.position.x - current.position.x)
            errors[1] = abs(PubWP.position.y - current.position.y)
            errors[2] = abs(PubWP.position.z - current.position.z)

            while errors[0] > self.th or errors[1] > self.th or errors[2] > self.th:
                self.waypointPub.publish(PubWP)
                

                feedback.current_pose = current
                self.waypoint_action_server.publish_feedback(feedback)

                current = self.get_current_position()
                errors[0] = abs(PubWP.position.x - current.position.x)
                errors[1] = abs(PubWP.position.y - current.position.y)
                errors[2] = abs(PubWP.position.z - current.position.z)
        
        print("done")
        result.success = success
        self.waypoint_action_server.set_succeeded(result)
        

        


if __name__ == "__main__":
    rospy.init_node("waypoint_server_node")
    WPServer()
    rospy.spin()