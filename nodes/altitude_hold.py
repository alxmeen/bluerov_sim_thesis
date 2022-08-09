#! /usr/bin/env python3

import rospy
import time 
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose


class PIDController():

    def __init__(self):

        self.setpoint = Pose()
        #self.setpoint.position.x = 0.0
        #self.setpoint.position.y = 2.0
        #self.setpoint.position.z = - 1.0

        self.Kp = 1
        self.Ki = 0
        self.Kd = 0
        self.maximum_distance_threshold = 5
        self.minimum_error_threshold = 0.1
        self.sample_time = 0.01
        self.curr_alt = 0.0
        self.int_x = 0
        self.int_y = 0
        self.int_z = 0

        self.last_time = None 

        self.last_input_x = None 
        self.last_input_y = None 
        self.last_input_z = None 
        self.last_output_x = None 
        self.last_output_y = None 
        self.last_output_z = None 

        self.curr_pose = Pose()
        #Sub
        #rospy.Subscriber("/bluerov/mavros/global_position/rel_alt",Float64,self.set_alt)
        rospy.Subscriber("/bluerov/SetWayPoint", Pose,self.set_WP)
        rospy.Subscriber("/bluerov/mavros/global_position/local", Odometry, self.set_pose)
        #Pub
        self.vertical_thrust_pub = rospy.Publisher("vertical_thrust", Float64, queue_size=1)
        self.thrust_pub = rospy.Publisher("thrust", Float64, queue_size=1)
        self.lateral_thrust_pub = rospy.Publisher("lateral_thrust", Float64, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(self.sample_time), self.do_work)




    def set_pose(self, data):
        
        self.curr_pose.position.x = data.pose.pose.position.x
        self.curr_pose.position.y = data.pose.pose.position.y
        self.curr_pose.position.z = data.pose.pose.position.z


    def get_pose(self):
        return self.curr_pose
    
    def set_WP(self, data):
        self.setpoint = data
    
    def get_WP(self):
        return self.setpoint

    def get_current_time(self):
        return time.time()



    def do_work(self, timer):

        dt = 0.01
        now = self.get_current_time()
        pid_out_x = 0
        pid_out_y = 0
        pid_out_z = 0

        if(self.setpoint is not None):      
            curr_pose = Pose()                   
            curr_pose = self.get_pose()

            

            

            err_x = self.setpoint.position.x - curr_pose.position.x
            err_y = self.setpoint.position.y - curr_pose.position.y
            err_z = self.setpoint.position.z - curr_pose.position.z
            
            self.prop_x = self.Kp * err_x
            self.prop_y = self.Kp * err_y
            self.prop_z = self.Kp * err_z

            self.int_x += self.Ki * err_x * dt
            self.int_y += self.Ki * err_y * dt
            self.int_z += self.Ki * err_z * dt

            self.int_x = max(-1, min(1, self.int_x))
            self.int_y = max(-1, min(1, self.int_y))
            self.int_z = max(-1, min(1, self.int_z))

            self.der_x = -self.Kd * err_x / dt
            self.der_y = -self.Kd * err_y / dt
            self.der_z = -self.Kd * err_z / dt

            out_x = self.prop_x + self.int_x + self.der_x
            out_y = self.prop_y + self.int_y + self.der_y
            out_z = self.prop_z + self.int_z + self.der_z
            #print(out_x, out_y, out_z)

            if abs(out_x) < self.minimum_error_threshold:
                pid_out_x = 0
            if abs(out_y) < self.minimum_error_threshold:
                pid_out_y = 0
            if abs(out_z) < self.minimum_error_threshold:
                pid_out_z = 0

            if abs(out_x) < self.maximum_distance_threshold:
                pid_out_x = out_x/self.maximum_distance_threshold
                pid_out_x = max(-1, min(1, pid_out_x))
            #else: pid_out_x = 0
            if abs(out_y) < self.maximum_distance_threshold:
                pid_out_y = out_y/self.maximum_distance_threshold
                pid_out_y = max(-1, min(1, pid_out_y))  
            #else: pid_out_y = 0   
            if abs(out_z) < self.maximum_distance_threshold:
                pid_out_z = out_z/self.maximum_distance_threshold
                pid_out_z = max(-1, min(1, pid_out_z))
            #else: pid_out_z = 0


                
        else:
            pid_out_x = 0
            pid_out_y = 0
            pid_out_z = 0
        
        #print(pid_out_x, pid_out_y, pid_out_z)
        
        self.last_output_x = pid_out_x
        self.last_output_y = pid_out_y
        self.last_output_z = pid_out_z

        self.last_input_x = self.setpoint.position.x
        self.last_input_y = self.setpoint.position.y
        self.last_input_z = self.setpoint.position.z

        self.last_time = now

        
        self.vertical_thrust_pub.publish(pid_out_z)
        self.thrust_pub.publish(pid_out_x)
        self.lateral_thrust_pub.publish(pid_out_y)

        return pid_out_x, pid_out_y, pid_out_z

            
if __name__ == "__main__":
    rospy.init_node("pid_node")
    PIDController()
    rospy.spin()

            
