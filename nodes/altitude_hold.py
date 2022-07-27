import rospy 
from std_msgs.msg import Float64 

class AltitudeHold():

    def __init__(self):

        self.setpoint = -3
        self.Kp = None
        self.Ki = None
        self.Kd = None
        self.maximum_distance_threshold = 5
        self.minimum_error_threshold = 0.2
        self.sample_time = 0.01
        #Sub
        rospy.Subscriber("/bluerov/mavros/global_position/rel_alt",Float64,self.set_alt)
        #Pub
        self.vertical_thrust_pub = rospy.Publisher("vertical_thrust",
                                                   Float64,
                                                   queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(self.sample_time,self.Alt_hold))

    def clamp(self, output, max, min):
        if output is None: 
            return output
        elif (max is not None) and (output > max):
            return max 
        elif (min is not None) and (output < min):
            return min
        return 

    def set_alt(self,data):
        self.curr_alt = data

    def get_alt(self):
        return self.curr_alt

    def Alt_hold(self):

        #mid = 0.5
        #max = 1
        #min = 0
        dt = 0.01

        if(self.setpoint is not None):                         
            curr_alt = self.get_alt()
            err = self.setpoint - curr_alt

            self.prop = self.Kp * err
            self.int = self.Ki * err * dt
            #self.int = self.clamp()

            self.der = self.Kd * err / dt

            out = float(self.prop + self.int + self.der)

            if abs(out) < self.minimum_error_threshold:
                pid_out = 0

            if abs(out) < self.maximum_distance_threshold:
                pid_out = out/self.maximum_distance_threshold
        else:
            pid_out = 0

        
        self.vertical_thrust_pub.publish(pid_out)

        return pid_out

            
if __name__ == "__main__":
    rospy.init_node("alt_hold")
    AltitudeHold()
    rospy.spin()

            
