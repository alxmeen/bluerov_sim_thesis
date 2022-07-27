from lib2to3.refactor import get_all_fix_names
from socket import J1939_EE_INFO_NONE
from tkinter.messagebox import NO
import rospy 
from std_msgs.msg import Float64 

class AltitudeHold():

    def __init__(self):

        self.setpoint = -1
        self.Kp = None
        self.Ki = None
        self.Kd = None
        #Sub
        rospy.Subscriber("/bluerov/mavros/global_position/rel_alt",Float64,self.set_alt)
        #Pub
        self.vertical_thrust_pub = rospy.Publisher("vertical_thrust",
                                                   Float64,
                                                   queue_size=1)

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

        mid = 0.5
        max = 1
        min = 0
        dt = 0.01

        if(self.setpoint is not None):                         
            curr_alt = self.get_alt()
            err = self.setpoint - curr_alt

            self.pro_x = self.Kp * err
            self.int_x = self.Ki * err * dt
            self.int_x = self.clamp()
