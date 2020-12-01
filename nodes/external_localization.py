#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class ExternalLocalizationNode():
    def __init__(self):
        rospy.init_node("ext_localization")

        self.pose_pub = rospy.Publisher("mavros/vision_pose/pose",
                                        PoseStamped,
                                        queue_size=1)
        self.ground_truth = PoseStamped()

        rospy.Subscriber("ground_truth/state",
                         Odometry,
                         self.on_ground_truth,
                         queue_size=1)

    def on_ground_truth(self, msg):
        pose_msg = PoseStamped()
        # pose_msg.header = msg.header
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = rospy.Time.now()
        
        pose_msg.pose = msg.pose.pose

        # add noise to avoid rejection
        # print(np.random.normal(0.0, 0.05))
        pose_msg.pose.position.x += np.random.normal(0.0, 0.05)
        pose_msg.pose.position.y += np.random.normal(0.0, 0.05)
        pose_msg.pose.position.z += np.random.normal(0.0, 0.05)
        # quaternion should be normalized in ekf2
        pose_msg.pose.orientation.w += np.random.normal(0.0, 0.001)
        pose_msg.pose.orientation.x += np.random.normal(0.0, 0.001)
        pose_msg.pose.orientation.y += np.random.normal(0.0, 0.001)
        pose_msg.pose.orientation.z += np.random.normal(0.0, 0.001)
        self.ground_truth = pose_msg

    def run(self):
        rate = rospy.Rate(30.0)
        while not rospy.is_shutdown():
            self.pose_pub.publish(self.ground_truth)
            rate.sleep()


def main():
    node = ExternalLocalizationNode()
    node.run()


if __name__ == "__main__":
    main()
