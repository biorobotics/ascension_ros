#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import time


class SavePoses(object):
    def __init__(self):
        
        self._pose = Pose()
        self.poses_dict = {"pose1":self._pose, "pose2":self._pose, "pose3":self._pose}
        self._pose_sub = rospy.Subscriber('/ascension_node/target_poses', Odometry , self.sub_callback)
        self.write_to_file()

    def sub_callback(self, msg):
        
        self._pose = msg.pose.pose
    
    def write_to_file(self):
        
        time.sleep(5)
        self.poses_dict["pose1"] = self._pose
        rospy.loginfo("Written pose1")
        time.sleep(5)
        self.poses_dict["pose2"] = self._pose
        rospy.loginfo("Written pose2")
        time.sleep(5)
        self.poses_dict["pose3"] = self._pose
        rospy.loginfo("Written pose3")
            
        
        with open('poses.txt', 'w') as file:
            
            for key, value in self.poses_dict.iteritems():
                if value:
                    file.write(str(key) + ':\n----------\n' + str(value) + '\n===========\n')
                    
        rospy.loginfo("Written all Poses to poses.txt file")
        


if __name__ == "__main__":
    rospy.init_node('spot_recorder', log_level=rospy.INFO) 
    save_spots_object = SavePoses()
    #rospy.spin() # mantain the service open.