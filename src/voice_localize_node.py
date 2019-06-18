#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Int16, Float32
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from geometry_msgs.msg import Quaternion
import actionlib
import tf_conversions
import math
from move_base_msgs.msg import MoveBaseGoal

class voice_localize:
    def __init__(self):
        self.keyword_sub = rospy.Subscriber('/kws_data', String, self.process)
        self.client = rospy.Publisher('/move_base_simple/goal',PoseStamped , queue_size=1)
        rospy.loginfo('Connected to move_base.')
    def process(self,data):
        doa = rospy.wait_for_message('/Doa',Float32)
        pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
        rpy = self.quaternion_rpy(pose.pose.pose.orientation)
        radian = self.angle_rad(doa.data)
        quaternion_calculated = self.calculate(radian,rpy)
        new_goal = self.create_goal(pose,quaternion_calculated)
        self.client.publish(new_goal)
        print('sending goal')
        # self.client.wait_for_result()
        print('reached')

    def quaternion_rpy(self,orientation):
        quaternion = (orientation.x,
                      orientation.y,
                      orientation.z,
                      orientation.w)
        rpy = tf_conversions.transformations.euler_from_quaternion(quaternion)
        print(rpy)
        return rpy
    def angle_rad(self,angle):
        ang = angle
        return math.radians(ang)

    #calcuate the required angle
    def calculate(self,radian,rpy):
        #print(radian)
        yaw_ = rpy[2] + radian
        if yaw_ > 6.28:
            yaw_-= 6.28
        print(radian,rpy,yaw_)
        roll_ = 0
        pitch_ = 0
        quaternian_ = tf_conversions.transformations.quaternion_from_euler(roll_,pitch_,yaw_)
        quaternian_out = Quaternion()
        quaternian_out.x = quaternian_[0]
        quaternian_out.y = quaternian_[1]
        quaternian_out.z = quaternian_[2]
        quaternian_out.w = quaternian_[3]
        self.quaternion_rpy(quaternian_out)
        return quaternian_out
    #create goal for move base
    def create_goal(self,pose, quaternion):
        # old_pose = PoseWithCovarianceStamped()
        old_pose = pose
        goal = PoseStamped()
        goal.pose.position =old_pose.pose.pose.position
        goal.pose.orientation = quaternion
        goal.header.frame_id = 'map'
        return goal

if __name__ == '__main__':
    rospy.init_node('voice_command', anonymous=True)
    voice = voice_localize()
    rospy.spin()
