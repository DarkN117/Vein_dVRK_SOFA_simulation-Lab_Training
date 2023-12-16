#!/usr/bin/env python3

import rospy

import geometry_msgs.msg # PoseStamped message from PSM pose (position+quaternion)
import sensor_msgs.msg # JointState message from PSM jaws angle (single value)

pose_stamped = geometry_msgs.msg.PoseStamped()
angle_info = sensor_msgs.msg.JointState()

def jaws_angle_callback(angle_msg):
    global angle_info
    angle_info = angle_msg

def pose_stamped_callback(pose_stamped_msg):
    global pose_stamped
    pose_stamped = pose_stamped_msg

def turtle_info_republisher():
    rospy.Subscriber("dvrk/PSM2/position_cartesian_current", geometry_msgs.msg.PoseStamped, pose_stamped_callback)
    rospy.Subscriber("dvrk/PSM2/state_jaw_current", sensor_msgs.msg.JointState, jaws_angle_callback)
    
    pub_angle = rospy.Publisher('PSM_Angle_info', sensor_msgs.msg.JointState, queue_size=10)
    pub_pose = rospy.Publisher('PSM_Pose_info', geometry_msgs.msg.PoseStamped, queue_size=10)
    rospy.init_node('PSM_info_republisher', anonymous=False)
    rate = rospy.Rate(10) # 20hz was working before
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        pub_angle.publish(angle_info)
        pub_pose.publish(pose_stamped)
        rate.sleep()

if __name__ == '__main__':
    try:
        turtle_info_republisher()
    except rospy.ROSInterruptException:
        pass
