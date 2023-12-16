#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

# FUNDAMENTAL: declare the global variable we are going to use as the msg type we want to save/send!
velocity_command = Twist()
pose_1 = Pose()
pose_2 = Pose()


def pose_1_callback(pose_msg):
    global pose_1
    pose_1 = pose_msg
    #rospy.loginfo('The copied turtle1 pose is:\n' + str(pose_1) + '\n')


def pose_2_callback(pose_msg):
    global pose_2
    pose_2 = pose_msg
    #rospy.loginfo('The copied turtle2 pose is:\n' + str(pose_2) + '\n')


def distance():
    x_squared = math.pow((pose_2.x - pose_1.x), 2)
    y_squared = math.pow((pose_2.y - pose_1.y), 2)
    calc_distance = math.sqrt(x_squared + y_squared)
    return calc_distance


def command_turtle():
	rospy.init_node('turtle_control', anonymous=True)
	rospy.Subscriber("/turtle1/pose", Pose, pose_1_callback)
	rospy.Subscriber("/turtle2/pose", Pose, pose_2_callback)
	twist = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
	# Subscriber,Publisher e Rate sono classi del rospy module
	rate = rospy.Rate(60)  # 60 Hz

	while not rospy.is_shutdown():
		arc_tan = math.atan2( (pose_1.y - pose_2.y), (pose_1.x - pose_2.x) )
		if distance() > 0.2:
			velocity_command.linear.x = distance()
			velocity_command.linear.y = 0
			velocity_command.linear.z = 0
			velocity_command.angular.x = 0
			velocity_command.angular.y = 0
			print("++++++++++++++++++++")
			print(distance())
			print(pose_2)	
			print(arc_tan)
			incident_turtles = (pose_2.theta>=(arc_tan-0.2)) and (pose_2.theta<=(arc_tan+0.2))
			if incident_turtles == False:
				velocity_command.angular.z = (arc_tan + pose_2.theta)*(distance()/3)
			else:
				velocity_command.angular.z = 0			
		else:
		    velocity_command.linear.x = 0
		    velocity_command.linear.y = 0
		    velocity_command.linear.z = 0
		    velocity_command.angular.x = 0
		    velocity_command.angular.y = 0
		    velocity_command.angular.z = 0

		twist.publish(velocity_command)
		rate.sleep()


if __name__ == '__main__':
    try:
        command_turtle()
    except rospy.ROSInterruptException:
        pass
	
