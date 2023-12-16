#!/usr/bin/env python
import rospy
#usare Pose,Twist:tartaruga, PoseStamped dVRK MTMR
import geometry_msgs.msg #Twist,PoseStamped
from turtlesim.msg import Pose #Pose
import math

# FUNDAMENTAL: declare the global variable we are going to use as the msg type we want to save/send!
velocity_command = geometry_msgs.msg.Twist()
turtle_pose = Pose()
pose_stamped = geometry_msgs.msg.PoseStamped()

def pose_stamped_callback(pose_stamped_msg):
    global pose_stamped
    pose_stamped = pose_stamped_msg
    #rospy.loginfo('The copied MTMR pose is:\n' + str(pose_stamped) + '\n')


def turtle_pose_callback(pose_msg):
    global turtle_pose
    turtle_pose = pose_msg
    #rospy.loginfo('The copied turtle_pose is:\n' + str(turtle_pose) + '\n')


#Euclidean distance between turtle and goal pose(commanded by only x and y components of MTMR pose)
def distance(x_desired,y_desired):
    x_squared = math.pow((x_desired - turtle_pose.x), 2)
    y_squared = math.pow((y_desired - turtle_pose.y), 2)
    calc_distance = math.sqrt(x_squared + y_squared)
    return calc_distance


def command_turtle():
	rospy.init_node('turtle_control', anonymous=True)
	rospy.Subscriber("dvrk/MTMR/position_cartesian_current", geometry_msgs.msg.PoseStamped, pose_stamped_callback)
	rospy.Subscriber("/turtle1/pose", Pose, turtle_pose_callback)
	twist = rospy.Publisher('/turtle1/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
	rate = rospy.Rate(60)  # 60 Hz

	while not rospy.is_shutdown():
		
		#Remapping of x and y coord. of MTMR to reference frame of turtle.
		#(probably values are slightly wrong but the formulas are right)
		#(RE-DO REMAPPING PROPERLY IF NEEDED)
		desired_pos_x = ( ( pose_stamped.pose.position.x + 0.5) / 1.1 )* 11
		desired_pos_y = ( ( pose_stamped.pose.position.y + 0.48 ) / 0.35 )* 11
		
		actual_distance = distance(desired_pos_x,desired_pos_y)

		angle = math.atan2( ( desired_pos_y - turtle_pose.y ),( desired_pos_x -turtle_pose.x ) )

		#This is a bool. If the turtle_pose.theta is more or less equal to the tangent angle
		#(the direction vector linking turtle and goal position) it becomes TRUE
		same_angle = ( turtle_pose.theta >= (angle-0.2) ) and ( turtle_pose.theta <= (angle+0.2))
		

		print('------------------------------------')	
		print( '>The goal pose are: \n' + str(desired_pos_x) + '\n'+str(desired_pos_y))
		print( '\n>The tan_angle and distance are: \n'+str(angle)+'\n'+str( actual_distance ) )
		
		
		#The turtle is moved only if we are far enough from the goal: linear velocity is
		#proportional to de distance, angular velocity is proportional to
		#to angle displacement between turtle angle and goal point(so theta+angle)
		if actual_distance>0.2:
			velocity_command.linear.x = actual_distance	
			velocity_command.linear.y = 0
			velocity_command.linear.z = 0
			velocity_command.angular.x = 0
			velocity_command.angular.y = 0

			if not same_angle:
				velocity_command.angular.z = angle + turtle_pose.theta
			else:
				velocity_command.angular.z = 0
		else:
			velocity_command.linear.x = 0
			velocity_command.angular.z = 0
			#If we are near the goal turtle is stopped
	
		twist.publish(velocity_command)
		rate.sleep()


if __name__ == '__main__':
    try:
        command_turtle()
    except rospy.ROSInterruptException:
        pass
	
