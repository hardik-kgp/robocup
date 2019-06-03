#alligns bots with id 1 ,2 and 3 
#in a square 


import os
import rospy
from krssg_ssl_msgs.msg import target_square
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from utils.config import *
from math import sqrt
BOT_1 = 0
BOT_2 = 1

target_1 = point_2d()
target_2 = point_2d()
homePos = None
awayPos = None
FIRST_CALL = 1
THRESHOLD = 2*BOT_BALL_THRESH
pub = rospy.Publisher('target_params', target_square)

def distance_(a, b):
    dx = a.x-b.x
    dy = a.y-b.y
    return sqrt(dx*dx+dy*dy)

def change_target():
	global target_1, target_2, homePos
	# temp = point_2d()
	# temp.x = target_1.x
	# temp.y = target_1.y

	# target_1.x = target_2.x
	# target_1.y = target_2.y

	# target_2.x = target_3.x
	# target_2.y = target_3.y

	# target_3.x = temp.x
	# target_3.y = temp.y

	if target_1.x>0:
		if target_1.y>0:
			target_1.x*=-1
		else:
			target_1.y*=-1
	

	else:
		if target_1.y>0:
			target_1.y*=-1
		else:
			target_1.x*=-1

	if target_2.x>0:
		if target_2.y>0:
			target_2.x*=-1
		else:
			target_2.y*=-1
	

	else:
		if target_2.y>0:
			target_2.y*=-1
		else:
			target_2.x*=-1




def check_target():
	global target_1, target_2
	dist1 = distance_(target_1, homePos[BOT_1])
	dist2 = distance_(target_2, homePos[BOT_2])

	flag1 = dist1<THRESHOLD
	flag2 = dist2<THRESHOLD

	print("flag1 = ",flag1," flag2 = ",flag2)
	if(flag1 and flag2):
		change_target()


def BS_callback(msg):
	global homePos, awayPos, FIRST_CALL, target_1, target_2
	homePos = msg.homePos
	awayPos = msg.awayPos
	msgs = target_square()
	
	if(FIRST_CALL):
		target_1.x=500
		target_1.y=500	
		target_2.x=-500
		target_2.y=-500
		FIRST_CALL = 0

	msgs.tx0 = target_1.x
	msgs.ty0 = target_1.y

	msgs.tx1 = target_2.x
	msgs.ty1 = target_2.y
	check_target()

	
	

	pub.publish(msgs)


if __name__ == "__main__":

	rospy.init_node('test_square',anonymous=False)
	rospy.Subscriber('/belief_state', BeliefState, BS_callback, queue_size=1000)
	# change_target()
	rospy.spin()
