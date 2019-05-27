import rospy,sys
from utils.geometry import Vector2D
from utils.functions import *
from utils.config import *
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from krssg_ssl_msgs.msg import BeliefState
from role import  GoToBall, GoToPoint, KickToPoint
from multiprocessing import Process
from kubs import kubs
from krssg_ssl_msgs.srv import bsServer
from math import atan2,pi
from utils.functions import *
from tactics import receiver
import multiprocessing
import threading
#pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)

kicker_id=1
receiver_id=2


def receiver_function(id_,state,pub):
	global receiver_id,kicker_id
	receiver_ = kubs.kubs(id_,state,pub)	
	print(id_)
	receiver_.update_state(state)
	#print(kub1.kubs_id)
	g_fsm = receiver.receiver()
	# g_fsm = GoToPoint.GoToPoint()
	g_fsm.add_kub(receiver_)
	# g_fsm.add_point(point=kub.state.ballPos,orient=normalize_angle(pi+atan2(state.ballPos.y,state.ballPos.x-3000)))
	#g_fsm.add_theta(theta=normalize_angle(pi+atan2(state.ballPos.y,state.ballPos.x+3000)))
	g_fsm.as_graphviz()
	g_fsm.write_diagram_png()
	print("bbb")
	#print('something before spin')
	g_fsm.spin()

def kicker_function(id_,state,pub):
	kicker = kubs.kubs(id_,state,pub)
	global receiver_id,kicker_id
	kicker.update_state(state)
	print(id_)
	receiver_ = kubs.kubs(receiver_id,state,pub)
	receiver_.update_state(state)
	target = receiver_.get_pos()
	#print(kub1.kubs_id)
	f_fsm = KickToPoint.KickToPoint(target)
	# g_fsm = GoToPoint.GoToPoint()
	f_fsm.add_kub(kicker)
	# g_fsm.add_point(point=kub.state.ballPos,orient=normalize_angle(pi+atan2(state.ballPos.y,state.ballPos.x-3000)))
	#f_fsm.add_theta(theta=normalize_angle(pi+atan2(state.ballPos.y,state.ballPos.x+3000)))
	f_fsm.add_theta(theta=normalize_angle(atan2(target.y - state.ballPos.y,target.x - state.ballPos.x)))
	#print('something before spin')
	f_fsm.spin()

	print("done")
	 
def receiver_main(process_id):
	pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
	rospy.init_node('node' + str(process_id),anonymous=False)

	while True:
		state = None
		# state=shared.get('state')
		rospy.wait_for_service('bsServer',)
		getState = rospy.ServiceProxy('bsServer',bsServer)
		try:
				state = getState(state)
		except rospy.ServiceException, e:
			print("exception")
		if state:
				#print('lasknfcjscnajnstate',state.stateB.homePos)
				#p2 = multiprocessing.Process(target=function2, args=(2,state.stateB, )) 
			state=state.stateB
			ball_Pos=Vector2D(state.ballPos.x,state.ballPos.y)
			global receiver_id,kicker_id
			receiver_ = kubs.kubs(receiver_id,state,pub)
			receiver_.update_state(state)
			kicker_ = kubs.kubs(kicker_id,state,pub)
			kicker_.update_state(state)

			print("distance between ball and receiver = ",dist(ball_Pos,receiver_.get_pos())/BOT_RADIUS)	
			if dist(ball_Pos,receiver_.get_pos())< dist(ball_Pos,kicker_.get_pos()):
				temp_id=kicker_id
				kicker_id=receiver_id
				receiver_id=temp_id

			print("receiver_main")
			receiver_function(receiver_id,state,pub)

			
			print("kicker_id",kicker_id)
			print("receiver_id",receiver_id)
			#p2.start()
				#p1.join()
				#p2.join()
			   # print('chal ja')
				# break
		#rospy.spin()	

def kicker_main(process_id):
	pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
	rospy.init_node('node' + str(process_id),anonymous=False)

	while True:
		state = None
		# state=shared.get('state')
		rospy.wait_for_service('bsServer',)
		getState = rospy.ServiceProxy('bsServer',bsServer)
		try:
				state = getState(state)
		except rospy.ServiceException, e:
				print("chutiya")		
		if state:
				#print('lasknfcjscnajnstate',state.stateB.homePos)
				#p2 = multiprocessing.Process(target=function2, args=(2,state.stateB, )) 
			state=state.stateB
			ball_Pos=Vector2D(state.ballPos.x,state.ballPos.y)
			global receiver_id,kicker_id
			receiver_ = kubs.kubs(receiver_id,state,pub)
			receiver_.update_state(state)
			kicker_ = kubs.kubs(kicker_id,state,pub)
			kicker_.update_state(state)
			
			print("distance between ball and receiver = ",dist(ball_Pos,receiver_.get_pos()))
			if dist(ball_Pos,receiver_.get_pos())< dist(ball_Pos,kicker_.get_pos()):
				temp_id=kicker_id
				kicker_id=receiver_id
				receiver_id=temp_id

			ball_Pos=Vector2D(state.ballPos.x,state.ballPos.y)

			print("kicker_id",kicker_id)
			print("receiver_id",receiver_id)

	
			

			print("process 2")
			kicker_function(kicker_id,state,pub)


#print str(kub.kubs_id) + str('***********')
p1 = multiprocessing.Process(target=kicker_main, args=(1,))
p2 = multiprocessing.Process(target=receiver_main, args=(2,))

p2.start()
p1.start()


p1.join()
p2.join()
