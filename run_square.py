import rospy,sys
from utils.geometry import Vector2D
from utils.functions import *
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from krssg_ssl_msgs.msg import BeliefState
from role import  GoToBall, GoToPoint
from multiprocessing import Process
from kubs import kubs
from krssg_ssl_msgs.srv import bsServer
from math import atan2,pi
from utils.functions import *
from role import test_square
pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)

def function(state):
	kub0 = kubs.kubs(0,state,pub)
	kub1 = kubs.kubs(1,state,pub)
	kub0.update_state(state)
	kub1.update_state(state)
	print(kub0.kubs_id)
	print(kub1.kubs_id)
	g_fsm = test_square.test_square()                      
	g_fsm.add_kub(kub0)
	g_fsm.add_kub(kub1) 
	g_fsm.spin()
	
rospy.init_node('node',anonymous=False)
start_time = rospy.Time.now()
start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   

while True:
	state = None
	rospy.wait_for_service('bsServer',)
	getState = rospy.ServiceProxy('bsServer',bsServer)
	try:
		state = getState(state)
	except rospy.ServiceException, e:
		print("exception found")		
	if state:
		print(state.stateB.homePos)
		function(state.stateB)
rospy.spin()	




