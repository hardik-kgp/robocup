import composite_behavior
import behavior
import time
import enum
import logging
import cmd_node
import rospy
from krssg_ssl_msgs.msg import BeliefState
import rospy
from utils.functions import *
from utils.config import *
pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)

class test_square(behaviour.Behaviour):


	class State(enum.Enum):
		setup = 1
		edge1 = 2
		edge2 = 3
		edge3 = 4
		edge4 = 5


	
	def __init__(self):
		# print "gtp"
		#GoToPoint.behavior.Behavior()
		#g = behavior.Behavior()
		#print "gtp2"
		#self.state = state

		super(GoToBall,self).__init__()

		self.corner1=Vector2D(200,2000)
		self.corner2=Vector2D(-200,2000)
		self.corner3=Vector2D(-200,1800)
		self.corner4=Vector2D(200,1800)
		self.kubs=[]
		self.target_point0=self.corner1
		self.target_point1=self.corner3

		self.name = "GoToPoint"

		self.behavior_failed = False
		self.DISTANCE_THRESH = DISTANCE_THRESH

		self.add_state(GoToPoint.State.setup,
			behavior.Behavior.State.running)
		self.add_state(GoToPoint.State.edge1,
			behavior.Behavior.State.running)
		self.add_state(GoToPoint.State.edge2,
			behavior.Behavior.State.running)
		self.add_state(GoToPoint.State.edge3,
			behavior.Behavior.State.running)
		self.add_state(GoToPoint.State.edge4,
			behavior.Behavior.State.running) 

		self.add_transition(behavior.Behavior.State.start,
			GoToPoint.State.setup,lambda: True,'immediately')

		self.add_transition(GoToPoint.State.setup,
			GoToPoint.State.edge1,lambda: True,'setup')

		#self.add_transition(GoToPoint.State.drive,
		#    GoToPoint.State.drive,lambda: not self.at_new_point(),'restart')

		self.add_transition(GoToPoint.State.edge1,
			behavior.Behavior.State.edge2,lambda:self.atcorner1(1) and self.atcorner1(0),'complete')
		self.add_transition(GoToPoint.State.edge2,
			behavior.Behavior.State.edge3,lambda:self.atcorner2(1) and self.atcorner2(0),'complete')        
		self.add_transition(GoToPoint.State.edge3,
			behavior.Behavior.State.edge4,lambda:self.atcorner3(1) and self.atcorner3(0),'complete')
		self.add_transition(GoToPoint.State.edge4,
			behavior.Behavior.State.edge1,lambda: self.atcorner4(1) and self.atcorner4(0),'complete')


	def atcorner1(self,BOT_ID):
		#print (dist(self.target_point,self.new_point),210)
		return dist(self.corner1,self.kubs[BOT_ID].get_pos()) < self.DISTANCE_THRESH
	def atcorner2(self,BOT_ID):
		#print (dist(self.target_point,self.new_point),210)
		return dist(self.corner2,self.kubs[BOT_ID].get_pos()) < self.DISTANCE_THRESH
	def atcorner3(self,BOT_ID):
		#print (dist(self.target_point,self.new_point),210)
		return dist(self.corner3,self.kubs[BOT_ID].get_pos()) < self.DISTANCE_THRESH
	def atcorner4(self,BOT_ID):
		#print (dist(self.target_point,self.new_point),210)
		return dist(self.corner4,self.kubs[BOT_ID].get_pos()) < self.DISTANCE_THRESH

	def add_kub(self,kub):
		self.kubs.append(kub)

	def on_enter_setup(self):
		self.target_point0 = self.corner1
		_GoToPoint_.init(self.kub[0], self.target_point0,0)
		self.target_point1 = self.corner3
		_GoToPoint_.init(self.kub[1], self.target_point1,0) 
		pass

	def execute_setup(self):
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
		generatingfunction = _GoToPoint_.execute(start_time,self.Di,True)
		#self.behavior_failed = False
		for gf in generatingfunction:
			self.kub[0],target_point = gf
			# self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
			self.target_point0 = self.corner1
			if not vicinity_points(self.target_point0,target_point,thresh=BOT_RADIUS*2.0):
				self.behavior_failed = True
				break
		
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
		generatingfunction = _GoToPoint_.execute(start_time,self.Di,True)
		#self.behavior_failed = False
		for gf in generatingfunction:
			self.kub[1],target_point = gf
			# self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
			self.target_point1 = self.corner3
			if not vicinity_points(self.target_point1,target_point,thresh=BOT_RADIUS*2.0):
				self.behavior_failed = True
				break

	def on_exit_setup(self):
		pass

	def on_enter_edge1(self):
		self.target_point0 = self.corner2
		_GoToPoint_.init(self.kub[0], self.target_point0,0)
		self.target_point1 = self.corner4
		_GoToPoint_.init(self.kub[1], self.target_point1,0)
		pass 

	def execute_edge1(self):
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
		generatingfunction = _GoToPoint_.execute(start_time,self.Di,True)
		#self.behavior_failed = False
		for gf in generatingfunction:
			self.kub[0],target_point = gf
			# self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
			self.target_point0 = self.corner2
			if not vicinity_points(self.target_point0,target_point,thresh=BOT_RADIUS*2.0):
				self.behavior_failed = True
				break
		
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
		generatingfunction = _GoToPoint_.execute(start_time,self.Di,True)
		#self.behavior_failed = False
		for gf in generatingfunction:
			self.kub[1],target_point = gf
			# self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
			self.target_point1 = self.corner4
			if not vicinity_points(self.target_point1,target_point,thresh=BOT_RADIUS*2.0):
				self.behavior_failed = True
				break

	def on_exit_edge1(self):
		pass

	def on_enter_edge2(self):
		self.target_point0 = self.corner3
		_GoToPoint_.init(self.kub[0], self.target_point0,0)
		self.target_point1 = self.corner1
		_GoToPoint_.init(self.kub[1], self.target_point1,0)
		pass 

	def execute_edge2(self):
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
		generatingfunction = _GoToPoint_.execute(start_time,self.Di,True)
		#self.behavior_failed = False
		for gf in generatingfunction:
			self.kub[0],target_point = gf
			# self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
			self.target_point0 = self.corner3
			if not vicinity_points(self.target_point0,target_point,thresh=BOT_RADIUS*2.0):
				self.behavior_failed = True
				break
		
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
		generatingfunction = _GoToPoint_.execute(start_time,self.Di,True)
		#self.behavior_failed = False
		for gf in generatingfunction:
			self.kub[1],target_point = gf
			# self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
			self.target_point1 = self.corner1
			if not vicinity_points(self.target_point1,target_point,thresh=BOT_RADIUS*2.0):
				self.behavior_failed = True
				break

	def on_exit_edge2(self):
		pass

	def on_enter_edge3(self):
		self.target_point0 = self.corner4
		_GoToPoint_.init(self.kub[0], self.target_point0,0)
		self.target_point1 = self.corner2
		_GoToPoint_.init(self.kub[1], self.target_point1,0)
		pass 

	def execute_edge3(self):
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
		generatingfunction = _GoToPoint_.execute(start_time,self.Di,True)
		#self.behavior_failed = False
		for gf in generatingfunction:
			self.kub[0],target_point = gf
			# self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
			self.target_point0 = self.corner4
			if not vicinity_points(self.target_point0,target_point,thresh=BOT_RADIUS*2.0):
				self.behavior_failed = True
				break
		
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
		generatingfunction = _GoToPoint_.execute(start_time,self.Di,True)
		#self.behavior_failed = False
		for gf in generatingfunction:
			self.kub[1],target_point = gf
			# self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
			self.target_point1 = self.corner2
			if not vicinity_points(self.target_point1,target_point,thresh=BOT_RADIUS*2.0):
				self.behavior_failed = True
				break

	def on_exit_edge3(self):
		pass

	def on_enter_edge4(self):
		self.target_point0 = self.corner1
		_GoToPoint_.init(self.kub[0], self.target_point0,0)
		self.target_point1 = self.corner3
		_GoToPoint_.init(self.kub[1], self.target_point1,0)
		pass 

	def execute_edge4(self):
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
		generatingfunction = _GoToPoint_.execute(start_time,self.Di,True)
		#self.behavior_failed = False
		for gf in generatingfunction:
			self.kub[0],target_point = gf
			# self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
			self.target_point0 = self.corner1
			if not vicinity_points(self.target_point0,target_point,thresh=BOT_RADIUS*2.0):
				self.behavior_failed = True
				break
		
		start_time = rospy.Time.now()
		start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
		generatingfunction = _GoToPoint_.execute(start_time,self.Di,True)
		#self.behavior_failed = False
		for gf in generatingfunction:
			self.kub[1],target_point = gf
			# self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
			self.target_point1 = self.corner3
			if not vicinity_points(self.target_point1,target_point,thresh=BOT_RADIUS*2.0):
				self.behavior_failed = True
				break

	def on_exit_edge4(self):
		pass

