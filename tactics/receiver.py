import composite_behavior
import behavior
from utils.config import *
import enum
import math
import time
from utils.functions import *
from role import GoToPoint
from krssg_ssl_msgs.srv import bsServer
import memcache
import rospy,sys
shared = memcache.Client(['127.0.0.1:11211'],debug=False)




# receiver accepts a receive_point as a parameter and gets setup there to catch the ball
# It transitions to the 'aligned' state once it's there within its error thresholds and is steady
# Set its 'ball_kicked' property to True to tell it to dynamically update its position based on where
# the ball is moving and attempt to catch it.
# It will move to the 'completed' state if it catches the ball, otherwise
# it will go to 'failed'.
class receiver(composite_behavior.CompositeBehavior):


    class State(enum.Enum):
        
        receiving=1
        received=2

    def __init__(self):
        super(receiver,self).__init__()
        # super().__init__(continuous=False,
        #                  # Don't restart play if we change robots while kicking
        #                  # the ball
        #                  autorestart=lambda: not self.ball_kicked)
        for state in receiver.State:
            self.add_state(state, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            receiver.State.receiving, lambda: True,
                            'immediately')

        self.add_transition(
            receiver.State.receiving, receiver.State.received,
            lambda: dist(self.kub.get_pos(),self.kub.state.ballPos)<2*BOT_RADIUS,
            'steady and in position to receive')

        self.add_transition(receiver.State.received,
                            behavior.Behavior.State.completed,
                            lambda: True, 'ball received!')

        # self.add_transition(
        #                     receiver.State.receiving, behavior.Behavior.State.failed,
        #                     lambda:  self.check_failure(),
        #                     'ball missed :(')


    # @property
    # def ball_kicked(self):
    #     return self._ball_kicked
    # def ball_kicked(self, value):
    #     self._ball_kicked = value
    #     if value:
    #         self._ball_kick_time = time.time()

    def add_kub(self, kub):
        self.kub = kub

    def execute_receiving(self):
        state = None
        rospy.wait_for_service('bsServer',)
        getState = rospy.ServiceProxy('bsServer',bsServer)
        try:
            state = getState(state)
        except rospy.ServiceException, e:
            print("exception found")        
        if state:
            state=state.stateB
            self.kub.update_state(state)
            theta=angle_diff(self.kub.get_pos(),self.kub.state.ballPos)
            g_fsm = GoToPoint.GoToPoint()       
            g_fsm.add_point(self.kub.get_pos(),theta)               
            g_fsm.add_kub(self.kub)
            print('in receiver executing go to point')
            g_fsm.spin()
        while True:    
            state = None
            rospy.wait_for_service('bsServer',)
            getState = rospy.ServiceProxy('bsServer',bsServer)
            try:
                state = getState(state)
            except rospy.ServiceException, e:
                print("exception found")        
            if state:
                state=state.stateB
                self.kub.update_state(state)
                ball_vel = Vector2D(self.kub.state.ballVel.x, self.kub.state.ballVel.y)
                ball_angle = ball_vel.angle()
                velocity_magnitude = MAX_BOT_SPEED/(5*BOT_RADIUS)
                face_angle = Vector2D().normalizeAngle(ball_angle + math.pi)
                vx = velocity_magnitude*math.cos(face_angle)
                vy = velocity_magnitude*math.sin(face_angle)
                self.kub.move(vx, vy)
                self.kub.dribble(True)
                self.kub.execute()
                print("in state receiving waiting for ball")

                if (self.kub.state.ballVel.x)**2 + (self.kub.state.ballVel.y)**2 == 0  or (self.kub.has_ball()):
                    break;
                


    def execute_received(self):
        state = None
        rospy.wait_for_service('bsServer',)
        getState = rospy.ServiceProxy('bsServer',bsServer)
        try:
            state = getState(state)
        except rospy.ServiceException, e:
            print("exception found")        
        if state:
            state=state.stateB
            self.kub.update_state(state)
        print("enter received")
        self.kub.dribble(False)
        self.kub.execute()
        state = None
        rospy.wait_for_service('bsServer',)
        getState = rospy.ServiceProxy('bsServer',bsServer)
        try:
            state = getState(state)
        except rospy.ServiceException, e:
            print("exception found")        
        if state:
            state=state.stateB
            self.kub.update_state(state)

    def on_enter_received(self):
        pass

    def on_enter_receiving(self):
        pass



 

    def on_exit_receiving(self):
        pass


    def check_failure(self):
        THRESH = 1.5 * BOT_RADIUS
        ball_pos = Vector2D(self.kub.state.ballPos)
        kub_pos = Vector2D(self.kub.get_pos())
        ball_vel = Vector2D(self.kub.state.ballVel.x, self.kub.state.ballVel.y)
        ball_angle = ball_vel.angle()
        self.pass_line = Line(point1=ball_pos, angle=ball_angle)
        pass_line = self.pass_line
        ball_line_distance = pass_line.distance_from_point(ball_pos)
        kick_angle = pass_line.angle
        bot_ball_angle = kub_pos.angle(ball_pos)

        opp_kick_angle = Vector2D().normalizeAngle(kick_angle + math.pi)
        diff_angle = Vector2D().normalizeAngle(opp_kick_angle - bot_ball_angle)
        if abs(diff_angle) < math.pi / 2 and ball_line_distance < THRESH:
            return False
        else:
            return True
        state = None
        rospy.wait_for_service('bsServer',)
        getState = rospy.ServiceProxy('bsServer',bsServer)
        try:
            state = getState(state)
        except rospy.ServiceException, e:
            print("exception found")        
        if state:
            state=state.stateB
            self.kub.update_state(state)