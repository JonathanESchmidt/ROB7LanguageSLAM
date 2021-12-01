import sys

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

from languageslam.srv import toggleexploration

class robot_command():
    def __init__(self, robotno):
        self.robotno = robotno
        self.robotname = "robot" + str(robotno)

    def exploration_client(self, state):
        rospy.wait_for_service('/'+ self.robotname + '/toggleexploration')
        try:
            exploration = rospy.ServiceProxy('/'+ self.robotname + '/toggleexploration', toggleexploration)
            resp1 = exploration(state)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def moverobot(self, move):
        
        if move == 'left':
            self.sendGoal(0, 1, 1.5708)
        elif move == 'right':
            self.sendGoal(0, -1, -1.5708)
        elif move == 'forward':
            self.sendGoal(1, 0, 0)
        elif move == 'backward':
            self.sendGoal(-1, 0, -3.1415)

    def sendGoal(self, x, y, theta):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.robotname + "/base_footprint"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        # RPY to quaternion convert: 90deg, 0, -90deg
        orientation = quaternion_from_euler(0, 0, theta)
        goal.target_pose.pose.orientation.x = orientation[0]
        goal.target_pose.pose.orientation.y = orientation[1]
        goal.target_pose.pose.orientation.z = orientation[2]
        goal.target_pose.pose.orientation.w = orientation[3]
        
        #print(str(goal))

        try:
            self.client = actionlib.SimpleActionClient('/' + self.robotname + '/move_base', MoveBaseAction)
            print("Client started")
            self.client.wait_for_server(rospy.Duration(1))
            self.client.send_goal(goal)
        except:
            print("Could not wait for sever")

    def stoprobot(self):
        try:
            self.client = actionlib.SimpleActionClient('/' + self.robotname + '/move_base', MoveBaseAction)
            print("Client started")
            self.client.wait_for_server(rospy.Duration(1))
            self.client.cancel_goal()
        except:
            print("Could not stop robot")