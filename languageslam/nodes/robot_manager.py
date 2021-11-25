#!/usr/bin/env python

import rospy
import subprocess
import signal

from languageslam.srv import toggleexploration, toggleexplorationResponse

class robot:
    def __init__(self):
        self.explore=False
        self.service=rospy.Service('toogleexploration',toggleexploration,self.handle_exploration_srv)
    

    def handle_exploration_srv(self,req):
        if req.state: self.startExplore()
        else: self.killExplore()

        return 1 #this should be a verdict if the launch/kill was successfull
    

    def startExplore(self):
        # TODO add explore launchfile
        if not self.explore:
            self.child = subprocess.Popen(["roslaunch","languageslam","explore.launch"])
            print("parent process")
            print(self.child.poll())

            rospy.loginfo('The PID of child: %d', self.child.pid)
            print ("The PID of child:", self.child.pid)
            self.explore=True

    def killExplore(self):
        if self.explore:
            self.child.send_signal(signal.SIGINT) #You may also use .terminate() method
            self.explore=False

#for more: https://docs.python.org/2/library/subprocess.html
if __name__=="__main__":
    rospy.init_node('robot_manager')
    bot=robot(7)
    rospy.spin()
