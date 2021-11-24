#!/usr/bin/env python

import rospy
import subprocess
import signal

from languageslam.srv import toggleexploration, toggleexplorationResponse

# TODO create callback functions on service
class robot:
    def __init__(self,robotno):
        self.explore=False
        self.robotno=robotno
        self.address='192.168.0.11'+str(robotno)
        self.robotname='robot'+str(robotno)
        

        self.service=rospy.Service('toogleexploration',toggleexploration,self.handle_exploration_srv)
    

    def handle_exploration_srv(self,req):
        if req.state: self.startExplore
        else: self.killExplore

        return True #this should be a verdict if the launch/kill was successfull
    # TODO add callback function with service

    def startExplore(self):
        # TODO add explore launchfile
        if not self.explore:
            self.child = subprocess.Popen(["roslaunch","turtlebot3_gazebo","turtlebot3_house.launch"])
            print("parent process")
            print(self.child.poll())

            rospy.loginfo('The PID of child: %d', self.child.pid)
            print ("The PID of child:", self.child.pid)
            self.explore=True

    def killExplore(self):
        if self.explore:
            self.child.send_signal(signal.SIGINT) #You may also use .terminate() method

#for more: https://docs.python.org/2/library/subprocess.html
if __name__=="__main__":
    rospy.init_node('robot_manager')
    bot=robot(7)
    rospy.spin()

'''
import roslaunch
import rospy
from dynamic_reconfigure.server import Server
from languageslam.cfg import managerConfig

class robot:
    def __init__(self,uuid,robotno):
        self.active=False
        self.explore=False
        self.robotno=robotno
        self.address='192.168.0.11'+str(robotno)
        self.robotname='robot'+str(robotno)

        #generating object for robots.launch
        self.cli_args = ['languageslam', 'robots.launch', 'robot_name:='+self.robotname, 'robot_ip:='+self.address, 'robot_port:='+':50051']
        self.launch_file = roslaunch.rlutil.resolve_launch_arguments(self.cli_args)[0]
        self.args = self.cli_args[2:]
        self.launch=roslaunch.parent.ROSLaunchParent(uuid, [(self.launch_file,self.args)])
        #TODO make and check namespace and machine tag in robots.launch autoadjusting moving the machine assignement to the launchfile makes this way easier

    def compareactive(self,value):
        print(self.address)
        if self.active != value:
            if value:
                self.__initrobot()
            else:
                self.__killrobot()

    def compareexplore(self,value):
        if self.explore != value and self.active:
            if value:
                self.initexplore()
            else:
                self.killexplore()


    #private methods These should never be called directly because I have no idea what happens if you kill a dead node or launch a live one
    def __initrobot(self):
        try:
            
            self.launch.start()
        except Exception as e:
            print("Init of " + str(self.robotname) + " failed because "+ str(e))
            self.active=False
        else:
            self.active=True
    def __killrobot(self):
        try:
            self.launch.shutdown()
        except Exception as e:
            print("Shutdown of " + str(self.robotname) + " failed because "+ str(e)) 
            self.active=True
        else:
            self.active=False

    # def __initexplore(self):
    #     placeholder
    # def __killexplore(self):
    #     placeholder



class robot_manager:
    def __init__(self):
        #init list of robots
        #adding 8 robots although most of them wont be initialized ever but it keeps it cleaner when calling the elements of the list
        
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        #roslaunch.configure_logging(uuid)
        
        self.robots=[robot(uuid,i+1) for i in range(8)]
        self.reconfigure=Server(managerConfig, self.callback)

    def callback(self, config, level):
        #check the activity of the robots
        
        self.robots[0].compareactive(config.robot1)
        self.robots[1].compareactive(config.robot2)
        self.robots[2].compareactive(config.robot3)
        self.robots[3].compareactive(config.robot4)
        self.robots[4].compareactive(config.robot5)
        self.robots[5].compareactive(config.robot6)
        self.robots[6].compareactive(config.robot7)
        self.robots[7].compareactive(config.robot8)
        return config

    

if __name__=="__main__":

    rospy.init_node('robot_manager', anonymous=False, disable_signals=True)
    robotmanager=robot_manager()
    rospy.spin()
'''