#!/usr/bin/env python


import roslaunch
import rospy
from dynamic_reconfigure.server import Server
from languageslam.cfg import managerConfig

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

class robot:
    def __init__(self,robotno):
        
        self.active=False
        self.explore=False
        self.robotno=robotno
        self.address='192.168.0.11'+str(robotno)
        self.robotname='robot'+str(robotno)
        self.config=roslaunch.config.ROSLaunchConfig()
        self.__initrobot()
        '''TODO make and check namespace and machine tag in robots.launch autoadjusting moving the machine assignement to the launchfile makes this way easier'''
        self.machine=roslaunch.core.Machine(name=self.robotname,address=self.address,env_loader="/home/ubuntu/languageslam_ws/devel/env.sh", user="ubuntu", password="SwarmBot")
        print(self.machine)
        self.config.add_machine(self.machine)
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
                self.__initexplore()
            else:
                self.__killexplore()

    def move(self,move):
        if move == 'left':
            self.__sendGoal( 0, 1, 1.5708)
        elif move == 'right':
            self.__sendGoal( 0, -1, -1.5708)
        elif move == 'forward':
            self.__sendGoal( 1, 0, 0)
        elif move == 'backward':
            self.__sendGoal( -1, 0, -3.1415)

    #private methods These should never be called directly because I have no idea what happens if you kill a dead node or launch a live one
    def __initrobot(self):
        try:
            self.cli_args = ['languageslam', 'robots.launch', 'robot_name:='+self.robotname, 'robot_ip:='+self.address, 'robot_port:='+':50051']
            self.launch_file = roslaunch.rlutil.resolve_launch_arguments(self.cli_args)[0]
            self.args = self.cli_args[2:]
            self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            self.launch = roslaunch.scriptapi.ROSLaunch()
            self.launch.parent=roslaunch.parent.ROSLaunchParent(self.uuid, [(self.launch_file,self.args)])
            self.launch.start()
        except Exception as e:
            print("Init of " + str(self.robotname) + " failed because "+ str(e))
            self.active=False
        else:
            self.active=True
    def __killrobot(self):
        try:
            self.launch.parent.stop()
        except Exception as e:
            print("Shutdown of " + str(self.robotname) + " failed because "+ str(e)) 
            self.active=True
        else:
            self.active=False

    def __initexplore(self):
        try:
            self.exp_args = "{} {} {} {} {} {} {} {}".format(   "base_link", 
                                                                "/"+self.robotname+"/map",
                                                                "/"+self.robotname+"/map_updates",
                                                                "true", 
                                                                "0.1",
                                                                "30", 
                                                                "0.0",
                                                                "1.0",
                                                                "0.3",
                                                                "0.75")
            self.node=roslaunch.core.Node(package="explore_lite",node_type="explore",name="explore", namespace="/"+self.robotname ,machine_name=self.robotname, args=self.exp_args)
            self.launch.launch(self.node)
        except Exception as e:
            print("Init of exploration of  " + str(self.robotname) + " failed because "+ str(e))
            self.explore=False
        else:
            self.explore=True
    def __killexplore(self):
        try:
            self.explaunch.shutdown()
        except Exception as e:
            print("Shutdown of exploration of " + str(self.robotname) + " failed because "+ str(e)) 
            self.explore=True
        else:
            self.explore=False

    def __sendGoal(self, x, y, theta):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "robot" + str(self.robotno) + "/base_footprint"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        # RPY to quaternion convert: 90deg, 0, -90deg
        orientation = quaternion_from_euler(0, 0, theta)
        goal.target_pose.pose.orientation.x = orientation[0]
        goal.target_pose.pose.orientation.y = orientation[1]
        goal.target_pose.pose.orientation.z = orientation[2]
        goal.target_pose.pose.orientation.w = orientation[3]
        
        try:
            client = actionlib.SimpleActionClient('/robot8/move_base', MoveBaseAction)
            print("Client started")
            client.wait_for_server(rospy.Duration(10))
        except:
            print("Could not wait for sever")

        try:
            client.send_goal(goal)
        except:
            print("Could not send goal")


class robot_manager:
    def __init__(self):
        #init list of robots
        #adding 8 robots although most of them wont be initialized ever but it keeps it cleaner when calling the elements of the list
        
        self.robots=[robot(i+1) for i in range(8)]
        self.reconfigure=Server(managerConfig, self.callback)

    def callback(self, config, level):
        #To activate manually via rqt_reconfigure
        #mainly for testing
        self.robots[0].compareactive(config.robot1_active)
        self.robots[1].compareactive(config.robot2_active)
        self.robots[2].compareactive(config.robot3_active)
        self.robots[3].compareactive(config.robot4_active)
        self.robots[4].compareactive(config.robot5_active)
        self.robots[5].compareactive(config.robot6_active)
        self.robots[6].compareactive(config.robot7_active)
        self.robots[7].compareactive(config.robot8_active)


        self.robots[0].compareexplore(config.robot1_explore)
        self.robots[1].compareexplore(config.robot2_explore)
        self.robots[2].compareexplore(config.robot3_explore)
        self.robots[3].compareexplore(config.robot4_explore)
        self.robots[4].compareexplore(config.robot5_explore)
        self.robots[5].compareexplore(config.robot6_explore)
        self.robots[6].compareexplore(config.robot7_explore)
        self.robots[7].compareexplore(config.robot8_explore)
        return config
    
    def activate(self, robotno,state):
        self.robots[robotno-1].compareactive(state)
    
    def explore(self, robotno,state):
        self.robots[robotno-1].compareexplore(state)

    def move(self,robotno,move):
        '''TODO: Do we want to disable the exploration for this?'''
        self.robots[robotno-1].move(move)


if __name__=="__main__":
    rospy.init_node("robot_manager",anonymous=True,disable_signals=False)



    manager=robot_manager()
    rospy.spin()