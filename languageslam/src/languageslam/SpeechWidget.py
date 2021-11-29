import sys
from PyQt5.QtWidgets import (QApplication, QGridLayout, QWidget,
 QPushButton, QCheckBox, QLabel)
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import pyqtSlot

## Using python==2.7 with PyAudio
import speech_recognition as sr
from nltk.tokenize import word_tokenize
from nltk.stem.porter import PorterStemmer

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

from languageslam.srv import *

class App(QWidget):

    def __init__(self):
        # Setup of the QT widget
        super(App, self).__init__()
        self.setWindowTitle('Voice Control')
        
        self.check_robot7 = QCheckBox("Robot 7", self)
        self.check_robot8 = QCheckBox("Robot 8", self)
        self.check_robot7.setToolTip('Control Robot 7')
        self.check_robot8.setToolTip('Control Robot 8')
        self.label = QLabel("")
        self.label2 = QLabel("")
        self.commands = [ "stop", "move", "explore", "go"]
        self.movements = ["left", "right", "forward", "backward"]
        self.r = sr.Recognizer()
        self.stemmer = PorterStemmer()

        self.button = QPushButton('Record command', self)
        self.button.setToolTip('Record command for selected robots')
        self.button.clicked.connect(self.on_click)

        layout = QGridLayout(self)
        layout.addWidget(QLabel("Choose robots to control"), 0, 0)
        layout.addWidget(self.check_robot7, 1, 0)
        layout.addWidget(self.check_robot8, 1, 1)
        layout.addWidget(self.button, 2, 0)
        layout.addWidget(self.label, 3, 0)
        layout.addWidget(self.label2, 4, 0)
        #rospy.init_node('SpeechCommand')
        #self.client7 = actionlib.SimpleActionClient('/robot7/move_base',MoveBaseAction)#for now commented out
        
        #wait for action client server
        
        self.show()

    def exploration_client(self,robotname,state):
        rospy.wait_for_service('/'+robotname + '/toggleexploration')
        try:
            toggleexploration = rospy.ServiceProxy('/'+robotname + '/toggleexploration', toggleexploration)
            resp1 = toggleexploration(state)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def moverobot(self, robotno, move):
        
        if move == 'left':
            self.sendGoal(robotno, 0, 1, 1.5708)
        elif move == 'right':
            self.sendGoal(robotno, 0, -1, -1.5708)
        elif move == 'forward':
            self.sendGoal(robotno, 1, 0, 0)
        elif move == 'backward':
            self.sendGoal(robotno, -1, 0, -3.1415)

    def sendGoal(self, robotno, x, y, theta):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "robot" + str(robotno) + "/base_footprint"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        # RPY to quaternion convert: 90deg, 0, -90deg
        orientation = quaternion_from_euler(0, 0, theta)
        goal.target_pose.pose.orientation.x = orientation[0]
        goal.target_pose.pose.orientation.y = orientation[1]
        goal.target_pose.pose.orientation.z = orientation[2]
        goal.target_pose.pose.orientation.w = orientation[3]
        
        print(str(goal))
        if robotno == 7:
            try:
                client7 = actionlib.SimpleActionClient('/robot7/move_base', MoveBaseAction)
                print("Client started")
                client7.wait_for_server(rospy.Duration(10))
            except:
                print("Could not wait for sever")

            try:
                client7.send_goal(goal)
            except:
                print("Could not send goal")
        elif robotno == 8:
            try:
                client8 = actionlib.SimpleActionClient('/robot8/move_base', MoveBaseAction)
                print("Client started")
                client8.wait_for_server(rospy.Duration(10))
            except:
                print("Could not wait for sever")

            try:
                client8.send_goal(goal)
            except:
                print("Could not send goal")

        '''wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return client.get_result()'''
    @pyqtSlot()
    def on_click(self):
        text = ''
        with sr.Microphone() as source:
            # read the audio data from the default microphone'
            audio_data = self.r.listen(source)

            try:
                text = self.r.recognize_google(audio_data)
                text = str(text)
                print(text)
            except:
                print("No response from Google")
        
        if text != '':
            # if any words are recognised convert and extract commands
            text = str.lower(text)
            token = word_tokenize(text)
            stems = []
            for w in token:
                stems.append(w)

            command = set(self.commands).intersection(stems)
        
        else:
            self.label.setText("No commands recognised. Please try again.")
            return

        if len(command) != 1:
            self.label.setText("Command not recognised. Please try again.")

        else:
            # TODO add navigation message
            command = list(command)
            output = "Command not recognised. Please try again."
            if command[0] == "stop":
                output = "Command recognised: Stopping"
            elif command[0] == "move":

                try:
                    move = set(self.movements).intersection(stems)
                    move = str(list(move)[0])
                    print(move)
                except:
                    move = None
                    print("No move")
                if len(command) != 1:
                    output = "Command not recognised. Please try again."
                else:
                    if move != None:
                        output = "Command recognised: Moving " + move
                        if self.check_robot7.isChecked() and self.check_robot8.isChecked():
                            self.label2.setText("Sending command to robot 7 and robot 8")
                            self.moverobot(7, move)
                            self.moverobot(8, move)
                        elif self.check_robot7.isChecked():
                            self.label2.setText("Sending command to robot 7")
                            self.moverobot(7, move)
                        elif self.check_robot8.isChecked():
                            self.label2.setText("Sending command to robot 8")
                            self.moverobot(8, move)
                        else:
                            self.label2.setText("No robot selected. Command not sent")
                    else:
                         output = "Command not recognised: Missing direction"
                         print("No direction")

            
            elif command[0] == "explore":
                if self.check_robot7.isChecked():
                    self.label2.setText("Starting exploration on robot 7")
                    self.exploration_client('robot7', 1)
                elif self.check_robot8.isChecked():
                    self.label2.setText("Starting exploration on robot 8")
                    self.exploration_client('robot7', 1)
                #TODO add function to launch exploration
                output = "Command recognised. Exploring"

            elif command[0] == "go":
                #TODO add function to choose posiiton
                position = "position placeholder"
                output = "Command recognised. Going to " + position


            self.label.setText(output)
            
if __name__ == '__main__':
    
    app = QApplication(sys.argv)
    
    ex = App()
    sys.exit(app.exec_())
