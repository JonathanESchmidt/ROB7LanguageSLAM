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
from tf.transformations import quaternion_from_euler

from .robotCommands import robot_command as rc
from languageslam.srv import toggleexploration

class App(QWidget):

    def __init__(self):
        # Setup of the QT widget
        super(App, self).__init__()
        self.setWindowTitle('Voice Control')

        # TODO add check for online robots robots
        self.robot7 = rc(7)
        self.robot8 = rc(8)
        
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

        self.show()

    
    @pyqtSlot()
    def on_click(self):
        if self.check_robot7.isChecked() or self.check_robot8.isChecked():
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

                            if self.check_robot7.isChecked():
                                self.label2.setText("Sending command to robot 7")
                                self.robot7.moverobot(move)

                            if self.check_robot8.isChecked():
                                self.label2.setText("Sending command to robot 8")
                                self.robot8.moverobot(move)

                        else:
                            output = "Command not recognised: Missing direction"
                            print("No direction")

                
                elif command[0] == "explore":
                    print("Went to exploration")

                    if self.check_robot7.isChecked():
                        self.label2.setText("Starting exploration on robot 7")
                        self.robot7.exploration_client(1)
                        print("Exploration 7 going")

                    if self.check_robot8.isChecked():
                        self.label2.setText("Starting exploration on robot 8")
                        self.robot8.exploration_client(1)
                        print("Exploration 8 going")
                    output = "Command recognised. Exploring"

                elif command[0] == "go":
                    #TODO add function to choose posiiton
                    position = "position placeholder"
                    output = "Command recognised. Going to " + position


                self.label.setText(output)
        else:
            self.label2.setText("No robot selected. Command not sent")
            
if __name__ == '__main__':
    
    app = QApplication(sys.argv)
    
    ex = App()
    sys.exit(app.exec_())
