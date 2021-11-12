import sys
from PyQt5.QtWidgets import (QApplication, QGridLayout, QWidget,
 QPushButton, QCheckBox, QLabel)
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import pyqtSlot

## Using python==3.6 with PyAudio
import speech_recognition as sr
from nltk.tokenize import word_tokenize
from nltk.stem.porter import PorterStemmer

class App(QWidget):

    def __init__(self):
        # Setup of the QT widget
        super(App, self).__init__()
        self.setWindowTitle('Voice Control')
        
        self.check_robot1 = QCheckBox("Robot 1", self)
        self.check_robot2 = QCheckBox("Robot 2", self)
        self.check_robot1.setToolTip('Control Robot 1')
        self.check_robot2.setToolTip('Control Robot 2')
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
        layout.addWidget(QLabel("Choose robots to control"),0,0)
        layout.addWidget(self.check_robot1, 1, 0)
        layout.addWidget(self.check_robot2, 1, 1)
        layout.addWidget(self.button, 2, 0)
        layout.addWidget(self.label, 3, 0)
        layout.addWidget(self.label2, 4, 0)
        
        self.show()

    @pyqtSlot()
    def on_click(self):
        text = ''
        with sr.Microphone() as source:
            # read the audio data from the default microphone'
            audio_data = self.r.listen(source)

            try:
                text = self.r.recognize_google(audio_data)
            
            except:
                pass
        
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
                move = set(self.movements).intersection(stems)
                if len(command) != 1:
                    output = "Command not recognised. Please try again."
                else:
                    output = "Command recognised: Moving " + move
            
            elif command[0] == "explore":
                output = "Command recognised. Exploring"

            elif command[0] == "go":
                #TODO add function to choose posiiton
                position = "position placeholder"
                output = "Command recognised. Going to " + position


            self.label.setText(output)
            # TODO add commands to multiple robots
            if self.check_robot1.isChecked() and self.check_robot2.isChecked():
                self.label2.setText("Sending command to robot 1 and robot 2")
            elif self.check_robot1.isChecked():
                self.label2.setText("Sending command to robot 1")
            elif self.check_robot2.isChecked():
                self.label2.setText("Sending command to robot 2")
            else:
                self.label2.setText("No robot selected. Command not sent")


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = App()
    sys.exit(app.exec_())
