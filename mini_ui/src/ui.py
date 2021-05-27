#!/usr/bin/env python3

import subprocess
import sys

import rospy
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWebEngineWidgets import *
from PyQt5.QtWebKitWidgets import *
from PyQt5.QtWidgets import *


class ImageView(QMainWindow):
    def __init__(self, parent=None):
        super(ImageView, self).__init__(parent)


class Example(QWidget):

    def __init__(self):
        super().__init__()

        self.initUI()

    def initUI(self):
        

        # background colour
        p = self.palette()
        p.setColor(self.backgroundRole(), Qt.black)
        self.setPalette(p)

        self.auto_mode_button = QPushButton("Autonomous Mode", self)
        self.start_button = QPushButton("Start System", self)
        self.record_button = QPushButton("Record Data", self)
        cam_0_button = QPushButton("Camera 0", self)
        cam_1_button = QPushButton("Camera 1", self)
        

        self.start_button.setStyleSheet("background-color: green")
        self.record_button.setStyleSheet("background-color: red")
        self.auto_mode_button.setStyleSheet("background-color: orange")



        self.cam_0_url = "http://localhost:8080/stream?topic=/jetbot_camera/0&type=ros_compressed&width=360&height=480"
        self.cam_1_url = "http://localhost:8080/stream?topic=/jetbot_camera/1&type=ros_compressed&width=360&height=480"


        hbox = QHBoxLayout()
        hbox.addStretch(1)
        hbox.addWidget(self.start_button)
        hbox.addWidget(self.record_button)
        hbox.addWidget(self.auto_mode_button)
        
        hbox2 = QHBoxLayout()
        hbox2.addStretch(1)
        hbox2.addWidget(cam_0_button)
        hbox2.addWidget(cam_1_button)

        self.image_view = ImageView(self)

        vbox = QVBoxLayout()
        vbox.addStretch(1)
        # vbox.addWidget(self.image_view)
        vbox.addLayout(hbox2)
        vbox.addLayout(hbox)

        self.setLayout(vbox)

        cam_0_button.clicked.connect(self.buttonClicked)
        cam_1_button.clicked.connect(self.buttonClicked)

        self.start_button.clicked.connect(self.start_button_clicked)
        self.record_button.clicked.connect(self.record_button_clicked)
        self.auto_mode_button.clicked.connect(self.auto_mode_button_clicked)

        # self.statusBar()

        self.setGeometry(300, 500, 450, 350)
        self.setWindowTitle('Mini UI')
        self.setWindowIcon(QIcon('white_logo.png'))
        self.show()

    def buttonClicked(self):
        sender = self.sender()
        # self.statusBar().showMessage(sender.text() + ' was pressed')

        rospy.loginfo(sender.text() + " button was pressed")

        if sender.text() == "Camera 0":
            self.image_view.browser = QWebEngineView()
            self.image_view.browser.setUrl(QUrl(self.cam_0_url))
            self.image_view.setCentralWidget(self.image_view.browser)
            self.image_view.setWindowTitle(sender.text())
            self.image_view.show()
        
        if sender.text() == "Camera 1":
            self.image_view.browser = QWebEngineView()
            self.image_view.browser.setUrl(QUrl(self.cam_1_url))
            self.image_view.setCentralWidget(self.image_view.browser)
            self.image_view.setWindowTitle(sender.text())
            self.image_view.show()


    def start_button_clicked(self):
        rospy.loginfo(self.sender().text() + " button was pressed")
        # out = subprocess.Popen(["roslaunch",  "jetbot_ros",  "jetbot_cam_compressed_one.launch"], stdout=subprocess.PIPE)
        self.start_button.setText("Stop System")

    
    def record_button_clicked(self):
        rospy.loginfo(self.sender().text() + " button was pressed")
        self.record_button.setText("Stop Recording")
    
    def auto_mode_button_clicked(self):
        rospy.loginfo(self.sender().text() + " button was pressed")
        self.auto_mode_button.setText("Manual Mode")



# TODO: create a main window properly
# TODO: jetson stats
# TODO: manager info
# TODO: 

def main():
    app = QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())


if __name__ == '__main__':

    rospy.init_node("ui_node")
    main()

# button press

# # self.setWindowIcon(QIcon('white_logo.png'))
#         okButton = QPushButton("OK")
#         cancelButton = QPushButton("Cancel")

#         okButton.clicked.connect(self.ok_button_pushed)
# # 
#ref
# https://zetcode.com/gui/pyqt5/eventssignals/



# respond to key press
    # def keyPressEvent(self, e):
    #     if e.key() == Qt.Key_Escape:
    #         self.close()

    #     if e.key() == 82:   # if r is pressed?
    #         # print("RECORDING DATA")

    #         self.record = not self.record
    #         print("Recording Data: ", self.record)


# sender of button

    # def initUI(self):
    #     btn1 = QPushButton("Button 1", self)
    #     btn1.move(30, 50)

    #     btn2 = QPushButton("Button 2", self)
    #     btn2.move(150, 50)

    #     btn1.clicked.connect(self.buttonClicked)
    #     btn2.clicked.connect(self.buttonClicked)

    #     self.statusBar()

    #     self.setGeometry(300, 300, 450, 350)
    #     self.setWindowTitle('Event sender')
    #     self.show()

    # def buttonClicked(self):
    #     sender = self.sender()
    #     self.statusBar().showMessage(sender.text() + ' was pressed')


# emit signals
# file dialog?
# toggle buttons 
# qslider
# qsplitter


# https://github.com/Geekgineer/ros_web_gui/blob/master/README.md

# https://github.com/chrisspen/homebot/blob/master/src/ros/src/ros_homebot_teleop/nodes/mjpeg_streamer.py
