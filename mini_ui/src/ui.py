#!/usr/bin/env python3

import subprocess
import sys

import rospy
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWebEngineWidgets import *
from PyQt5.QtWebKitWidgets import *
from PyQt5.QtWidgets import *


import glob
import re


def setup_camera_urls():
    """ return the url for viewing compressed image stream over http
    assumes compressed image stream, and jetbot_camera
    
    """

    published_topics = rospy.get_published_topics()
    published_topic_names = [x[0] for x in published_topics]
    published_topic_types = [x[1] for x in published_topics]
    #\/jetbot_camera\/.*
    r = re.compile(".*\/compressed$")
    compressed_camera_topics = list(filter(r.match, published_topic_names))

    # TODO: make more dynamic
    # TODO: probably need to sort the list

    cam_urls = []
    for cam_topic in compressed_camera_topics:

        cam_topic = "/".join(cam_topic.split("/")[:-1])
        cam_url = f"http://localhost:8080/stream?topic={cam_topic}&type=ros_compressed&width=360&height=480"

        cam_urls.append(cam_url)

    return cam_urls



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

        self.auto_mode_button = QPushButton("Manual Mode", self)
        self.start_button = QPushButton("Start System", self)
        self.record_button = QPushButton("Record Data", self)


        self.start_button.setStyleSheet("background-color: green")
        self.record_button.setStyleSheet("background-color: red")
        self.auto_mode_button.setStyleSheet("background-color: orange")



        # setup camera buttons dynamically
        # based on the number of compressed streams detected
        self.cam_urls = setup_camera_urls()
        print(self.cam_urls)

        camera_button_layout = QHBoxLayout()
        camera_button_layout.addStretch(1)
        
        for i, cam_url in enumerate(self.cam_urls):

            cam_button = QPushButton(f"Camera {i}", self)
            cam_button.clicked.connect(self.camera_button_clicked)
            camera_button_layout.addWidget(cam_button)


        action_button_layout = QHBoxLayout()
        action_button_layout.addStretch(1)
        action_button_layout.addWidget(self.start_button)
        action_button_layout.addWidget(self.record_button)
        action_button_layout.addWidget(self.auto_mode_button)
        


        self.image_view = ImageView(self)

        vbox = QVBoxLayout()
        vbox.addStretch(1)
        vbox.addLayout(camera_button_layout)
        vbox.addLayout(action_button_layout)

        self.setLayout(vbox)



        self.start_button.clicked.connect(self.start_button_clicked)
        self.record_button.clicked.connect(self.record_button_clicked)
        self.auto_mode_button.clicked.connect(self.auto_mode_button_clicked)

        # self.statusBar()

        # self.setGeometry(300, 500, 450, 350)
        self.setWindowTitle('Mini UI')
        self.setWindowIcon(QIcon('logo.png'))
        self.show()

    def camera_button_clicked(self):
        sender = self.sender()
        # self.statusBar().showMessage(sender.text() + ' was pressed')

        rospy.loginfo(sender.text() + " button was pressed")

        if sender.text() == "Camera 0":
            self.image_view.browser = QWebEngineView()
            self.image_view.browser.setUrl(QUrl(self.cam_urls[0]))

        
        if sender.text() == "Camera 1":
            self.image_view.browser = QWebEngineView()
            self.image_view.browser.setUrl(QUrl(self.cam_urls[1]))
        
        if sender.text() == "Camera 2":
            self.image_view.browser = QWebEngineView()
            self.image_view.browser.setUrl(QUrl(self.cam_urls[2]))
        
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
        self.auto_mode_button.setText("Autonomous Mode")



# TODO: create a main window properly
# TODO: jetson stats
# TODO: manager info
# TODO: rosrun web_video_server web_video_server
# TODO: make buttons bigger
# TODO: make buttons do stuff.
# TODO: close on main ui button close


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
