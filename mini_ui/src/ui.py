#!/usr/bin/env python3

import glob
import re
import subprocess
import sys

import roslaunch
import rospy
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWebEngineWidgets import *
from PyQt5.QtWebKitWidgets import *
from PyQt5.QtWidgets import *
from std_msgs.msg import Bool

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

class MainInterface(QMainWindow):

    def __init__(self):
        super().__init__()

        self.initUI()

        self.record_pub = rospy.Publisher("/recorder/recording", Bool, queue_size=1)

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


        # def setup_ui():
        self.setup_camera_buttons()


        self.action_button_layout = QHBoxLayout()
        self.action_button_layout.addStretch(1)
        self.action_button_layout.addWidget(self.start_button)
        self.action_button_layout.addWidget(self.record_button)
        self.action_button_layout.addWidget(self.auto_mode_button)
        
        # load map? not ready
        # from gps import GPSReader
        # self.gps_reader = GPSReader()
        # self.fig = self.gps_reader.fig2

        self.map_view = QWebEngineView()

        # # # self.map_view.setUrl(QUrl("https://www.google.com"))
        # # self.map_view.setHtml(self.fig.to_html(include_plotlyjs="cdn"))
        self.map_view_layout = QVBoxLayout()
        self.map_view_layout.addWidget(self.map_view)

        vbox = QVBoxLayout()
        vbox.addStretch(1)
        vbox.addLayout(self.map_view_layout)
        vbox.addLayout(self.camera_button_layout)
        vbox.addLayout(self.action_button_layout)

        self.main_widget = QWidget()
        self.main_widget.setLayout(vbox)
        self.setCentralWidget(self.main_widget)

        
        self.start_button.clicked.connect(self.start_button_clicked)
        self.record_button.clicked.connect(self.record_button_clicked)
        self.auto_mode_button.clicked.connect(self.auto_mode_button_clicked)

        self.image_view = ImageView(self)


        self.setGeometry(300, 300, 1000, 700)
        self.setWindowTitle('Mini UI')
        self.setWindowIcon(QIcon('/home/jetson03/mcav/catkin_ws/src/minidrone/mini_ui/src/logo.png'))
        self.show()


    def setup_camera_buttons(self):
        # setup camera buttons dynamically
        # based on the number of compressed streams detected
        self.cam_urls = setup_camera_urls()
        print(self.cam_urls)

        self.camera_button_layout = QHBoxLayout()
        self.camera_button_layout.addStretch(1)
        
        for i, cam_url in enumerate(self.cam_urls):

            cam_button = QPushButton(f"Camera {i}", self)
            cam_button.clicked.connect(self.camera_button_clicked)
            self.camera_button_layout.addWidget(cam_button)

    def camera_button_clicked(self):
        sender = self.sender()
        # self.statusBar().showMessage(sender.text() + ' was pressed')

        rospy.loginfo(sender.text() + " button was pressed")


        idx = int(sender.text()[-1])
        self.image_view.browser = QWebEngineView()
        self.image_view.browser.setUrl(QUrl(self.cam_urls[idx]))
        self.image_view.setCentralWidget(self.image_view.browser)
        self.image_view.setWindowTitle(sender.text())
        self.image_view.show()


    def start_button_clicked(self):
        rospy.loginfo(self.sender().text() + " button was pressed")

        if self.sender().text() == "Start System":

            self.start_button.setText("Stop System")
            
            # roslaunch
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launch_file = "/home/jetson03/mcav/catkin_ws/src/minidrone/mini_ui/launch/mini_ui_cams.launch"
            self.launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
            self.launch.start()
            rospy.loginfo("roslaunch successful. ")

            # TODO: refresh camera buttons here
            # http://wiki.ros.org/roslaunch/API%20Usage
        else:
            self.launch.shutdown()
            self.start_button.setText("Start System")




    def record_button_clicked(self):
        rospy.loginfo(self.sender().text() + " button was pressed")

        record_msg = Bool()
        # toggle between recording states
        if self.sender().text() == "Record Data":
            
            # set the recording msg
            self.record_button.setText("Stop Recording")
            record_msg.data = True
            
        else:
            # set the stop recording msg
            self.record_button.setText("Record Data")
            record_msg.data = False

        # publish the recording message        
        self.record_pub.publish(record_msg)
        print("Recording Status: ", record_msg.data)

    def auto_mode_button_clicked(self):
        rospy.loginfo(self.sender().text() + " button was pressed")
        self.auto_mode_button.setText("Autonomous Mode")
        print(rospy.get_published_topics())



# TODO: create a main window properly
# TODO: jetson stats
# TODO: manager info
# TODO: make buttons bigger
# TODO: make buttons do stuff.
# TODO: close on main ui button close


def main():
    app = QApplication(sys.argv)
    main_ui = MainInterface()
    sys.exit(app.exec_())


if __name__ == '__main__':

    rospy.init_node("ui_node")
    main()



#ref
# https://zetcode.com/gui/pyqt5/eventssignals/


# emit signals
# file dialog?
# toggle buttons 
# qslider
# qsplitter


# https://github.com/Geekgineer/ros_web_gui/blob/master/README.md

# https://github.com/chrisspen/homebot/blob/master/src/ros/src/ros_homebot_teleop/nodes/mjpeg_streamer.py
