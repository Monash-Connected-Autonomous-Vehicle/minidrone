#!/usr/bin/env python3

import glob
import re
import yaml
import sys

import roslaunch
import rospy
import rosnode
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWebEngineWidgets import *
from PyQt5.QtWebKitWidgets import *
from PyQt5.QtWidgets import *
from std_msgs.msg import Bool

from gps import GPSReader

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

        self.config = yaml.safe_load(open("/home/jetson03/mcav/catkin_ws/src/minidrone/mini_ui/src/config.yaml"))

        self.init_ui()

        self.record_pub = rospy.Publisher("/recorder/recording", Bool, queue_size=1)

    def init_ui(self):
        

        # background colour
        p = self.palette()
        p.setColor(self.backgroundRole(), Qt.black)
        self.setPalette(p)


        # setup action buttons
        self.setup_action_buttons()

        # refreshable parts of the ui
        self.setup_ui_layout()

        self.image_view = ImageView(self)

        self.setGeometry(300, 300, 1000, 700)
        self.setWindowTitle('Mini UI')
        self.setWindowIcon(QIcon(self.config["data"]["logo"]))
        self.show()

    def setup_ui_layout(self):

        self.setup_camera_buttons()
        self.setup_node_buttons()
        self.setup_map_view()

        self.action_button_layout = QHBoxLayout()
        self.action_button_layout.addStretch(0)
        self.action_button_layout.addWidget(self.start_button)
        self.action_button_layout.addWidget(self.record_button)
        self.action_button_layout.addWidget(self.auto_mode_button)

        vbox = QVBoxLayout()
        vbox.addStretch(0)
        vbox.addLayout(self.map_view_layout)
        # vbox.addLayout(self.node_button_layout)
        vbox.addLayout(self.camera_button_layout)
        vbox.addLayout(self.action_button_layout)

        main_hbox = QHBoxLayout()
        main_hbox.addLayout(self.node_button_layout)
        main_hbox.addLayout(vbox)


        self.main_widget = QWidget()
        self.main_widget.setLayout(main_hbox)
        self.setCentralWidget(self.main_widget)
    
    def setup_node_buttons(self):

        # TODO: self, config this
        requred_nodes = ["jetbot_camera_0", "jetbot_camera_1", "web_video_server", "mini_ui", "recorder", "joystick_twist_node", "joy_node"]
        self.node_button_layout = QVBoxLayout()
        self.node_button_layout.addStretch(0)
        


        for i, node_name in enumerate(requred_nodes):
            
            node_button = QPushButton(f"{node_name}", self)
            node_button.clicked.connect(self.node_button_clicked)
            node_button.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding)
            self.node_button_layout.addWidget(node_button)
            if "/" + node_name in rosnode.get_node_names():
                node_button.setStyleSheet("background-color: green")
            else: 
                node_button.setStyleSheet("background-color: red")
            # name = node_name.split("/")[-1]
            


        # TODO: change so it is looking for a select group of nodes in get_node_names
        # TODO: make this a grid?
        # TODO: this view is too big


 
    
    def setup_action_buttons(self):

        self.auto_mode_button = QPushButton("Manual Mode", self)
        self.start_button = QPushButton("Start System", self)
        self.record_button = QPushButton("Record Data", self)

        self.start_button.setStyleSheet("background-color: green")
        self.record_button.setStyleSheet("background-color: red")
        self.auto_mode_button.setStyleSheet("background-color: orange")

        self.start_button.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding)
        self.record_button.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding)
        self.auto_mode_button.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding)

        self.start_button.clicked.connect(self.start_button_clicked)
        self.record_button.clicked.connect(self.record_button_clicked)
        self.auto_mode_button.clicked.connect(self.auto_mode_button_clicked)
    
    def setup_map_view(self):
        # map view (not working)
        # self.gps_reader = GPSReader()
        # self.fig = self.gps_reader.fig2
        self.map_view = QWebEngineView()
        # self.map_view.setUrl(QUrl("https://www.google.com.au/maps/@-37.9105836,145.133697,16.71z")) # just for testing view
        # self.map_view.setHtml(self.fig.to_html(include_plotlyjs="cdn"))
        self.map_view_layout = QVBoxLayout()
        self.map_view_layout.addWidget(self.map_view)
        # pass

    def setup_camera_buttons(self):
        # setup camera buttons dynamically
        # based on the number of compressed streams detected
        self.cam_urls = setup_camera_urls()
        # print(self.cam_urls)

        self.camera_button_layout = QHBoxLayout()
        self.camera_button_layout.addStretch(0)
        
        for i, cam_url in enumerate(self.cam_urls):

            cam_button = QPushButton(f"Camera {i}", self)
            cam_button.clicked.connect(self.camera_button_clicked)
            cam_button.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding)
            self.camera_button_layout.addWidget(cam_button)

    def camera_button_clicked(self):
        sender = self.sender()
        self.statusBar().showMessage(sender.text() + ' was pressed')

        rospy.loginfo(sender.text() + " button was pressed")

        idx = int(sender.text()[-1])
        self.image_view.browser = QWebEngineView()
        self.image_view.browser.setUrl(QUrl(self.cam_urls[idx]))
        self.image_view.setCentralWidget(self.image_view.browser)
        self.image_view.setWindowTitle(sender.text())
        self.image_view.show()


    def start_button_clicked(self):

        self.statusBar().showMessage(self.sender().text() + ' was pressed')
        rospy.loginfo(self.sender().text() + " button was pressed")

        if self.sender().text() == "Start System":

            self.start_button.setText("Stop System")
            self.launched_nodes = []
            
            for node in self.config["nodes"]:
                
                rospy.loginfo("Launching %s", node)
                launch_file = self.config["nodes"][node]["launch"]

                # roslaunch
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(uuid)
                self.launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
                self.launch.start()

                self.launched_nodes.append(self.launch)

            rospy.loginfo("roslaunch successful. ")

            # TODO: refresh camera buttons here
            import time 
            time.sleep(5) # do this better. need to wait for nodes to actuaally launch before refreshing
            self.setup_ui_layout()

            # http://wiki.ros.org/roslaunch/API%20Usage
        else:
            for launch_file in self.launched_nodes:
                launch_file.shutdown()
            self.start_button.setText("Start System")

    def node_button_clicked(self):
        self.statusBar().showMessage(self.sender().text() + ' was pressed')
        rospy.loginfo(self.sender().text() + " button was pressed")

    def record_button_clicked(self):

        self.statusBar().showMessage(self.sender().text() + ' was pressed')
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
        self.statusBar().showMessage(self.sender().text() + ' was pressed')
        rospy.loginfo(self.sender().text() + " button was pressed")
        print(rosnode.get_node_names())
        if self.sender().text() == "Manual Mode":
            self.auto_mode_button.setText("Autonomous Mode")
        else:
            self.auto_mode_button.setText("Manual Mode")
        
        # refresh the ui
        self.setup_ui_layout()



# TODO: create a main window properly
# TODO: jetson stats
# TODO: manager info
# TODO: make buttons bigger
# TODO: make buttons do stuff.
# TODO: close on main ui button close
# TODO: change so statusbar text is visible


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
