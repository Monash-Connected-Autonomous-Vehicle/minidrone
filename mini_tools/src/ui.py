#!/usr/bin/env python3

import glob
import re
import yaml
import sys
import time 

import roslaunch
import rospy
import rosnode
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWebEngineWidgets import *
from PyQt5.QtWidgets import *
from std_msgs.msg import Bool

from gps import GPSReader

def get_camera_urls():
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

def launch_node(launch_file):
    # roslaunch
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
    launch.start()

    return launch

class ImageView(QMainWindow):
    def __init__(self, parent=None):
        super(ImageView, self).__init__(parent)

class MainInterface(QMainWindow):

    def __init__(self):
        super().__init__()

        config_file = sys.argv[1]
        self.config = yaml.safe_load(open(config_file))
        self.camera_button_pressed = False

        self.init_ui()

        self.record_pub = rospy.Publisher("/recorder/recording", Bool, queue_size=1)
        self.auto_pub = rospy.Publisher("/carla/hero/vehicle_control_manual_override", Bool, queue_size=1) #/carla/patrick/enable_autopilot

    def keyPressEvent(self, event):

        if event.key() == Qt.Key_Escape: 
            # Exit application on Esc press
            self.close()
        
        if event.key() == Qt.Key_R: 
            # Start / Stop Recording  on R button press
            self.record_button.click()
        

    def init_ui(self):
        
        
        # background colour
        p = self.palette()
        p.setColor(self.backgroundRole(), Qt.black)
        self.setPalette(p)

        self.timer=QTimer()
        self.timer.timeout.connect(self.setup_ui_layout)


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

        self.setup_main_view_buttons()
        self.setup_node_buttons()
        if self.camera_button_pressed:
            pass
        else:
            self.setup_map_view()

        self.setup_main_view()

        self.action_button_layout = QHBoxLayout()
        self.action_button_layout.addStretch(0)
        self.action_button_layout.addWidget(self.start_button)
        self.action_button_layout.addWidget(self.record_button)
        self.action_button_layout.addWidget(self.auto_mode_button)

        vbox = QVBoxLayout()
        vbox.addStretch(0)
        vbox.addLayout(self.main_view_layout)
        vbox.addLayout(self.main_view_button_layout)
        vbox.addLayout(self.action_button_layout)

        main_hbox = QHBoxLayout()
        main_hbox.addLayout(self.node_button_layout, 1)
        main_hbox.addLayout(vbox, 4)

        self.statusBar().setStyleSheet("color: white")
        self.main_widget = QWidget()
        self.main_widget.setLayout(main_hbox)
        self.setCentralWidget(self.main_widget)

        self.timer.stop()
    
    def setup_node_buttons(self):
        """ Setup the buttons displaying the status of required nodes"""
        requred_nodes = self.config["required_nodes"]
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

    
    def setup_action_buttons(self):
        """ Setup the buttons for performing actions"""
        self.auto_mode_button = QPushButton("Manual Mode", self)
        self.start_button = QPushButton("Start System", self)
        self.record_button = QPushButton("Record Data", self)

        self.start_button.setStyleSheet("background-color: green")
        self.record_button.setStyleSheet("background-color: red")
        self.auto_mode_button.setStyleSheet("background-color: gray")

        self.start_button.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding)
        self.record_button.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding)
        self.auto_mode_button.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding)

        self.start_button.clicked.connect(self.start_button_clicked)
        self.record_button.clicked.connect(self.record_button_clicked)
        self.auto_mode_button.clicked.connect(self.auto_mode_button_clicked)
    
    def setup_main_view(self):
        """ Setup the layout for the main view """
        self.main_view_layout = QVBoxLayout()
        self.main_view_layout.addWidget(self.main_view)

    def setup_map_view(self):
        """Setup map data and browser based map view"""
        # map view (not working)
        # self.gps_reader = GPSReader()
        # self.fig = self.gps_reader.fig2
        self.main_view = QWebEngineView()
        self.main_view.setUrl(QUrl("https://www.google.com.au/maps/@-37.9105836,145.133697,16.71z")) # just for testing view

        # self.main_view.setHtml(self.fig.to_html(include_plotlyjs="cdn"))


    def setup_main_view_buttons(self):
        """ Setup buttons for controlling the main view (map / cameras)"""
        self.main_view_button_layout = QHBoxLayout()
        self.main_view_button_layout.addStretch(0)


        # setup map view button
        map_button = QPushButton("Map View", self)
        map_button.clicked.connect(self.main_view_button_clicked)
        map_button.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding)
        self.main_view_button_layout.addWidget(map_button)
        
        # setup camera buttons dynamically based on detected streams
        self.cam_urls = get_camera_urls()

        for i, cam_url in enumerate(self.cam_urls):

            cam_button = QPushButton(f"Camera {i}", self)
            cam_button.clicked.connect(self.main_view_button_clicked)
            cam_button.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding)
            self.main_view_button_layout.addWidget(cam_button)

    def main_view_button_clicked(self):
        """ Functionality for when a main view button is clicked"""
        sender = self.sender()
        self.statusBar().showMessage(sender.text() + ' was pressed')
        rospy.loginfo(sender.text() + " button was pressed")

        if sender.text() == "Map View":
            self.camera_button_pressed = False
        else:
            idx = int(sender.text()[-1])
            self.main_view = QWebEngineView()
            self.main_view.setUrl(QUrl(self.cam_urls[idx]))
            self.camera_button_pressed = True

        self.setup_ui_layout()


    def start_button_clicked(self):
        """ Functionality for when the start/stop system button is pressed""" 
        self.statusBar().showMessage(self.sender().text() + ' was pressed')
        rospy.loginfo(self.sender().text() + " button was pressed")

        if self.sender().text() == "Start System":

            self.start_button.setText("Stop System")
            self.launched_nodes = []
            
            for node in self.config["nodes"]:
                
                rospy.loginfo("Launching %s", node)
                launch_file = self.config["nodes"][node]["launch"]

                launch = launch_node(launch_file)

                self.launched_nodes.append(launch)

            rospy.loginfo("roslaunch successful. ")

            time.sleep(3) # TODO: do this better. need to wait for nodes to actuaally launch before refreshing
                           


            self.timer.start(3000)
            # self.setup_ui_layout()

        else:
            for launch_file in self.launched_nodes:
                launch_file.shutdown()

            self.camera_button_pressed = False # reset to map view
            self.setup_ui_layout()
            self.start_button.setText("Start System")

    def node_button_clicked(self):
        """Functionality for when the node button is pressed"""
        self.statusBar().showMessage(self.sender().text() + ' was pressed')
        rospy.loginfo(self.sender().text() + " button was pressed")

    def record_button_clicked(self):
        """ Functionality for when the record button is pressed"""
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
        """Functionality for when the auto mode button is clicked"""
        self.statusBar().showMessage(self.sender().text() + ' was pressed')
        rospy.loginfo(self.sender().text() + " button was pressed")

        auto_msg = Bool()
        if self.sender().text() == "Manual Mode":
            auto_msg.data = False
            self.auto_mode_button.setText("Autonomous Mode")
            self.auto_mode_button.setStyleSheet("background-color: green")
        else:
            auto_msg.data = True
            self.auto_mode_button.setText("Manual Mode")
            self.auto_mode_button.setStyleSheet("background-color: gray")
        
        # publish the recording message        
        self.auto_pub.publish(auto_msg)
        # refresh the ui
        self.setup_ui_layout()

# TODO: jetson stats

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
