from calendar import c
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import time
import numpy as np
import torch
from matplotlib import pyplot as plt
import matplotlib.pylab as pl
from matplotlib.colors import ListedColormap

from md_lane_detection import letterbox



class Lane_Detector_Yolo(Node):
    def __init__(self):
        super().__init__('lane_detector_yolo')

        self.subscription = self.create_subscription(
            Image,
            '/stereo_camera/stereo_camera/left/image_raw',
            self.lane_detect_callback,
            10)
        self.publisher_ = self.create_publisher(Image, '/test_md_ld', 10)

    
    def lane_detect_callback(self, msg):
        with torch.no_grad():
            
            # Determine device and load model
            device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
            model = torch.jit.load('yolopv2.pt', device)
            model.eval()
            print("cuda:", next(model.parameters()).is_cuda)
            
        try:
            print("start")
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')  
            img_box = letterbox(cv_image, 640, stride=32)[0]
            img = img_box[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB
            img = np.ascontiguousarray(img)
            img = torch.from_numpy(img).to(device)/255.0
            if img.ndimension() == 3:
                img = img.unsqueeze(0)
            # Run the model and get lane lines
            t0 = time.time()
            # for _ in range(100):
            _, _, lane_lines = model(img)
            print(lane_lines)

            # print("inference")
            t1 = time.time()
            print('Average inference time:', (t1-t0)/100)  # Over 100 images (or, the same image 100 times)

            # Process lane lines into numpy array (360x640)
            print(lane_lines.shape)
            lane_lines = torch.round(lane_lines).squeeze(1).squeeze().cpu().numpy()
            lane_lines = cv2.resize(lane_lines, dsize=(640, 480), interpolation=cv2.INTER_CUBIC)
            print(type(lane_lines), lane_lines.shape)
            print(img_box.shape)

            # Colormap
            cmap = pl.cm.Reds
            my_cmap = cmap(np.arange(cmap.N))

            # Set alpha
            my_cmap[:,-1] = np.linspace(0, 1, cmap.N)

            # Create new colormap
            my_cmap = ListedColormap(my_cmap)

            # Show lane lines
            plt.imshow(img_box)
            plt.imshow(lane_lines, alpha=0.5, cmap=my_cmap)
            #   plt.imshow(lane_lines)
            plt.show()
            print("SUCCESSFUL \n\n")
        except Exception as e:
            print(e)

    

def main(args=None):
    rclpy.init(args=args)
    lane_detector_yolo = Lane_Detector_Yolo()
    rclpy.spin(lane_detector_yolo)
    lane_detector_yolo.destroy_node()
    rclpy.shutdown()

if __name__=="main":
    main()
