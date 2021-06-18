#!/usr/bin/env python3 


# import folium
import plotly.graph_objects as go
import matplotlib.pyplot as plt
import pandas as pd
import plotly.express as px
import rospy
from sensor_msgs.msg import NavSatFix

# url: http://127.0.0.1:34011/

class GPSReader():

    def __init__(self):

        # us_cities = pd.read_csv("https://raw.githubusercontent.com/plotly/datasets/master/us-cities-top-1k.csv")

        df_gps = pd.read_csv("/home/patrick/mcav/catkin_ws/src/minidrone/mini_tools/src/gps_data.csv")

        # m = folium.Map(location=[45.5236, -122.6750])

        # m.save("plot.html")

        fig = px.scatter_mapbox(df_gps, lat="field.latitude", lon="field.longitude", 
                                color_discrete_sequence=["blue"])
        fig.update_layout(mapbox_style="open-street-map")
        fig.update_layout(margin={"r":0,"t":0,"l":0,"b":0})
        fig.update_layout(
            hovermode='closest',
            mapbox=dict(
                bearing=0,
                center=go.layout.mapbox.Center(
                    lat=df_gps.iloc[0]["field.latitude"],
                    lon=df_gps.iloc[0]["field.longitude"]
                ),
                pitch=0,
                zoom=12
            )
        )



        # TODO: figure out how to get the map tiles with cross origin error
        #TODO: try out folium?

        # fig.show()


        self.fig2 = go.FigureWidget(fig)

        # fig2.show()
        # for data in fig2.data:
        #     print(data)


        self.fig2.data[0].lat = [0]
        self.fig2.data[0].lon = [0]  
        self.fig2.data[0].marker["color"]="red"
        self.fig2.data[0].marker["size"] = 10


        # fig2.show()
        # TODO: implement as FigureWidget for dynamic updates?
        # https://plotly.com/python/figurewidget/
        # https://community.plotly.com/t/updating-data-on-mapbox-without-updating-the-whole-map/24778/15
        # https://stackoverflow.com/questions/63716543/plotly-how-to-update-redraw-a-plotly-express-figure-with-new-data
        # https://stackoverflow.com/questions/63589249/plotly-dash-display-real-time-data-in-smooth-animation/63677698#63677698
        # http://127.0.0.1:34011/

        # 38, -84
        # fig.append_trace

        # self.latitude = [145.030456]
        # self.longitude = [-37.802112]

        # # plt.ion()
        # self.fig = plt.figure()
        # self.ax = self.fig.add_subplot(111)
        # self.line1,  = self.ax.plot(self.latitude, self.longitude)

        # self.sub = rospy.Subscriber("/gps", NavSatFix, self.gps_callback)
        # plt.show()

    def gps_callback(self, msg):


        print("GPS: ", msg.latitude, ",", msg.longitude, ",", msg.altitude)
        self.latitude.append(msg.latitude)
        self.longitude.append(msg.longitude)

        # self.ax.plot(self.longitude, self.latitude)
        self.line1.set_xdata(self.latitude)
        self.line1.set_ydata(self.longitude)
        self.fig.canvas.draw()
        # self.fig.canvas.flush_events()


        # https://geopandas.org/getting_started.html
        


if __name__ == "__main__":
        

    # import matplotlib.pyplot as plt
    # import numpy as np

    # x = np.linspace(0, 6*np.pi, 100)
    # y = np.sin(x)

    # # You probably won't need this if you're embedding things in a tkinter plot...
    # plt.ion()

    # fig = plt.figure()
    # ax = fig.add_subplot(111)
    # line1, = ax.plot(x, y, 'r-') # Returns a tuple of line objects, thus the comma

    # for phase in np.linspace(0, 10*np.pi, 500):
    #     line1.set_ydata(np.sin(x + phase))
    #     fig.canvas.draw()
    #     fig.canvas.flush_events()

    rospy.init_node("gps_test")

    gps_reader = GPSReader()
    
    rospy.spin()
