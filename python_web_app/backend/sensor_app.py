from flask import Flask, render_template, request
from flask_socketio import SocketIO
from random import random
from threading import Lock
from datetime import datetime
from periphery import I2C 
import minidrone_pb2
import os
import subprocess

"""
Background Thread
"""
thread = None
thread_lock = Lock()

app = Flask(__name__, template_folder = '../frontend2/templates', static_folder = '../frontend2/static')
app.config['SECRET_KEY'] = 'donsky!'
socketio = SocketIO(app, cors_allowed_origins='*')

i2c_address = 0x50
i2c = I2C("/dev/i2c-0")

"""
Get current Jetson TX2 Temperature
"""
def get_tx2_temp():
    command = "cat /sys/devices/virtual/thermal/thermal_zone*/temp"
    output = subprocess.check_output(command, shell=True)
    temperature_str = output.decode("utf-8").split("\n")[0].strip()
    print("TEMP STR: " + temperature_str)
    temperature = float(temperature_str) / 1000.0  # Convert to Celsius
    return temperature


"""
Get current date time
"""
def get_current_datetime():
    now = datetime.now()
    return now.strftime("%H:%M:%S")

""" 
Generate random sequence of dummy sensor values and send it to our clients
"""
def background_thread():
    print("Generating random sensor values")
    with open('log_1.csv', "w") as file:
        file.write('time,fan_speed,temp1,temp2,temp3,tx2_temp\n')
        while True:
            try:
                msg = I2C.Message(bytearray(19), read=True)
                i2c.transfer(i2c_address, [msg])

                message = minidrone_pb2.QDES()
                message.ParseFromString(msg.data)
            except:
                print("Waiting for response from Arduino...")
                continue

            # print("TEMP: " + str(message.computeUnit.temp) + "FAN SPEED: " + str(message.computeUnit.fan_speed))

            fanSpeed = message.computeUnit.fan_speed
            temp1 = message.computeUnit.temp1
            temp2 = message.computeUnit.temp2
            temp3 = message.computeUnit.temp3
            time = get_current_datetime()
            tx2_temp = get_tx2_temp()

            log_entry = "{},{},{},{},{},{}\n".format(time, fanSpeed, temp1, temp2, temp3, tx2_temp)

            print(log_entry)

            socketio.emit('QDES', {'time': time,'fan_speed': fanSpeed,'temp1': temp1,'temp2': temp2,'temp3': temp3,'TX2_temp': tx2_temp})
            # file.write(str(time) + "," + str(fan_speed) + "," + str(temp) + "," + str(tx2_temp))        
            file.write(log_entry)
            file.flush()
            socketio.sleep(1)



"""
Serve root index file
"""
@app.route('/')
def index():
    return render_template('./index.html')

"""
Decorator for connect
"""
@socketio.on('connect')
def connect():
    global thread
    print('Client connected')

    global thread
    with thread_lock:
        if thread is None:
            thread = socketio.start_background_task(background_thread)

"""
Decorator for disconnect
"""
@socketio.on('disconnect')
def disconnect():
    print('Client disconnected',  request.sid)

if __name__ == '__main__':
    socketio.run(app)