from flask import Flask
from flask_socketio import SocketIO, emit
from flask_cors import CORS
import time
from periphery import I2C 
import minidrone_pb2

app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins='*', engineio_logger=True, logger=True)

i2c_address = 0x50
i2c = I2C("/dev/i2c-1")

@socketio.on('connect')
def handle_connect():
    print("CLIENT CONNECTED")
    while True:
        # counter += 1
        # emit('counter_update', {'value': counter})
        # time.sleep(1)
        msg = I2C.Message(bytearray(9), read=True)
        i2c.transfer(i2c_address, [msg])

        print("input data raw length: ", len(msg.data))
        print("input data (raw): ", msg.data)

        message = minidrone_pb2.QDES()
        message.ParseFromString(msg.data)
        print("TEMP: " + str(message.computeUnit.temp))
        print("FAN SPEED: " + str(message.computeUnit.fan_speed))

        emit('QDES', {'fan_speed': message.computeUnit.fan_speed, 'temp': message.computeUnit.temp})
        time.sleep(2)

    i2c.close()

@app.route("/")
def index():
    return "Hello, world"


if __name__ == '__main__':
    socketio.run(app)