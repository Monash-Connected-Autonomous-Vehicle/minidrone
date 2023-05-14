from flask import Flask
from flask_socketio import SocketIO, emit
from flask_cors import CORS
import time

app = Flask(__name__)
CORS(app)
socketio = SocketIO(app)

counter = 0

@socketio.on('connect')
def handle_connect():
    global counter
    while True:
        counter += 1
        emit('counter_update', {'value': counter})
        time.sleep(1)

if __name__ == '__main__':
    socketio.run(app)