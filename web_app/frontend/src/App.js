import logo from './logo.svg';
import './App.css';
import {useState, useEffect} from 'react';
// import Scene from './Scene';
import io from 'socket.io-client';

const socket = io.connect('http://localhost:5000')

function App() {
  const [data, setdata] = useState({fan_speed: 0, temp:0});

  useEffect(() => {
    socket.on('QDES', data => {
      setdata({fan_speed: data.fan_speed, temp: data.temp})
    });
  }, [])

  return (
    <div className="App">
      <div className="flask-data">
        <h1>Fan Speed:</h1>
        <p>{data.fan_speed}</p>
        <h1>Temp:</h1>
        <p>{data.temp}</p>
      </div>
      {/* <div className="scene">
        <Scene />
      </div> */}
    </div>
  );
}

export default App;
