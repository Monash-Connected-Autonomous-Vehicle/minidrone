import logo from './logo.svg';
import './App.css';
import {useState, useEffect} from 'react';
// import Scene from './Scene';
import io from 'socket.io-client';

// const socket = io.connect('ws://localhost:5000')
// console.log(socket)

function App() {
  const [data, setdata] = useState({fan_speed: 0, temp:0});
  const [counter, setCounter] = useState(0);

  useEffect(() => {
    const socket = io.connect('ws://localhost:5000')
    // console.log("test")
    console.log(socket)

    // socket.on('QDES', data => {
    //   console.log(data.fan_speed)
    //   setdata({fan_speed: data.fan_speed, temp: data.temp})
    // });
    socket.on('counter_update', data => {
      console.log("YES")
      setCounter(data);

    })

    // return () => {
    //   socket.disconnect();
    // };
  }, [])

  return (
    <div className="App">
      <div className="flask-data">
        <h1>Fan Speed:</h1>
        <p>{data.fan_speed}</p>
        <h1>Temp:</h1>
        <p>{data.temp}</p>
        <h1>Counter:</h1>
        <p>{counter}</p>
      </div>
      {/* <div className="scene">
        <Scene />
      </div> */}
    </div>
  );
}

export default App;
