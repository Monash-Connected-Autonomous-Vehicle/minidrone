import logo from './logo.svg';
import './App.css';
import {useState, useEffect} from 'react';
import Scene from './Scene';
import io from 'socket.io-client';

const socket = io.connect('http://localhost:5000')

function App() {
  const [data, setdata] = useState(0);

  useEffect(() => {
    socket.on('counter_update', data => {
      setdata(data.value);
    });
  }, [])

  return (
    <div className="App">
      <div className="flask-data">
        <h1>Flask data:</h1>
        <p>{data}</p>
      </div>
      <div className="scene">
        <Scene />
      </div>
    </div>
  );
}

export default App;
