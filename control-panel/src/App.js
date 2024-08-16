/*
import React from 'react';
import './App.css';
import Sensor from './Sensor';
//import { BrowserRouter as Route,  Routes } from 'react-router-dom';
import { BrowserRouter as Router, Route, Routes, Link } from 'react-router-dom';

function App() {
  const sendCommand = (direction) => {
    fetch('http://192.168.0.109:8080/command', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ direction }),
    })
    .then(response => response.text())
    .then(data => console.log('Response:', data))
    .catch((error) => console.error('Error:', error));
  };

  return (
    <div className="App">
      <h1>Control Panel</h1>
      <div className="buttons">
        <button onClick={() => sendCommand('left')}>Left</button>
        <button onClick={() => sendCommand('right')}>Right</button>
        <button onClick={() => sendCommand('forward')}>Forward</button>
        <button onClick={() => sendCommand('backward')}>Backward</button>
      </div>
      <Routes>
          <Route path="/sensor" element={<Sensor />} />
      </Routes>
    </div>
  );
}

export default App;
*/
import React from 'react';
import './App.css';
import { Route, Routes} from 'react-router-dom';
import Sensor from './Pages/Sensor';
import Motor from './Pages/Motor';
import Homepage from './Pages/Homepage';

function App() {
 

  return (
    <div className="App">
      
      

      {/* Define your routes here */}
      <Routes>

        <Route path="/" element={<Homepage />} />
        <Route path="/sensor" element={<Sensor />} />
        <Route path="/motor" element={<Motor/>} />
      </Routes>
    </div>
  );
}

export default App;
