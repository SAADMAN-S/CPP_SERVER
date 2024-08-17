
import React from 'react';
import './App.css';
import { Route, Routes} from 'react-router-dom';
import Sensor from './Pages/Sensor';
import Motor from './Pages/Motor';
import Homepage from './Pages/Homepage';
import Videostream from './Pages/Videostream';

function App() {
 

  return (
    <div className="App">
      
      

      {/* Define your routes here */}
      <Routes>

        <Route path="/" element={<Homepage />} />
        <Route path="/sensor" element={<Sensor />} />
        <Route path="/motor" element={<Motor/>} />
        <Route path="/videostream" element={<Videostream/>} />
      </Routes>
    </div>
  );
}

export default App;
