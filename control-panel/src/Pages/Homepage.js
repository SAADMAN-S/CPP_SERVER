import React from 'react';
import { Link } from 'react-router-dom';

const Homepage = () => {
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
    return(
        <div>
        <h1>Control Panel</h1>
        <div className="buttons">
          <button onClick={() => sendCommand('left')}>Left</button>
          <button onClick={() => sendCommand('right')}>Right</button>
          <button onClick={() => sendCommand('forward')}>Forward</button>
          <button onClick={() => sendCommand('backward')}>Backward</button>
          <Link to="/sensor">
            <button>Go to Sensor Page</button>
          </Link>
          <Link to="/motor">
          <button>Motor</button>
  
          </Link>
          <Link to="/videostream">
          <button>Video Stream</button>
  
          </Link>

        </div>
        </div>
    )
      
      
}

export default Homepage;
