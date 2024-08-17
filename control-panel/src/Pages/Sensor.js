
import React, { useEffect, useState } from 'react';
import { useWebSocket } from '../Context/WebSocketContext';

const Sensor = () => {
  const { wsRef, connected } = useWebSocket();
  const [sensorData, setSensorData] = useState({ temperature: [], pressure: [] });

  useEffect(() => {
    const ws = wsRef.current;

    const handleMessage = (event) => {
      console.log('Received data:', event.data);

      const match = event.data.match(/\[.*?\]/);
      if (match) {
        const jsonString = match[0];

        try {
          const data = JSON.parse(jsonString);

          if (Array.isArray(data)) {
            const timestamp = new Date().toLocaleTimeString(); // Get the current time

            if (event.data.includes("temperature")) {
              const temperatureData = data.map((temp, index) => ({
                sensor: `Temperature Sensor ${index + 1}`,
                value: temp.toFixed(2),
                unit: '°C',
                time: timestamp
              }));
              setSensorData(prevData => ({ ...prevData, temperature: temperatureData }));
            } else if (event.data.includes("pressure")) {
              const pressureData = data.map((pressure, index) => ({
                sensor: `Pressure Sensor ${index + 1}`,
                value: pressure.toFixed(2),
                unit: 'Pa',
                time: timestamp
              }));
              setSensorData(prevData => ({ ...prevData, pressure: pressureData }));
            }
          } else {
            console.error('Received data is not an array:', data);
          }
        } catch (error) {
          console.error('Error parsing JSON data:', error);
        }
      } else {
        console.error('No JSON data found in message:', event.data);
      }
    };

    if (ws) {
      ws.onmessage = handleMessage;
    }

    // Cleanup function
    return () => {
      if (ws) {
        ws.onmessage = null;
      }
    };
  }, [wsRef]);

  const statusStyle = {
    padding: '10px',
    color: 'white',
    borderRadius: '5px',
    textAlign: 'center',
    backgroundColor: connected ? 'green' : 'red',
  };

  return (
    <div>
      <div style={statusStyle}>
        {connected ? 'Connected' : 'Disconnected'}
      </div>
      <div>
        <h3>Temperature Data:</h3>
        {sensorData.temperature.length > 0 ? (
          sensorData.temperature.map((sensor, index) => (
            <p key={index}>
              {sensor.sensor}: {sensor.value} {sensor.unit} (Time: {sensor.time})
            </p>
          ))
        ) : (
          <p>No temperature data available</p>
        )}
        <h3>Pressure Data:</h3>
        {sensorData.pressure.length > 0 ? (
          sensorData.pressure.map((sensor, index) => (
            <p key={index}>
              {sensor.sensor}: {sensor.value} {sensor.unit} (Time: {sensor.time})
            </p>
          ))
        ) : (
          <p>No pressure data available</p>
        )}
      </div>
    </div>
  );
};

export default Sensor;
