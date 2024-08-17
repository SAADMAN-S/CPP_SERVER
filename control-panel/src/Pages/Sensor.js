/*
import React, { useEffect, useRef, useState } from 'react';

const Sensor = () => {
  const wsRef = useRef(null);
  const [connected, setConnected] = useState(false);
  const [temperatures, setTemperatures] = useState([]);

  useEffect(() => {
    const connectWebSocket = () => {
      wsRef.current = new WebSocket('ws://192.168.0.109:9002/');
      
      wsRef.current.onopen = () => {
        console.log('Connected to server');
        setConnected(true);
      };

      wsRef.current.onmessage = (event) => {
        console.log('Received data:', event.data);

        // Extract the JSON part from the message
        const match = event.data.match(/\[.*?\]/);
        if (match) {
          const jsonString = match[0];

          try {
            const data = JSON.parse(jsonString);

            if (Array.isArray(data)) {
              setTemperatures(data);
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

      wsRef.current.onerror = (error) => {
        console.error('WebSocket error:', error);
      };

      wsRef.current.onclose = () => {
        console.log('Disconnected from server');
        setConnected(false);
      };
    };

    connectWebSocket();

    return () => {
      if (wsRef.current) {
        wsRef.current.close();
      }
    };
  }, []);

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
        {temperatures.length > 0 ? (
          temperatures.map((temp, index) => (
            <p key={index}>Temperature Sensor {index + 1}: {temp.toFixed(2)} °C</p>
          ))
        ) : (
          <p>No temperature data available</p>
        )}
      </div>
    </div>
  );
};

export default Sensor;
*/
/*
import React, { useEffect, useRef, useState } from 'react';

const Sensor = () => {
  const wsRef = useRef(null);
  const [connected, setConnected] = useState(false);
  const [sensorData, setSensorData] = useState([]);

  useEffect(() => {
    const connectWebSocket = () => {
      wsRef.current = new WebSocket('ws://192.168.0.109:9002/');
      
      wsRef.current.onopen = () => {
        console.log('Connected to server');
        setConnected(true);
      };

      wsRef.current.onmessage = (event) => {
        console.log('Received data:', event.data);

        // Extract the JSON part from the message
        const match = event.data.match(/\[.*?\]/);
        if (match) {
          const jsonString = match[0];

          try {
            const data = JSON.parse(jsonString);

            if (Array.isArray(data)) {
              const timestamp = new Date().toLocaleTimeString(); // Get the current time
              const sensorDataWithTime = data.map((temp, index) => ({
                sensor: `Temperature Sensor ${index + 1}`,
                temperature: temp.toFixed(2),
                time: timestamp
              }));
              setSensorData(sensorDataWithTime);
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

      wsRef.current.onerror = (error) => {
        console.error('WebSocket error:', error);
      };

      wsRef.current.onclose = () => {
        console.log('Disconnected from server');
        setConnected(false);
      };
    };

    connectWebSocket();

    return () => {
      if (wsRef.current) {
        wsRef.current.close();
      }
    };
  }, []);

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
        {sensorData.length > 0 ? (
          sensorData.map((sensor, index) => (
            <p key={index}>
              {sensor.sensor}: {sensor.temperature} °C (Time: {sensor.time})
            </p>
          ))
        ) : (
          <p>No temperature data available</p>
        )}
      </div>
    </div>
  );
};

export default Sensor;
*/
import React, { useEffect, useRef, useState } from 'react';

const Sensor = () => {
  const wsRef = useRef(null);
  const [connected, setConnected] = useState(false);
  const [sensorData, setSensorData] = useState({ temperature: [], pressure: [] });

  useEffect(() => {
    const connectWebSocket = () => {
      wsRef.current = new WebSocket('ws://192.168.0.109:9002/');
      
      wsRef.current.onopen = () => {
        console.log('Connected to server');
        setConnected(true);
      };

      wsRef.current.onmessage = (event) => {
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

      wsRef.current.onerror = (error) => {
        console.error('WebSocket error:', error);
      };

      wsRef.current.onclose = () => {
        console.log('Disconnected from server');
        setConnected(false);
      };
    };

    connectWebSocket();

    return () => {
      if (wsRef.current) {
        wsRef.current.close();
      }
    };
  }, []);

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
