
import React, { useEffect, useRef } from 'react';

const VideoStream = () => {
  const imgRef = useRef(null);

  useEffect(() => {
    const img = imgRef.current;
    if (img) {
      const interval = setInterval(() => {
        img.src = "http://192.168.0.109:8080/video";
      }, 100);

      return () => clearInterval(interval); // Clear the interval when the component unmounts
    }
  }, []);

  return <img ref={imgRef} alt="Video stream" />;
};

export default VideoStream;
