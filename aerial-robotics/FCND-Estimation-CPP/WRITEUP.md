Write up that includes all the rubric points

1. Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data.

   The following log tracing from 06_SensorNoise.txt is creating two logs file */config/log/Graph1.txt* for GPS x location  and */config/log/Graph2.txt* accelerometer x velocity .  

   ```reStructuredText
   # graphing commands
   Commands += SetTitle(1,"GPS")
   Commands += Plot(1,Quad.GPS.X,"x position",0,1,1)
   Commands += AddGraph1.SigmaThreshold(Quad.GPS.X, Quad.Pos.X, MeasuredStdDev_GPSPosXY,64,73,2)
   Commands += AddGraph1.LogToFile
   
   Commands += SetTitle(2,"Accelerometer")
   Commands += Plot(2,Quad.IMU.AX,"x accel",.5,.7,1)
   Commands += AddGraph2.SigmaThreshold(Quad.IMU.AX, 0, MeasuredStdDev_AccelXY,64,73,2)
   Commands += AddGraph2.LogToFile
   ```

     I have simple added a python script under */script/sensor_noise.py* to calculate standard deviation for GPS and Accelerometer.

2.  Implement a better rate gyro attitude integration scheme in the `updateFromIMU()` function.

3.  Implement all of the elements of the prediction step for the estimator.

4.  Implement the magnetometer update

5.  Implement the GPS update.