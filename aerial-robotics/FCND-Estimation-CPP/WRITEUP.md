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

     I have simply added a python script under */script/sensor_noise.py* to calculate standard deviation for GPS and Accelerometer. Calculate values are : 

   ```
   GPS   : 0.6964075466374975
   Accel : 0.47488011936473634
   ```

   Screen shot is attached in images/binit/06_sensor_noise.png

2. Implement a better rate gyro attitude integration scheme in the `updateFromIMU()` function.

   + Use FromEuler123_RPY function to create a quaternion from the current euler angle estimate
   + Do Integrated the body rate into the quaternion.
   + Convert back quaternion to euler angle. 

3. Implement all of the elements of the prediction step for the estimator.

   + Add 08_PredictState.txt scenerio at first to predict the state on acceleration measurement. It shows that estimated state doesn't follow the true state 
   + Implement the state prediction step in Predict State function. The estimator state is able to track the actual state with required slow drift. Gif is included inside /image/binit/
   + On running 09_PredictCovariance.txt scenerio, the estimated covariance is not capturing the increasing errors.
   + Implement partial derivative of the body to global rotation matrix in GetRbgPrime function. The RbgPrime is used calculate Jacobian matrix in predict method. After this the estimated covariance grows with errors. 

4. Implement the magnetometer update

   + On running 10_MagUpdate scenerio, I can see the estimate yaw and estimated standard deviation is increasing from the real value 
   + Update the magnetometer update within UpdatefromMag function using equations from section 7.3.2 Magnetometer from [Estimation For Quadrotors](https://www.overleaf.com/read/vymfngphcccj#/54894644/) paper. Later Update method is called to do the state update and covariance update.  It is crucial to normalize the difference of measurement and estimated yaw before the magnetometer update.
   + Now 10_MagUpdate scenerio, estimated standard deviation accurately captures the yaw error.

5. Implement the GPS update.

   + On running scenario 11_GPSUpdate with default code, we can see the watch the position and velocity error drifting away. They are using ideal estimator and ideal IMU. Drone goes wild if we use my estimated and the realistic IMU.
   + Update the EKF GPS Update in the function update from gps with equations from section 7.3.1 GPS from the Estimation For Quadrotor paper. 
   + After this change, we can see position error and sigma decreases in scenerio 11_GPSUpdate