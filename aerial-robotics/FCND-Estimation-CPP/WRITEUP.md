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
   GPS   : 0.6394973781911066
   Accel : 0.6394973781911066
   ```

   

2. Implement a better rate gyro attitude integration scheme in the `updateFromIMU()` function.

   + Use FromEuler123_RPY function to create a quaternion from the current euler angle estimate
   + Do Integrated the body rate into the quaternion.
   + Convert back quaternion to euler angle. 

3. Implement all of the elements of the prediction step for the estimator.

   + Add 08_PredictState.txt scenerio at first to predict the state on acceleration measurement. It shows that estimated state doesn't follow the true state 
   + Implement the state prediction step in Predict State function. The estimator state is able to track the actual state with required slow drift. Gif is included inside /image/binit/
   + On running 09_PredictCovariance.txt scenerio, the estimated covariance is not capturing the increasing errors.
   + Implement partial derivative of the body to global rotation matrix in GetRbgPrime function. The RbgPrime is used calculate Jacobian matrix in predict method. After this the estimated covariance grows with errors. 

4. Implement the magnetometer updatee 

5. Implement the GPS update.