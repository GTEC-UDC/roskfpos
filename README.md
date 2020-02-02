**NOTE:** This repository is related with the next scientific work:

**Barral, V.**; **Escudero, C.J.**; **García-Naya, J.A.**; **Maneiro-Catoira, R.** *NLOS Identification and Mitigation Using Low-Cost UWB Devices.* Sensors 2019, 19, 3464.[https://doi.org/10.3390/s19163464](https://doi.org/10.3390/s19163464)

If you use this code for your scientific activities, a citation is appreciated.

# README (DEPRECATED, NEEDS UPDATE)

This node implements a positioning algorithm based on EKF to generate positions based on data coming from one or various sensors:

- Ranging values (distance between a tag and some reference anchors).
- Magnetic field values.
- Acceleration and gyroscope values. 
- Rotation and displacement from an optical sensor.  

To launch the node ```kfpos```:

```bash
$ roslaunch gtec_roskfpos kfpos.launch 
```
Also there is another version that can be configured from command line to activate or deactivate the use of one or various sensors:

```bash
$ roslaunch gtec_roskfpos kfpos_with_args.launch 
```

This launcher file admits the following arguments:

* **use_uwb** Value of 1 to use the ranging measurements, 0 to ignore them.
* **use_imu** Value of 1 to use the IMU measurements, 0 to ignore them.
* **use_mag** Value of 1 to use the magnetometer measurements, 0 to ignore them.
* **use_px4** Value of 1 to use the Px4Flow measurements, 0 to ignore them.

### Subscriptions configuration

The node can be configured with the *ROS* topics that provide the sensors data. To do that, the next file must be edited:

```catkin_ws/src/gtec/roskfpos/src/kfpos/config/config_subscriptions.xml```

The file contains the name of the topics that publish the sensor data. It looks like:

```xml
<config> 	
	<ranging use="1" topic="/gtec/toa/ranging"/>
	<erlemag use="1" topic="/gtec/gazebo/erle/mag" interference="1"   	topicInterference="/gtec/gazebo/erle/maginterfered"/>
	<erleimu use="1" topic="/gtec/gazebo/erle/imu"/>
	<px4flow use="1" topic="/gtec/gazebo/px4flow"/>
	<anchors use="1" topic="/gtec/gazebo/anchors"/>   
</config>
```

Value ```use="1"``` of each parameter indicates if the location algorithm must subscribe or not to this source of information. 

**Note**: if *ranging* sensor is used, is also mandatory to be subscribed to the topic *anchors*, as this one publish the anchor positions and this information is needed by the location algorithm to process the *ranging* values.

### Configuration of *Tag* height

The *kfpos* node can locate the objects in 2D or 3D. If the object is for example a vehicle, and its movement in Z axis is always zero, then the algorithm can be configured with a fixed value of height in order to improve the accuracy in the 2D estimation. To set the height of the ranging tag the next file should be edited:

```catkin_ws/src/gtec/roskfpos/src/kfpos/config/config_t0.xml```

Contents of this file look like:

```xml
<config>
 <uwb useFixedHeight="1" fixedHeight="1.995"/>
</config>
```

If parameter ```useFixedHeight="1"``` is set to *1*, then the value set in  ```fixedHeight``` (in meters) will be used to perform a 2D location.

### Configuration of internal options

The implemented  algorithm has some internal options that can be used. The options are located in the following file:

```catkin_ws/src/gtec/roskfpos/src/kfpos/config/config_pos.xml```

This is the content of the file with a description of each option:

```xml
<config>
 <!--
	Algorithm
	*********
	type= 0: ML
	type= 1: EKF

 	Variant: Algorithm variants
 	**************************************************
 	Variant= 0: Normal mode.
 	Variant= 1: The N rangings con more error are ignored.
 			- numIgnoredRangings Number of rangings to ignore.
 	Variant= 2: Only the best 3 anchors are used. 			- bestMode: Criterion to select the best anchors: 
				* 0: less error in x,y,z. 
				* 1: less error in z.

	minZ, maxZ : Limits of Z coordinate.
	***************************************
	minZ: min Z value. Position is not generated if the estimation is lower.
 	maxZ: max Z value. Position is not generated if the estimation is greater.

	EKF Initial position
	****************
	useInitPosition = 0: A random initial position is used to initialize the algorithm.
	useInitPosition = 1: A fixed position is used. Position values are set using the next parameters: 
		- initX 
		- initY
		- initZ
 -->
 <algorithm type="0" variant="0" numIgnoredRangings="2" bestMode="0" minZ="0.0" maxZ="3.0"  useInitPosition="0" initX="8" initY="0" initZ="2"/>
</config>
```

### IMU configuration.

If a *IMU* is used, there are some configuration values that can be modify inside the next file:

```catkin_ws/src/gtec/roskfpos/src/kfpos/config/config_erleimu.xml```

Contents of file look like:

```xml
<config> 
<erleimu useFixedCovarianceAcceleration="1" covarianceAcceleration="0.01"  useFixedCovarianceAngularVelocityZ="1" covarianceAngularVelocityZ="0.000786"/> 
</config>
```

Parameters ```useFixedCovarianceAcceleration``` and ```useFixedCovarianceAngularVelocityZ``` indicate if a fixed value of covariance for the linear acceleration and for the angular velocity must be used. These fixed values and later set using the parameters ```covarianceAcceleration``` and ```covarianceAngularVelocityZ```.

### Magnetometer configuration

As in the previous case, there are some configuration parameters related with the magnetometer sensors that can be edited in the file:

```catkin_ws/src/gtec/roskfpos/src/kfpos/config/config_erlemag.xml```

File looks like:

```xml
<config>
<erlemag angleOffset="1.31559699349" covarianceMag="0.001"/>
</config>
```

Parameter ```angleOffset``` can be used to set the sensors angle (in radians) between the sensor and the real north. Parameter ```covarianceMag``` can be used to set the covariance error of the sensor.

### Px4Flow configuration:

Px4Flow options can be configured using the next file:

```catkin_ws/src/gtec/roskfpos/src/kfpos/config/config_px4flow.xml```

File looks like:

```xml
<config>
<px4flow armP0="0" armP1="0.493" useFixedSensorHeight="1" sensorHeight="3.0" sensorInitAngle ="-1.570796326794897" covarianceVelocity="0.4" covarianceGyroZ="0.4"/> 
</config>
```

Parameters ```armP0``` y ```armP1``` can be used to select the coordinates of the arm where the Px4Flow sensor is placed on the side of the vehicle. Using ```useFixedSensorHeight``` parameter a fixed value of height can be used if the sensors is placed always at the same height. Parameter ```sensorHeight``` define this height. 

Parameter ```sensorInitAngle``` (in radians) sets the initial angle between the vehicles orientation and the sensor. For example, an angle of *0 rads* means that the vehicle is placed with the front looking at the positive X axis, whereas a *-pi/2 rads* angle would mean that the fronts vehicle is looking at the negative Y axis. 

The last options, ```covarianceVelocity``` and ```covarianceGyroZ```, can be used to set the covariance error of the velocity and gyro around Z axis. 
