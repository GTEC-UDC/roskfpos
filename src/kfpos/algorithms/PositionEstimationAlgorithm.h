#ifndef POSITION_ESTIMATION_ALGORITHM_H
#define POSITION_ESTIMATION_ALGORITHM_H

#include "ros/ros.h"
#include "sensor_types.h"


class PositionEstimationAlgorithm {

public:

	virtual ~PositionEstimationAlgorithm();
	virtual bool init();
	virtual bool getPose(Vector3& pose);
    virtual void newPX4FlowMeasurement(double integrationX, double integrationY, double integrationRotationZ, double integrationTime, int quality);
    virtual void newTOAMeasurement(const std::vector<double>& rangings, const std::vector<Beacon>& beacons, const std::vector<double>& errorEstimations, double timeLag);
    virtual void newIMUMeasurement( VectorDim3 angularVelocity,double covarianceAngularVelocity[9],VectorDim3 linearAcceleration, double covarianceAcceleration[9]);
    virtual void newMAGMeasurement( VectorDim3 mag, double covarianceMag[9]);
    virtual void newCompassMeasurement( double compass);


protected:

	struct PX4FLOWOutput {
        double vX;
        double vY;
        double gyroZ;
    };

    struct ImuOutput {
        double accelX;
        double accelY;
        double gyroZ;
        double orientationW;
    };
   
};

#endif // ML_LOCATION_H