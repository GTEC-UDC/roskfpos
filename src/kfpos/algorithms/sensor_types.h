#ifndef SENSOR_TYPES_H
#define SENSOR_TYPES_H

#include <armadillo>


struct Vector3 {
	double x, y, z;
	double rotX, rotY, rotZ, rotW;
	double linearSpeedX, linearSpeedY, linearSpeedZ;
	double angularSpeedX, angularSpeedY, angularSpeedZ;
	arma::mat covarianceMatrix;
};


struct Beacon {
	int id;
	int index;
	Vector3 position;
};

typedef struct
{
	double ranging;
	double errorEstimation;
	Beacon beacon;
} RangingMeasurement;

typedef struct
{
	double integrationTime;
	double vx;
	double vy;
	double gyroz;
	double covarianceVelocity;
	double covarianceGyroZ;
} PX4FlowMeasurement;


typedef struct
{
	double angularVelocityZ;
	double covarianceAngularVelocityZ;
	double linearAccelerationX;
	double linearAccelerationY;
	double covarianceAccelerationXY[4];

} ImuMeasurement;


typedef struct
{
	double angle;
	double covarianceMag;

} MagMeasurement;


struct StateWithIgnoredAnchor {
	int indexIgnoredAnchor;
	double cost;
	arma::vec state;
	arma::mat estimationCovariance;
};

struct StateWithCovariance {
	arma::vec state;
	arma::mat estimationCovariance;
};

#endif