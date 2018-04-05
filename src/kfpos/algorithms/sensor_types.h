/*
MIT License

Copyright (c) 2018 Group of Electronic Technology and Communications. University of A Coruña.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#ifndef SENSOR_TYPES_H
#define SENSOR_TYPES_H

#include <armadillo>


struct Vector3 {
	double x, y, z;
	double rotX, rotY, rotZ, rotW;
	arma::mat covarianceMatrix;
};


struct Beacon {
	int id;
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

} ErleImuMeasurement;


typedef struct
{
	double angle;
	double covarianceMag;

} ErleMagMeasurement;

#endif
