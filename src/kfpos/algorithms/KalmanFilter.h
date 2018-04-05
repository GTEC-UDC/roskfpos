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
#ifndef KALMAN_FILTER_PX4FLOW_H
#define KALMAN_FILTER_PX4FLOW_H

#include "MLLocation.h"
#include "ros/ros.h"
#include <math.h>
#include <chrono>
#include "sensor_types.h"


#define UWB_RANGING 0
#define PX4FLOW 1
#define ERLE_IMU 2
#define ERLE_MAG 3



/**
 * Localiza en base a un filtro de Kalman extendido.
 */
class KalmanFilter {
public:
    /**
     * Inicializa el filtro para un periodo de muestreo del ranging y una
     * varianza de aceleración entre observaciones sucesivas de la posición.
     */
    KalmanFilter(double accelerationNoise, double uwbTagZ, double px4FlowArmP1, double px4FlowArmP2, Vector3 initialPosition, double px4flowHeight, double initialAngle, double magAngleOffset, double jolt);
    KalmanFilter(double accelerationNoise, double uwbTagZ, double px4FlowArmP1, double px4FlowArmP2, double px4flowHeight, double initialAngle, double magAngleOffset, double jolt);

    //Vector3 position() const;


    //Vector3 newPX4FlowMeasurement(double integrationX, double integrationY, double integrationRotationZ, double integrationTime, double covarianceVelocity, double covarianceGyroZ);
    void newPX4FlowMeasurement(double integrationX, double integrationY, double integrationRotationZ, double integrationTime, double covarianceVelocity, double covarianceGyroZ, int quality);
    void newUWBMeasurement(const std::vector<double>& rangings, const std::vector<Beacon>& beacons, const std::vector<double>& errorEstimations, double timeLag);
    void newErleImuMeasurement( double angularVelocityZ,double covarianceAngularVelocityZ,double linearAccelerationX,double linearAccelerationY, double covarianceAccelerationXY[4]);
    void newErleMagMeasurement( double magX,double magY,double covarianceMag);
    void newErleCompassMeasurement( double compass,double covarianceCompass);
    bool getPose(Vector3& pose);

private:

    void estimatePositionKF(bool hasRangingMeasurements, const std::vector<RangingMeasurement>& rangingMeasurements,
                                                bool hasPX4Measurement, const PX4FlowMeasurement& px4flowMeasurement, 
                                                bool hasImuMeasurement,const ErleImuMeasurement& erleImuMeasurement,
                                                bool hasMagMeasurement,const ErleMagMeasurement& erleMagMeasurement);

    struct PX4FLOWOutput {
        double vX;
        double vY;
        double gyroZ;
    };

    struct ErleImuOutput {
        double accelX;
        double accelY;
        double gyroZ;
        double orientationW;
    };
    
    arma::vec kalmanStep3D(const arma::vec& predictedState, 
                            bool hasRangingMeasurements, const std::vector<RangingMeasurement>& allRangingMeasurements, 
                            bool hasPX4Measurement, const PX4FlowMeasurement& px4flowMeasurement, 
                            bool hasImuMeasurement,const ErleImuMeasurement& erleImuMeasurement,
                            bool hasMagMeasurement,const ErleMagMeasurement& erleMagMeasurement,
                            int maxSteps, 
                            double minRelativeError,
                            double timeLag);
    void predictionMatrix(arma::mat& matrix, double time) const;
    void predictionErrorCovariance(arma::mat& matrix, double timeLag) const;
    void stateToPose(Vector3& pose,const arma::vec& state, const arma::mat& estimationCovariance);
    void jacobianRangings(arma::mat& jacobian, const Vector3& position, const std::vector<RangingMeasurement>& rangingMeasurements, int indexStartRow) const;
    void jacobianPx4flow(arma::mat& jacobian, const Vector3& velocity, const double angle, const double angularSpeed, const double timeLag, int indexStartRow) const;
    void jacobianErleBrain(arma::mat& jacobian, const Vector3& acceleration, double angle, double angularSpeed, int indexStartRow) const;
    void jacobianErleMag(arma::mat& jacobian, int indexStartRow) const;
    
    double normalizeAngle(double angle);

    arma::vec sensorOutputs(const Vector3& position, const Vector3& speed, const Vector3& acceleration, double angle, double angularSpeed,double timeLag,
            bool hasRangingMeasurements, bool hasPX4Measurement, bool hasImuMeasurement,bool hasMagMeasurement,const std::vector<RangingMeasurement>& rangingMeasurements) const;
    PX4FLOWOutput px4flowOutput(double speedX, double speedY, double angle, double angularSpeed, double timeLag) const;
    ErleImuOutput erleImuOutput(const Vector3& acceleration, double angle, double angularSpeed) const;



    double mJolt;

    Vector3 mPosition;
    Vector3 mVelocity;
    Vector3 mAcceleration;
    double mAngle;
    double mAngularSpeed;

    double mTimeLag;
    double mAccelerationNoise;
    arma::mat estimationCovariance;

    double mUWBtagZ;

    bool mUseFixedInitialPosition;
    double mPX4FlowArmP1, mPX4FlowArmP2;
    double mPX4flowHeight;

    double mMagAngleOffset;

    MLLocation *mlLocation;

    std::chrono::steady_clock::time_point mLastKFTimestamp;


    ErleMagMeasurement lastErleMagMeasurement;
    bool mHasMagMeasurement;
    PX4FlowMeasurement lastPX4FlowMeasurement;
    bool mHasPX4FlowMeasurement;
    ErleImuMeasurement lastErleImuMeasurement;
    bool mHasImuMeasurement;
    std::vector<RangingMeasurement> lastUwbMeasurements;
    bool mHasUwbMeasurement;

};

#endif // KALMAN_FILTER_H
