#ifndef KALMAN_FILTER_PX4FLOW_H
#define KALMAN_FILTER_PX4FLOW_H

#include "MLLocation.h"
#include "ros/ros.h"
#include <math.h>
#include <chrono>
#include "sensor_types.h"


#define UWB_RANGING 0
#define PX4FLOW 1
#define IMU 2
#define MAG 3

class KalmanFilter {
public:

    KalmanFilter(double accelerationNoise, double uwbTagZ, double px4FlowArmP1, double px4FlowArmP2, Vector3 initialPosition, double px4flowHeight, double initialAngle, double magAngleOffset, double jolt);
    KalmanFilter(double accelerationNoise, double uwbTagZ, double px4FlowArmP1, double px4FlowArmP2, double px4flowHeight, double initialAngle, double magAngleOffset, double jolt);

     void newPX4FlowMeasurement(double integrationX, double integrationY, double integrationRotationZ, double integrationTime, double covarianceVelocity, double covarianceGyroZ, int quality);
    void newUWBMeasurement(const std::vector<double>& rangings, const std::vector<Beacon>& beacons, const std::vector<double>& errorEstimations, double timeLag);
    void newIMUMeasurement( double angularVelocityZ,double covarianceAngularVelocityZ,double linearAccelerationX,double linearAccelerationY, double covarianceAccelerationXY[4]);
    void newMAGMeasurement( double magX,double magY,double covarianceMag);
    void newCompassMeasurement( double compass,double covarianceCompass);
    bool getPose(Vector3& pose);

private:

    void estimatePositionKF(bool hasRangingMeasurements, const std::vector<RangingMeasurement>& rangingMeasurements,
                                                bool hasPX4Measurement, const PX4FlowMeasurement& px4flowMeasurement, 
                                                bool hasImuMeasurement,const ImuMeasurement& imuMeasurement,
                                                bool hasMagMeasurement,const MagMeasurement& magMeasurement);

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
    
    arma::vec kalmanStep3D(const arma::vec& predictedState, 
                            bool hasRangingMeasurements, const std::vector<RangingMeasurement>& allRangingMeasurements, 
                            bool hasPX4Measurement, const PX4FlowMeasurement& px4flowMeasurement, 
                            bool hasImuMeasurement,const ImuMeasurement& imuMeasurement,
                            bool hasMagMeasurement,const MagMeasurement& magMeasurement,
                            int maxSteps, 
                            double minRelativeError,
                            double timeLag);
    void predictionMatrix(arma::mat& matrix, double time) const;
    void predictionErrorCovariance(arma::mat& matrix, double timeLag) const;
    void stateToPose(Vector3& pose,const arma::vec& state, const arma::mat& estimationCovariance);
    void jacobianRangings(arma::mat& jacobian, const Vector3& position, const std::vector<RangingMeasurement>& rangingMeasurements, int indexStartRow) const;
    void jacobianPx4flow(arma::mat& jacobian, const Vector3& velocity, const double angle, const double angularSpeed, const double timeLag, int indexStartRow) const;
    void jacobianImu(arma::mat& jacobian, const Vector3& acceleration, double angle, double angularSpeed, int indexStartRow) const;
    void jacobianMag(arma::mat& jacobian, int indexStartRow) const;
    
    double normalizeAngle(double angle);

    arma::vec sensorOutputs(const Vector3& position, const Vector3& speed, const Vector3& acceleration, double angle, double angularSpeed,double timeLag,
            bool hasRangingMeasurements, bool hasPX4Measurement, bool hasImuMeasurement,bool hasMagMeasurement,const std::vector<RangingMeasurement>& rangingMeasurements) const;
    PX4FLOWOutput px4flowOutput(double speedX, double speedY, double angle, double angularSpeed, double timeLag) const;
    ImuOutput imuOutput(const Vector3& acceleration, double angle, double angularSpeed) const;



    double mJolt;

    Vector3 mPosition;
    Vector3 mVelocity;
    Vector3 mAcceleration;
    double mAngle;
    double mAngularSpeed;

    double mTimeLag;
    double mAccelerationNoise;
    arma::mat mEstimationCovariance;

    double mUWBtagZ;

    bool mUseFixedInitialPosition;
    double mPX4FlowArmP1, mPX4FlowArmP2;
    double mPX4flowHeight;

    double mMagAngleOffset;

    MLLocation *mlLocation;

    std::chrono::steady_clock::time_point mLastKFTimestamp;


    MagMeasurement lastMagMeasurement;
    bool mHasMagMeasurement;
    PX4FlowMeasurement lastPX4FlowMeasurement;
    bool mHasPX4FlowMeasurement;
    ImuMeasurement lastImuMeasurement;
    bool mHasImuMeasurement;

};

#endif // KALMAN_FILTER_H
