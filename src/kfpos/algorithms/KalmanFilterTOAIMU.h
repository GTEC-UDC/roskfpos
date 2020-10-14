#ifndef KALMAN_FILTER_TOAIMU_H
#define KALMAN_FILTER_TOAIMU_H

#include "PositionEstimationAlgorithm.h"
#include "MLLocation.h"
#include "ros/ros.h"
#include <math.h>
#include <chrono>
#include "sensor_types.h"


#define UWB_RANGING 0
#define IMU 2

class KalmanFilterTOAIMU : public PositionEstimationAlgorithm {
public:

    
    KalmanFilterTOAIMU(double accelerationNoise, double jolt);
    KalmanFilterTOAIMU(double accelerationNoise, double jolt, Vector3 initialPosition);


    bool init() override;
    bool getPose(Vector3& pose) override;
    void newPX4FlowMeasurement(double integrationX, double integrationY, double integrationRotationZ, double integrationTime, int quality) override;
    void newTOAMeasurement(const std::vector<double>& rangings, const std::vector<Beacon>& beacons, const std::vector<double>& errorEstimations, double timeLag) override;
    void newIMUMeasurement( VectorDim3 angularVelocity,double covarianceAngularVelocity[9],VectorDim3 linearAcceleration, double covarianceAcceleration[9]) override;
    void newMAGMeasurement( VectorDim3 mag, double covarianceMag[9]) override;
    void newCompassMeasurement( double compass) override;

private:

    void estimatePositionKF(bool hasRangingMeasurements, const std::vector<RangingMeasurement>& rangingMeasurements, 
                                                bool hasImuMeasurement,const ImuMeasurement3D& imuMeasurement);

    struct ImuOutput3D {
        VectorDim3 linearAcceleration;
        VectorDim3 angularVelocity;
    };

    struct Quaternion
    {
        double w, x, y, z;
    };
        
    arma::vec kalmanStep3D(const arma::vec& predictedState, 
                            bool hasRangingMeasurements, const std::vector<RangingMeasurement>& allRangingMeasurements, 
                            bool hasImuMeasurement,const ImuMeasurement3D& imuMeasurement,
                            int maxSteps, 
                            double minRelativeError,
                            double timeLag);
    void predictionMatrix(arma::mat& matrix, double time) const;
    void predictionErrorCovariance(arma::mat& matrix, double timeLag) const;
    void stateToPose(Vector3& pose,const arma::vec& state, const arma::mat& estimationCovariance);
    void jacobianRangings(arma::mat& jacobian, const Vector3& position, const std::vector<RangingMeasurement>& rangingMeasurements, int indexStartRow) const;
    void jacobianImu(arma::mat& jacobian, const Vector3& acceleration, const Vector3& angularVelocity, int indexStartRow) const;
    

    arma::vec sensorOutputs(const Vector3& currentPosition, const Vector3& currentLinearSpeed,const Vector3& currentAcceleration,
    const Vector3 currentAngles, const Vector3 currentAngularVelocity, double timeLag,  bool hasRangingMeasurements, 
    bool hasImuMeasurement,const std::vector<RangingMeasurement>& rangingMeasurements) const;

    ImuOutput3D imuOutput(const Vector3& acceleration, const Vector3& angularVelocity, const Vector3 angles, double timeLag) const;
    Quaternion ToQuaternion(double yaw, double pitch, double roll);

    double mJolt;

    Vector3 mPosition;
    Vector3 mVelocity;
    Vector3 mAcceleration;
    Vector3 mAngles;
    Vector3 mAngularVelocity;

    double mTimeLag;
    double mAccelerationNoise;
    arma::mat mEstimationCovariance;

    bool mUseFixedInitialPosition;

    MLLocation *mlLocation;

    std::chrono::steady_clock::time_point mLastKFTimestamp;

    ImuMeasurement3D lastImuMeasurement;
    bool mHasImuMeasurement;

};

#endif // KALMAN_FILTER_H
