#ifndef KALMAN_FILTER_UWB_3D
#define KALMAN_FILTER_UWB_3D

#include "PositionEstimationAlgorithm.h"
#include "MLLocation.h"
#include "ros/ros.h"
#include <math.h>
#include <chrono>
#include "sensor_types.h"

#define UWB_RANGING 0



/**
 * Only TOA
 */
class KalmanFilterTOA : public PositionEstimationAlgorithm {
public:

    KalmanFilterTOA(double accelerationNoise, bool ignoreWorstAnchorMode, double ignoreCostThreshold);
    KalmanFilterTOA(double accelerationNoise, bool ignoreWorstAnchorMode, double ignoreCostThreshold, Vector3 initialPosition);
    

    bool init() override;
    bool getPose(Vector3& pose) override;
    void newPX4FlowMeasurement(double integrationX, double integrationY, double integrationRotationZ, double integrationTime, int quality) override;
    void newTOAMeasurement(const std::vector<double>& rangings, const std::vector<Beacon>& beacons, const std::vector<double>& errorEstimations, double timeLag) override;
    void newIMUMeasurement( VectorDim3 angularVelocity,double covarianceAngularVelocity[9],VectorDim3 linearAcceleration, double covarianceAcceleration[9]) override;
    void newMAGMeasurement( VectorDim3 mag, double covarianceMag[9]) override;
    void newCompassMeasurement( double compass) override;

private:

    void estimatePositionKF(const std::vector<RangingMeasurement>& rangingMeasurements);
    void predictionMatrix(arma::mat& matrix, double time) const;
    void predictionErrorCovariance(arma::mat& matrix, double timeLag) const;
    void stateToPose(Vector3& pose,const arma::vec& state, const arma::mat& estimationCovariance);
    void jacobianRangings(arma::mat& jacobian, const Vector3& position, const std::vector<RangingMeasurement>& rangingMeasurements, int indexStartRow) const;
    StateWithIgnoredAnchor kalmanStep3DIgnoreAnchor(const arma::vec& predictedState, const std::vector<RangingMeasurement>& allRangingMeasurements, int maxSteps, double minRelativeError, double timeLag, int indexIgnoredAnchor);
    StateWithCovariance kalmanStep3DCanIgnoreAnAnchor(const arma::vec& predictedState, const std::vector<RangingMeasurement>& allRangingMeasurements, int maxSteps, double minRelativeError, double timeLag, double costThreshold);
    arma::vec sensorOutputs(const Vector3& position, const Vector3& speed,double timeLag,const std::vector<RangingMeasurement>& rangingMeasurements) const;
    
    double mJolt;
    Vector3 mPosition;
    Vector3 mVelocity;
    double mTimeLag;
    double mAccelerationNoise;
    arma::mat estimationCovariance;
    bool mUseFixedInitialPosition;
    MLLocation *mlLocation;
    std::chrono::steady_clock::time_point mLastKFTimestamp;
    bool _ignoreWorstAnchorMode;
    double _ignoreCostThreshold;
};

#endif 
