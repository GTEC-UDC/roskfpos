#ifndef KALMAN_FILTER_UWB_3D
#define KALMAN_FILTER_UWB_3D

#include "MLLocation.h"
#include "ros/ros.h"
#include <math.h>
#include <chrono>
#include "sensor_types.h"

#define UWB_RANGING 0



/**
 * Localiza en base a un filtro de Kalman extendido. SOLO CON UWB
 */
class KalmanFilterUWB {
public:
    /**
     * Inicializa el filtro para un periodo de muestreo del ranging y una
     * varianza de aceleración entre observaciones sucesivas de la posición.
     */
    KalmanFilterUWB(double accelerationNoise, Vector3 initialPosition,  bool ignoreWorstAnchorMode, double ignoreCostThreshold);
    KalmanFilterUWB(double accelerationNoise,  bool ignoreWorstAnchorMode, double ignoreCostThreshold);

    //Vector3 position() const;

    void newUWBMeasurement(const std::vector<double>& rangings, const std::vector<Beacon>& beacons, const std::vector<double>& errorEstimations, double timeLag);

    bool getPose(Vector3& pose);

private:

    void estimatePositionKF(const std::vector<RangingMeasurement>& rangingMeasurements);

    
    //StateWithCovariance kalmanStep3D(const arma::vec& predictedState, const std::vector<RangingMeasurement>& allRangingMeasurements, int maxSteps, 
    //    double minRelativeError, double timeLag);
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

#endif // KALMAN_FILTER_H
