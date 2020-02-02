#ifndef KALMAN_FILTER_PX4FLOW_H
#define KALMAN_FILTER_PX4FLOW_H


#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/assert.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>

#include "PositionEstimationAlgorithm.h"
#include "MLLocation.h"
#include <math.h>
#include <chrono>

#define UWB_RANGING 0
#define PX4FLOW 1
#define IMU 2
#define MAG 3

class KalmanFilter: public PositionEstimationAlgorithm {
public:

    KalmanFilter(double accelerationNoise, double initialAngle, double jolt, std::string filenamePos, std::string filenamePX4Flow, std::string filenameTag, std::string filenameImu, std::string filenameMag);
    KalmanFilter(double accelerationNoise, double initialAngle, double jolt, std::string filenamePos, std::string filenamePX4Flow, std::string filenameTag, std::string filenameImu, std::string filenameMag, Vector3 initialPosition);


    

/*    void setUWBOptions(bool useFixedHeight,  double uwbTagZ);
    void setPx4Options(double px4FlowArmP1, double px4FlowArmP2, double px4flowHeight);
    void setMagOptions(double magAngleOffset);
    void setImuOptions(bool useFixedCovarianceAcceleration, double fixedCovarianceAcceleration);*/


/*    void newPX4FlowMeasurement(double integrationX, double integrationY, double integrationRotationZ, double integrationTime, double covarianceVelocity, double covarianceGyroZ, int quality);
    void newUWBMeasurement(const std::vector<double>& rangings, const std::vector<Beacon>& beacons, const std::vector<double>& errorEstimations, double timeLag);
    void newIMUMeasurement( double angularVelocityZ,double covarianceAngularVelocityZ,double linearAccelerationX,double linearAccelerationY, double covarianceAccelerationXY[4]);
    void newMAGMeasurement( double magX,double magY,double covarianceMag);
    void newCompassMeasurement( double compass,double covarianceCompass);
    bool getPose(Vector3& pose);*/

    bool init() override;
    bool getPose(Vector3& pose) override;
    void newPX4FlowMeasurement(double integrationX, double integrationY, double integrationRotationZ, double integrationTime, int quality) override;
    void newTOAMeasurement(const std::vector<double>& rangings, const std::vector<Beacon>& beacons, const std::vector<double>& errorEstimations, double timeLag) override;
    void newIMUMeasurement( VectorDim3 angularVelocity,double covarianceAngularVelocity[9],VectorDim3 linearAcceleration, double covarianceAcceleration[9]) override;
    void newMAGMeasurement( VectorDim3 mag, double covarianceMag[9]) override;
    void newCompassMeasurement( double compass) override;

private:
    bool loadConfigurationFiles(std::string filenamePos, std::string filenamePX4Flow, std::string filenameTag, std::string filenameImu, std::string filenameMag);
    void estimatePositionKF(bool hasRangingMeasurements, const std::vector<RangingMeasurement>& rangingMeasurements,
                                                bool hasPX4Measurement, const PX4FlowMeasurement& px4flowMeasurement, 
                                                bool hasImuMeasurement,const ImuMeasurement& imuMeasurement,
                                                bool hasMagMeasurement,const MagMeasurement& magMeasurement);

    
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


    //Algorithm
    double mJolt;
    Vector3 mPosition;
    Vector3 mVelocity;
    Vector3 mAcceleration;
    double mAngle;
    double mAngularSpeed;
    double mTimeLag;
    double mAccelerationNoise;
    arma::mat mEstimationCovariance;
    bool mUseFixedInitialPosition;
    MLLocation *mlLocation;
    std::chrono::steady_clock::time_point mLastKFTimestamp;
    MagMeasurement lastMagMeasurement;
    bool mHasMagMeasurement;
    PX4FlowMeasurement lastPX4FlowMeasurement;
    bool mHasPX4FlowMeasurement;
    ImuMeasurement lastImuMeasurement;
    bool mHasImuMeasurement;


    std::string mFilenamePos, mFilenamePX4Flow, mFilenameTag, mFilenameImu, mFilenameMag;

    //TOA
    double mUWBtagZ;
    bool mUseFixedHeight;
    int mTagIdUWB;

    //IMU
    bool mUseImuFixedCovarianceAcceleration, mUseImuFixedCovarianceAngularVelocityZ;
    double mImuCovarianceAcceleration, mUmuCovarianceAngularVelocityZ;
    
    //PX4Flow
    double mPX4FlowArmP1, mPX4FlowArmP2;
    double mPX4flowHeight;
    bool mUseFixedHeightPX4Flow;
    double mInitAnglePX4Flow, mCovarianceVelocityPX4Flow, mCovarianceGyroZPX4Flow;

    //MAG
    double mMagAngleOffset, mCovarianceMag;



};

#endif // KALMAN_FILTER_H
