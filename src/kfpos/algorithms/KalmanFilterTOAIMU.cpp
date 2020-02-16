#include "KalmanFilterTOAIMU.h"




KalmanFilterTOAIMU::KalmanFilterTOAIMU(double accelerationNoise, double jolt) :
    mAccelerationNoise(accelerationNoise),
    mUseFixedInitialPosition(false),
    mJolt(jolt)
{
    MLLocation *mlLocation = new MLLocation();

    mPosition = { NAN, NAN, NAN};
    mVelocity = { 0, 0, 0};
    mAcceleration = { 0, 0, 0};

    mHasImuMeasurement=false;

    mEstimationCovariance.zeros(9, 9);
    mLastKFTimestamp =  std::chrono::steady_clock::time_point::min();
}




KalmanFilterTOAIMU::KalmanFilterTOAIMU(double accelerationNoise, double jolt, Vector3 initialPosition) : 
    mAccelerationNoise(accelerationNoise),
    mUseFixedInitialPosition(true),
    mJolt(jolt)
{
    MLLocation *mlLocation = new MLLocation();

    mPosition = initialPosition;
    mVelocity = { 0, 0, 0};
    mAcceleration = { 0, 0, 0};

    mHasImuMeasurement=false;

    mEstimationCovariance.zeros(9, 9);
    mLastKFTimestamp =  std::chrono::steady_clock::time_point::min();
}


bool KalmanFilterTOAIMU::init(){
    return true;
}


void KalmanFilterTOAIMU::newTOAMeasurement(const std::vector<double>& rangings,
        const std::vector<Beacon>& beacons, const std::vector<double>& errorEstimations, double timeLag) {

    std::vector<RangingMeasurement> measurements;

    for (int i = 0; i < rangings.size(); ++i)
    {
        if (rangings[i]>0){
            RangingMeasurement measurement;
            measurement.ranging = rangings[i];
            measurement.errorEstimation = errorEstimations[i];
            measurement.beacon = beacons[i];
            measurements.push_back(measurement);
        }
    }

    ImuMeasurement3D aImuMeasurement;


    if (mHasImuMeasurement){
        aImuMeasurement = lastImuMeasurement;
    }

    estimatePositionKF(true, measurements , mHasImuMeasurement, aImuMeasurement);
}


void KalmanFilterTOAIMU::newIMUMeasurement( VectorDim3 angularVelocity,double covarianceAngularVelocity[9],VectorDim3 linearAcceleration, double covarianceAcceleration[9]) {
   
    ImuMeasurement3D measurement;
    measurement.linearAccelerationX = linearAcceleration.x;
    measurement.linearAccelerationY = linearAcceleration.y;
    measurement.linearAccelerationZ = linearAcceleration.z;

    for(int i=0;i<9;i++){
        measurement.covarianceAccelerationXYZ[i] = covarianceAcceleration[i];
    }


    lastImuMeasurement = measurement;
    mHasImuMeasurement = true;

    estimatePositionKF(false, std::vector<RangingMeasurement>() , true, measurement);
}


void KalmanFilterTOAIMU::newPX4FlowMeasurement(double integrationX, double integrationY, double integrationRotationZ, double integrationTime, int quality)  {};
void KalmanFilterTOAIMU::newMAGMeasurement( VectorDim3 mag,double covarianceMag[9])  {};
void KalmanFilterTOAIMU::newCompassMeasurement( double compass)  {};


void KalmanFilterTOAIMU::estimatePositionKF(bool hasRangingMeasurements, const std::vector<RangingMeasurement>& allRangingMeasurements,
                                                bool hasImuMeasurement,const ImuMeasurement3D& imuMeasurement) {


    std::vector<RangingMeasurement> rangingMeasurements(allRangingMeasurements);

    double timeLag;

      auto now = std::chrono::steady_clock::now();

      if (mLastKFTimestamp == std::chrono::steady_clock::time_point::min()) {
        //First estimation, we have no timeLag
        timeLag = 0.1;
      } else {
        std::chrono::duration<double> diff = now -  mLastKFTimestamp;
        timeLag = diff.count();
      }
      mLastKFTimestamp = now;

    if (!mUseFixedInitialPosition) {
        //We try to find the first position using ML
        if (std::isnan(mPosition.x) ||
                std::isnan(mPosition.y)) {
            ROS_INFO("KalmanFilter initPosition is NAN"); 
            if (hasRangingMeasurements) {
                ROS_INFO("KalmanFilter Ranging mode"); 
                mPosition = mlLocation->estimatePosition(rangingMeasurements, { 1.0, 1.0, 4.0 });
                ROS_INFO("KalmanFilter new mPosition [%f %f %f]", mPosition.x, mPosition.y, mPosition.z); 

                mPosition.rotX = 0.0;
                mPosition.rotY = 0.0;
                mPosition.rotZ = 0.0;
                mPosition.rotW = 0.0;

                mEstimationCovariance(0,0) = mPosition.covarianceMatrix(0,0);
                mEstimationCovariance(1,0) = mPosition.covarianceMatrix(1,0);
                mEstimationCovariance(0,1) = mPosition.covarianceMatrix(0,1);
                mEstimationCovariance(1,1) = mPosition.covarianceMatrix(1,1);

                mPosition.covarianceMatrix = arma::eye<arma::mat>(9, 9) * 0.01;

                mPosition.covarianceMatrix(0,0) = mPosition.covarianceMatrix(0,0);
                mPosition.covarianceMatrix(0,1) = mPosition.covarianceMatrix(0,1);
                mPosition.covarianceMatrix(0,2) = mPosition.covarianceMatrix(0,2);
                mPosition.covarianceMatrix(1,0) = mPosition.covarianceMatrix(1,0);
                mPosition.covarianceMatrix(1,1) = mPosition.covarianceMatrix(1,1);
                mPosition.covarianceMatrix(1,2) = mPosition.covarianceMatrix(1,2);
                mPosition.covarianceMatrix(2,0) = mPosition.covarianceMatrix(2,0);
                mPosition.covarianceMatrix(2,1) = mPosition.covarianceMatrix(2,1);
                mPosition.covarianceMatrix(2,2) = mPosition.covarianceMatrix(2,2);

                mPosition.covarianceMatrix(0,7) = mEstimationCovariance(0, 8);
                mPosition.covarianceMatrix(1,7) = mEstimationCovariance(1, 8);
                mPosition.covarianceMatrix(2,7) = mEstimationCovariance(2, 8);

                mPosition.covarianceMatrix(7,0) = mEstimationCovariance(8, 0);
                mPosition.covarianceMatrix(7,1) = mEstimationCovariance(8, 1);
                mPosition.covarianceMatrix(7,2) = mEstimationCovariance(8, 2);

                mPosition.covarianceMatrix(7,7) = mEstimationCovariance(8, 8);
            }
            return;
        }
    }

    arma::vec state = { mPosition.x, mPosition.y, mPosition.z,
                        mVelocity.x, mVelocity.y, mVelocity.z,
                        mAcceleration.x, mAcceleration.y, mAcceleration.z
                      };

    // Prediction
    arma::mat predictionStep(9, 9);
    arma::mat predictionCovariance(9, 9);

    
    predictionMatrix(predictionStep, timeLag);
    predictionErrorCovariance(predictionCovariance, timeLag);
    arma::vec predictedState = predictionStep * state;

    mEstimationCovariance = predictionStep * mEstimationCovariance * predictionStep.t() +
                           predictionCovariance;



    state= kalmanStep3D(predictedState, 
        hasRangingMeasurements, allRangingMeasurements,
        hasImuMeasurement, imuMeasurement,
         20, 1e-4, timeLag);

    mVelocity.x = state(3);
    mVelocity.y = state(4);
    mVelocity.z = state(5);


    stateToPose(mPosition, state, mEstimationCovariance);
}


void KalmanFilterTOAIMU::stateToPose(Vector3& pose, const arma::vec& state, const arma::mat& estimationCovariance){

    pose.x = state(0);
    pose.y = state(1);
    pose.z = state(2);

    pose.rotX = 0.0;
    pose.rotY = 0.0;
    pose.rotZ = 0.0;
    pose.rotW = 0.0;

    pose.linearSpeedX = state(3);
    pose.linearSpeedY = state(4);
    pose.linearSpeedZ = state(5);

    pose.angularSpeedX = state(6);
    pose.angularSpeedY = state(7);
    pose.angularSpeedZ = state(8);

    pose.covarianceMatrix = arma::eye<arma::mat>(9, 9) * 0.01;

    pose.covarianceMatrix(0,0) =  estimationCovariance(0, 0);
    pose.covarianceMatrix(0,1) =  estimationCovariance(0, 1);
    pose.covarianceMatrix(0,2) =  estimationCovariance(0, 2);

    pose.covarianceMatrix(1,0) =  estimationCovariance(1, 0);
    pose.covarianceMatrix(1,1) =  estimationCovariance(1, 1);
    pose.covarianceMatrix(1,2) =  estimationCovariance(1, 2);

    pose.covarianceMatrix(2,0) =  estimationCovariance(2, 0);
    pose.covarianceMatrix(2,1) =  estimationCovariance(2, 1);
    pose.covarianceMatrix(2,2) =  estimationCovariance(2, 2);

    pose.covarianceMatrix(0,7) = estimationCovariance(0, 8);
    pose.covarianceMatrix(1,7) = estimationCovariance(1, 8);
    pose.covarianceMatrix(2,7) = estimationCovariance(2, 8);

    pose.covarianceMatrix(7,0) = estimationCovariance(8, 0);
    pose.covarianceMatrix(7,1) = estimationCovariance(8, 1);
    pose.covarianceMatrix(7,2) = estimationCovariance(8, 2);

    pose.covarianceMatrix(7,7) = estimationCovariance(8, 8);   
}

arma::vec KalmanFilterTOAIMU::kalmanStep3D(const arma::vec& predictedState, 
    bool hasRangingMeasurements, const std::vector<RangingMeasurement>& allRangingMeasurements, 
    bool hasImuMeasurement,const ImuMeasurement3D& imuMeasurement,
    int maxSteps, 
    double minRelativeError,
    double timeLag) {

    int countValid = 0;

    int indexRanging= 0, indexImu= 0;
    std::vector<RangingMeasurement> rangingMeasurements(allRangingMeasurements);
    arma::vec state(predictedState);

    if (hasRangingMeasurements){
        countValid = rangingMeasurements.size();
    }

    if (hasImuMeasurement){
        indexImu = countValid;
        countValid+=9;
    }

    arma::mat observationCovariance = arma::eye<arma::mat>(countValid, countValid);
    arma::vec measurements(countValid);


    if (hasRangingMeasurements){
        Vector3 mlTempPosition = mlLocation->estimatePosition(rangingMeasurements, { state(0), state(1), state(2) });
        double mlRangingError = mlLocation->estimationError(rangingMeasurements, mlTempPosition);
        for (int i = 0; i < rangingMeasurements.size(); ++i)
        {
            measurements(i) = rangingMeasurements[i].ranging;
            observationCovariance(i, i) =  std::max(mlRangingError, rangingMeasurements[i].errorEstimation);
        }
    }

    if (hasImuMeasurement){
        measurements(indexImu) = imuMeasurement.linearAccelerationX;
        measurements(indexImu+1) = imuMeasurement.linearAccelerationY;
        measurements(indexImu+2) = imuMeasurement.linearAccelerationZ;

        observationCovariance(indexImu, indexImu) = imuMeasurement.covarianceAccelerationXYZ[0];
        observationCovariance(indexImu, indexImu+1) = imuMeasurement.covarianceAccelerationXYZ[1];
        observationCovariance(indexImu, indexImu+2) = imuMeasurement.covarianceAccelerationXYZ[2];

        observationCovariance(indexImu+1, indexImu) = imuMeasurement.covarianceAccelerationXYZ[3];
        observationCovariance(indexImu+1, indexImu+1) = imuMeasurement.covarianceAccelerationXYZ[4];
        observationCovariance(indexImu+1, indexImu+2) = imuMeasurement.covarianceAccelerationXYZ[5];

        observationCovariance(indexImu+2, indexImu) = imuMeasurement.covarianceAccelerationXYZ[6];
        observationCovariance(indexImu+2, indexImu+1) = imuMeasurement.covarianceAccelerationXYZ[7];
        observationCovariance(indexImu+2, indexImu+2) = imuMeasurement.covarianceAccelerationXYZ[8];
    }

    arma::mat jacobian(countValid, 9);
    arma::mat kalmanGain;
    arma::mat invObsCovariance = arma::inv(observationCovariance);
    arma::mat invEstCovariance = arma::pinv(mEstimationCovariance);

    double cost = 1e20;
    for (int iter = 0; iter < maxSteps; iter++) {
        Vector3 currentPosition = { state(0), state(1), state(2) };
        Vector3 currentSpeed = { state(3), state(4), state(5) };
        Vector3 currentAcceleration = { state(6), state(7), state(8) };
        
        arma::vec output = sensorOutputs(currentPosition, currentSpeed, currentAcceleration, timeLag,
                      hasRangingMeasurements, hasImuMeasurement,rangingMeasurements);
        arma::vec predictionError = measurements - output;

        arma::vec predictionDiff = predictedState - state;
        arma::vec costMat = predictionError.t() * invObsCovariance * predictionError
                    + predictionDiff.t() * invEstCovariance * predictionDiff;

        double newCost = costMat(0);           
        if (std::abs(cost - newCost) / cost < minRelativeError) {
            break;
        }
        cost = newCost;

        if (hasRangingMeasurements) {
            jacobianRangings(jacobian, currentPosition, rangingMeasurements, indexRanging);
        }

        if (hasImuMeasurement) {
            jacobianImu(jacobian, currentAcceleration, indexImu);
        }


        kalmanGain = mEstimationCovariance * jacobian.t() *
                           inv(jacobian * mEstimationCovariance * jacobian.t() + observationCovariance);


        arma::vec direction = predictionDiff + kalmanGain * (predictionError - jacobian*predictionDiff);
        state = state + direction;
    }

    mEstimationCovariance = (arma::eye<arma::mat>(9, 9) - kalmanGain * jacobian) * mEstimationCovariance;
    return state;
}




arma::vec KalmanFilterTOAIMU::sensorOutputs(const Vector3& position, const Vector3& speed, const Vector3& acceleration,double timeLag,
    bool hasRangingMeasurements, bool hasImuMeasurement,const std::vector<RangingMeasurement>& rangingMeasurements) const {
    int countValid = 0;
    int indexImu= 0, indexRanging= 0;

    if (hasRangingMeasurements){
        countValid = rangingMeasurements.size();
    }

    if (hasImuMeasurement){
        indexImu = countValid;
        countValid+=9;
    }
    
    arma::vec output(countValid);
    if (hasRangingMeasurements) {
        std::vector<double> distances = mlLocation->distanceToBeacons(position, rangingMeasurements);
        int v = 0;
        for (const auto& r : rangingMeasurements) {
            output(v + indexRanging) = distances[v];
            v++;
        }
    }

    if (hasImuMeasurement) {
        ImuOutput3D sensorPrediction = imuOutput(acceleration);
        output(indexImu) = sensorPrediction.accelX;
        output(indexImu + 1) = sensorPrediction.accelY;
        output(indexImu + 2) = sensorPrediction.accelZ;
    }

    
    return output;
}


KalmanFilterTOAIMU::ImuOutput3D KalmanFilterTOAIMU::imuOutput(const Vector3& acceleration) const{
    ImuOutput3D output;

    output.accelX =acceleration.x;
    output.accelY =acceleration.y;
    output.accelZ = acceleration.z;

    return output;
}


void KalmanFilterTOAIMU::predictionMatrix(arma::mat& matrix, double timeLag) const {
    matrix << 1 << 0 << 0 << timeLag << 0 << 0 << timeLag*timeLag/2 << 0 << 0  << arma::endr
           << 0 << 1 << 0 << 0 << timeLag << 0 << 0 << timeLag*timeLag/2 << 0  << arma::endr
           << 0 << 0 << 1 << 0 << 0 << timeLag << 0 << 0 << timeLag*timeLag/2  << arma::endr
           << 0 << 0 << 0 << 1 << 0 << 0 << timeLag << 0 << 0 << arma::endr
           << 0 << 0 << 0 << 0 << 1 << 0 << 0 << timeLag << 0 << arma::endr
           << 0 << 0 << 0 << 0 << 0 << 1 << 0 << 0 << timeLag << arma::endr
           << 0 << 0 << 0 << 0 << 0 << 0 << 1 << 0 << 0 << arma::endr
           << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 1 << 0 << arma::endr
           << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 1 << arma::endr;
}


void KalmanFilterTOAIMU::predictionErrorCovariance(arma::mat& matrix, double timeLag) const {
    double t3 = pow(timeLag, 3)/6;
    double t2 = pow(timeLag, 2)/2;
    double t = timeLag;
    double a = mAccelerationNoise;
    double j = mJolt;

    matrix << j*t3*t3 << 0 << 0 << j*t3*t2 << 0 << 0 << j*t3*t << 0 << 0 << arma::endr
           << 0 << j*t3*t3 << 0 << 0 << j*t3*t2 << 0 << 0 << j*t3*t << 0 << arma::endr
           << 0 << 0 << j*t3*t3 << 0 << 0 << j*t3*t2 << 0 << 0 << j*t3*t << arma::endr
           << j*t3*t2 << 0 << 0 << j*t2*t2 << 0 << 0 << j*t2*t << 0 << 0 << arma::endr
           << 0 << j*t3*t2 << 0 << 0 << j*t2*t2 << 0 << 0 << j*t2*t << 0  << arma::endr
           << 0 << 0 << j*t3*t2 << 0 << 0 << j*t2*t2 << 0 << 0 << j*t2*t  << arma::endr
           << j*t3*t << 0 << 0 << j*t2*t << 0 << 0 << j*t*t << 0 << 0 << arma::endr 
           << 0 << j*t3*t << 0 << 0 << j*t2*t << 0 << 0 << j*t*t << 0 << arma::endr
           << 0 << 0 << j*t3*t << 0 << 0 << j*t2*t << 0 << 0 << j*t*t << arma::endr;
}

void KalmanFilterTOAIMU::jacobianRangings(arma::mat& jacobian, const Vector3& position, const std::vector<RangingMeasurement>& rangingMeasurements, int indexStartRow) const {
    std::vector<double> distances = mlLocation->distanceToBeacons(position, rangingMeasurements);
    int i = 0;
    for (auto const& r : rangingMeasurements){
        jacobian(i+indexStartRow, 0) = (position.x - r.beacon.position.x) / distances[i];
        jacobian(i+indexStartRow, 1) = (position.y - r.beacon.position.y) / distances[i];
        jacobian(i+indexStartRow, 2) = (position.z - r.beacon.position.z) / distances[i];;
        jacobian(i+indexStartRow, 3) = 0;
        jacobian(i+indexStartRow, 4) = 0;
        jacobian(i+indexStartRow, 5) = 0;
        jacobian(i+indexStartRow, 6) = 0;
        jacobian(i+indexStartRow, 7) = 0;
        jacobian(i+indexStartRow, 8) = 0;
        i++;
    }
}


void KalmanFilterTOAIMU::jacobianImu(arma::mat& jacobian, const Vector3& acceleration, int indexStartRow) const {

    jacobian(indexStartRow, 0) = 0;
    jacobian(indexStartRow, 1) = 0;
    jacobian(indexStartRow, 2) = 0;
    jacobian(indexStartRow, 3) = 0;
    jacobian(indexStartRow, 4) = 0;
    jacobian(indexStartRow, 5) = 0;
    jacobian(indexStartRow, 6) = acceleration.x;
    jacobian(indexStartRow, 7) = 0;
    jacobian(indexStartRow, 9) = 0;
    
    jacobian(indexStartRow +1, 0) = 0;
    jacobian(indexStartRow +1, 1) = 0;
    jacobian(indexStartRow +1, 2) = 0;
    jacobian(indexStartRow +1, 3) = 0;
    jacobian(indexStartRow +1, 4) = 0;
    jacobian(indexStartRow +1, 5) = 0;
    jacobian(indexStartRow +1, 6) = 0;
    jacobian(indexStartRow +1, 7) = acceleration.y;
    jacobian(indexStartRow +1, 8) = 0;

    jacobian(indexStartRow +2, 0) = 0;
    jacobian(indexStartRow +2, 1) = 0;
    jacobian(indexStartRow +2, 2) = 0;
    jacobian(indexStartRow +2, 3) = 0;
    jacobian(indexStartRow +2, 4) = 0;
    jacobian(indexStartRow +2, 5) = 0;
    jacobian(indexStartRow +2, 6) = 0;
    jacobian(indexStartRow +2, 7) = 0; 
    jacobian(indexStartRow +2, 8) = acceleration.z;

}


bool KalmanFilterTOAIMU::getPose(Vector3& pose) {
    double timeLag;
    auto now = std::chrono::steady_clock::now();

    if (mLastKFTimestamp == std::chrono::steady_clock::time_point::min()) {

        timeLag = 0.1;
        //TODO: We dont received any measurement, we cant return any estimation
        return false;

    } else {

        std::chrono::duration<double> diff = now -  mLastKFTimestamp;
        timeLag = diff.count();
    }
    
    arma::vec state = { mPosition.x, mPosition.y, mPosition.z,
                        mVelocity.x, mVelocity.y, mVelocity.z,
                        mAcceleration.x, mAcceleration.y, mAcceleration.z,
                      };

    // Prediction
    arma::mat predictionStep(9, 9);
    arma::mat predictionCovariance(9, 9);

    predictionMatrix(predictionStep, timeLag);
    predictionErrorCovariance(predictionCovariance, timeLag);
    arma::vec predictedState = predictionStep * state;

    arma::mat predictedEstimationCovariance = predictionStep * mEstimationCovariance * predictionStep.t() +
                           predictionCovariance;

    stateToPose(pose, predictedState, predictedEstimationCovariance);
    return true;
}