#include "KalmanFilter.h"




KalmanFilter::KalmanFilter(double accelerationNoise, double initialAngle, double jolt, std::string filenamePos, std::string filenamePX4Flow, std::string filenameTag, std::string filenameImu, std::string filenameMag) :
    mAccelerationNoise(accelerationNoise),
    mAngularSpeed(0.0),
    mAngle(initialAngle),
    mUseFixedInitialPosition(false),
    mJolt(jolt),
    mFilenamePos(filenamePos), 
    mFilenamePX4Flow(filenamePX4Flow), 
    mFilenameTag(filenameTag), 
    mFilenameImu(filenameImu), 
    mFilenameMag(filenameMag)
{
    MLLocation *mlLocation = new MLLocation();

    mPosition = { NAN, NAN, NAN};
    mVelocity = { 0, 0, 0};
    mAcceleration = { 0, 0, 0};

    mHasMagMeasurement=false;
    mHasPX4FlowMeasurement=false;
    mHasImuMeasurement=false;

    mEstimationCovariance.zeros(8, 8);
    mLastKFTimestamp =  std::chrono::steady_clock::time_point::min();
}



KalmanFilter::KalmanFilter(double accelerationNoise, double initialAngle, double jolt , std::string filenamePos, std::string filenamePX4Flow, std::string filenameTag, std::string filenameImu, std::string filenameMag, Vector3 initialPosition):
    mAccelerationNoise(accelerationNoise),
    mAngularSpeed(0.0),
    mAngle(initialAngle),
    mUseFixedInitialPosition(true),
    mJolt(jolt),
    mFilenamePos(filenamePos), 
    mFilenamePX4Flow(filenamePX4Flow), 
    mFilenameTag(filenameTag), 
    mFilenameImu(filenameImu), 
    mFilenameMag(filenameMag)
{
    MLLocation *mlLocation = new MLLocation();

    mPosition = initialPosition;
    mVelocity = { 0, 0, 0};
    mAcceleration = { 0, 0, 0};

    mHasMagMeasurement=false;
    mHasPX4FlowMeasurement=false;
    mHasImuMeasurement=false;

    mEstimationCovariance.zeros(8, 8);
    mLastKFTimestamp =  std::chrono::steady_clock::time_point::min();
}

bool KalmanFilter::init()  {
    return loadConfigurationFiles(mFilenamePos, mFilenamePX4Flow, mFilenameTag, mFilenameImu, mFilenameMag);
}

void KalmanFilter::newTOAMeasurement(const std::vector<double>& rangings,
        const std::vector<Beacon>& beacons, const std::vector<double>& errorEstimations, double timeLag){

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

    ImuMeasurement aImuMeasurement;
    PX4FlowMeasurement aPX4Measurement;
    MagMeasurement aMagMeasurement;

    if (mHasMagMeasurement){
        aMagMeasurement = lastMagMeasurement;    
    }

    if (mHasImuMeasurement){
        aImuMeasurement = lastImuMeasurement;
    }

    if (mHasPX4FlowMeasurement){
        aPX4Measurement = lastPX4FlowMeasurement;
    }

    estimatePositionKF(true, measurements , mHasPX4FlowMeasurement, aPX4Measurement, mHasImuMeasurement, aImuMeasurement, mHasMagMeasurement, aMagMeasurement);
}


void KalmanFilter::newPX4FlowMeasurement(double integrationX, double integrationY, double integrationRotationZ, double integrationTime, int quality) {


    PX4FlowMeasurement measurement;

    measurement.vy= integrationY/(integrationTime/1000000.0)*mPX4flowHeight;
    measurement.vx= integrationX/(integrationTime/1000000.0)*mPX4flowHeight;

    measurement.gyroz = integrationRotationZ/(integrationTime/1000000.0);
    measurement.integrationTime = integrationTime/1000000.0;

    if (quality==0){
        return;
    }

    if (integrationTime>0){
       measurement.covarianceVelocity = mCovarianceVelocityPX4Flow/measurement.integrationTime*mPX4flowHeight/quality;  
   } else {
    //TODO: We artificially increase the covariance error if integrationTime=0
    //This shouldn't happen if quality > 0
       measurement.covarianceVelocity = mCovarianceVelocityPX4Flow*quality;
   }
   
    measurement.covarianceGyroZ = mCovarianceGyroZPX4Flow;

    ImuMeasurement voidMeasurement;
    MagMeasurement voidMeasurementMag;

    lastPX4FlowMeasurement = measurement;

    mHasPX4FlowMeasurement = true;

    estimatePositionKF(false, std::vector<RangingMeasurement>() , true, measurement , false, voidMeasurement, false, voidMeasurementMag);
}



void KalmanFilter::newIMUMeasurement( VectorDim3 angularVelocity,double covarianceAngularVelocity[9],VectorDim3 linearAcceleration, double covarianceAcceleration[9]) {

    double covarianceAccelerationXY[4];

    if (mUseImuFixedCovarianceAcceleration) {
      covarianceAccelerationXY[0] = mImuCovarianceAcceleration;
      covarianceAccelerationXY[1] = covarianceAcceleration[1];
      covarianceAccelerationXY[2] = covarianceAcceleration[3];
      covarianceAccelerationXY[3] = mImuCovarianceAcceleration;
    } else {
      covarianceAccelerationXY[0] = covarianceAcceleration[0];
      covarianceAccelerationXY[1] = covarianceAcceleration[1];
      covarianceAccelerationXY[2] = covarianceAcceleration[3];
      covarianceAccelerationXY[3] = covarianceAcceleration[4];
    }

    double covAngularVelocityZ = covarianceAngularVelocity[8];
    if (mUseImuFixedCovarianceAngularVelocityZ) {
      covAngularVelocityZ = mUmuCovarianceAngularVelocityZ;
    }


    ImuMeasurement measurement;
    measurement.angularVelocityZ= angularVelocity.z;
    measurement.covarianceAngularVelocityZ= covAngularVelocityZ;
    measurement.linearAccelerationX = linearAcceleration.x;
    measurement.linearAccelerationY = linearAcceleration.y;
    measurement.covarianceAccelerationXY[0] = covarianceAccelerationXY[0];
    measurement.covarianceAccelerationXY[1] = covarianceAccelerationXY[1];
    measurement.covarianceAccelerationXY[2] = covarianceAccelerationXY[2];
    measurement.covarianceAccelerationXY[3] = covarianceAccelerationXY[3];

    PX4FlowMeasurement voidMeasurement;
    MagMeasurement voidMeasurementMag;

    lastImuMeasurement = measurement;
    mHasImuMeasurement = true;

    estimatePositionKF(false, std::vector<RangingMeasurement>() , false, voidMeasurement , true, measurement, false, voidMeasurementMag);
}
 

void KalmanFilter::newMAGMeasurement( VectorDim3 mag, double covarianceMag[9])  {

    MagMeasurement measurement;
    measurement.angle = atan2(mag.y, mag.x) - mMagAngleOffset;
    measurement.covarianceMag = mCovarianceMag;

    PX4FlowMeasurement voidMeasurementPx4;
    ImuMeasurement voidMeasurement;

    lastMagMeasurement = measurement;
    mHasMagMeasurement = true;

    estimatePositionKF(false, std::vector<RangingMeasurement>() , false, voidMeasurementPx4, false, voidMeasurement , true, measurement);

}

void KalmanFilter::newCompassMeasurement( double compass) {

    MagMeasurement measurement;


   double compassCorrected = compass;
    measurement.angle = normalizeAngle(compassCorrected);
    measurement.covarianceMag = mCovarianceMag;

    ImuMeasurement aImuMeasurement;
    PX4FlowMeasurement aPX4Measurement;

    if (mHasImuMeasurement){
        aImuMeasurement = lastImuMeasurement;
    }

    if (mHasPX4FlowMeasurement){
        aPX4Measurement = lastPX4FlowMeasurement;
    }


    lastMagMeasurement = measurement;
    mHasMagMeasurement = true;

    estimatePositionKF(false, std::vector<RangingMeasurement>() , mHasPX4FlowMeasurement, aPX4Measurement, mHasImuMeasurement, aImuMeasurement , true, measurement);

}


void KalmanFilter::estimatePositionKF(bool hasRangingMeasurements, const std::vector<RangingMeasurement>& allRangingMeasurements,
                                                bool hasPX4Measurement, const PX4FlowMeasurement& px4flowMeasurement, 
                                                bool hasImuMeasurement,const ImuMeasurement& imuMeasurement,
                                                bool hasMagMeasurement,const MagMeasurement& magMeasurement) {


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
                if (mUseFixedHeight) {
                    mPosition = mlLocation->estimatePosition2D(rangingMeasurements, { 1.0, 1.0, mUWBtagZ });   
                } else {
                    
                    mPosition = mlLocation->estimatePosition(rangingMeasurements, { 1.0, 1.0, 4.0 });
                    mUWBtagZ = mPosition.z; 
                }
                
                ROS_INFO("KalmanFilter new mPosition [%f %f %f]", mPosition.x, mPosition.y, mPosition.z); 

                double halfAngle = mAngle*0.5;
                mPosition.rotX = 0.0;
                mPosition.rotY = 0.0;
                mPosition.rotZ = sin(halfAngle);
                mPosition.rotW = cos(halfAngle);

                mEstimationCovariance(0,0) = mPosition.covarianceMatrix(0,0);
                mEstimationCovariance(1,0) = mPosition.covarianceMatrix(1,0);
                mEstimationCovariance(0,1) = mPosition.covarianceMatrix(0,1);
                mEstimationCovariance(1,1) = mPosition.covarianceMatrix(1,1);
                mPosition.covarianceMatrix = arma::eye<arma::mat>(6, 6) * 0.01;
                mPosition.covarianceMatrix(0,0) = mEstimationCovariance(0, 0);
                mPosition.covarianceMatrix(0,1) = mEstimationCovariance(0, 1);
                mPosition.covarianceMatrix(1,0) = mEstimationCovariance(1, 0);
                mPosition.covarianceMatrix(1,1) = mEstimationCovariance(1, 1);
                mPosition.covarianceMatrix(0,5) = mEstimationCovariance(0, 6);
                mPosition.covarianceMatrix(1,5) = mEstimationCovariance(1, 6);
                mPosition.covarianceMatrix(5,0) = mEstimationCovariance(6, 0);
                mPosition.covarianceMatrix(5,1) = mEstimationCovariance(6, 1);
                mPosition.covarianceMatrix(5,5) = mEstimationCovariance(6, 6);
            }
            return;
        }
    }

    arma::vec state = { mPosition.x, mPosition.y,
                        mVelocity.x, mVelocity.y,
                        mAcceleration.x, mAcceleration.y,
                        mAngle, mAngularSpeed
                      };

    // Prediction
    arma::mat predictionStep(8, 8);
    arma::mat predictionCovariance(8, 8);

    
    predictionMatrix(predictionStep, timeLag);
    predictionErrorCovariance(predictionCovariance, timeLag);
    arma::vec predictedState = predictionStep * state;

    mEstimationCovariance = predictionStep * mEstimationCovariance * predictionStep.t() +
                           predictionCovariance;

    predictedState(6) = normalizeAngle(predictedState(6));


    state= kalmanStep3D(predictedState, 
        hasRangingMeasurements, allRangingMeasurements,
        hasPX4Measurement, px4flowMeasurement,
        hasImuMeasurement, imuMeasurement,
        hasMagMeasurement, magMeasurement, 
         20, 1e-4, timeLag);

    mVelocity.x = state(2);
    mVelocity.y = state(3);
    mAngle = state(6);
    mAngularSpeed = state(7);

    stateToPose(mPosition, state, mEstimationCovariance);
}


void KalmanFilter::stateToPose(Vector3& pose, const arma::vec& state, const arma::mat& estimationCovariance){

    pose.x = state(0);
    pose.y = state(1);
    if (mUseFixedHeight){
        pose.z = mUWBtagZ;
    } else {
        pose.z = mUWBtagZ;
    }
    

    double halfAngle = state(6)*0.5;

    pose.rotX = 0.0;
    pose.rotY = 0.0;
    pose.rotZ = sin(halfAngle);
    pose.rotW = cos(halfAngle);


    pose.linearSpeedX = state(2);
    pose.linearSpeedY = state(3);
    pose.linearSpeedZ = 0.0;

    pose.angularSpeedX = 0.0;
    pose.angularSpeedY = 0.0;
    pose.angularSpeedZ = state(7);

    pose.covarianceMatrix = arma::eye<arma::mat>(6, 6) * 0.01;

    pose.covarianceMatrix(0,0) =  estimationCovariance(0, 0);
    pose.covarianceMatrix(0,1) =  estimationCovariance(0, 1);
    pose.covarianceMatrix(1,0) =  estimationCovariance(1, 0);
    pose.covarianceMatrix(1,1) =  estimationCovariance(1, 1);

    pose.covarianceMatrix(0,5) = estimationCovariance(0, 6);
    pose.covarianceMatrix(1,5) = estimationCovariance(1, 6);
    pose.covarianceMatrix(5,0) = estimationCovariance(6, 0);
    pose.covarianceMatrix(5,1) = estimationCovariance(6, 1);
    pose.covarianceMatrix(5,5) = estimationCovariance(6, 6);   
}

arma::vec KalmanFilter::kalmanStep3D(const arma::vec& predictedState, 
    bool hasRangingMeasurements, const std::vector<RangingMeasurement>& allRangingMeasurements, 
    bool hasPX4Measurement, const PX4FlowMeasurement& px4flowMeasurement, 
    bool hasImuMeasurement,const ImuMeasurement& imuMeasurement,
    bool hasMagMeasurement,const MagMeasurement& magMeasurement,
    int maxSteps, 
    double minRelativeError,
    double timeLag) {

    int countValid = 0;

    int indexRanging= 0, indexPX4= 0, indexImu= 0, indexMag = 0;
    std::vector<RangingMeasurement> rangingMeasurements(allRangingMeasurements);
    arma::vec state(predictedState);

    if (hasRangingMeasurements){
        countValid = rangingMeasurements.size();
    }

    if(hasPX4Measurement){
        indexPX4 = countValid;
        countValid+=3;
    }

    if (hasImuMeasurement){
        indexImu = countValid;
        countValid+=3;
    }

    if (hasMagMeasurement){
        indexMag = countValid;
        countValid+=1;
    }

    arma::mat observationCovariance = arma::eye<arma::mat>(countValid, countValid);
    arma::vec measurements(countValid);


    if (hasRangingMeasurements){
        Vector3 mlTempPosition = mlLocation->estimatePosition2D(rangingMeasurements, { state(0), state(1), mUWBtagZ });
        double mlRangingError = mlLocation->estimationError(rangingMeasurements, mlTempPosition);
        for (int i = 0; i < rangingMeasurements.size(); ++i)
        {
            measurements(i) = rangingMeasurements[i].ranging;
            observationCovariance(i, i) =  std::max(mlRangingError, rangingMeasurements[i].errorEstimation);
        }
    }

    if(hasPX4Measurement){
        measurements(indexPX4) = px4flowMeasurement.vx;
        measurements(indexPX4+1) = px4flowMeasurement.vy;
        measurements(indexPX4+2) = px4flowMeasurement.gyroz;
        double absSpeed = sqrt(predictedState(2)*predictedState(2) 
                    + predictedState(3)*predictedState(3));

        observationCovariance(indexPX4,indexPX4) = px4flowMeasurement.covarianceVelocity;
        observationCovariance(indexPX4+1,indexPX4+1) = px4flowMeasurement.covarianceVelocity;
        observationCovariance(indexPX4+2,indexPX4+2) = px4flowMeasurement.covarianceGyroZ;

    }

    if (hasImuMeasurement){
        measurements(indexImu) = imuMeasurement.linearAccelerationX;
        measurements(indexImu+1) = imuMeasurement.linearAccelerationY;
        measurements(indexImu+2) = imuMeasurement.angularVelocityZ;

        observationCovariance(indexImu, indexImu) = imuMeasurement.covarianceAccelerationXY[0];
        observationCovariance(indexImu, indexImu+1) = imuMeasurement.covarianceAccelerationXY[1];
        observationCovariance(indexImu+1, indexImu) = imuMeasurement.covarianceAccelerationXY[2];
        observationCovariance(indexImu+1, indexImu+1) = imuMeasurement.covarianceAccelerationXY[3];
        observationCovariance(indexImu+2, indexImu+2) = imuMeasurement.covarianceAngularVelocityZ;
    }

    if (hasMagMeasurement){
        measurements(indexMag) = magMeasurement.angle;

        observationCovariance(indexMag, indexMag) = magMeasurement.covarianceMag;
    }

    arma::mat jacobian(countValid, 8);
    arma::mat kalmanGain;
    arma::mat invObsCovariance = arma::inv(observationCovariance);
    arma::mat invEstCovariance = arma::pinv(mEstimationCovariance);

    double cost = 1e20;
    for (int iter = 0; iter < maxSteps; iter++) {
        Vector3 currentPosition = { state(0), state(1), mUWBtagZ };
        Vector3 currentSpeed = { state(2), state(3), 0.0 };
        Vector3 currentAcceleration = { state(4), state(5), 0.0 };
        double currentAngle = state(6);
        double currentAngularSpeed = state(7);
        
        arma::vec output = sensorOutputs(currentPosition, currentSpeed, currentAcceleration, currentAngle, currentAngularSpeed, timeLag,
                      hasRangingMeasurements,  hasPX4Measurement,  hasImuMeasurement, hasMagMeasurement,rangingMeasurements);
        arma::vec predictionError = measurements - output;

        if (hasMagMeasurement) {
            predictionError(indexMag) = normalizeAngle(predictionError(indexMag));
        }

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

        if (hasPX4Measurement) {
            jacobianPx4flow(jacobian, currentSpeed, currentAngle, currentAngularSpeed, timeLag, indexPX4);
        }

        if (hasImuMeasurement) {
            jacobianImu(jacobian, currentAcceleration, currentAngle, currentAngularSpeed, indexImu);
        }

        if (hasMagMeasurement) {
            jacobianMag(jacobian, indexMag);
        }

        kalmanGain = mEstimationCovariance * jacobian.t() *
                           inv(jacobian * mEstimationCovariance * jacobian.t() + observationCovariance);


        arma::vec direction = predictionDiff + kalmanGain * (predictionError - jacobian*predictionDiff);
        state = state + direction;
    }

    mEstimationCovariance = (arma::eye<arma::mat>(8, 8) - kalmanGain * jacobian) * mEstimationCovariance;
    return state;
}




arma::vec KalmanFilter::sensorOutputs(const Vector3& position, const Vector3& speed, const Vector3& acceleration, double angle, double angularSpeed,double timeLag,
    bool hasRangingMeasurements, bool hasPX4Measurement, bool hasImuMeasurement,bool hasMagMeasurement,const std::vector<RangingMeasurement>& rangingMeasurements) const {
    int countValid = 0;

    int indexRanging= 0, indexPX4= 0, indexImu= 0, indexMag = 0;

    if (hasRangingMeasurements){
        countValid = rangingMeasurements.size();
    }

    if(hasPX4Measurement){
        indexPX4 = countValid;
        countValid+=3;
    }

    if (hasImuMeasurement){
        indexImu = countValid;
        countValid+=3;
    }

    if (hasMagMeasurement){
        indexMag = countValid;
        countValid+=1;
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


    if (hasPX4Measurement) {
        PX4FLOWOutput sensorPrediction = px4flowOutput(speed.x, speed.y, angle, angularSpeed, timeLag);
        output(indexPX4) = sensorPrediction.vX;
        output(indexPX4 + 1) = sensorPrediction.vY;
        output(indexPX4 + 2) = sensorPrediction.gyroZ;
    }

    if (hasImuMeasurement) {
        ImuOutput sensorPrediction = imuOutput(acceleration, angle, angularSpeed);
        output(indexImu) = sensorPrediction.accelX;
        output(indexImu + 1) = sensorPrediction.accelY;
        output(indexImu + 2) = sensorPrediction.gyroZ;
    }

    if (hasMagMeasurement) {
        output(indexMag) = angle;
    }
    
    return output;
}

KalmanFilter::PX4FLOWOutput KalmanFilter::px4flowOutput(double speedX, double speedY, double angle, double angularSpeed, double timeLag) const {
    PX4FLOWOutput output;

    output.vX = cos(angle) * speedX + sin(angle) * speedY + 1 / timeLag * ((1 - cos(angularSpeed * timeLag)) * mPX4FlowArmP1 - sin(angularSpeed * timeLag) * mPX4FlowArmP2);
    output.vY = -sin(angle) * speedX + cos(angle) * speedY + 1 / timeLag * (sin(angularSpeed * timeLag) * mPX4FlowArmP1 + (1 - cos(angularSpeed * timeLag)) * mPX4FlowArmP2);
    output.gyroZ = angularSpeed;

    return output;
}

KalmanFilter::ImuOutput KalmanFilter::imuOutput(const Vector3& acceleration, double angle, double angularSpeed) const{
    ImuOutput output;

    output.accelX = cos(angle)*acceleration.x + sin(angle)*acceleration.y;
    output.accelY = -sin(angle)*acceleration.x + cos(angle)*acceleration.y;
    output.gyroZ = angularSpeed;

    return output;
}

void KalmanFilter::predictionMatrix(arma::mat& matrix, double timeLag) const {
    matrix << 1 << 0 << timeLag << 0 << timeLag*timeLag/2 << 0 << 0 << 0 << arma::endr
           << 0 << 1<< 0 << timeLag << 0 << timeLag*timeLag/2 << 0 << 0 << arma::endr
           << 0 << 0 << 1 << 0 << timeLag << 0 << 0 << 0 << arma::endr
           << 0 << 0 << 0 << 1 << 0 << timeLag << 0 << 0 << arma::endr
           << 0 << 0 << 0 << 0 << 1 << 0 << 0 << 0 << arma::endr
           << 0 << 0 << 0 << 0 << 0 << 1 << 0 << 0 << arma::endr
           << 0 << 0 << 0 << 0 << 0 << 0 << 1 << timeLag << arma::endr
           << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 1 << arma::endr;
}

void KalmanFilter::predictionErrorCovariance(arma::mat& matrix, double timeLag) const {
    double t3 = pow(timeLag, 3)/6;
    double t2 = pow(timeLag, 2)/2;
    double t = timeLag;
    double a = mAccelerationNoise;
    double j = mJolt;
    // TODO: We wait that linear and angular accelerations are similar, this may be incorrect
    matrix << j*t3*t3 << 0 << j*t3*t2 << 0 << j*t3*t << 0 << 0 << 0 << arma::endr
           << 0 << j*t3*t3 << 0 << j*t3*t2 << 0 << j*t3*t << 0 << 0 << arma::endr
           << j*t3*t2 << 0 << j*t2*t2 << 0 << j*t2*t << 0 << 0 << 0 << arma::endr
           << 0 << j*t3*t2 << 0 << j*t2*t2 << 0 << j*t2*t << 0 << 0 << arma::endr
           << j*t3*t << 0 << j*t2*t << 0 << j*t*t << 0 << 0 << 0 << arma::endr 
           << 0 << j*t3*t << 0 << j*t2*t << 0 << j*t*t << 0 << 0 << arma::endr
           << 0 << 0 << 0 << 0 << 0 << 0 << a*t2*t2 << a*t2*t << arma::endr
           << 0 << 0 << 0 << 0 << 0 << 0 << a*t2*t << a*t*t << arma::endr;
}

void KalmanFilter::jacobianRangings(arma::mat& jacobian, const Vector3& position, const std::vector<RangingMeasurement>& rangingMeasurements, int indexStartRow) const {
    std::vector<double> distances = mlLocation->distanceToBeacons(position, rangingMeasurements);
    int i = 0;
    for (auto const& r : rangingMeasurements){
        jacobian(i+indexStartRow, 0) = (position.x - r.beacon.position.x) / distances[i];
        jacobian(i+indexStartRow, 1) = (position.y - r.beacon.position.y) / distances[i];
        jacobian(i+indexStartRow, 2) = 0;
        jacobian(i+indexStartRow, 3) = 0;
        jacobian(i+indexStartRow, 4) = 0;
        jacobian(i+indexStartRow, 5) = 0;
        jacobian(i+indexStartRow, 6) = 0;
        jacobian(i+indexStartRow, 7) = 0;
        i++;
    }
}

void KalmanFilter::jacobianPx4flow(arma::mat& jacobian, const Vector3& velocity, const double angle, const double angularSpeed, const double timeLag, int indexStartRow) const {
    jacobian(indexStartRow, 0) = 0;
    jacobian(indexStartRow, 1) = 0;
    jacobian(indexStartRow, 2) = cos(angle);
    jacobian(indexStartRow, 3) = sin(angle);
    jacobian(indexStartRow, 4) = 0;
    jacobian(indexStartRow, 5) = 0;
    jacobian(indexStartRow, 6) = -sin(angle) * velocity.x + cos(angle) * velocity.y;
    jacobian(indexStartRow, 7) = mPX4FlowArmP1 * sin(angularSpeed * timeLag) - mPX4FlowArmP2 * cos(angularSpeed * timeLag);

    jacobian(indexStartRow +1, 0) = 0;
    jacobian(indexStartRow +1, 1) = 0;
    jacobian(indexStartRow +1, 2) = -sin(angle);
    jacobian(indexStartRow +1, 3) = cos(angle);
    jacobian(indexStartRow +1, 4) = 0;
    jacobian(indexStartRow +1, 5) = 0;
    jacobian(indexStartRow +1, 6) =  -cos(angle) * velocity.x - sin(angle) * velocity.y;
    jacobian(indexStartRow +1, 7) = mPX4FlowArmP1 * cos(angularSpeed * timeLag) + mPX4FlowArmP2 * sin(angularSpeed * timeLag);

    jacobian(indexStartRow +2, 0) = 0;
    jacobian(indexStartRow +2, 1) = 0;
    jacobian(indexStartRow +2, 2) = 0;
    jacobian(indexStartRow +2, 3) = 0;
    jacobian(indexStartRow +2, 4) = 0;
    jacobian(indexStartRow +2, 5) = 0;
    jacobian(indexStartRow +2, 6) = 0;
    jacobian(indexStartRow +2, 7) = 1; //<-?
}

void KalmanFilter::jacobianImu(arma::mat& jacobian, const Vector3& acceleration, double angle, double angularSpeed, int indexStartRow) const {

    jacobian(indexStartRow, 0) = 0;
    jacobian(indexStartRow, 1) = 0;
    jacobian(indexStartRow, 2) = 0;
    jacobian(indexStartRow, 3) = 0;
    jacobian(indexStartRow, 4) = cos(angle);
    jacobian(indexStartRow, 5) = sin(angle);
    jacobian(indexStartRow, 6) = -sin(angle)*acceleration.x + cos(angle)*acceleration.y;
    jacobian(indexStartRow, 7) = 0;
    
    jacobian(indexStartRow +1, 0) = 0;
    jacobian(indexStartRow +1, 1) = 0;
    jacobian(indexStartRow +1, 2) = 0;
    jacobian(indexStartRow +1, 3) = 0;
    jacobian(indexStartRow +1, 4) = -sin(angle);
    jacobian(indexStartRow +1, 5) = cos(angle);
    jacobian(indexStartRow +1, 6) = -cos(angle)*acceleration.x - sin(angle)*acceleration.y;
    jacobian(indexStartRow +1, 7) = 0;
    
    jacobian(indexStartRow +2, 0) = 0;
    jacobian(indexStartRow +2, 1) = 0;
    jacobian(indexStartRow +2, 2) = 0;
    jacobian(indexStartRow +2, 3) = 0;
    jacobian(indexStartRow +2, 4) = 0;
    jacobian(indexStartRow +2, 5) = 0;
    jacobian(indexStartRow +2, 6) = 0;
    jacobian(indexStartRow +2, 7) = 1; //<-?

}


void KalmanFilter::jacobianMag(arma::mat& jacobian, int indexStartRow) const {
    jacobian(indexStartRow , 0) = 0;
    jacobian(indexStartRow , 1) = 0;
    jacobian(indexStartRow , 2) = 0;
    jacobian(indexStartRow , 3) = 0;
    jacobian(indexStartRow , 4) = 0;
    jacobian(indexStartRow , 5) = 0;
    jacobian(indexStartRow , 6) = 1;
    jacobian(indexStartRow , 7) = 0;
}

double KalmanFilter::normalizeAngle(double angle){
    if (angle>M_PI){
        return angle - 2*M_PI;
    } else if (angle<=-M_PI){
        return angle + 2*M_PI;
    }
    return angle;
}


bool KalmanFilter::getPose(Vector3& pose) {
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
    
    arma::vec state = { mPosition.x, mPosition.y,
                        mVelocity.x, mVelocity.y,
                        mAcceleration.x, mAcceleration.y,
                        mAngle, mAngularSpeed
                      };

    // Prediction
    arma::mat predictionStep(8, 8);
    arma::mat predictionCovariance(8, 8);

    predictionMatrix(predictionStep, timeLag);
    predictionErrorCovariance(predictionCovariance, timeLag);
    arma::vec predictedState = predictionStep * state;

    arma::mat predictedEstimationCovariance = predictionStep * mEstimationCovariance * predictionStep.t() +
                           predictionCovariance;


    predictedState(6) = normalizeAngle(predictedState(6));

    stateToPose(pose, predictedState, predictedEstimationCovariance);
    return true;
}

bool KalmanFilter::loadConfigurationFiles(std::string filenamePos, std::string filenamePX4Flow, std::string filenameTag, std::string filenameImu, std::string filenameMag) {

  try {

    //************************************
    // PX4Flow
    //************************************

    boost::property_tree::ptree configTreePX4Flow;

    ros::NodeHandle node_handlePX4Flow("~");
    std::string configContentPX4Flow;
    node_handlePX4Flow.getParam(filenamePX4Flow, configContentPX4Flow);
    std::stringstream ssPX4Flow;
    ssPX4Flow << configContentPX4Flow;
    boost::property_tree::read_xml(ssPX4Flow, configTreePX4Flow);

    BOOST_FOREACH(const boost::property_tree::ptree::value_type & v, configTreePX4Flow.get_child("config")) {
      if (v.first.compare("px4flow") == 0) {
        int useFH = v.second.get<int>("<xmlattr>.useFixedSensorHeight", 0);
        mUseFixedHeightPX4Flow = (useFH == 1);

        mPX4flowHeight = v.second.get<double>("<xmlattr>.sensorHeight", 0);
        mPX4FlowArmP1 = v.second.get<double>("<xmlattr>.armP0", 0);
        mPX4FlowArmP2 = v.second.get<double>("<xmlattr>.armP1", 0);
        mInitAnglePX4Flow = v.second.get<double>("<xmlattr>.sensorInitAngle", 0);

        mCovarianceVelocityPX4Flow = v.second.get<double>("<xmlattr>.covarianceVelocity", 0);
        mCovarianceGyroZPX4Flow = v.second.get<double>("<xmlattr>.covarianceGyroZ", 0);
      }
    }

    //************************************
    // UWB
    //************************************

    boost::property_tree::ptree configTreeTag;
    ros::NodeHandle node_handleTag("~");
    std::string configContentTag;
    node_handleTag.getParam(filenameTag, configContentTag);
    std::stringstream ssTag;
    ssTag << configContentTag;
    boost::property_tree::read_xml(ssTag, configTreeTag);

    BOOST_FOREACH(const boost::property_tree::ptree::value_type & v, configTreeTag.get_child("config")) {
      if (v.first.compare("uwb") == 0) {
        int use = v.second.get<int>("<xmlattr>.useFixedHeight", 0);
        mUseFixedHeight = (use == 1);
        mUWBtagZ = v.second.get<double>("<xmlattr>.fixedHeight", 0);
        mTagIdUWB = v.second.get<int>("<xmlattr>.tagId", 0);
      }
    }


    //************************************
    // IMU
    //************************************

    boost::property_tree::ptree configTreeImu;
    ros::NodeHandle node_handleImu("~");
    std::string configContentImu;
    node_handleImu.getParam(filenameImu, configContentImu);
    std::stringstream ssImu;
    ssImu << configContentImu;
    boost::property_tree::read_xml(ssImu, configTreeImu);

    BOOST_FOREACH(const boost::property_tree::ptree::value_type & v, configTreeImu.get_child("config")) {
      if (v.first.compare("imu") == 0) {
        int useFCA = v.second.get<int>("<xmlattr>.useFixedCovarianceAcceleration", 0);
        mUseImuFixedCovarianceAcceleration = (useFCA == 1);
        mImuCovarianceAcceleration = v.second.get<double>("<xmlattr>.covarianceAcceleration", 0);
        int useFCAVZ = v.second.get<int>("<xmlattr>.useFixedCovarianceAngularVelocityZ", 0);
        mUseImuFixedCovarianceAngularVelocityZ = (useFCAVZ == 1);
        mUmuCovarianceAngularVelocityZ = v.second.get<double>("<xmlattr>.covarianceAngularVelocityZ", 0);
      }
    }


    //************************************
    // Mag
    //************************************

    boost::property_tree::ptree configTreeMag;
    ros::NodeHandle node_handleMag("~");
    std::string configContentMag;
    node_handleMag.getParam(filenameMag, configContentMag);
    std::stringstream ssMag;
    ssMag << configContentMag;
    boost::property_tree::read_xml(ssMag, configTreeMag);

    BOOST_FOREACH(const boost::property_tree::ptree::value_type & v, configTreeMag.get_child("config")) {
      if (v.first.compare("mag") == 0) {
        mMagAngleOffset = v.second.get<double>("<xmlattr>.angleOffset", 0);
        mCovarianceMag = v.second.get<double>("<xmlattr>.covarianceMag", 0);
      }
    }


    //************************************
    // Position
    //************************************

    ROS_INFO("POSGEN: Intentando cargar propiedad %s", filenamePos.c_str());
    boost::property_tree::ptree configTreePos;
    ros::NodeHandle node_handlePos("~");
    std::string configContentPos;
    node_handlePos.getParam(filenamePos, configContentPos);
    std::stringstream ssPos;
    ssPos << configContentPos;
    boost::property_tree::read_xml(ssPos, configTreePos);


    BOOST_FOREACH(const boost::property_tree::ptree::value_type & v, configTreePos.get_child("config")) {
      if (v.first.compare("algorithm") == 0) {

/*        int type = v.second.get<int>("<xmlattr>.type", 0);
        int variant = v.second.get<int>("<xmlattr>.variant", 0);
        int numIgnoredRangings = v.second.get<int>("<xmlattr>.numIgnoredRangings", 0);
        int bestMode = v.second.get<int>("<xmlattr>.bestMode", 0);
        double minZ = v.second.get<double>("<xmlattr>.minZ", 0);
        double maxZ = v.second.get<double>("<xmlattr>.maxZ", 0);

        if (variant == 0) {
          setVariantNormal();
          ROS_INFO("POSGEN: Variant NORMAL");
        } else if (variant == 1) {
          setVariantIgnoreN(numIgnoredRangings);
          ROS_INFO("POSGEN: Variant IGNORE N. Ignored: %d", numIgnoredRangings);
        } else  if (variant == 2) {
          setVariantOnlyBest(bestMode, minZ, maxZ);
          ROS_INFO("POSGEN: Variant Best N:. BestMode: %d minZ: %f mazZ: %f", bestMode, minZ, maxZ);
        }*/

      }
    }

    return true;

  } catch (boost::exception const &ex) {
    return false;
  }


  return false;
}