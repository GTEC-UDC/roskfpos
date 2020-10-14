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
    mAngles = {0, 0,0};
    mAngularVelocity = {0,0,0};



    mHasImuMeasurement=false;

    mEstimationCovariance.zeros(15, 15);
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
    mAngles = {0,0,0};
    mAngularVelocity = {0,0,0};

    mHasImuMeasurement=false;

    mEstimationCovariance.zeros(15, 15);
    mLastKFTimestamp =  std::chrono::steady_clock::time_point::min();
}


bool KalmanFilterTOAIMU::init(){


    return true;
}


void KalmanFilterTOAIMU::newTOAMeasurement(const std::vector<double>& rangings,
        const std::vector<Beacon>& beacons, const std::vector<double>& errorEstimations, double timeLag) {

ROS_INFO("KalmanFilterTOAIMU new TOA");
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
   
    ROS_INFO("KalmanFilterTOAIMU new IMU");

    ImuMeasurement3D measurement;
    measurement.linearAcceleration.x = linearAcceleration.x;
    measurement.linearAcceleration.y = linearAcceleration.y;
    measurement.linearAcceleration.z = linearAcceleration.z;

    measurement.angularVelocity.x = angularVelocity.x;
    measurement.angularVelocity.y = angularVelocity.y;
    measurement.angularVelocity.z = angularVelocity.z;

    for(int i=0;i<9;i++){
        measurement.covarianceLinearAccelerationXYZ[i] = covarianceAcceleration[i];
        measurement.covarianceAngularVelocityXYZ[i] = covarianceAngularVelocity[i];
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
        if (std::isnan(mPosition.x) || std::isnan(mPosition.y)  || std::isnan(mPosition.z)) {
            ROS_INFO("KalmanFilter initPosition is NAN"); 
            if (hasRangingMeasurements) {
                ROS_INFO("KalmanFilter Ranging mode"); 
                Vector3 defaultPos = { 1.0, 1.0, 4.0 }; 
                mPosition = mlLocation->estimatePosition(rangingMeasurements, defaultPos);
                ROS_INFO("KalmanFilter new mPosition [%f %f %f]", mPosition.x, mPosition.y, mPosition.z); 

                if (mPosition.x!=defaultPos.x || mPosition.y!=defaultPos.y || mPosition.z!=defaultPos.z){
                mEstimationCovariance(0,0) = mPosition.covarianceMatrix(0,0);
                mEstimationCovariance(1,0) = mPosition.covarianceMatrix(1,0);
                mEstimationCovariance(0,1) = mPosition.covarianceMatrix(0,1);
                mEstimationCovariance(1,1) = mPosition.covarianceMatrix(1,1);

                mPosition.covarianceMatrix = arma::eye<arma::mat>(6, 6) * 0.01;

/*                mPosition.covarianceMatrix(0,0) = mPosition.covarianceMatrix(0,0);
                mPosition.covarianceMatrix(0,1) = mPosition.covarianceMatrix(0,1);
                mPosition.covarianceMatrix(0,2) = mPosition.covarianceMatrix(0,2);
                mPosition.covarianceMatrix(1,0) = mPosition.covarianceMatrix(1,0);
                mPosition.covarianceMatrix(1,1) = mPosition.covarianceMatrix(1,1);
                mPosition.covarianceMatrix(1,2) = mPosition.covarianceMatrix(1,2);
                mPosition.covarianceMatrix(2,0) = mPosition.covarianceMatrix(2,0);
                mPosition.covarianceMatrix(2,1) = mPosition.covarianceMatrix(2,1);
                mPosition.covarianceMatrix(2,2) = mPosition.covarianceMatrix(2,2);*/

                }



                ROS_INFO("KalmanFilterTOAIMU END new Position ML"); 

            }
            return;
        }
    }

    arma::vec state = { mPosition.x, mPosition.y, mPosition.z,
                        mVelocity.x, mVelocity.y, mVelocity.z,
                        mAcceleration.x, mAcceleration.y, mAcceleration.z,
                        mAngles.x, mAngles.y, mAngles.z,
                        mAngularVelocity.x, mAngularVelocity.y, mAngularVelocity.z
                      };

    // Prediction
    arma::mat predictionStep(15, 15);
    arma::mat predictionCovariance(15, 15);

    
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
    mAcceleration.x = state(6);
    mAcceleration.y = state(7);
    mAcceleration.z = state(8);
    mAngles.x = state(9);
    mAngles.y = state(10);
    mAngles.z = state(11);
    mAngularVelocity.x = state(12);
    mAngularVelocity.y = state(13);
    mAngularVelocity.z = state(14);

    //Position is set within the next function
    stateToPose(mPosition, state, mEstimationCovariance);
}


void KalmanFilterTOAIMU::stateToPose(Vector3& pose, const arma::vec& state, const arma::mat& estimationCovariance){

    pose.x = state(0);
    pose.y = state(1);
    pose.z = state(2);

    Quaternion quat = ToQuaternion(state(9), state(10), state(11));

    pose.rotX = quat.x;
    pose.rotY = quat.y;
    pose.rotZ = quat.z;
    pose.rotW = quat.w;

    pose.linearSpeedX = state(3);
    pose.linearSpeedY = state(4);
    pose.linearSpeedZ = state(5);

    pose.angularSpeedX = state(12);
    pose.angularSpeedY = state(13);
    pose.angularSpeedZ = state(14);

    pose.covarianceMatrix = arma::eye<arma::mat>(6, 6) * 0.01;

    pose.covarianceMatrix(0,0) =  estimationCovariance(0, 0);
    pose.covarianceMatrix(0,1) =  estimationCovariance(0, 1);
    pose.covarianceMatrix(0,2) =  estimationCovariance(0, 2);

    pose.covarianceMatrix(1,0) =  estimationCovariance(1, 0);
    pose.covarianceMatrix(1,1) =  estimationCovariance(1, 1);
    pose.covarianceMatrix(1,2) =  estimationCovariance(1, 2);

    pose.covarianceMatrix(2,0) =  estimationCovariance(2, 0);
    pose.covarianceMatrix(2,1) =  estimationCovariance(2, 1);
    pose.covarianceMatrix(2,2) =  estimationCovariance(2, 2);

    pose.covarianceMatrix(3,0) = estimationCovariance(9, 9);
    pose.covarianceMatrix(3,1) = estimationCovariance(9, 10);
    pose.covarianceMatrix(3,2) = estimationCovariance(9, 11);

    pose.covarianceMatrix(4,0) = estimationCovariance(10, 9);
    pose.covarianceMatrix(4,1) = estimationCovariance(10, 10);
    pose.covarianceMatrix(4,2) = estimationCovariance(10, 11);

    pose.covarianceMatrix(5,0) = estimationCovariance(11, 9);
    pose.covarianceMatrix(5,1) = estimationCovariance(11, 10);
    pose.covarianceMatrix(5,2) = estimationCovariance(11, 11);
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
        countValid+=6;
    }

    ROS_INFO("Count valid %d", countValid);
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
        ROS_INFO("Has imu measurement");
        measurements(indexImu) = imuMeasurement.linearAcceleration.x;
        measurements(indexImu+1) = imuMeasurement.linearAcceleration.y;
        measurements(indexImu+2) = imuMeasurement.linearAcceleration.z;

        measurements(indexImu+3) = imuMeasurement.angularVelocity.x;
        measurements(indexImu+4) = imuMeasurement.angularVelocity.y;
        measurements(indexImu+5) = imuMeasurement.angularVelocity.z;

        observationCovariance(indexImu, indexImu) = imuMeasurement.covarianceLinearAccelerationXYZ[0];
        observationCovariance(indexImu, indexImu+1) = imuMeasurement.covarianceLinearAccelerationXYZ[1];
        observationCovariance(indexImu, indexImu+2) = imuMeasurement.covarianceLinearAccelerationXYZ[2];
        observationCovariance(indexImu+1, indexImu) = imuMeasurement.covarianceLinearAccelerationXYZ[3];
        observationCovariance(indexImu+1, indexImu+1) = imuMeasurement.covarianceLinearAccelerationXYZ[4];
        observationCovariance(indexImu+1, indexImu+2) = imuMeasurement.covarianceLinearAccelerationXYZ[5];
        observationCovariance(indexImu+2, indexImu) = imuMeasurement.covarianceLinearAccelerationXYZ[6];
        observationCovariance(indexImu+2, indexImu+1) = imuMeasurement.covarianceLinearAccelerationXYZ[7];
        observationCovariance(indexImu+2, indexImu+2) = imuMeasurement.covarianceLinearAccelerationXYZ[8];

        observationCovariance(indexImu+3, indexImu+3) = imuMeasurement.covarianceAngularVelocityXYZ[0];
        observationCovariance(indexImu+3, indexImu+4) = imuMeasurement.covarianceAngularVelocityXYZ[1];
        observationCovariance(indexImu+3, indexImu+5) = imuMeasurement.covarianceAngularVelocityXYZ[2];
        observationCovariance(indexImu+4, indexImu+3) = imuMeasurement.covarianceAngularVelocityXYZ[3];
        observationCovariance(indexImu+4, indexImu+4) = imuMeasurement.covarianceAngularVelocityXYZ[4];
        observationCovariance(indexImu+4, indexImu+5) = imuMeasurement.covarianceAngularVelocityXYZ[5];
        observationCovariance(indexImu+5, indexImu+3) = imuMeasurement.covarianceAngularVelocityXYZ[6];
        observationCovariance(indexImu+5, indexImu+4) = imuMeasurement.covarianceAngularVelocityXYZ[7];
        observationCovariance(indexImu+5, indexImu+5) = imuMeasurement.covarianceAngularVelocityXYZ[8];
    }

    //observationCovariance.print();
    arma::mat jacobian(countValid, 15);
    arma::mat kalmanGain;
    arma::mat invObsCovariance = arma::inv(observationCovariance);
    arma::mat invEstCovariance = arma::pinv(mEstimationCovariance);

    double cost = 1e20;
    for (int iter = 0; iter < maxSteps; iter++) {
        Vector3 currentPosition = { state(0), state(1), state(2) };
        Vector3 currentSpeed = { state(3), state(4), state(5) };
        Vector3 currentAcceleration = { state(6), state(7), state(8) };
        Vector3 currentAngles = { state(9), state(10), state(11) };
        Vector3 currentAngularVelocity = { state(12), state(13), state(14)};

        ROS_INFO("SensorsOutput");
        
        arma::vec output = sensorOutputs(currentPosition, currentSpeed, currentAcceleration, currentAngles, currentAngularVelocity, timeLag,
                      hasRangingMeasurements, hasImuMeasurement,rangingMeasurements);
        arma::vec predictionError = measurements - output;

        ROS_INFO("After sensorOutputs");

        arma::vec predictionDiff = predictedState - state;
        arma::vec costMat = predictionError.t() * invObsCovariance * predictionError
                    + predictionDiff.t() * invEstCovariance * predictionDiff;

        double newCost = costMat(0);           

        ROS_INFO("New cost: %f", newCost);

        if (std::abs(cost - newCost) / cost < minRelativeError) {
            break;
        }
        cost = newCost;

        if (hasRangingMeasurements) {
            jacobianRangings(jacobian, currentPosition, rangingMeasurements, indexRanging);
        }

        ROS_INFO("After jacobianRangings");

        if (hasImuMeasurement) {
            jacobianImu(jacobian, currentAcceleration, currentAngularVelocity, indexImu);
        }

        ROS_INFO("After jacobianImu");

        kalmanGain = mEstimationCovariance * jacobian.t() *
                           inv(jacobian * mEstimationCovariance * jacobian.t() + observationCovariance);


        arma::vec direction = predictionDiff + kalmanGain * (predictionError - jacobian*predictionDiff);
        state = state + direction;
    }

    mEstimationCovariance = (arma::eye<arma::mat>(15, 15) - kalmanGain * jacobian) * mEstimationCovariance;
    return state;
}




arma::vec KalmanFilterTOAIMU::sensorOutputs(const Vector3& currentPosition, const Vector3& currentLinearSpeed,const Vector3& currentAcceleration,
    const Vector3 currentAngles, const Vector3 currentAngularVelocity, double timeLag,  bool hasRangingMeasurements, 
    bool hasImuMeasurement,const std::vector<RangingMeasurement>& rangingMeasurements) const {
    int countValid = 0;
    int indexImu= 0, indexRanging= 0;

    if (hasRangingMeasurements){
        countValid = rangingMeasurements.size();
    }

    if (hasImuMeasurement){
        indexImu = countValid;
        countValid+=6;
    }
    
    arma::vec output(countValid);
    if (hasRangingMeasurements) {
        std::vector<double> distances = mlLocation->distanceToBeacons(currentPosition, rangingMeasurements);
        int v = 0;
        for (const auto& r : rangingMeasurements) {
            output(v + indexRanging) = distances[v];
            v++;
        }
    }

    if (hasImuMeasurement) {
        ImuOutput3D sensorPrediction = imuOutput(currentAcceleration, currentAngularVelocity, currentAngles, timeLag);
        output(indexImu) = sensorPrediction.linearAcceleration.x;
        output(indexImu + 1) = sensorPrediction.linearAcceleration.y;
        output(indexImu + 2) = sensorPrediction.linearAcceleration.z;
        output(indexImu + 3) = sensorPrediction.angularVelocity.x;
        output(indexImu + 4) = sensorPrediction.angularVelocity.y;
        output(indexImu + 5) = sensorPrediction.angularVelocity.z;
    }

    
    return output;
}


KalmanFilterTOAIMU::ImuOutput3D KalmanFilterTOAIMU::imuOutput(const Vector3& acceleration, const Vector3& angularVelocity, const Vector3 angles, double timeLag) const{
    ImuOutput3D output;
    arma::mat rotationMatrix;

    ROS_INFO("ImuOutput");

    rotationMatrix << cos(mAngles.x)*cos(mAngles.y) << cos(mAngles.x)*sin(mAngles.y)*sin(mAngles.z) - sin(mAngles.x)*cos(mAngles.z) <<  cos(mAngles.x)*sin(mAngles.y)*cos(mAngles.z) + sin(mAngles.x)*sin(mAngles.z) << arma::endr
                   << sin(mAngles.x)*cos(mAngles.y) << sin(mAngles.x)*sin(mAngles.y)*sin(mAngles.z) + cos(mAngles.x)*cos(mAngles.z) <<  sin(mAngles.x)*sin(mAngles.y)*cos(mAngles.z) - cos(mAngles.x)*sin(mAngles.z) << arma::endr
                   << -sin(mAngles.y) << cos(mAngles.y)*sin(mAngles.z) <<  cos(mAngles.y)*cos(mAngles.z) << arma::endr;

    arma::mat invRotationMatrix = arma::inv(rotationMatrix);

    arma::vec accel({acceleration.x, acceleration.y, acceleration.z});
    arma::vec angVel({angularVelocity.x, angularVelocity.y, angularVelocity.z});

    arma::mat accelTag = invRotationMatrix*accel;
    arma::mat velTag = invRotationMatrix*angVel;

    output.linearAcceleration.x = accelTag(0);
    output.linearAcceleration.y = accelTag(1);
    output.linearAcceleration.z = accelTag(2);

    output.angularVelocity.x = angVel(0);
    output.angularVelocity.y = angVel(1);
    output.angularVelocity.z = angVel(2);

     ROS_INFO("ImuOutput END");

    return output;
}


void KalmanFilterTOAIMU::predictionMatrix(arma::mat& matrix, double timeLag) const {
    //<< x << y << z << vx << vy << vz << ax << ay << az << an_x << an_y << an_z << va_x << va_y << va_z 


    matrix = {{1,0,0,timeLag,0,0, timeLag*timeLag/2,0,0,0,0,0,0,0,0},
              {0,1,0,0,timeLag,0, 0,timeLag*timeLag/2,0,0,0,0,0,0,0},
              {0,0,1,0,0,timeLag, 0,0,timeLag*timeLag/2,0,0,0,0,0,0},
              {0,0,0,1,0,0,timeLag,0,0,0,0,0,0,0,0},
              {0,0,0,0,1,0,0,timeLag,0,0,0,0,0,0,0},
              {0,0,0,0,0,1,0,0,timeLag,0,0,0,0,0,0},
              {0,0,0,0,0,0,1,0,0,0,0,0,0,0,0},
              {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0},
              {0,0,0,0,0,0,0,0,1,0,0,0,0,0,0},
              {0,0,0,0,0,0,0,0,0,1,0,0,timeLag,0,0},
              {0,0,0,0,0,0,0,0,0,0,1,0,0,timeLag,0},
              {0,0,0,0,0,0,0,0,0,0,0,1,0,0,timeLag},
              {0,0,0,0,0,0,0,0,0,0,0,0,1,0,0},
              {0,0,0,0,0,0,0,0,0,0,0,0,0,1,0},
              {0,0,0,0,0,0,0,0,0,0,0,0,0,0,1}};

/*    matrix << 1 << 0 << 0   << timeLag << 0       << 0 << timeLag*timeLag/2 << 0 << 0  << 0 << 0 << 0 << 0 << 0 << 0<< arma::endr
           << 0 << 1 << 0   << 0       << timeLag << 0 << 0                 << timeLag*timeLag/2 << 0  << 0 << 0 << 0 << 0 << 0 << 0<< arma::endr


           << 0 << 0 << 0 << 1 << 0 << 0 << timeLag << 0 << 0 << arma::endr
           << 0 << 0 << 0 << 0 << 1 << 0 << 0 << timeLag << 0 << arma::endr
           << 0 << 0 << 0 << 0 << 0 << 1 << 0 << 0 << timeLag << arma::endr
           << 0 << 0 << 0 << 0 << 0 << 0 << 1 << 0 << 0 << arma::endr
           << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 1 << 0 << arma::endr
           << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 1 << arma::endr;*/
}


void KalmanFilterTOAIMU::predictionErrorCovariance(arma::mat& matrix, double timeLag) const {
    double t3 = pow(timeLag, 3)/6;
    double t2 = pow(timeLag, 2)/2;
    double t = timeLag;
    double a = mAccelerationNoise;
    double j = mJolt;

    matrix.zeros(); 
    matrix.eye(); 
    matrix = matrix*mJolt*t3*t3;
/*    matrix << j*t3*t3 << 0 << 0 << j*t3*t2 << 0 << 0 << j*t3*t << 0 << 0 << arma::endr
           << 0 << j*t3*t3 << 0 << 0 << j*t3*t2 << 0 << 0 << j*t3*t << 0 << arma::endr
           << 0 << 0 << j*t3*t3 << 0 << 0 << j*t3*t2 << 0 << 0 << j*t3*t << arma::endr
           << j*t3*t2 << 0 << 0 << j*t2*t2 << 0 << 0 << j*t2*t << 0 << 0 << arma::endr
           << 0 << j*t3*t2 << 0 << 0 << j*t2*t2 << 0 << 0 << j*t2*t << 0  << arma::endr
           << 0 << 0 << j*t3*t2 << 0 << 0 << j*t2*t2 << 0 << 0 << j*t2*t  << arma::endr
           << j*t3*t << 0 << 0 << j*t2*t << 0 << 0 << j*t*t << 0 << 0 << arma::endr 
           << 0 << j*t3*t << 0 << 0 << j*t2*t << 0 << 0 << j*t*t << 0 << arma::endr
           << 0 << 0 << j*t3*t << 0 << 0 << j*t2*t << 0 << 0 << j*t*t << arma::endr;*/
}

void KalmanFilterTOAIMU::jacobianRangings(arma::mat& jacobian, const Vector3& position, const std::vector<RangingMeasurement>& rangingMeasurements, int indexStartRow) const {
    std::vector<double> distances = mlLocation->distanceToBeacons(position, rangingMeasurements);
    int i = 0;
    for (auto const& r : rangingMeasurements){
        jacobian.row(i+indexStartRow) = arma::vec({(position.x - r.beacon.position.x) / distances[i],
            (position.y - r.beacon.position.y) / distances[i],
            (position.z - r.beacon.position.z) / distances[i],0,0,0,0,0,0,0,0,0,0,0,0}).t();
        i++;
    }
}


void KalmanFilterTOAIMU::jacobianImu(arma::mat& jacobian, const Vector3& acceleration, const Vector3& angularVelocity, int indexStartRow) const {
    jacobian.row(indexStartRow) = arma::vec({0,0,0,0,0,0,acceleration.x,0,0,0,0,0,0,0,0}).t();
    jacobian.row(indexStartRow+1) = arma::vec({0,0,0,0,0,0,0,acceleration.y,0,0,0,0,0,0,0}).t();
    jacobian.row(indexStartRow+2) = arma::vec({0,0,0,0,0,0,0,0,acceleration.z,0,0,0,0,0,0}).t();

    jacobian.row(indexStartRow+3) = arma::vec({0,0,0,0,0,0,0,0,0,0,0,0,angularVelocity.x,0,0}).t();
    jacobian.row(indexStartRow+4) = arma::vec({0,0,0,0,0,0,0,0,0,0,0,0,0,angularVelocity.y,0}).t();
    jacobian.row(indexStartRow+5) = arma::vec({0,0,0,0,0,0,0,0,0,0,0,0,0,0,angularVelocity.z}).t();
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
                        mAngles.x, mAngles.y, mAngles.z,
                        mAngularVelocity.x, mAngularVelocity.y, mAngularVelocity.z
                      };

    ROS_INFO("KalmanFilterTOAIMU get Pose");
    // Prediction
    arma::mat predictionStep(15, 15);
    arma::mat predictionCovariance(15, 15);

    predictionMatrix(predictionStep, timeLag);
    predictionErrorCovariance(predictionCovariance, timeLag);
    arma::vec predictedState = predictionStep * state;

    arma::mat predictedEstimationCovariance = predictionStep * mEstimationCovariance * predictionStep.t() +
                           predictionCovariance;

    stateToPose(pose, predictedState, predictedEstimationCovariance);
    return true;
}


KalmanFilterTOAIMU::Quaternion KalmanFilterTOAIMU::ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaternion q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;

    return q;
}