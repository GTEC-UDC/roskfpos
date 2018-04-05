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
#include "KalmanFilter.h"


KalmanFilter::KalmanFilter(double accelerationNoise, double uwbTagZ, double px4FlowArmP1, double px4FlowArmP2, Vector3 initialPosition, double px4flowHeight, double initialAngle,double magAngleOffset, double jolt) :
    mAccelerationNoise(accelerationNoise),
    mAngularSpeed(0.0),
    mUWBtagZ(uwbTagZ),
    mPX4FlowArmP1(px4FlowArmP1),
    mPX4FlowArmP2(px4FlowArmP2),
    mPX4flowHeight(px4flowHeight),
    mPosition(initialPosition),
    mAngle(initialAngle),
    mUseFixedInitialPosition(true),
    mMagAngleOffset(magAngleOffset),
    mJolt(jolt)
{
    MLLocation *mlLocation = new MLLocation();

    mVelocity = { 0, 0, 0};
    mAcceleration = { 0, 0, 0};

     mHasMagMeasurement=false;
     mHasPX4FlowMeasurement=false;
     mHasImuMeasurement=false;
     mHasUwbMeasurement = false;

    estimationCovariance.zeros(8, 8);
    mLastKFTimestamp =  std::chrono::steady_clock::time_point::min();
    ROS_INFO("KalmanFilter constructor end wit FixedInitialPosition = TRUE");
}


KalmanFilter::KalmanFilter(double accelerationNoise, double uwbTagZ, double px4FlowArmP1, double px4FlowArmP2, double px4flowHeight, double initialAngle,double magAngleOffset, double jolt) :
    mAccelerationNoise(accelerationNoise),
    mAngularSpeed(0.0),
    mUWBtagZ(uwbTagZ),
    mPX4FlowArmP1(px4FlowArmP1),
    mPX4FlowArmP2(px4FlowArmP2),
    mPX4flowHeight(px4flowHeight),
    mAngle(initialAngle),
    mUseFixedInitialPosition(false),
    mMagAngleOffset(magAngleOffset),
    mJolt(jolt)
{
    MLLocation *mlLocation = new MLLocation();

    mPosition = { NAN, NAN, NAN};
    mVelocity = { 0, 0, 0};
    mAcceleration = { 0, 0, 0};

    mHasMagMeasurement=false;
    mHasPX4FlowMeasurement=false;
    mHasImuMeasurement=false;
    mHasUwbMeasurement = false;

    estimationCovariance.zeros(8, 8);
    mLastKFTimestamp =  std::chrono::steady_clock::time_point::min();
    ROS_INFO("KalmanFilter constructor end wit FixedInitialPosition = FALSE");
}


void KalmanFilter::newUWBMeasurement(const std::vector<double>& rangings,
        const std::vector<Beacon>& beacons, const std::vector<double>& errorEstimations, double timeLag) {

    //std::vector<RangingMeasurement> measurements;

    lastUwbMeasurements.clear();
    for (int i = 0; i < rangings.size(); ++i)
    {
        if (rangings[i]>0){
            RangingMeasurement measurement;
            measurement.ranging = rangings[i];
            measurement.errorEstimation = errorEstimations[i];
            measurement.beacon = beacons[i];
            lastUwbMeasurements.push_back(measurement);
        }
    }
    mHasUwbMeasurement = true;

    // ErleImuMeasurement aImuMeasurement;
    // PX4FlowMeasurement aPX4Measurement;
    // ErleMagMeasurement aMagMeasurement;

    // if (mHasMagMeasurement){
    //     aMagMeasurement = lastErleMagMeasurement;    
    // }

    // if (mHasImuMeasurement){
    //     aImuMeasurement = lastErleImuMeasurement;
    // }

    // if (mHasPX4FlowMeasurement){
    //     aPX4Measurement = lastPX4FlowMeasurement;
    // }

   // estimatePositionKF(true, measurements , mHasPX4FlowMeasurement, aPX4Measurement, mHasImuMeasurement, aImuMeasurement, mHasMagMeasurement, aMagMeasurement);
}


void KalmanFilter::newPX4FlowMeasurement(double integrationX, double integrationY, double integrationRotationZ, double integrationTime, double covarianceVelocity, double covarianceGyroZ, int quality) {

    //ROS_INFO("KalmanFilter newPX4FlowMeasurement 444444444444");
    //TODO: comprobar lo siguiente

    PX4FlowMeasurement measurement;

    measurement.vy= integrationY/(integrationTime/1000000.0)*mPX4flowHeight;
    measurement.vx= integrationX/(integrationTime/1000000.0)*mPX4flowHeight;


    //ROS_INFO("KalmanFilter PX4Flow vx: %f vy: %f quality: %d",measurement.vx, measurement.vy, quality); 


    measurement.gyroz = integrationRotationZ/(integrationTime/1000000.0);
    //measurement.gyroz = integrationRotationZ;

    measurement.integrationTime = integrationTime/1000000.0;


    //Ya filtramos en Posgenerator las medidas con quality =0
    //if (quality>0){
    

    //Modificar varianza para que tenga en cuanta el integrationTime

    if (integrationTime>0){
       measurement.covarianceVelocity = covarianceVelocity/measurement.integrationTime*mPX4flowHeight/quality;  
   } else {
    //TODO: revisar esto, artificialmente aumentamos la varianza si el integration time no llega a 0
    //Aunque esto no deberia pasar si quality > 0
       measurement.covarianceVelocity = covarianceVelocity*quality;
   }
   
    measurement.covarianceGyroZ = covarianceGyroZ;

    if(integrationTime<0.1){
        ROS_INFO("KalmanFilter integrationTime %f ", integrationTime); 
    }

    // ROS_INFO("*******************************************");
    // ROS_INFO("Px4Flow vx = %f", measurement.vx); 
    // ROS_INFO("Px4Flow vy = %f",measurement.vy); 
    // ROS_INFO("Px4Flow gyroz = %f",measurement.gyroz); 
    // ROS_INFO("*******************************************");

    // ROS_INFO("KalmanFilter mPX4flowHeight = %f",mPX4flowHeight); 
    // ROS_INFO("KalmanFilter integrationX = %f",integrationX); 
    // ROS_INFO("KalmanFilter integrationY = %f",integrationY); 
    // ROS_INFO("KalmanFilter integrationRotationZ = %f",integrationRotationZ); 
    // ROS_INFO("KalmanFilter integrationTime = %f",integrationTime); 

    //ErleImuMeasurement voidMeasurement;
    //ErleMagMeasurement voidMeasurementMag;

    lastPX4FlowMeasurement = measurement;
     mHasPX4FlowMeasurement = true;

    //estimatePositionKF(false, std::vector<RangingMeasurement>() , true, measurement , false, voidMeasurement, false, voidMeasurementMag);
  //  }
   


}


void KalmanFilter::newErleImuMeasurement( double angularVelocityZ,double covarianceAngularVelocityZ,double linearAccelerationX,double linearAccelerationY, double covarianceAccelerationXY[4]){
   
    //ROS_INFO("KalmanFilter newErleImuMeasurement EEEEEEEEEEEEEEEEE");
    ErleImuMeasurement measurement;
    measurement.angularVelocityZ= angularVelocityZ;
    measurement.covarianceAngularVelocityZ= covarianceAngularVelocityZ;
    measurement.linearAccelerationX = linearAccelerationX;
    measurement.linearAccelerationY = linearAccelerationY;
    measurement.covarianceAccelerationXY[0] = covarianceAccelerationXY[0];
    measurement.covarianceAccelerationXY[1] = covarianceAccelerationXY[1];
    measurement.covarianceAccelerationXY[2] = covarianceAccelerationXY[2];
    measurement.covarianceAccelerationXY[3] = covarianceAccelerationXY[3];

    //PX4FlowMeasurement voidMeasurement;
    //ErleMagMeasurement voidMeasurementMag;

    lastErleImuMeasurement = measurement;
    mHasImuMeasurement = true;

   //estimatePositionKF(false, std::vector<RangingMeasurement>() , false, voidMeasurement , true, measurement, false, voidMeasurementMag);
}


void KalmanFilter::newErleMagMeasurement( double magX,double magY,double covarianceMag){

    ErleMagMeasurement measurement;
    measurement.angle = atan2(magY, magX) - mMagAngleOffset;
    measurement.covarianceMag = covarianceMag;

    //ROS_INFO("KalmanFilter  newErleMagMeasurement :[angle:%f]",atan2(magY, magX));
    // ROS_INFO("KalmanFilter  newErleMagMeasurement :[angle with correction:%f]",measurement.angle);

    //PX4FlowMeasurement voidMeasurementPx4;
    //ErleImuMeasurement voidMeasurement;


    lastErleMagMeasurement = measurement;
    mHasMagMeasurement = true;

   // estimatePositionKF(false, std::vector<RangingMeasurement>() , false, voidMeasurementPx4, false, voidMeasurement , true, measurement);

}

void KalmanFilter::newErleCompassMeasurement( double compass,double covarianceCompass){

    ErleMagMeasurement measurement;

    double compassCorrected = compass*M_PI/180;
    compassCorrected = mMagAngleOffset - compassCorrected;

    measurement.angle = normalizeAngle(compassCorrected);
    measurement.covarianceMag = covarianceCompass;

   // ROS_INFO("KalmanFilter  newErleMagMeasurement :[angle:%f]", compass*M_PI/180);
   // ROS_INFO("KalmanFilter  newErleMagMeasurement :[angle corrected:%f]", compassCorrected);
   // ROS_INFO("KalmanFilter  newErleMagMeasurement :[angle corrected rads:%f]",measurement.angle);

    // ErleImuMeasurement aImuMeasurement;
    // PX4FlowMeasurement aPX4Measurement;

    // if (mHasImuMeasurement){
    //     aImuMeasurement = lastErleImuMeasurement;
    // }

    // if (mHasPX4FlowMeasurement){
    //     aPX4Measurement = lastPX4FlowMeasurement;
    // }


    lastErleMagMeasurement = measurement;
    mHasMagMeasurement = true;

    //estimatePositionKF(false, std::vector<RangingMeasurement>() , mHasPX4FlowMeasurement, aPX4Measurement, mHasImuMeasurement, aImuMeasurement , true, measurement);

}



void KalmanFilter::estimatePositionKF(bool hasRangingMeasurements, const std::vector<RangingMeasurement>& allRangingMeasurements,
                                                bool hasPX4Measurement, const PX4FlowMeasurement& px4flowMeasurement, 
                                                bool hasImuMeasurement,const ErleImuMeasurement& erleImuMeasurement,
                                                bool hasMagMeasurement,const ErleMagMeasurement& erleMagMeasurement) {


    std::vector<RangingMeasurement> rangingMeasurements(allRangingMeasurements);

    //ROS_INFO("KalmanFilter estimatePositionKF start"); 

    double timeLag;

      auto now = std::chrono::steady_clock::now();

      if (mLastKFTimestamp == std::chrono::steady_clock::time_point::min()) {
        //Es la primera estimacion
        //TODO: AHORA NO TENEMOS timelag, mirar esto
        timeLag = 0.1;
      } else {

        std::chrono::duration<double> diff = now -  mLastKFTimestamp;
        timeLag = diff.count();

      }
      mLastKFTimestamp = now;


    //ROS_INFO("KalmanFilter mUseFixedInitialPosition %d",mUseFixedInitialPosition); 

    if (!mUseFixedInitialPosition) {
        //Si no se usa una posicion inicial fija, se necesita calcular una mediante UWB antes de poder seguir
        
        if (std::isnan(mPosition.x) ||
                std::isnan(mPosition.y)) {
            ROS_INFO("KalmanFilter initPosition is NAN"); 
            if (hasRangingMeasurements) {
                ROS_INFO("KalmanFilter Ranging mode"); 
                //TODO: hay que modificar lo siguiente para que acepte el nuevo formato de datos
                mPosition = mlLocation->estimatePosition2D(rangingMeasurements, { 0.0, 0.0, mUWBtagZ });
                ROS_INFO("KalmanFilter new mPosition [%f %f %f]", mPosition.x, mPosition.y, mPosition.z); 

                double halfAngle = mAngle*0.5;
                mPosition.rotX = 0.0;
                mPosition.rotY = 0.0;
                mPosition.rotZ = sin(halfAngle);
                mPosition.rotW = cos(halfAngle);

                estimationCovariance(0,0) = mPosition.covarianceMatrix(0,0);
                estimationCovariance(1,0) = mPosition.covarianceMatrix(1,0);
                estimationCovariance(0,1) = mPosition.covarianceMatrix(0,1);
                estimationCovariance(1,1) = mPosition.covarianceMatrix(1,1);

                mPosition.covarianceMatrix = arma::eye<arma::mat>(6, 6) * 0.01;
                //Los valores relacionados con X e Y
                mPosition.covarianceMatrix(0,0) = estimationCovariance(0, 0);
                mPosition.covarianceMatrix(0,1) = estimationCovariance(0, 1);
                mPosition.covarianceMatrix(1,0) = estimationCovariance(1, 0);
                mPosition.covarianceMatrix(1,1) = estimationCovariance(1, 1);

                //Los valores relacionados con el angulo
                mPosition.covarianceMatrix(0,5) = estimationCovariance(0, 6);
                mPosition.covarianceMatrix(1,5) = estimationCovariance(1, 6);
                mPosition.covarianceMatrix(5,0) = estimationCovariance(6, 0);
                mPosition.covarianceMatrix(5,1) = estimationCovariance(6, 1);
                mPosition.covarianceMatrix(5,5) = estimationCovariance(6, 6);
            }
            //return mPosition;
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

    estimationCovariance = predictionStep * estimationCovariance * predictionStep.t() +
                           predictionCovariance;


    predictedState(6) = normalizeAngle(predictedState(6));

    //arma::mat lastValidEstimationCovariance(estimationCovariance);

    //state = predictedState;
    state= kalmanStep3D(predictedState, 
        hasRangingMeasurements, allRangingMeasurements,
        hasPX4Measurement, px4flowMeasurement,
        hasImuMeasurement, erleImuMeasurement,
        hasMagMeasurement, erleMagMeasurement, 
         10, 1e-3, timeLag);
    //ROS_INFO("KalmanFilter estimatePositionKF end state"); 


    //Actualizamos el resto de estados que no son pose
    mVelocity.x = state(2);
    mVelocity.y = state(3);
    mAngle = state(6);
    mAngularSpeed = state(7);

    mPosition.x= state(0); 
    mPosition.y=state(1);
    mAcceleration.x=state(4);
    mAcceleration.y = state(5);



    //Actualizamos la pose
    stateToPose(mPosition, state, estimationCovariance);

    //return mPosition;
}



/**
* Usa los valores que vienen en el estado y en la estimacion de covarianza
* para rellenar la variable pose 
*/
void KalmanFilter::stateToPose(Vector3& pose, const arma::vec& state, const arma::mat& estimationCovariance){

    pose.x = state(0);
    pose.y = state(1);
    pose.z = mUWBtagZ;


    double halfAngle = mAngle*0.5;

    pose.rotX = 0.0;
    pose.rotY = 0.0;
    pose.rotZ = sin(halfAngle);
    pose.rotW = cos(halfAngle);


 // Radian fHalfAngle ( 0.5*rfAngle );
 //        Real fSin = Math::Sin(fHalfAngle);
 //        w = Math::Cos(fHalfAngle);
 //        x = fSin*rkAxis.x;
 //        y = fSin*rkAxis.y;
 //        z = fSin*rkAxis.z;


    // for (int i = 0; i < countValid; ++i)
    // {
    //    ROS_INFO("KalmanFilter predictionError(%d) = %f",i, predictionError(i)); 
    // }

    // ROS_INFO("KalmanFilter mPosition.x = %f",mPosition.x); 
    // ROS_INFO("KalmanFilter mPosition.y = %f",mPosition.y); 
    // ROS_INFO("KalmanFilter mVelocity.x = %f",mVelocity.x); 
    // ROS_INFO("KalmanFilter mVelocity.y = %f",mVelocity.y); 
    // ROS_INFO("KalmanFilter mAngle.x = %f",mAngle); 
    // ROS_INFO("KalmanFilter mAngularSpeed = %f",mAngularSpeed); 

    pose.covarianceMatrix = arma::eye<arma::mat>(6, 6) * 0.01;
    //Los valores relacionados con X e Y
    pose.covarianceMatrix(0,0) =  estimationCovariance(0, 0);
    pose.covarianceMatrix(0,1) =  estimationCovariance(0, 1);
    pose.covarianceMatrix(1,0) =  estimationCovariance(1, 0);
    pose.covarianceMatrix(1,1) =  estimationCovariance(1, 1);

    //Los valores relacionados con el angulo
    pose.covarianceMatrix(0,5) = estimationCovariance(0, 6);
    pose.covarianceMatrix(1,5) = estimationCovariance(1, 6);
    pose.covarianceMatrix(5,0) = estimationCovariance(6, 0);
    pose.covarianceMatrix(5,1) = estimationCovariance(6, 1);
    pose.covarianceMatrix(5,5) = estimationCovariance(6, 6);   
}

arma::vec KalmanFilter::kalmanStep3D(const arma::vec& predictedState, 
    bool hasRangingMeasurements, const std::vector<RangingMeasurement>& allRangingMeasurements, 
    bool hasPX4Measurement, const PX4FlowMeasurement& px4flowMeasurement, 
    bool hasImuMeasurement,const ErleImuMeasurement& erleImuMeasurement,
    bool hasMagMeasurement,const ErleMagMeasurement& erleMagMeasurement,
    int maxSteps, 
    double minRelativeError,
    double timeLag) {

    // TODO: Si el tag UWB no se va a colocar en el centro, indicar un vector
    // respecto al punto del vehículo que queremos posicionar (la misma idea
    // que se hizo en el doble tag).
    int countValid = 0;

    int indexRanging= 0, indexPX4= 0, indexImu= 0, indexMag = 0;
    std::vector<RangingMeasurement> rangingMeasurements(allRangingMeasurements);
    arma::vec state(predictedState);


    //Comprobamos si tenemos algun rangin incorrecto y lo eliminamos

    if (hasRangingMeasurements){
        if (rangingMeasurements.size()>=4){
            double treshold = 2;
             Vector3 mlTempPosition = mlLocation->estimatePosition2D(rangingMeasurements, { state(0), state(1), mUWBtagZ });
            double mlRangingError = mlLocation->estimationError(rangingMeasurements, mlTempPosition);
           // ROS_INFO("KalmanFilter filtered rangings mlRanginError: %f", mlRangingError);
            if (mlRangingError>=treshold){
                //ROS_INFO("KalmanFilter filtered ON rangings mlRanginError: %f", mlRangingError); 
                std::vector<RangingMeasurement> filteredMeasurements;
                mlLocation->estimatePositionIgnoreN(rangingMeasurements,mlTempPosition,1, filteredMeasurements);
                rangingMeasurements = filteredMeasurements;
            }
        }

        if (rangingMeasurements.size() == 3) {
            for (int i = 0; i < rangingMeasurements.size(); ++i) {
                rangingMeasurements[i].errorEstimation = 100*rangingMeasurements[i].errorEstimation;
            }
        }


        //if (rangingMeasurements.size() == 3) {
        //    double predictedError = mlLocation->estimationError(rangingMeasurements, { predictedState(0), predictedState(1), mUWBtagZ});
        //    ROS_INFO("KalmanFilter predicted error with 3 beacons: %f", predictedError); 
        //    if (predictedError> 1) {
        //        ROS_INFO("KalmanFilter wrong ranging measurements mlRanginError: %f", predictedError); 
        //       return mPosition;
        //    }
        //}

    }


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
        //ROS_INFO("KalmanFilter mlRanginError: %f", mlRangingError); 

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
        // if (absSpeed < 0.1) {
        //     //ROS_INFO("KalmanFilter Bounding angular speed");
        //     measurements(indexPX4+2) = 0;
        // }
        observationCovariance(indexPX4,indexPX4) = px4flowMeasurement.covarianceVelocity;
        observationCovariance(indexPX4+1,indexPX4+1) = px4flowMeasurement.covarianceVelocity;
        observationCovariance(indexPX4+2,indexPX4+2) = px4flowMeasurement.covarianceGyroZ;

    }

    if (hasImuMeasurement){
        measurements(indexImu) = erleImuMeasurement.linearAccelerationX;
        measurements(indexImu+1) = erleImuMeasurement.linearAccelerationY;
        measurements(indexImu+2) = erleImuMeasurement.angularVelocityZ;
        double absSpeed = sqrt(predictedState(2)*predictedState(2) 
                    + predictedState(3)*predictedState(3));
        // if (absSpeed < 0.1) {
        //     //ROS_INFO("KalmanFilter Bounding angular speed");
        //     measurements(indexImu+2) = 0;
        // }
        observationCovariance(indexImu, indexImu) = erleImuMeasurement.covarianceAccelerationXY[0];
        observationCovariance(indexImu, indexImu+1) = erleImuMeasurement.covarianceAccelerationXY[1];
        observationCovariance(indexImu+1, indexImu) = erleImuMeasurement.covarianceAccelerationXY[2];
        observationCovariance(indexImu+1, indexImu+1) = erleImuMeasurement.covarianceAccelerationXY[3];
        observationCovariance(indexImu+2, indexImu+2) = erleImuMeasurement.covarianceAngularVelocityZ;
    }

    if (hasMagMeasurement){
        //ROS_INFO("KalmanFilter: Compass angle: %f", erleMagMeasurement.angle);
        measurements(indexMag) = erleMagMeasurement.angle;

        /*double absSpeed = sqrt(predictedState(2)*predictedState(2) 
                    + predictedState(3)*predictedState(3));
        if (absSpeed < 0.1) {
            ROS_INFO("KalmanFilter Bounding angular speed");
            measurements(indexMag) = state(6);
        }*/
        observationCovariance(indexMag, indexMag) = erleMagMeasurement.covarianceMag;
    }

    arma::mat jacobian(countValid, 8);
    arma::mat kalmanGain;
    arma::mat invObsCovariance = inv(observationCovariance);
    arma::mat invEstCovariance = pinv(estimationCovariance);
    // arma::vec predictionDiff = predictedState - state;
    double cost = 1e20;
    double step = 1;
    for (int iter = 0; iter < maxSteps; iter++) {
        Vector3 currentPosition = { state(0), state(1), mUWBtagZ };
        Vector3 currentSpeed = { state(2), state(3), 0.0 };
        Vector3 currentAcceleration = { state(4), state(5), 0.0 };
        double currentAngle = state(6);
        double currentAngularSpeed = state(7);
        
        arma::vec output = sensorOutputs(currentPosition, currentSpeed, currentAcceleration, currentAngle, currentAngularSpeed, timeLag,
                      hasRangingMeasurements,  hasPX4Measurement,  hasImuMeasurement, hasMagMeasurement,rangingMeasurements);
        arma::vec predictionError = measurements - output;

        //for (int i = 0; i < rangingMeasurements.size(); ++i)
        //{
        //    ROS_INFO("KalmanFilter Ranging %d PredictionError=%f", rangingMeasurements[i].beacon.id, predictionError(i));
        //}

        if (hasMagMeasurement) {
            predictionError(indexMag) = normalizeAngle(predictionError(indexMag));
        }


        if (hasRangingMeasurements) {
            jacobianRangings(jacobian, currentPosition, rangingMeasurements, indexRanging);
        }

        if (hasPX4Measurement) {
            jacobianPx4flow(jacobian, currentSpeed, currentAngle, currentAngularSpeed, timeLag, indexPX4);
        }

        if (hasImuMeasurement) {
            jacobianErleBrain(jacobian, currentAcceleration, currentAngle, currentAngularSpeed, indexImu);
        }

        if (hasMagMeasurement) {
            jacobianErleMag(jacobian, indexMag);
        }

        kalmanGain = estimationCovariance * jacobian.t() *
                           inv(jacobian * estimationCovariance * jacobian.t() + observationCovariance);

        /*for (int i = 0; i < kalmanGain.n_rows; ++i)
            {
                for (int j = 0; j < kalmanGain.n_cols; ++j)
                {
                    if (std::isnan(kalmanGain(i,j))){
                       ROS_INFO("KalmanFilter KalmanGain(%d, %d) is NaN", i, j);  
        
                    }
                }
            }   */                

        
        arma::vec predictionDiff = predictedState - state;
        arma::vec costMat = predictionError.t() * invObsCovariance * predictionError
                    + predictionDiff.t() * invEstCovariance * predictionDiff;

        double newCost = costMat(0);           

        arma::vec direction = predictionDiff + kalmanGain * (predictionError - jacobian*predictionDiff);

        

        //if (hasRangingMeasurements) {
        //    ROS_INFO("KalmanFilter iteration = %d: Cost=%f", iter, newCost);
        //}
        if (std::abs(cost - newCost) / cost < minRelativeError) {
            break;
        } else if (newCost<cost){
            state = state + step*direction;
            cost = newCost;
        } else {
            //Cambiamos step size
            step = step/2;
        }
        

        
        //state(6) = normalizeAngle(state(6));

            /*for (int i = 0; i < state.n_row   s; ++i)
            {
                    if (std::isnan(state(i))){
                       ROS_INFO("KalmanFilter state(%d) is NaN", i);  
                       //exit(-1);
                    }
            }  */

        // arma::vec newState = state
        //           + step*(predictionDiff + kalmanGain * (predictionError - jacobian*predictionDiff));
        // arma::vec newPredictionDiff = predictedState - newState;
        // arma::vec newPredictionError = ...;
        // double cost = newPredictionError.t() * inv(observationCovariance) * newPredictionError
        //             + newPredictionDiff.t() * inv(estimationCovariance) * newPredictionDiff;
        // if (cost < oldCost) {
        //     state = newState;
        //     predictionError = newPredictionError;
        //     predictionDiff = newPredictionDiff;
        //     oldCost = cost;
        // } else {
        //     step = step/2;
        // }
    }

    /*for (int i = 0; i < state.n_rows; ++i){
        ROS_INFO("KalmanFilter state(%d)= %f", i, state(i));  
      }  */

    estimationCovariance = (arma::eye<arma::mat>(8, 8) - kalmanGain * jacobian) * estimationCovariance;
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
        ErleImuOutput sensorPrediction = erleImuOutput(acceleration, angle, angularSpeed);

        //ROS_INFO("KalmanFilter ERLE vx = %f m/s",erleImuMeasurement.linearAccelerationX); 
        //ROS_INFO("KalmanFilter ERLE vy = %f m/s",erleImuMeasurement.linearAccelerationY);

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


     // ROS_INFO("*******************************************");
     //  ROS_INFO("Px4Flow speedX = %f", speedX); 
     // ROS_INFO("Px4Flow speedY = %f", speedY); 
     // ROS_INFO("Px4Flow angle = %f", angle); 
     // ROS_INFO("Px4Flow output vx = %f", output.vX); 
     // ROS_INFO("Px4Flow output vy = %f", output.vY); 
     // ROS_INFO("Px4Flow output gyroz = %f", output.gyroZ); 
     // ROS_INFO("*******************************************");

    return output;
}

KalmanFilter::ErleImuOutput KalmanFilter::erleImuOutput(const Vector3& acceleration, double angle, double angularSpeed) const{
    ErleImuOutput output;

    output.accelX = cos(angle)*acceleration.x + sin(angle)*acceleration.y;
    output.accelY = -sin(angle)*acceleration.x + cos(angle)*acceleration.y;
    output.gyroZ = angularSpeed;
    //output.orientationW = angle;

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
    // TODO: Se está suponiendo que la aceleración lineal es del mismo orden
    // que la aceleración angular. No sé si es correcto suponer eso.
    matrix << j*t3*t3 << 0 << j*t3*t2 << 0 << j*t3*t << 0 << 0 << 0 << arma::endr
           << 0 << j*t3*t3 << 0 << j*t3*t2 << 0 << j*t3*t << 0 << 0 << arma::endr
           << j*t3*t2 << 0 << j*t2*t2 << 0 << j*t2*t << 0 << 0 << 0 << arma::endr
           << 0 << j*t3*t2 << 0 << j*t2*t2 << 0 << j*t2*t << 0 << 0 << arma::endr
           << j*t3*t << 0 << j*t2*t << 0 << j*t*t << 0 << 0 << 0 << arma::endr // TODO
           << 0 << j*t3*t << 0 << j*t2*t << 0 << j*t*t << 0 << 0 << arma::endr // TODO
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

void KalmanFilter::jacobianErleBrain(arma::mat& jacobian, const Vector3& acceleration, double angle, double angularSpeed, int indexStartRow) const {
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
    jacobian(indexStartRow +2, 7) = 1; 


}


void KalmanFilter::jacobianErleMag(arma::mat& jacobian, int indexStartRow) const {
    jacobian(indexStartRow , 0) = 0;
    jacobian(indexStartRow , 1) = 0;
    jacobian(indexStartRow , 2) = 0;
    jacobian(indexStartRow , 3) = 0;
    jacobian(indexStartRow , 4) = 0;
    jacobian(indexStartRow , 5) = 0;
    jacobian(indexStartRow , 6) = 1;
    jacobian(indexStartRow , 7) = 0;
}

//Vector3 KalmanFilter::position() const { return mPosition; };


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

    //Llamamos a kf con las ultimas muestras de cada sensor que llegaron desde la ultima vez
    ErleImuMeasurement aImuMeasurement = lastErleImuMeasurement;    
    PX4FlowMeasurement aPX4Measurement = lastPX4FlowMeasurement;
    ErleMagMeasurement aMagMeasurement = lastErleMagMeasurement;
    std::vector<RangingMeasurement> anUwbMeasurements(lastUwbMeasurements); 

    estimatePositionKF(mHasUwbMeasurement, anUwbMeasurements , mHasPX4FlowMeasurement, lastPX4FlowMeasurement, mHasImuMeasurement, lastErleImuMeasurement, mHasMagMeasurement, lastErleMagMeasurement);
    
    mHasUwbMeasurement = false;
    mHasPX4FlowMeasurement = false;
    mHasImuMeasurement = false;
    mHasMagMeasurement = false;


    if (mLastKFTimestamp == std::chrono::steady_clock::time_point::min()) {

        timeLag = 0.1;
        //TODO: si estamos aqui es que todavia no ha llegado ninguna medida,
        //no podemos dar una posicion valida
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

    arma::mat predictedEstimationCovariance = predictionStep * estimationCovariance * predictionStep.t() +
                           predictionCovariance;

    predictedState(6) = normalizeAngle(predictedState(6));

    stateToPose(pose, predictedState, predictedEstimationCovariance);
    return true;
}
