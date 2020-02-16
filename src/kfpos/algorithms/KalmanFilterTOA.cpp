#include "KalmanFilterTOA.h"



KalmanFilterTOA::KalmanFilterTOA(double accelerationNoise, bool ignoreWorstAnchorMode, double ignoreCostThreshold) :
    mAccelerationNoise(accelerationNoise),
    mUseFixedInitialPosition(false),
    _ignoreWorstAnchorMode(ignoreWorstAnchorMode),
    _ignoreCostThreshold(ignoreCostThreshold)
{
    MLLocation *mlLocation = new MLLocation();

    mPosition = { NAN, NAN, NAN};
    mVelocity = { 0, 0, 0};

    estimationCovariance.zeros(6, 6);
    mLastKFTimestamp =  std::chrono::steady_clock::time_point::min();
}


KalmanFilterTOA::KalmanFilterTOA(double accelerationNoise, bool ignoreWorstAnchorMode, double ignoreCostThreshold,  Vector3 initialPosition): 
    mAccelerationNoise(accelerationNoise),
    mUseFixedInitialPosition(true),
    _ignoreWorstAnchorMode(ignoreWorstAnchorMode),
    _ignoreCostThreshold(ignoreCostThreshold)
{
    MLLocation *mlLocation = new MLLocation();

    mPosition = initialPosition;
    mVelocity = { 0, 0, 0};

    estimationCovariance.zeros(6, 6);
    mLastKFTimestamp =  std::chrono::steady_clock::time_point::min();
}



bool KalmanFilterTOA::init(){
    return true;
}


void KalmanFilterTOA::newTOAMeasurement(const std::vector<double>& rangings,
                                        const std::vector<Beacon>& beacons, const std::vector<double>& errorEstimations, double timeLag) {
    
    std::vector<RangingMeasurement> measurements;

    for (int i = 0; i < rangings.size(); ++i)
    {
        if (rangings[i] > 0) {
            RangingMeasurement measurement;
            measurement.ranging = rangings[i];
            measurement.errorEstimation = errorEstimations[i];
            measurement.beacon = beacons[i];
            measurements.push_back(measurement);
        }
    }


    estimatePositionKF(measurements);
}

void KalmanFilterTOA::newPX4FlowMeasurement(double integrationX, double integrationY, double integrationRotationZ, double integrationTime, int quality)  {};
void KalmanFilterTOA::newIMUMeasurement( VectorDim3 angularVelocity,double covarianceAngularVelocity[9],VectorDim3 linearAcceleration, double covarianceAcceleration[9])  {};
void KalmanFilterTOA::newMAGMeasurement( VectorDim3 mag,double covarianceMag[9])  {};
void KalmanFilterTOA::newCompassMeasurement( double compass)  {};



void KalmanFilterTOA::estimatePositionKF(const std::vector<RangingMeasurement>& allRangingMeasurements) {

    std::vector<RangingMeasurement> rangingMeasurements(allRangingMeasurements);

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

    if (!mUseFixedInitialPosition) {
        if (std::isnan(mPosition.x) || std::isnan(mPosition.y) || std::isnan(mPosition.z)) {

            mPosition = mlLocation->estimatePosition(rangingMeasurements, { 1.0, 1.0, 4.0 });
            ROS_DEBUG("KalmanFilterTOA new mPosition [%f %f %f]", mPosition.x, mPosition.y, mPosition.z);

            estimationCovariance(0, 0) = mPosition.covarianceMatrix(0, 0);
            estimationCovariance(1, 0) = mPosition.covarianceMatrix(1, 0);
            estimationCovariance(2, 0) = mPosition.covarianceMatrix(2, 0);
            estimationCovariance(0, 1) = mPosition.covarianceMatrix(0, 1);
            estimationCovariance(1, 1) = mPosition.covarianceMatrix(1, 1);
            estimationCovariance(2, 1) = mPosition.covarianceMatrix(2, 1);
            estimationCovariance(0, 2) = mPosition.covarianceMatrix(0, 1);
            estimationCovariance(1, 2) = mPosition.covarianceMatrix(1, 1);
            estimationCovariance(2, 2) = mPosition.covarianceMatrix(2, 1);

            return;
        }
    }

    arma::vec currentState = { mPosition.x, mPosition.y, mPosition.z,
                               mVelocity.x, mVelocity.y, mVelocity.z
                             };

    // Prediction
    arma::mat predictionStep(6, 6);
    arma::mat predictionCovariance(6, 6);

    predictionMatrix(predictionStep, timeLag);
    predictionErrorCovariance(predictionCovariance, timeLag);
    arma::vec predictedState = predictionStep * currentState;

    estimationCovariance = predictionStep * estimationCovariance * predictionStep.t() +
                           predictionCovariance;


    arma::mat lastValidEstimationCovariance(estimationCovariance);

    StateWithCovariance newState;


    try {

    if (allRangingMeasurements.size()>4 && _ignoreWorstAnchorMode){
        //We can ignore ONE anchor if its behaviour is different from the rest
        //double costThreshold = 0.5;
         //ROS_INFO("Cost Threshold %f", _ignoreCostThreshold);
        ROS_DEBUG("KalmanFilterTOA Ignore mode");
        newState = kalmanStep3DCanIgnoreAnAnchor(predictedState, allRangingMeasurements, 10, 1e-3, timeLag, _ignoreCostThreshold);
    } else {
        StateWithIgnoredAnchor stateWithIgnored = kalmanStep3DIgnoreAnchor(predictedState, allRangingMeasurements, 10, 1e-3, timeLag, -1); //-1 => all anchors used
        newState.state = stateWithIgnored.state;
        newState.estimationCovariance = stateWithIgnored.estimationCovariance;
    }

    //ROS_INFO("KalmanFilterTOA estimatePositionKF end state");

    estimationCovariance = newState.estimationCovariance;
    //Actualizamos la pose
    stateToPose(mPosition, newState.state, newState.estimationCovariance);

} catch (const std::runtime_error& e){

}

    //return mPosition;
}


void KalmanFilterTOA::stateToPose(Vector3& pose, const arma::vec& state, const arma::mat& estimationCovariance) {

    pose.x = state(0);
    pose.y = state(1);
    pose.z = state(2);

    pose.rotX = 0.0;
    pose.rotY = 0.0;
    pose.rotZ = 0.0;
    pose.rotW = 0.0;


    pose.covarianceMatrix = arma::eye<arma::mat>(6, 6) * 0.00;
    //Los valores relacionados con X e Y Z
    pose.covarianceMatrix(0, 0) =  estimationCovariance(0, 0);
    pose.covarianceMatrix(0, 1) =  estimationCovariance(0, 1);
    pose.covarianceMatrix(0, 2) =  estimationCovariance(0, 2);
    pose.covarianceMatrix(1, 0) =  estimationCovariance(1, 0);
    pose.covarianceMatrix(1, 1) =  estimationCovariance(1, 1);
    pose.covarianceMatrix(1, 2) =  estimationCovariance(1, 2);
    pose.covarianceMatrix(2, 0) =  estimationCovariance(2, 0);
    pose.covarianceMatrix(2, 1) =  estimationCovariance(2, 1);
    pose.covarianceMatrix(2, 2) =  estimationCovariance(2, 2);

}

StateWithCovariance KalmanFilterTOA::kalmanStep3DCanIgnoreAnAnchor(const arma::vec& predictedState, const std::vector<RangingMeasurement>& allRangingMeasurements, int maxSteps, double minRelativeError, double timeLag, double costThreshold) {

    double worstIgnoredCost = 0;
    double maxDistance = 0;
    StateWithIgnoredAnchor bestState;
    int ignoredAnchorIndex = -1;

    StateWithIgnoredAnchor stateWithAllAnchors = kalmanStep3DIgnoreAnchor(predictedState, allRangingMeasurements, maxSteps, minRelativeError, timeLag, -1);

    ROS_DEBUG("kalmanStep3DCanIgnoreAnAnchor stateWithAllAnchors end");
   

    for (int i = 0; i < allRangingMeasurements.size(); ++i)
    {
         ROS_DEBUG("kalmanStep3DCanIgnoreAnAnchor StateWithIgnored  Anchor: %d", i);
        auto& ignoredRanging = allRangingMeasurements[i];
        StateWithIgnoredAnchor stateWithIgnoredAnchor = kalmanStep3DIgnoreAnchor(predictedState, allRangingMeasurements, maxSteps, minRelativeError, timeLag, i);
        double x = stateWithIgnoredAnchor.state(0);
        double y = stateWithIgnoredAnchor.state(1);
        double z = stateWithIgnoredAnchor.state(2);
        double distanceToIgnoredAnchor = sqrt(pow(ignoredRanging.beacon.position.x - x, 2) + pow(ignoredRanging.beacon.position.y - y, 2) + pow(ignoredRanging.beacon.position.z - z, 2));

        double distanceDiffAnchor = ignoredRanging.ranging -distanceToIgnoredAnchor;
        ROS_DEBUG("kalmanStep3DCanIgnoreAnAnchor StateWithIgnored  Anchor: %d END", i);
        if (i == 0 || distanceDiffAnchor > maxDistance) {
            maxDistance = distanceDiffAnchor;
            worstIgnoredCost = stateWithIgnoredAnchor.cost;
            bestState = stateWithIgnoredAnchor;
            ignoredAnchorIndex = i;

        }

    }

    StateWithCovariance result;
    result.state = stateWithAllAnchors.state;
    result.estimationCovariance = stateWithAllAnchors.estimationCovariance;

    ROS_DEBUG("kalmanStep3DCanIgnoreAnAnchor Max distances %f  Anchor: %d", maxDistance, ignoredAnchorIndex);
    
    if (maxDistance > 0) {
        double costDiffWithWorstAnchor =  stateWithAllAnchors.cost -  worstIgnoredCost;
        //ROS_INFO("kalmanStep3DCanIgnoreAnAnchor Cost diff %f Cost Threshold %f", costDiffWithWorstAnchor, costThreshold);

        if (costDiffWithWorstAnchor > costThreshold) {
             //ROS_INFO("kalmanStep3DCanIgnoreAnAnchor Ignored Anchor:  [%d]", ignoredAnchorIndex);
             result.state = bestState.state;
            result.estimationCovariance = bestState.estimationCovariance;
        } 
    }

    return result;

}



StateWithIgnoredAnchor KalmanFilterTOA::kalmanStep3DIgnoreAnchor(const arma::vec& predictedState, const std::vector<RangingMeasurement>& allRangingMeasurements, int maxSteps, double minRelativeError, double timeLag, int indexIgnoredAnchor) {

    int countValid = 0;

    int indexRanging = 0;

    //We remove the anchor to ignore
    std::vector<RangingMeasurement> rangingMeasurements;
    for (int i = 0; i < allRangingMeasurements.size(); ++i)
    {
        if (i != indexIgnoredAnchor) {
            rangingMeasurements.push_back(allRangingMeasurements[i]);
        }
    }

    arma::vec newState(predictedState);


    countValid = rangingMeasurements.size();


    arma::mat observationCovariance = arma::eye<arma::mat>(countValid, countValid);
    arma::vec measurements(countValid);


    ROS_DEBUG("KalmanFilterTOA ML Position");
    Vector3 mlTempPosition = mlLocation->estimatePosition(rangingMeasurements, { newState(0), newState(1), newState(2) });

    if (std::isnan(mlTempPosition.x) || std::isnan(mlTempPosition.y) || std::isnan(mlTempPosition.z)){
        mlTempPosition = { newState(0), newState(1), newState(2) };
    }
    double mlRangingError = mlLocation->estimationError(rangingMeasurements, mlTempPosition);
    

    ROS_DEBUG("KalmanFilterTOA mlTempPosition: (%f, %f, %f),mlRanginError: %f", mlRangingError,mlTempPosition.x, mlTempPosition.y, mlTempPosition.z);

    for (int i = 0; i < rangingMeasurements.size(); ++i)
    {
        measurements(i) = rangingMeasurements[i].ranging;
        observationCovariance(i, i) =  std::max(mlRangingError, rangingMeasurements[i].errorEstimation);
    }


    arma::mat jacobian(countValid, 6);
    arma::mat kalmanGain;


    arma::mat invObsCovariance = inv(observationCovariance);
    arma::mat invEstCovariance = pinv(estimationCovariance);
    // arma::vec predictionDiff = predictedState - state;
    double cost = 1e20;
    for (int iter = 0; iter < maxSteps; iter++) {
        Vector3 currentPosition = { newState(0), newState(1), newState(2) };
        Vector3 currentSpeed = { newState(3), newState(4), newState(5) };

        arma::vec output = sensorOutputs(currentPosition, currentSpeed, timeLag, rangingMeasurements);
        arma::vec predictionError = measurements - output;


        arma::vec predictionDiff = predictedState - newState;
        arma::vec costMat = predictionError.t() * invObsCovariance * predictionError
                            + predictionDiff.t() * invEstCovariance * predictionDiff;

        double newCost = costMat(0);

        if (std::abs(cost - newCost) / cost < minRelativeError) {
            break;
        }
        cost = newCost;


        jacobianRangings(jacobian, currentPosition, rangingMeasurements, indexRanging);


        kalmanGain = estimationCovariance * jacobian.t() *
                     inv(jacobian * estimationCovariance * jacobian.t() + observationCovariance);

        arma::vec direction = predictionDiff + kalmanGain * (predictionError - jacobian * predictionDiff);
        newState = newState + direction;
        ROS_DEBUG("KalmanFilterTOA cost: %f", cost);


    }

    arma::mat newEstimationCovariance = (arma::eye<arma::mat>(6, 6) - kalmanGain * jacobian) * estimationCovariance;


    StateWithIgnoredAnchor stateWithIgnoredAnchor;
    stateWithIgnoredAnchor.state = newState;
    stateWithIgnoredAnchor.estimationCovariance = newEstimationCovariance;
    stateWithIgnoredAnchor.cost = cost;
    stateWithIgnoredAnchor.indexIgnoredAnchor = indexIgnoredAnchor;


    return stateWithIgnoredAnchor;

}


arma::vec KalmanFilterTOA::sensorOutputs(const Vector3& position, const Vector3& speed, double timeLag, const std::vector<RangingMeasurement>& rangingMeasurements) const {
    int countValid = 0;

    countValid = rangingMeasurements.size();

    arma::vec output(countValid);


    std::vector<double> distances = mlLocation->distanceToBeacons(position, rangingMeasurements);
    int v = 0;
    for (const auto& r : rangingMeasurements) {
        output(v) = distances[v];
        v++;
    }


    return output;
}



void KalmanFilterTOA::predictionMatrix(arma::mat& matrix, double timeLag) const {
    matrix << 1 << 0 << 0 << timeLag << 0  << 0 << arma::endr
           << 0 << 1 << 0 << 0 << timeLag  << 0 << arma::endr
           << 0 << 0 << 1 << 0 << 0 << timeLag << arma::endr
           << 0 << 0 << 0 << 1 << 0 << 0 << arma::endr
           << 0 << 0 << 0 << 0 << 1 << 0 << arma::endr
           << 0 << 0 << 0 << 0 << 0 << 1 << arma::endr;
}

void KalmanFilterTOA::predictionErrorCovariance(arma::mat& matrix, double timeLag) const {
    //double t3 = pow(timeLag, 3)/6;
    double t2 = pow(timeLag, 2) / 2;
    double t = timeLag;
    double a2 = mAccelerationNoise * mAccelerationNoise;
    //double j = 1;
    // TODO: Se está suponiendo que la aceleración lineal es del mismo orden
    // que la aceleración angular. No sé si es correcto suponer eso.
    /*matrix << j*t3*t3 << 0        << 0       << j*t3*t2 << 0        << 0 << arma::endr
           << 0       << j*t3*t3  << 0       << 0       << j*t3*t2  << 0 << arma::endr
           << 0       << 0        << j*t3*t3 << 0       << 0        << j*t3*t2 << arma::endr
           << j*t3*t2 << 0        << 0       << j*t2*t2 << 0        << 0 << arma::endr
           << 0       << j*t3*t2  << 0       << 0       << j*t2*t2  << 0 << arma::endr
           << 0       << 0        << j*t3*t2 << 0       << 0        << j*t2*t2 << arma::endr;*/

    matrix << a2*t2*t2 << 0        << 0       << a2*t2*t << 0        << 0 << arma::endr
           << 0       << a2*t2*t2  << 0       << 0       << a2*t2*t  << 0 << arma::endr
           << 0       << 0        << a2*t2*t2 << 0       << 0        << a2*t2*t << arma::endr
           << a2*t2*t << 0        << 0       << a2*t*t << 0        << 0 << arma::endr
           << 0       << a2*t2*t  << 0       << 0       << a2*t*t  << 0 << arma::endr
           << 0       << 0        << a2*t2*t << 0       << 0        << a2*t*t << arma::endr;


    /*  double timeLag4 = mAccelerationNoise*pow(timeLag, 4)/4;
      double timeLag3 = mAccelerationNoise*pow(timeLag, 3)/2;
      double timeLag2 = mAccelerationNoise*pow(timeLag, 2);
      matrix << timeLag4<< 0<< 0<< timeLag3<<0<< 0<< arma::endr
                 << 0<< timeLag4<< 0<< 0<< timeLag3<< 0<< arma::endr
                 << 0<< 0<< timeLag4<< 0<< 0<< timeLag3<< arma::endr
                 << timeLag3<< 0<< 0<< timeLag2<< 0<< 0<< arma::endr
                 << 0<< timeLag3<< 0<< 0<< timeLag2<< 0<< arma::endr
                << 0<< 0<< timeLag3<< 0<< 0<<timeLag2<< arma::endr;*/

    /*Veo un posible error en la función predictionCovarianceError().
    Lo que es t2 debería pasar a ser t, y lo que es t3 debería pasar a ser t2,
    dentro de la definición de esa matriz. Entiendo que aquí solo se considera hasta la velocidad,
    y que la aceleración ya forma parte del error de predicción.
    x = x0 + vt +1/2at²
    v = v0 + at
    implicaría que el error de velocidad
    debería ser a²t² (esquina inferior derecha),
    el de posición 1/4a²t⁴ (esquina superior izquierda)
    y las correlaciones cruzadas 1/2a²t³ (esquinas superior derecha e inferior izquierda de la matriz). El a² sería la variable j del código.*/

    /*  1/4a²t⁴  1/2a²t³
         1/2a²t³        a²t² */


}

void KalmanFilterTOA::jacobianRangings(arma::mat& jacobian, const Vector3& position, const std::vector<RangingMeasurement>& rangingMeasurements, int indexStartRow) const {
    std::vector<double> distances = mlLocation->distanceToBeacons(position, rangingMeasurements);
    int i = 0;
    for (auto const& r : rangingMeasurements) {
        jacobian(i + indexStartRow, 0) = (position.x - r.beacon.position.x) / distances[i];
        jacobian(i + indexStartRow, 1) = (position.y - r.beacon.position.y) / distances[i];
        jacobian(i + indexStartRow, 2) = (position.z - r.beacon.position.z) / distances[i];;
        jacobian(i + indexStartRow, 3) = 0;
        jacobian(i + indexStartRow, 4) = 0;
        jacobian(i + indexStartRow, 5) = 0;
        i++;
    }
}




bool KalmanFilterTOA::getPose(Vector3& pose) {
    double timeLag;
    auto now = std::chrono::steady_clock::now();

    if (mLastKFTimestamp == std::chrono::steady_clock::time_point::min()) {

        timeLag = 0.1;
        //TODO: si estamos aqui es que todavia no ha llegado ninguna medida,
        //no podemos dar una posicion valida
        return false;

    } else {

        std::chrono::duration<double> diff = now -  mLastKFTimestamp;
        timeLag = diff.count();
    }

    arma::vec state = { mPosition.x, mPosition.y, mPosition.z,
                        mVelocity.x, mVelocity.y, mVelocity.z
                      };

    // Prediction
    arma::mat predictionStep(6, 6);
    arma::mat predictionCovariance(6, 6);

    predictionMatrix(predictionStep, timeLag);
    predictionErrorCovariance(predictionCovariance, timeLag);
    arma::vec predictedState = predictionStep * state;

    arma::mat predictedEstimationCovariance = predictionStep * estimationCovariance * predictionStep.t() +
            predictionCovariance;


    stateToPose(pose, predictedState, predictedEstimationCovariance);
    return true;
}