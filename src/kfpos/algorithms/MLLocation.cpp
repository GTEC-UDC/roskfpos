#include "MLLocation.h"

MLLocation::MLLocation():
    _use2d(false),
    _variant(ML_VARIANT_NORMAL),
    _numRangingsToIgnore(0),
    _previousEstimation({1,1,4})
{
    _lastRangingMeasurements = {};
    _previousEstimation.covarianceMatrix= arma::eye<arma::mat>(6, 6) * 0.00;

}

MLLocation::MLLocation(bool use2d, int variant, int numRangingsToIgnore, const Vector3 &previousEstimation):
    _use2d(use2d),
    _variant(variant),
    _numRangingsToIgnore(numRangingsToIgnore),
    _previousEstimation(previousEstimation)
{
    _lastRangingMeasurements = {};
    _previousEstimation.covarianceMatrix= arma::eye<arma::mat>(6, 6) * 0.00;
}

std::vector<double> MLLocation::distanceToBeacons(const Vector3& position,
        const std::vector<RangingMeasurement>& rangingMeasurements) const {
    std::vector<double> distances;

    for (int i = 0; i < rangingMeasurements.size(); ++i)
    {
        auto& r = rangingMeasurements[i];
        double distance = sqrt((r.beacon.position.x - position.x) * (r.beacon.position.x - position.x) +
                               (r.beacon.position.y - position.y) * (r.beacon.position.y - position.y) +
                               (r.beacon.position.z - position.z) * (r.beacon.position.z - position.z));
        distances.push_back(distance);
    }
    return distances;
}



/*
   Position estimation in 2D. Z value is defined by the previousEstimation parameter.
   It uses the gradient descendent technique over the error defined as the difference
   among the rangings and the distance to the anchor from the estimated position.
   This method could diverge. This could be minimized passing to the function
   a previous estimation to start the search of the minimum.
*/
Vector3 MLLocation::estimatePosition2D(const std::vector<RangingMeasurement>& rangingMeasurements,
                                       const Vector3& previousEstimation) {

    ROS_DEBUG("MLLocation:estimatePosition2D: Start..");
    Vector3 position = previousEstimation;
    int numMeasurements = rangingMeasurements.size();
    if (numMeasurements < 3) {
        //Return previous position if cant calculate a new one
        ROS_DEBUG("MLLocation:estimatePosition2D: less than 3 rangings, cant generate a new position.");
        return position;
    }

    double cost = 1e20, newCost = 1;
    double step = 1;


    Vector3 tentativePos;
    newCost = estimationError(rangingMeasurements, position);

    int maxIters = 10000;
    int iter = 0;
    //&& (newCost > 1e-2)
    while ((std::abs(cost - newCost) / cost > 1e-3) && (iter<maxIters)) {
        iter+=1;
        cost = newCost;

        std::vector<double> distances = distanceToBeacons(position, rangingMeasurements);
        arma::vec gradient(2, arma::fill::zeros);
        arma::mat secondDeriv(2, 2, arma::fill::zeros);
        for (int i = 0; i < rangingMeasurements.size(); ++i)
        {
            auto& r = rangingMeasurements[i];
            gradient(0) += (r.ranging - distances[i]) *
                           (r.beacon.position.x - position.x) / (distances[i] * r.errorEstimation);
            gradient(1) += (r.ranging - distances[i]) *
                           (r.beacon.position.y - position.y) / (distances[i] * r.errorEstimation);

            double d3 = distances[i] * distances[i] * distances[i];
            secondDeriv(0, 0) += ( 1 - r.ranging / distances[i]
                                   + r.ranging * (r.beacon.position.x - position.x) * (r.beacon.position.x - position.x) / d3
                                 ) / r.errorEstimation;
            secondDeriv(1, 1) += ( 1 - r.ranging / distances[i]
                                   + r.ranging * (r.beacon.position.y - position.y) * (r.beacon.position.y - position.y) / d3
                                 ) / r.errorEstimation;
            double diffXY = r.ranging * (r.beacon.position.x - position.x) * (r.beacon.position.y - position.y) / (d3 * r.errorEstimation);
            secondDeriv(0, 1) += diffXY;
            secondDeriv(1, 0) += diffXY;
        }
        arma::vec pos(2);
        pos(0) = position.x;
        pos(1) = position.y;
        arma::vec rhs = secondDeriv * pos - gradient * step;
        arma::vec newPos = arma::solve(secondDeriv, rhs);

        tentativePos.x = newPos(0);
        tentativePos.y = newPos(1);

        distances = distanceToBeacons(tentativePos, rangingMeasurements);
        double tentativeCost = estimationError(rangingMeasurements, tentativePos);


        if (tentativeCost > cost) {
            step /= 2;
        } else {
            newCost = tentativeCost;
            step = 1;
            position.x = newPos(0);
            position.y = newPos(1);
        }
    }
    std::vector<double> distances = distanceToBeacons(position, rangingMeasurements);

    arma::mat jacobian(numMeasurements, 2);
    arma::vec realObservationError(numMeasurements);

    // To try to avoid the effect of NLOS measurement, we overestimate the covariance.
    // We estimate the MSE among the rangings and the position estimation, so if a ranging is NLOS
    // it will cause a huge covariance.
    double rangingError = estimationError(rangingMeasurements, position);

    // Covariance estimation using the linearization of the distance function.
    // This could underestimate.
    for (int i = 0; i < rangingMeasurements.size(); ++i)
    {
        auto& r = rangingMeasurements[i];
        jacobian(i, 0) = (position.x - r.beacon.position.x) / distances[i];
        jacobian(i, 1) = (position.y - r.beacon.position.y) / distances[i];

        realObservationError(i) = std::max(r.errorEstimation, rangingError);
    }

    arma::mat covarianceMatrix = inv(jacobian.t() * inv(arma::diagmat(realObservationError)) * jacobian);
    position.covarianceMatrix = covarianceMatrix;

    return position;
}


/*
   Position estimation in 3D.
   It uses the gradient descendent technique over the error defined as the difference
   among the rangings and the distance to the anchor from the estimated position.
   This method could diverge. This could be minimized passing to the function
   a previous estimation to start the search of the minimum.
*/
Vector3 MLLocation::estimatePosition(const std::vector<RangingMeasurement>& rangingMeasurements,
                                     const Vector3& previousEstimation) {
    ROS_DEBUG("MLLocation:estimatePosition: Start..");
    Vector3 position = previousEstimation;
    int numMeasurements = rangingMeasurements.size();
    if (numMeasurements < 4) {
        ROS_DEBUG("MLLocation:estimatePosition: less than 4 rangings, cant generate a new position.");
        return position;
    }
    //ROS_INFO("Previous pos: (%f, %f, %f)", position.x, position.y, position.z);

    double cost = 1e20, newCost = 1;
    int maxIters = 10000;
    int iter = 0;
    //&& (newCost > 1e-2)
    while ((std::abs(cost - newCost) / cost > 1e-3) && (iter<maxIters)) {
        iter+=1;
        cost = newCost;
        std::vector<double> distances = distanceToBeacons(position, rangingMeasurements);
        arma::vec gradient(3, arma::fill::zeros);
        arma::mat secondDeriv(3, 3, arma::fill::zeros);
        for (int i = 0; i < rangingMeasurements.size(); ++i)
        {

            auto& r = rangingMeasurements[i];
            gradient(0) += (r.ranging - distances[i]) *
                           (r.beacon.position.x - position.x) / (distances[i] * r.errorEstimation);
            gradient(1) += (r.ranging - distances[i]) *
                           (r.beacon.position.y - position.y) / (distances[i] * r.errorEstimation);
            gradient(2) += (r.ranging - distances[i]) *
                           (r.beacon.position.z - position.z) / (distances[i] * r.errorEstimation);

            double d3 = distances[i] * distances[i] * distances[i];
            secondDeriv(0, 0) += ( 1 - r.ranging / distances[i]
                                   + r.ranging * (r.beacon.position.x - position.x) * (r.beacon.position.x - position.x) / d3
                                 ) / r.errorEstimation;
            secondDeriv(1, 1) += ( 1 - r.ranging / distances[i]
                                   + r.ranging * (r.beacon.position.y - position.y) * (r.beacon.position.y - position.y) / d3
                                 ) / r.errorEstimation;
            secondDeriv(2, 2) += ( 1 - r.ranging / distances[i]
                                   + r.ranging * (r.beacon.position.z - position.z) * (r.beacon.position.z - position.z) / d3
                                 ) / r.errorEstimation;
            double diffXY = r.ranging * (r.beacon.position.x - position.x) * (r.beacon.position.y - position.y) / (d3 * r.errorEstimation);
            double diffXZ = r.ranging * (r.beacon.position.x - position.x) * (r.beacon.position.z - position.z) / (d3 * r.errorEstimation);
            double diffYZ = r.ranging * (r.beacon.position.y - position.y) * (r.beacon.position.z - position.z) / (d3 * r.errorEstimation);
            secondDeriv(0, 1) += diffXY;
            secondDeriv(0, 2) += diffXZ;
            secondDeriv(1, 2) += diffYZ;
            secondDeriv(1, 0) += diffXY;
            secondDeriv(2, 0) += diffXZ;
            secondDeriv(2, 1) += diffYZ;
        }
        arma::vec pos(3);
        pos(0) = position.x;
        pos(1) = position.y;
        pos(2) = position.z;
        arma::vec rhs = secondDeriv * pos - gradient;
        arma::vec newPos = arma::solve(secondDeriv, rhs, arma::solve_opts::equilibrate);
        position.x = newPos(0);
        position.y = newPos(1);
        position.z = newPos(2);

        //ROS_INFO("New pos: (%f, %f, %f)", position.x, position.y, position.z);
        distances = distanceToBeacons(position, rangingMeasurements);
        newCost = 0.0;
        for (int i = 0; i < rangingMeasurements.size(); ++i)
        {
            auto& r = rangingMeasurements[i];
            newCost += (r.ranging - distances[i]) * (r.ranging - distances[i]) / r.errorEstimation;
        }

        ROS_DEBUG("newCost: %f , cost: %f, cost - newCost / cost : %f", newCost, cost, cost - newCost / cost);
    }

    ROS_DEBUG("END newCost: %f", newCost);

    std::vector<double> distances = distanceToBeacons(position, rangingMeasurements);
    arma::mat jacobian(numMeasurements, 3);
    arma::vec realObservationError(numMeasurements);


    // To try to avoid the effect of NLOS measurement, we overestimate the covariance.
    // We estimate the MSE among the rangings and the position estimation, so if a ranging is NLOS
    // it will cause a huge covariance.
    double rangingError = estimationError(rangingMeasurements, position);

    // Covariance estimation using the linearization of the distance function.
    // This could underestimate.
    for (int i = 0; i < rangingMeasurements.size(); ++i)
    {
        auto& r = rangingMeasurements[i];
        jacobian(i, 0) = (position.x - r.beacon.position.x) / distances[i];
        jacobian(i, 1) = (position.y - r.beacon.position.y) / distances[i];
        jacobian(i, 2) = (position.z - r.beacon.position.z) / distances[i];

        realObservationError(i) = std::max(r.errorEstimation, rangingError);
    }


    arma::mat covarianceMatrix = inv(jacobian.t() * inv(arma::diagmat(realObservationError)) * jacobian);

    position.covarianceMatrix = covarianceMatrix;

    return position;
}


/*
    Get the error between the distances to the beacons and the measured values.
*/
double MLLocation::estimationError(const std::vector<RangingMeasurement>& rangingMeasurements,
                                   const Vector3& position) {
    if (rangingMeasurements.size() == 0) {
        return -1;
    }
    std::vector<double> distances = distanceToBeacons(position, rangingMeasurements);
    double error = 0.0;
    for (int i = 0; i < rangingMeasurements.size(); ++i)
    {
        auto& r = rangingMeasurements[i];
        error += (distances[i] - r.ranging) *
                 (distances[i] - r.ranging);
    }

    return error;
}

/*
    Calculates the error after removing one ranging measurement each time. Later, it sorts the
    results so we can know what measurement causes the higher error.
*/
void MLLocation::bestRangingsByDistance(const std::vector<RangingMeasurement>& rangingMeasurements,
                                        const Vector3& position, std::vector<RangingQuality>& rangingQualities) {

    std::vector<double> distancesT0 = distanceToBeacons(position, rangingMeasurements);

    for (int i = 0; i < rangingMeasurements.size(); ++i)
    {
        auto& r = rangingMeasurements[i];
        RangingQuality rangingQuality;
        rangingQuality.tagId = 0;
        rangingQuality.innerIndex = i;
        rangingQuality.error = (distancesT0[i] - r.ranging) *
                               (distancesT0[i] - r.ranging);
        rangingQualities.push_back(rangingQuality);
    }
    std::sort(rangingQualities.begin(), rangingQualities.end(), [](const RangingQuality & a, const RangingQuality & b) { return a.error < b.error; });
}



/*
    Removes the rangings that have a higher impact in the error before performing the estimate.
*/
Vector3 MLLocation::estimatePositionIgnoreN( const std::vector<RangingMeasurement>& rangingMeasurements,
        const Vector3& previousEstimation, int numRangingsToIgnore) {

    Vector3 position;
    int minRangings;
    int numMeasurements = rangingMeasurements.size();

    if (_use2d) {
        position = estimatePosition2D(rangingMeasurements, previousEstimation);
        minRangings = 3;
    } else {
        position = estimatePosition(rangingMeasurements, previousEstimation);
        minRangings = 4;
    }

    std::vector<RangingQuality> rangingQualities;
    bestRangingsByDistance(rangingMeasurements, position, rangingQualities);

    std::vector<RangingMeasurement> newRangings(rangingMeasurements);

    int i = 0;
    for (const auto& q : rangingQualities) {
        newRangings[i] = rangingMeasurements[q.innerIndex];
        i++;
    }

    //We need at least 3 o 4 anchors
    int minNumRangingsToIgnore = std::min(numMeasurements-minRangings, numRangingsToIgnore);


    for (i = 0; i < minNumRangingsToIgnore; i++) {
        newRangings.pop_back();
    }

    if (_use2d) {
        return estimatePosition2D(newRangings, previousEstimation);
    } else {
        return estimatePosition(newRangings, previousEstimation);
    }

}



Vector3 MLLocation::estimatePositionBestGroup(const std::vector<RangingMeasurement>& rangingMeasurements,
        const Vector3& previousEstimation) {

    Vector3 position;
    int minRangings;

    if (_use2d) {
        position = estimatePosition2D(rangingMeasurements, previousEstimation);
        minRangings = 3;
    } else {
        position = estimatePosition(rangingMeasurements, previousEstimation);
        minRangings = 4;
    }

    int numMeasurements = rangingMeasurements.size();

    if (numMeasurements < minRangings) {
        return position;
    }

    std::vector<Vector3> groupPositions;

    //We calculate the combinations
    std::vector<bool> v(numMeasurements);
    std::fill(v.begin(), v.begin() + minRangings, true);

    do {
        std::vector<RangingMeasurement> newRangings(rangingMeasurements);
        for (int i = 0; i < numMeasurements; ++i) {
            if (!v[i]) {
                newRangings.erase(newRangings.begin() + i);
            }
        }
        Vector3 newPos;
        if (_use2d) {
            newPos = estimatePosition2D(newRangings, previousEstimation);
        } else {
            newPos = estimatePosition(newRangings, previousEstimation);
        }
        groupPositions.push_back(newPos);

    } while (std::prev_permutation(v.begin(), v.end()));


    double minError = 0;
    int minIndex = -1;
    double currentError = 0.0;

    for (int i = 0; i < groupPositions.size(); i++) {
        currentError = groupPositions[i].covarianceMatrix(0, 0) + groupPositions[i].covarianceMatrix(1, 1) + groupPositions[i].covarianceMatrix(2, 2);

        if (minIndex == -1) {
            minIndex = i;
            minError = currentError;
        }

        if (currentError <= minError) {
            minIndex = i;
            minError = currentError;
        }
    }

    return groupPositions[minIndex];
}


bool MLLocation::init(){
    ROS_INFO("MLLocation:init");
}

bool MLLocation::getPose(Vector3& pose) {
    ROS_DEBUG("MLLocation:getPose: Start...");

    Vector3 estimation;
    if (_variant==ML_VARIANT_NORMAL){
        if (_use2d){
            ROS_DEBUG("MLLocation:getPose: Normal 2D");
            estimation = estimatePosition2D(_lastRangingMeasurements, _previousEstimation);
        } else {
            ROS_DEBUG("MLLocation:getPose: Normal 3D");
            estimation = estimatePosition(_lastRangingMeasurements, _previousEstimation);
        }
    } else if (_variant==ML_VARIANT_IGNORE_N){
        ROS_DEBUG("MLLocation:getPose: Ignore N");
        estimation = estimatePositionIgnoreN(_lastRangingMeasurements, _previousEstimation, _numRangingsToIgnore);
    } else if (_variant==ML_VARIANT_BEST){
        ROS_DEBUG("MLLocation:getPose: Best");
        estimation = estimatePositionBestGroup(_lastRangingMeasurements, _previousEstimation);
    }


 /*   if (std::isnan(estimation.x) || std::isnan(estimation.y) || std::isnan(estimation.z)) {
        return false;
    }
*/
    pose.x = estimation.x;
    pose.y = estimation.y;
    pose.z = estimation.z;

    pose.rotX = 0.0;
    pose.rotY = 0.0;
    pose.rotZ = 0.0;
    pose.rotW = 0.0;

    pose.covarianceMatrix = arma::eye<arma::mat>(6, 6) * 0.00;
    pose.covarianceMatrix(0, 0) =  estimation.covarianceMatrix(0, 0);
    pose.covarianceMatrix(0, 1) =  estimation.covarianceMatrix(0, 1);
    pose.covarianceMatrix(0, 2) =  estimation.covarianceMatrix(0, 2);
    pose.covarianceMatrix(1, 0) =  estimation.covarianceMatrix(1, 0);
    pose.covarianceMatrix(1, 1) =  estimation.covarianceMatrix(1, 1);
    pose.covarianceMatrix(1, 2) =  estimation.covarianceMatrix(1, 2);
    pose.covarianceMatrix(2, 0) =  estimation.covarianceMatrix(2, 0);
    pose.covarianceMatrix(2, 1) =  estimation.covarianceMatrix(2, 1);
    pose.covarianceMatrix(2, 2) =  estimation.covarianceMatrix(2, 2);

    ROS_DEBUG("MLLocation:getPose: End");

    return true;
};


void MLLocation::newTOAMeasurement(const std::vector<double>& rangings, const std::vector<Beacon>& beacons, const std::vector<double>& errorEstimations, double timeLag){
    ROS_DEBUG("MLLocation:newTOAMeasurement");
    _lastRangingMeasurements.clear();

    for (int i = 0; i < rangings.size(); ++i)
    {
        if (rangings[i] > 0) {
            RangingMeasurement measurement;
            measurement.ranging = rangings[i];
            measurement.errorEstimation = errorEstimations[i];
            measurement.beacon = beacons[i];
            _lastRangingMeasurements.push_back(measurement);
        }
    }
}



void MLLocation::newPX4FlowMeasurement(double integrationX, double integrationY, double integrationRotationZ, double integrationTime, int quality)  {};
void MLLocation::newIMUMeasurement( VectorDim3 angularVelocity,double covarianceAngularVelocity[9],VectorDim3 linearAcceleration, double covarianceAcceleration[9])  {};
void MLLocation::newMAGMeasurement( VectorDim3 mag,double covarianceMag[9])  {};
void MLLocation::newCompassMeasurement( double compass)  {};