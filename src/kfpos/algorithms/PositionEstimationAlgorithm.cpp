#include "PositionEstimationAlgorithm.h"



PositionEstimationAlgorithm::~PositionEstimationAlgorithm(){

}


bool PositionEstimationAlgorithm::init(){

}
bool PositionEstimationAlgorithm::getPose(Vector3& pose){

}


void PositionEstimationAlgorithm::newTOAMeasurement(const std::vector<double>& rangings,
                                        const std::vector<Beacon>& beacons, const std::vector<double>& errorEstimations, double timeLag) {}

void PositionEstimationAlgorithm::newPX4FlowMeasurement(double integrationX, double integrationY, double integrationRotationZ, double integrationTime, int quality)  {}
void PositionEstimationAlgorithm::newIMUMeasurement( VectorDim3 angularVelocity,double covarianceAngularVelocity[9],VectorDim3 linearAcceleration, double covarianceAcceleration[9])  {}
void PositionEstimationAlgorithm::newMAGMeasurement( VectorDim3 mag,double covarianceMag[9])  {}
void PositionEstimationAlgorithm::newCompassMeasurement( double compass)  {}