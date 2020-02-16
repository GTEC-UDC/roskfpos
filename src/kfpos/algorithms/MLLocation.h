#ifndef ML_LOCATION_H
#define ML_LOCATION_H


#define ML_VARIANT_NORMAL 0
#define ML_VARIANT_IGNORE_N 1
#define ML_VARIANT_BEST 2


#define SELECT_ANCHORS_USING_ERROR_XYZ 0
#define SELECT_ANCHORS_USING_ERROR_Z 1

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <cmath>
#include <vector>
#include <armadillo>
#include "ros/ros.h"
#include "sensor_types.h"
#include "PositionEstimationAlgorithm.h"

class MLLocation : public PositionEstimationAlgorithm
{
public:

    MLLocation();
    MLLocation(bool use2d, int variant, int numRangingsToIgnore, const Vector3 &previousEstimation);

    //Superclass
    bool init() override;
    bool getPose(Vector3& pose) override;
    void newPX4FlowMeasurement(double integrationX, double integrationY, double integrationRotationZ, double integrationTime, int quality) override;
    void newTOAMeasurement(const std::vector<double>& rangings, const std::vector<Beacon>& beacons, const std::vector<double>& errorEstimations, double timeLag) override;
    void newIMUMeasurement( VectorDim3 angularVelocity,double covarianceAngularVelocity[9],VectorDim3 linearAcceleration, double covarianceAcceleration[9]) override;
    void newMAGMeasurement( VectorDim3 mag, double covarianceMag[9]) override;
    void newCompassMeasurement( double compass) override;


    Vector3 estimatePosition(const std::vector<RangingMeasurement>& rangingMeasurements,
                         const Vector3& previousEstimationj = { 1, 1, 1});
    Vector3 estimatePosition2D(const std::vector<RangingMeasurement> &rangingMeasurements, const Vector3 &previousEstimation = {1,1,1});

    //Heuristic variants
    Vector3 estimatePositionBestGroup(const std::vector<RangingMeasurement>& rangingMeasurements, const Vector3& previousEstimation);
    Vector3 estimatePositionIgnoreN( const std::vector<RangingMeasurement>& rangingMeasurements, const Vector3& previousEstimation, int numRangingsToIgnore);

    double estimationError(const std::vector<RangingMeasurement> &rangingMeasurements,
                           const Vector3& position);
    std::vector<double> distanceToBeacons(const Vector3& position,
                                          const std::vector<RangingMeasurement>& beacons) const;
private:

    struct RangingQuality {
        int tagId;
        int innerIndex;
        double error;
    };

    void bestRangingsByDistance(const std::vector<RangingMeasurement>& rangingMeasurements, const Vector3& position, std::vector<RangingQuality>& rangingQualities);
    void bestRangingsByDistance(const std::vector<Beacon>& beacons, const std::vector<double>& rangingsT0, const std::vector<double>& rangingsT1,const Vector3& position, std::vector<RangingQuality>& rangingQualities);



    bool _use2d;
    int _variant;
    int _numRangingsToIgnore;

    Vector3 _previousEstimation = {1,1,1};
    std::vector<RangingMeasurement> _lastRangingMeasurements = {};



};

#endif // ML_LOCATION_H
