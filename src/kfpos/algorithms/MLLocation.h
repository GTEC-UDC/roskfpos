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

class MLLocation
{
public:

    struct RangingQuality {
        int tagId;
        int innerIndex;
        double error;
    };


    Vector3 estimatePosition(const std::vector<RangingMeasurement>& rangingMeasurements,
                         const Vector3& previousEstimationj = { 1, 1, 1});

    Vector3 estimatePositionTwoTags( const std::vector<Beacon>& beacons, const std::vector<double>& rangingsT0,const std::vector<double>& errorEstimationsT0,
                                     const std::vector<double>& rangingsT1,const std::vector<double>& errorEstimationsT1, const Vector3& previousEstimation = { 1, 1, 1});

    Vector3 estimatePositionTwoTagsIgnoreN( const std::vector<Beacon>& beacons, const std::vector<double>& rangingsT0,const std::vector<double>& errorEstimationsT0, const std::vector<double>& rangingsT1,const std::vector<double>& errorEstimationsT1, const Vector3& previousEstimation, int numRangingsToIgnore);
    Vector3 estimatePositionTwoTagsBestGroup( const std::vector<Beacon>& beacons, const std::vector<double>& rangingsT0,const std::vector<double>& errorEstimationsT0,const std::vector<double>& rangingsT1,const std::vector<double>& errorEstimationsT1, const Vector3& previousEstimation, int bestCriteria, double minZ, double maxZ);

    double estimationError(const std::vector<RangingMeasurement> &rangingMeasurements,
                           const Vector3& position);


    std::vector<double> distanceToBeacons(const Vector3& position,
                                          const std::vector<RangingMeasurement>& beacons) const;

    MLLocation();
    MLLocation(const Vector3& relPosTO, const Vector3& relPosT1);

    double estimationErrorTwoTags(const std::vector<Beacon> &beacons, const std::vector<double> &rangingsT0, const std::vector<double> &rangingsT1, const Vector3 &position);
    void bestRangingsByDistance(const std::vector<Beacon>& beacons, const std::vector<double>& rangingsT0, const std::vector<double>& rangingsT1,const Vector3& position, std::vector<RangingQuality>& rangingQualities);

    Vector3 estimatePositionBestGroup(const std::vector<RangingMeasurement>& rangingMeasurements,
                           const Vector3& previousEstimation, int bestCriteria, double minZ, double maxZ);


    Vector3 estimatePositionIgnoreN( const std::vector<RangingMeasurement>& rangingMeasurements,
                           const Vector3& previousEstimation, int numRangingsToIgnore, std::vector<RangingMeasurement>& newRangingMeasurements);
 Vector3 estimatePositionIgnoreN( const std::vector<RangingMeasurement>& rangingMeasurements,
                           const Vector3& previousEstimation, int numRangingsToIgnore);

    void bestRangingsByDistance(const std::vector<RangingMeasurement>& rangingMeasurements, const Vector3& position, std::vector<RangingQuality>& rangingQualities);


    Vector3 estimatePosition2D(const std::vector<RangingMeasurement> &rangingMeasurements, const Vector3 &previousEstimation = {1,1,1});
private:
    Vector3 relativePosTO;
    Vector3 relativePosT1;
};

#endif // ML_LOCATION_H
