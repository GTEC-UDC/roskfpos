#ifndef POSGENERATOR_PX4FLOW_H
#define POSGENERATOR_PX4FLOW_H


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

#include <string>
#include <vector>
#include <stdint.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <chrono>

#include "../algorithms/MLLocation.h"
#include "../algorithms/KalmanFilter.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/Int32MultiArray.h"
#include <sstream>
#include <gtec_msgs/Ranging.h>
#include <mavros_msgs/OpticalFlowRad.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Float64.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

#include "../algorithms/sensor_types.h"

#define HIS_LENGTH 100
#define PORT_TYPE_UWB 0

#define ALGORITHM_TRILATERATION  0
#define ALGORITHM_TRILATERATION_NEAR 1
#define ALGORITHM_ML 2
#define ALGORITHM_MAP 3
#define ALGORITHM_KF 4
#define ALGORITHM_KF_UWB 5

#define RESULT_ML 4
#define RESULT_MAP 5
#define RESULT_KF 6

#define MAX_NUM_TAGS (8)
#define MAX_NUM_ANCS (64)


#define MAX_TIME_TO_SEND_RANGING 0.05

typedef struct
{
    double x_arr[HIS_LENGTH];
    double y_arr[HIS_LENGTH];
    double z_arr[HIS_LENGTH];
    double av_x, av_y, av_z; //average
    double sqx_arr[HIS_LENGTH]; //square x
    double sqy_arr[HIS_LENGTH];
    double sqz_arr[HIS_LENGTH];
    double avsq_x, avsq_y, avsq_z; //average of squares
    double errx_arr[HIS_LENGTH]; //error x (x-av_x)
    double erry_arr[HIS_LENGTH];
    double errz_arr[HIS_LENGTH];
    double averr_x, averr_y, averr_z; //avearge error
    double variancex, variancey, variancez;
    double std_x, std_y, std_z;
    double r95;
    int id;
    int arr_idx;
    int count;
    int numberOfLEs;
    bool ready;
    int rangeSeq;
    int rangeCount[256];
    int rangeValue[256][MAX_NUM_ANCS]; //(mm) each tag ranges to 4 anchors - it has a range number which is modulo 256
    double errorEstimation[256][MAX_NUM_ANCS]; //GTEC ADD
} tag_reports_t;


typedef struct
{
    double x, y, z;
    uint64_t id;
    std::string label;
    int tagRangeCorection[MAX_NUM_TAGS];
} anc_struct_t;

typedef struct
{
    double x, y, z;
    uint64_t id;
} pos_report_t;

typedef struct
{
    double x;
    double y;
} vec2d;





class PosGenerator
{


public:
    PosGenerator();
    PosGenerator(int algorithm, bool useRawRange, int tagOffset0, int tagOffset1, int tagOffset2, int tagOffset3, bool useTwoTags);


    void start(std::string configFilenamePos, std::string configFilenamePX4Flow, std::string configFilenameTag, std::string configFilenameImu, std::string configFilenameMag,ros::Publisher aPub, ros::Publisher aPathPub, ros::Publisher aOdomPub,double jolt);
    void start(std::string configFilenamePos,  std::string configFilenameTag, ros::Publisher aPub, ros::Publisher aPathPub, ros::Publisher aOdomPub,  double jolt);


    void setUseRawRange(bool useRawRange);
    void setTagOffset(int tagOffset0, int tagOffset1, int tagOffset2, int tagOffset3);
    void setObservationCovariance(std::vector<double> obsCov);
    //void setAlgorithm(int algorithm);
    void setAlgorithm(int algorithm, double accelerationNoise, bool ignoreWorstAnchorMode, double ignoreCostThreshold, bool mode3d);
    void setTwoTagsMode(bool twoTagsMode);
    void setTag0RelativePosition(double x, double y, double z);
    void setTag1RelativePosition(double x, double y, double z);

    void setVariantNormal();
    void setVariantIgnoreN(int numIgnoredRangings);
    void setVariantOnlyBest(int bestMode, double minZ, double maxZ);

    void setImuPublisher(ros::Publisher anImuPub, bool enabled);
    void setCompassPublisher(ros::Publisher aCompassPub, bool enabled);
    void setAnchorsPublisher(ros::Publisher anAnchorsPub, bool enabled);
    void publishFixedRateReport();


    bool init( std::string filenamePos, std::string filenamePX4Flow, std::string filenameTag, std::string filenameImu, std::string filenameMag);
    bool init(std::string filenamePos, std::string filenameTag);
    void reset();

    void newUWBMeasurement(const gtec_msgs::Ranging::ConstPtr& uwbRanging);
    void newPX4FlowMeasurement(const mavros_msgs::OpticalFlowRad::ConstPtr& px4FlowMeasurement);
    void newIMUMeasurement(const sensor_msgs::Imu::ConstPtr& imuMeasurement);
    void newMAGMeasurement(const sensor_msgs::MagneticField::ConstPtr& magMeasurement);
    void newCompassMeasurement(const std_msgs::Float64 compasMeasurement);

    void newAnchorsMarkerArray(const visualization_msgs::MarkerArray::ConstPtr& anchorsMarkerArray);


private:
    ros::Publisher ros_pub;
    ros::Publisher ros_pub_path;
    ros::Publisher ros_pub_anchors;
    ros::Publisher ros_pub_imu;
    ros::Publisher ros_pub_compass;
    ros::Publisher ros_pub_odom;

    void timerRangingCallback(const ros::TimerEvent& event);

    ros::Timer timerRanging;
    bool rangingTimeout;

    bool publishImu;
    bool publishAnchors;
    bool publishCompass;
    double mJolt;

    int maxPathSize = 1000;
    int currentPathIndex = 0;
    std::vector<geometry_msgs::PoseStamped> path;

    void publishPositionReport(Vector3 report);

    anc_struct_t _ancArray[MAX_NUM_ANCS];
    std::vector <tag_reports_t> _tagList;
    bool useRawRange;
    bool useTwoTags;
    bool useInitPosition;
    Vector3 initPosition;

    bool useFixedHeightUWB;
    double fixedHeightUWB;
    bool usePX4Flow;
    bool useFixedHeightPX4Flow;
    double armP0PX4Flow;
    double armP1PX4Flow;
    double fixedHeightPX4Flow;
    double initAnglePX4Flow;
    double covarianceVelocityPX4Flow;
    double covarianceGyroZPX4Flow;


    bool useImu;
    double imuCovarianceAcceleration;
    bool useImuFixedCovarianceAngularVelocityZ;
    bool useImuFixedCovarianceAcceleration;
    double imuCovarianceAngularVelocityZ;

    bool useMag;
    double magAngleOffset;
    double covarianceMag;

    int algorithm;
    int tagOffset0;
    int tagOffset1;
    int tagOffset2;
    int tagOffset3;
    std::vector<bool> twoTagsReady;
    std::vector<double> observationCovariance;

    int twoTagsLastRanges[2][4];
    double twoTagsLastErrorEstimation[2][4];
    void initialiseTagList(int id);
    int calculateTagLocationWithRangings(Vector3 *report, int count, int *ranges, double *errorEstimation, double timeLag);
    void processRanging(int anchorId, int tagId, double range, double rawrange, double errorEstimation, int seq, bool withErrorEstimation);
    void processRangingNow(int anchorId, int tagId, double range, double rawrange, double errorEstimation, int seq, bool withErrorEstimation);
    

    void sendRangingMeasurementIfAvailable(tag_reports_t tagReport);
    Vector3 previousPos;
    std::unique_ptr<KalmanFilter> kalmanFilter;
    std::vector<Beacon> beacons;
    Vector3 tag0RelativePosition;
    Vector3 tag1RelativePosition;

    std::chrono::steady_clock::time_point mLastRangingPositionTimestamp;

    //ML Variants
    int variant;

    int numIgnoredRangings;

    int bestMode;
    double minZ;
    double maxZ;

    int lastRangingSeq;
    std::chrono::steady_clock::time_point timestampLastRanging;

    bool anchorsSet;
    bool mode3d;

    tf::TransformBroadcaster mBroadcaster;


};

#endif // POSGENERATOR_PX4FLOW_H
