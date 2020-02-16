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
#include <unordered_map>

#include "../algorithms/MLLocation.h"
#include "../algorithms/KalmanFilter.h"
#include "../algorithms/KalmanFilterTOA.h"
#include "../algorithms/KalmanFilterTOAIMU.h"
#include "../algorithms/PositionEstimationAlgorithm.h"

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
#define ALGORITHM_KF_TOA 5
#define ALGORITHM_KF_TOA_IMU 6

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
    int rangeValue[256][MAX_NUM_ANCS];
    double errorEstimation[256][MAX_NUM_ANCS];
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

    void start(std::string configFilenamePos, std::string configFilenamePX4Flow, std::string configFilenameTag, std::string configFilenameImu, std::string configFilenameMag,ros::Publisher aPub, ros::Publisher aPathPub, ros::Publisher aOdomPub,double jolt);
    void start(std::string configFilenamePos,  std::string configFilenameTag, ros::Publisher aPub, ros::Publisher aPathPub, ros::Publisher aOdomPub,  double jolt);
    void start(ros::Publisher aPub, ros::Publisher aPathPub, ros::Publisher aOdomPub, int tagId, double jolt);


    void setPublishers(ros::Publisher aPub, ros::Publisher aPathPub, ros::Publisher aOdomPub);
    void setDynamicParameters(double accelerationNoise, double jolt);
    void setExternalFilesParameters(std::string configFilenamePos, std::string configFilenamePX4Flow, std::string configFilenameTag, std::string configFilenameImu, std::string configFilenameMag);
    void setStartParameters(bool useStartPosition, double startPositionX, double startPositionY,double startPositionZ,double  startAngle);
    void setHeuristicIgnore(bool ignoreWorstAnchorMode, double ignoreCostThreshold);
    void setDeviceIdentifiers(int toaTagId);
    void setHeuristicML(bool use2d, int variant, int numRangingsToIgnore);


    void setAlgorithm(int algorithm);

    void publishFixedRateReport();
    void reset();

    void newTOAMeasurement(const gtec_msgs::Ranging::ConstPtr& uwbRanging);
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

    double mInitAngle;
    double mAccelerationNoise;
    double mJolt;

    bool mIgnoreWorstAnchorMode = false;
    double mIgnoreCostThreshold = 0.0;

    int mTOATagId;

    std::string mFilenamePos, mFilenamePX4Flow, mFilenameTag, mFilenameImu, mFilenameMag;

    int maxPathSize = 1000;
    int currentPathIndex = 0;
    std::vector<geometry_msgs::PoseStamped> path;

    void publishPositionReport(Vector3 report);

    anc_struct_t _ancArray[MAX_NUM_ANCS];
    std::vector <tag_reports_t> _tagList;
    std::unordered_map<int, int> _anchorIndexById;

    bool mUseInitPosition;
    Vector3 mInitPosition;


    bool mUse2d;
    int mVariant;
    int mNumRangingsToIgnore;



    void initialiseTagList(int id);
    int calculateTagLocationWithRangings(Vector3 *report, int count, int *ranges, double *errorEstimation, double timeLag);
    void processRanging(int anchorId, int tagId, double range, double rawrange, double errorEstimation, int seq, bool withErrorEstimation);
    void processRangingNow(int anchorId, int tagId, double range, double rawrange, double errorEstimation, int seq, bool withErrorEstimation);
    

    void sendRangingMeasurementIfAvailable(tag_reports_t tagReport);
    Vector3 previousPos;

    std::unique_ptr<PositionEstimationAlgorithm> mPositionAlgorithm;
    std::vector<Beacon> beacons;

    std::chrono::steady_clock::time_point mLastRangingPositionTimestamp;

    int lastRangingSeq;
    std::chrono::steady_clock::time_point timestampLastRanging;

    bool anchorsSet;
    bool mode3d;

    tf::TransformBroadcaster mBroadcaster;


};

#endif // POSGENERATOR_PX4FLOW_H
