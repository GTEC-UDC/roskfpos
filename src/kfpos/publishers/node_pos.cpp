
#include "Posgenerator.h"
#include "PositionEstimationAlgorithm.h"
#include <mavros_msgs/OpticalFlowRad.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Twist.h>


int main(int argc, char *argv[])
{

    ros::init(argc, argv, "kfpos");
    ros::NodeHandle n("~");

    ros::Subscriber sub0, sub1, sub2, sub3, sub4;
    PosGenerator aPosGenerator;
    int currentAlgorithm = ALGORITHM_KF;
    int rate;
    std::string targetDeviceId, tagId, algorithm, rangingTopic, anchorsTopic, imuTopic, magTopic, px4Topic;
    int usePX4Flow = 0, useTOA = 1, useImu = 0, useMag = 0;

    //Dynamic properties
    double jolt, accelNoise;

    //Init position
    double initAngle = 0;
    double initPositionX = 0, initPositionY = 0, initPositionZ = 0;
    int useStartPosition;

    //KF ignore
    bool useHeuristicIgnoreWorst = false;
    double heuristicIgnoreThreshold = 0;

    //ML
    bool use2d = false;
    int variant = 0;
    int numRangingsToIgnore = 0;


    std::string nodeName = ros::this_node::getName();

    n.getParam("algorithm", algorithm);

    if ((algorithm.compare("ALGORITHM_KF_TOA")) == 0){
        currentAlgorithm = ALGORITHM_KF_TOA;
    } else if ((algorithm.compare("ALGORITHM_KF_TOA_IMU")) == 0){
        currentAlgorithm = ALGORITHM_KF_TOA_IMU;
    } else if ((algorithm.compare("ALGORITHM_KF")) == 0) {
        currentAlgorithm = ALGORITHM_KF;
    } else if ((algorithm.compare("ALGORITHM_ML")) == 0) {
        currentAlgorithm = ALGORITHM_ML;
    }

    //Common parameters
    n.getParam("rate", rate);
    n.getParam("targetDeviceId", targetDeviceId);
    n.getParam("toaTagId", tagId);
    n.getParam("useStartPosition", useStartPosition);
    n.getParam("rangingTopic", rangingTopic);
    n.getParam("anchorsTopic", anchorsTopic);

    if (useStartPosition==1){
        n.getParam("initPositionX", initPositionX);
        n.getParam("initPositionY", initPositionY);
        n.getParam("initPositionZ", initPositionZ);
        n.getParam("initAngle", initAngle); 
    }

    if (currentAlgorithm == ALGORITHM_ML){
        //ML
        ROS_INFO("Algorithm: ALGORITHM_ML");

        n.getParam("use2d", use2d);
        n.getParam("variant", variant);
        n.getParam("numRangingsToIgnore", numRangingsToIgnore);

        aPosGenerator.setStartParameters(useStartPosition==1, initPositionX, initPositionY, initPositionZ, initAngle);
        aPosGenerator.setHeuristicML(use2d, variant, numRangingsToIgnore);
    } else {
        //KF variants
        n.getParam("accelNoise", accelNoise);
        n.getParam("jolt", jolt);
        
        if (currentAlgorithm == ALGORITHM_KF){
            ROS_INFO("Algorithm: ALGORITHM_KF");
            n.getParam("imuTopic", imuTopic);
            n.getParam("magTopic", anchorsTopic);
            n.getParam("px4Topic", px4Topic);
            n.getParam("usePX4Flow", usePX4Flow);
            n.getParam("useTOA", useTOA);
            n.getParam("useIMU", useImu);
            n.getParam("useMAG", useMag);
        } else if (currentAlgorithm == ALGORITHM_KF_TOA){
            ROS_INFO("Algorithm: ALGORITHM_KF_TOA");
            n.getParam("useHeuristicIgnoreWorst", useHeuristicIgnoreWorst);
            n.getParam("heuristicIgnoreThreshold", heuristicIgnoreThreshold);
            useTOA = 1;
        } else if (currentAlgorithm == ALGORITHM_KF_TOA_IMU){
            ROS_INFO("Algorithm: ALGORITHM_KF_TOA_IMU");
            n.getParam("imuTopic", imuTopic);
            useTOA = 1;
            useImu = 1;
        }

        aPosGenerator.setExternalFilesParameters("configPos", "configPX4Flow", "configUWB", "configIMU", "configMAG");
        aPosGenerator.setDynamicParameters(accelNoise, jolt);
        aPosGenerator.setStartParameters(useStartPosition==1, initPositionX, initPositionY, initPositionZ, initAngle);
        aPosGenerator.setHeuristicIgnore(useHeuristicIgnoreWorst, heuristicIgnoreThreshold);
    }


    //Publishers creation
    std::ostringstream stringStream;
    stringStream << "/gtec/" << nodeName.c_str() << "/" << targetDeviceId.c_str();
    std::string topicPos = stringStream.str();

    stringStream.str("");
    stringStream.clear();
    stringStream << "/gtec/" << nodeName.c_str() << "/path" << "/" << targetDeviceId.c_str();
    std::string topicPathPos = stringStream.str();

    stringStream.str("");
    stringStream.clear();
    stringStream << "/gtec/" << nodeName.c_str() << "/odom" << "/" << targetDeviceId.c_str();
    std::string topicOdomPos = stringStream.str();

    ros::Publisher gtec_pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(topicPos, 200);
    ros::Publisher gtec_pose_parth_pub = n.advertise<nav_msgs::Path>(topicPathPos, 200);
    ros::Publisher gtec_odom_pub = n.advertise<nav_msgs::Odometry>(topicOdomPos, 200);

    aPosGenerator.setPublishers(gtec_pose_pub, gtec_pose_parth_pub, gtec_odom_pub);
    
    int tagIdInt;   
    std::stringstream ss;
    ss << std::hex << tagId.c_str();
    ss >> tagIdInt;

    aPosGenerator.setDeviceIdentifiers(tagIdInt);
    aPosGenerator.setAlgorithm(currentAlgorithm);

    if (useTOA==1)
    {
        ROS_INFO("Position Generator: TOA ON. Ranging Topic: %s", rangingTopic.c_str());
        ROS_INFO("Position Generator: Anchors Topic: %s", anchorsTopic.c_str());
        sub0 = n.subscribe<gtec_msgs::Ranging>(rangingTopic, 20, &PosGenerator::newTOAMeasurement, &aPosGenerator);
        sub1 = n.subscribe<visualization_msgs::MarkerArray>(anchorsTopic, 20, &PosGenerator::newAnchorsMarkerArray, &aPosGenerator);
    }

    if (usePX4Flow==1)
    {
        ROS_INFO("Position Generator: PX4Flow ON. Topic: %s", px4Topic.c_str());
        sub2 = n.subscribe<mavros_msgs::OpticalFlowRad>(px4Topic, 20, &PosGenerator::newPX4FlowMeasurement, &aPosGenerator);
    }

    if (useImu==1)
    {
        ROS_INFO("Position Generator: IMU ON. Topic: %s", imuTopic.c_str());
        sub3 = n.subscribe<sensor_msgs::Imu>(imuTopic, 20, &PosGenerator::newIMUMeasurement, &aPosGenerator);
    }

    if (useMag==1)
    {
        ROS_INFO("Position Generator: MAG ON. Topic: %s", magTopic.c_str());
        sub4 = n.subscribe<std_msgs::Float64>(magTopic, 20, &PosGenerator::newCompassMeasurement, &aPosGenerator);
    }


    ROS_INFO("Position Generator. Rate: %d Hz", rate);
    ros::Rate r(rate);
    while (ros::ok())
    {
        ros::spinOnce();
        aPosGenerator.publishFixedRateReport();
        r.sleep();
    }

    return 0;
}
