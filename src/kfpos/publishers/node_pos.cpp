
#include "Posgenerator.h"
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

    int rate;
    double jolt, accelNoise, magAngleOffset;

    std::string nodeName = ros::this_node::getName();

    n.getParam("rate", rate);
    n.getParam("jolt", jolt);
    n.getParam("accelNoise", accelNoise);
    n.getParam("magAngleOffset", magAngleOffset);

     std::ostringstream stringStream;
     stringStream << "/gtec/" << nodeName.c_str();
     std::string topicPos = stringStream.str();

     stringStream.str("");
     stringStream.clear();
    stringStream << "/gtec/" << nodeName.c_str() << "/path";
    std::string topicPathPos = stringStream.str();

    stringStream.str("");
    stringStream.clear();
    stringStream << "/gtec/" << nodeName.c_str() << "/odom";
    std::string topicOdomPos = stringStream.str();

    ros::Publisher gtec_uwb_pos_px4flow_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(topicPos, 200);
    ros::Publisher gtec_uwb_pos_px4flow_pub_path = n.advertise<nav_msgs::Path>(topicPathPos, 200);
    ros::Publisher gtec_uwb_pos_odom = n.advertise<nav_msgs::Odometry>(topicOdomPos, 200);

    PosGenerator aPosGenerator;
    aPosGenerator.start("configPos", "configPX4Flow", "configUWB", "configIMU", "configMAG", gtec_uwb_pos_px4flow_pub, gtec_uwb_pos_px4flow_pub_path, gtec_uwb_pos_odom, jolt);
    aPosGenerator.setAlgorithm(ALGORITHM_KF, accelNoise, false, magAngleOffset, false);

    bool usePX4Flow = false, useUwb = false, useImu = false, useMag = false;
    std::string px4flowTopic, rangingTopic, imuTopic, magTopic, anchorsTopic;

    try
    {

        boost::property_tree::ptree configTree;
        std::string configContent;
        n.getParam("configSubscriptions", configContent);
        std::stringstream ssConfig;
        ssConfig << configContent;
        boost::property_tree::read_xml(ssConfig, configTree);

        BOOST_FOREACH(const boost::property_tree::ptree::value_type & v, configTree.get_child("config"))
        {
            if (v.first.compare("px4flow") == 0)
            {
                usePX4Flow = (v.second.get<int>("<xmlattr>.use", 0)) == 1;
                px4flowTopic = v.second.get<std::string>("<xmlattr>.topic", "");
            }
            else if (v.first.compare("uwb") == 0)
            {
                useUwb = (v.second.get<int>("<xmlattr>.use", 0)) == 1;
                rangingTopic = v.second.get<std::string>("<xmlattr>.topic", "");
                anchorsTopic = v.second.get<std::string>("<xmlattr>.anchors_topic", "");
            }
            else if (v.first.compare("imu") == 0)
            {
                useImu = (v.second.get<int>("<xmlattr>.use", 0)) == 1;
                imuTopic = v.second.get<std::string>("<xmlattr>.topic", "");
            }
            else if (v.first.compare("mag") == 0)
            {
                useMag = (v.second.get<int>("<xmlattr>.use", 0)) == 1;
                bool withInterference = (v.second.get<int>("<xmlattr>.interference", 0)) == 1;
                if (withInterference)
                {
                    magTopic = v.second.get<std::string>("<xmlattr>.topicInterference", "");
                }
                else
                {
                    magTopic = v.second.get<std::string>("<xmlattr>.topic", "");
                }
            }
        }

    }
    catch (boost::exception const &ex)
    {
        ROS_INFO("Read error in config_subscriptions.xml");
    }

    if (n.hasParam("use_uwb"))
    {
        int use_uwb_param;
        n.getParam("use_uwb", use_uwb_param);
        useUwb = (use_uwb_param == 1);
    }

    if (n.hasParam("use_imu"))
    {
        int use_imu_param;
        n.getParam("use_imu", use_imu_param);
        useImu = (use_imu_param == 1);
    }

    if (n.hasParam("use_mag"))
    {
        int use_mag_param;
        n.getParam("use_mag", use_mag_param);
        useMag = (use_mag_param == 1);
    }

    if (n.hasParam("use_px4"))
    {
        int use_px4_param;
        n.getParam("use_px4", use_px4_param);
        usePX4Flow = (use_px4_param == 1);
    }

    ros::Subscriber sub0, sub1, sub2, sub3, sub4;

    if (useUwb)
    {
        ROS_INFO("KFPOS: UWB ON. Ranging Topic: %s", rangingTopic.c_str());
        ROS_INFO("KFPOS: Anchors Topic: %s", anchorsTopic.c_str());
        sub0 = n.subscribe<gtec_msgs::Ranging>(rangingTopic, 20, &PosGenerator::newUWBMeasurement, &aPosGenerator);
        sub4 = n.subscribe<visualization_msgs::MarkerArray>(anchorsTopic, 20, &PosGenerator::newAnchorsMarkerArray, &aPosGenerator);
    }

    if (usePX4Flow)
    {
        ROS_INFO("KFPOS: PX4Flow ON. Topic: %s", px4flowTopic.c_str());
        sub1 = n.subscribe<mavros_msgs::OpticalFlowRad>(px4flowTopic, 20, &PosGenerator::newPX4FlowMeasurement, &aPosGenerator);
    }

    if (useImu)
    {
        ROS_INFO("KFPOS: IMU ON. Topic: %s", imuTopic.c_str());
        sub2 = n.subscribe<sensor_msgs::Imu>(imuTopic, 20, &PosGenerator::newIMUMeasurement, &aPosGenerator);
    }

    if (useMag)
    {
        ROS_INFO("KFPOS: MAG ON. Topic: %s", magTopic.c_str());
        sub3 = n.subscribe<std_msgs::Float64>(magTopic, 20, &PosGenerator::newCompassMeasurement, &aPosGenerator);
    }

    ros::Rate r(rate);
    while (ros::ok())
    {
        ros::spinOnce();
        aPosGenerator.publishFixedRateReport();
        r.sleep();
    }

    return 0;
}
