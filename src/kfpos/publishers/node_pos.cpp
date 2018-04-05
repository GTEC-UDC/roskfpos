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

#include "Posgenerator.h"
#include <mavros_msgs/OpticalFlowRad.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Float64.h>


int main(int argc, char *argv[])
{

    ros::init(argc, argv, "kfpos");
    ros::NodeHandle n("~");

    int rate;
    n.getParam("rate", rate);
    
    ros::Publisher gtec_uwb_pos_px4flow_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/gtec/kfpos", 200);
    ros::Publisher gtec_uwb_pos_px4flow_pub_path = n.advertise<nav_msgs::Path>("/gtec/kfpos/path", 200);
    //ros::Publisher gtec_uwb_pos_px4flow_pub_imu = n.advertise<sensor_msgs::Imu>("gtec/mavros/imu", 200);
    //ros::Publisher gtec_uwb_pos_px4flow_pub_compass = n.advertise<std_msgs::Float64>("gtec/mavros/compass", 200);

    PosGenerator aPosGenerator;
    aPosGenerator.start("configPos", "configPX4Flow", "configTag", "configErleImu", "configErleMag",gtec_uwb_pos_px4flow_pub, gtec_uwb_pos_px4flow_pub_path);
    aPosGenerator.setAlgorithm(ALGORITHM_KF); 


    bool usePX4Flow=false, useRanging=false, useErleImu=false, useErleMag=false, useAnchors=false;
    std::string px4flowTopic, rangingTopic, erleImuTopic, erleMagTopic, anchorsTopic;

  try {

    //************************************
    //Cargamos la configuracion de las suscripciones
    //************************************

    boost::property_tree::ptree configTree;
    std::string configContent;
    n.getParam("configSubscriptions", configContent);
    std::stringstream ssConfig;
    ssConfig << configContent;
    boost::property_tree::read_xml(ssConfig, configTree);

    BOOST_FOREACH(const boost::property_tree::ptree::value_type & v, configTree.get_child("config")) {
      if (v.first.compare("px4flow") == 0) {
        usePX4Flow = (v.second.get<int>("<xmlattr>.use", 0))==1;
        px4flowTopic = v.second.get<std::string>("<xmlattr>.topic", "");
      } else if (v.first.compare("ranging") == 0) {
        useRanging = (v.second.get<int>("<xmlattr>.use", 0))==1;
        rangingTopic = v.second.get<std::string>("<xmlattr>.topic", "");
      }else if (v.first.compare("erleimu") == 0) {
        useErleImu = (v.second.get<int>("<xmlattr>.use", 0))==1;
        erleImuTopic = v.second.get<std::string>("<xmlattr>.topic", "");
      }else if (v.first.compare("erlemag") == 0) {
        useErleMag = (v.second.get<int>("<xmlattr>.use", 0))==1;
        bool withInterference = (v.second.get<int>("<xmlattr>.interference", 0))==1;
        if (withInterference){
           erleMagTopic = v.second.get<std::string>("<xmlattr>.topicInterference", "");
        } else {
           erleMagTopic = v.second.get<std::string>("<xmlattr>.topic", ""); 
        }
        
      }else if (v.first.compare("anchors") == 0) {
        useAnchors = (v.second.get<int>("<xmlattr>.use", 0))==1;
        anchorsTopic = v.second.get<std::string>("<xmlattr>.topic", "");
      }
    }

  } catch (boost::exception const &ex) {
    ROS_INFO("Read error in config_subscriptions.xml");
  }

  if (n.hasParam("use_uwb")){
    int use_uwb_param;
    n.getParam("use_uwb", use_uwb_param);
    useRanging = (use_uwb_param ==1);
  }

if (n.hasParam("use_imu")){
    int use_imu_param;
    n.getParam("use_imu", use_imu_param);
    useErleImu = (use_imu_param ==1);
  }

if (n.hasParam("use_mag")){
    int use_mag_param;
    n.getParam("use_mag", use_mag_param);
    useErleMag = (use_mag_param ==1);
  }

if (n.hasParam("use_px4")){
    int use_px4_param;
    n.getParam("use_px4", use_px4_param);
    usePX4Flow = (use_px4_param ==1);
  }

    //ros::Subscriber sub0 = n.subscribe<gtec_msgs::Ranging>("/gtec/toa/ranging", 12, &PosGenerator::newRanging, &aPosGenerator);
    //ros::Subscriber sub1 = n.subscribe<mavros_msgs::OpticalFlowRad>("/px4flow/px4flow/raw/optical_flow_rad", 200, &PosGenerator::newPX4FlowMeasurement, &aPosGenerator);
    
    //ros::Subscriber sub2 = n.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 200, &PosGenerator::newErleImuMeasurement, &aPosGenerator);
    //ros::Subscriber sub2 = n.subscribe<sensor_msgs::Imu>("/gtec/mavros/imu", 200, &PosGenerator::newErleImuMeasurement, &aPosGenerator);
    

    //ros::Subscriber sub3 = n.subscribe<sensor_msgs::MagneticField>("/mavros/imu/mag", 200, &PosGenerator::newErleMagMeasurement, &aPosGenerator);
    //ros::Subscriber sub3 = n.subscribe<std_msgs::Float64>("/mavros/global_position/compass_hdg", 200, &PosGenerator::newErleCompassMeasurement, &aPosGenerator);
    // Lo siguiente es para leer desde el log.
    //ros::Subscriber sub3 = n.subscribe<std_msgs::Float64>("/gtec/mavros/compass", 200, &PosGenerator::newErleCompassMeasurement, &aPosGenerator);

    //ros::Subscriber sub4 = n.subscribe<visualization_msgs::MarkerArray>("/gtec/toa/anchors", 2, &PosGenerator::newAnchorsMarkerArray, &aPosGenerator);

    ros::Subscriber sub0, sub1, sub2, sub3, sub4;

    if (useRanging){
        ROS_INFO("KFPOS: Ranging ON. Topic: %s", rangingTopic.c_str());
        sub0 = n.subscribe<gtec_msgs::Ranging>(rangingTopic, 20, &PosGenerator::newRanging, &aPosGenerator); 
    }

    if (usePX4Flow){
        ROS_INFO("KFPOS: PX4Flow ON. Topic: %s", px4flowTopic.c_str());
        sub1 = n.subscribe<mavros_msgs::OpticalFlowRad>(px4flowTopic, 20, &PosGenerator::newPX4FlowMeasurement, &aPosGenerator);
    }

    if (useErleImu){
        ROS_INFO("KFPOS: ErleIMU ON. Topic: %s", erleImuTopic.c_str());
        sub2 = n.subscribe<sensor_msgs::Imu>(erleImuTopic, 20, &PosGenerator::newErleImuMeasurement, &aPosGenerator);
    }

    if (useErleMag){
        ROS_INFO("KFPOS: ErleMag ON. Topic: %s", erleMagTopic.c_str());
        sub3 = n.subscribe<std_msgs::Float64>(erleMagTopic, 20, &PosGenerator::newErleCompassMeasurement, &aPosGenerator);
   
    }
    if (useAnchors){
        ROS_INFO("KFPOS: Anchors ON. Topic: %s", anchorsTopic.c_str());
        sub4 = n.subscribe<visualization_msgs::MarkerArray>(anchorsTopic, 20, &PosGenerator::newAnchorsMarkerArray, &aPosGenerator);
    }


  // ros::Rate rate(100.0);
  // while (n.ok()){
  //   ros::spinOnce();
  //   rate.sleep();
  // }

    ros::Rate r(rate);
    while (ros::ok())
    {
        ros::spinOnce();  
        aPosGenerator.publishFixedRateReport();
        r.sleep();
    }

    return 0;
}
