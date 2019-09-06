#include "Posgenerator.h"


PosGenerator::PosGenerator(int algorithm, bool useRawRange, int tagOffset0, int tagOffset1, int tagOffset2, int tagOffset3, bool useTwoTags) {
  this->useRawRange = useRawRange;
  this->tagOffset0 = tagOffset0;
  this->tagOffset1 = tagOffset1;
  this->tagOffset2 = tagOffset2;
  this->tagOffset3 = tagOffset3;
  this->algorithm = algorithm;
  this->useTwoTags = useTwoTags;
  this->variant = ML_VARIANT_NORMAL;
  this->mLastRangingPositionTimestamp =  std::chrono::steady_clock::time_point::min();
  this->timestampLastRanging = std::chrono::steady_clock::time_point::min();
  this->lastRangingSeq = -1;

  this->publishImu = false;
  this->publishAnchors = false;
  this->publishCompass = false;

  for (int i = 0; i < 2; i++) {
    twoTagsReady.push_back((bool) false);
  }
  this->anchorsSet = false;

  this->rangingTimeout = false;
  ros::NodeHandle n("~");
  this->timerRanging = n.createTimer(ros::Duration(MAX_TIME_TO_SEND_RANGING),  &PosGenerator::timerRangingCallback,this, true);

}




PosGenerator::PosGenerator() {
  this->useRawRange = false;
  this->tagOffset0 = 0;
  this->tagOffset1 = 0;
  this->tagOffset2 = 0;
  this->tagOffset3 = 0;
  this->algorithm = ALGORITHM_ML;
  this->useTwoTags = false;
  this->variant = ML_VARIANT_NORMAL;
  this->mLastRangingPositionTimestamp =  std::chrono::steady_clock::time_point::min();
  this->timestampLastRanging = std::chrono::steady_clock::time_point::min();
  this->lastRangingSeq = -1;

  this->publishImu = false;
  this->publishAnchors = false;
  this->publishCompass = false;

  for (int i = 0; i < 2; i++) {
    twoTagsReady.push_back((bool) false);
  }

  this->anchorsSet = false;
  this->rangingTimeout = false;
  ros::NodeHandle n("~");
  this->timerRanging = n.createTimer(ros::Duration(MAX_TIME_TO_SEND_RANGING), &PosGenerator::timerRangingCallback,this, true);


}


void PosGenerator::newAnchorsMarkerArray(const visualization_msgs::MarkerArray::ConstPtr& anchorsMarkerArray){


  if (!anchorsSet && anchorsMarkerArray->markers.size()<=MAX_NUM_ANCS){
    for (int i = 0; i < anchorsMarkerArray->markers.size(); ++i)
    {
      visualization_msgs::Marker marker = anchorsMarkerArray->markers[i];
      _ancArray[i].id = i;
      _ancArray[i].label = "";
      _ancArray[i].x = marker.pose.position.x;
      _ancArray[i].y = marker.pose.position.y;
      _ancArray[i].z = marker.pose.position.z;
      beacons.push_back({ i, { _ancArray[i].x, _ancArray[i].y, _ancArray[i].z } });
    }
    anchorsSet = true;
    ROS_INFO("Anchors set");

    for (int i = 0; i < beacons.size(); ++i)
    {
      
      Beacon beacon= beacons[i];
      ROS_INFO("Anchors: %d  (%f, %f, %f)", i, beacon.position.x, beacon.position.y, beacon.position.z);
    }
  }
  
}


void PosGenerator::start(std::string configFilenamePos, std::string configFilenamePX4Flow, std::string configFilenameTag, std::string configFilenameImu, std::string configFilenameMag, ros::Publisher aPub, ros::Publisher aPathPub, ros::Publisher aOdomPub, double jolt) {

  bool configLoaded = init(configFilenamePos, configFilenamePX4Flow, configFilenameTag, configFilenameImu, configFilenameMag);
  ros_pub = aPub;
  ros_pub_path = aPathPub;
  ros_pub_odom = aOdomPub;
  mJolt = jolt;
}

void PosGenerator::start(std::string configFilenamePos,  std::string configFilenameTag, ros::Publisher aPub, ros::Publisher aPathPub, ros::Publisher aOdomPub,  double jolt) {
  bool configLoaded = init(configFilenamePos, configFilenameTag);
  ros_pub = aPub;
  ros_pub_path = aPathPub;
  ros_pub_odom = aOdomPub;
  mJolt = jolt;
}

bool PosGenerator::init(std::string filenamePos, std::string filenameTag) {

  try {

    //************************************
    // UWB Tag
    //************************************

    boost::property_tree::ptree configTreeTag;
    ros::NodeHandle node_handleTag("~");
    std::string configContentTag;
    node_handleTag.getParam(filenameTag, configContentTag);
    std::stringstream ssTag;
    ssTag << configContentTag;
    boost::property_tree::read_xml(ssTag, configTreeTag);

    BOOST_FOREACH(const boost::property_tree::ptree::value_type & v, configTreeTag.get_child("config")) {
      if (v.first.compare("uwb") == 0) {
        int use = v.second.get<int>("<xmlattr>.useFixedHeight", 0);
        useFixedHeightUWB = (use == 1);
        fixedHeightUWB = v.second.get<double>("<xmlattr>.fixedHeight", 0);
      }
    }

    //************************************
    // Position Algorithm
    //************************************

    boost::property_tree::ptree configTreePos;
    ros::NodeHandle node_handlePos("~");
    std::string configContentPos;
    node_handlePos.getParam(filenamePos, configContentPos);
    std::stringstream ssPos;
    ssPos << configContentPos;
    boost::property_tree::read_xml(ssPos, configTreePos);


    BOOST_FOREACH(const boost::property_tree::ptree::value_type & v, configTreePos.get_child("config")) {
      if (v.first.compare("algorithm") == 0) {

        int type = v.second.get<int>("<xmlattr>.type", 0);
        int variant = v.second.get<int>("<xmlattr>.variant", 0);
        int numIgnoredRangings = v.second.get<int>("<xmlattr>.numIgnoredRangings", 0);
        int bestMode = v.second.get<int>("<xmlattr>.bestMode", 0);
        double minZ = v.second.get<double>("<xmlattr>.minZ", 0);
        double maxZ = v.second.get<double>("<xmlattr>.maxZ", 0);

        useInitPosition = 0;

        int hasInitPosition = v.second.get<int>("<xmlattr>.useInitPosition", 0);
        useInitPosition = (hasInitPosition == 1);
        if (useInitPosition) {
          initPosition.x = v.second.get<double>("<xmlattr>.initX", 0);
          initPosition.y = v.second.get<double>("<xmlattr>.initY", 0);
          initPosition.z = v.second.get<double>("<xmlattr>.initZ", 0);
        }

        if (variant == 0) {
          setVariantNormal();
          ROS_INFO("POSGEN: Variant NORMAL");
        } else if (variant == 1) {
          setVariantIgnoreN(numIgnoredRangings);
          ROS_INFO("POSGEN: Variant IGNORE N. Ignored: %d", numIgnoredRangings);
        } else  if (variant == 2) {
          setVariantOnlyBest(bestMode, minZ, maxZ);
          ROS_INFO("POSGEN: Variant Best N:. BestMode: %d minZ: %f mazZ: %f", bestMode, minZ, maxZ);
        }

      }
    }

    return true;

  } catch (boost::exception const &ex) {
    return false;
  }


  return false;
}




bool PosGenerator::init(std::string filenamePos, std::string filenamePX4Flow, std::string filenameTag, std::string filenameImu, std::string filenameMag) {

  try {

    //************************************
    // PX4Flow
    //************************************

    boost::property_tree::ptree configTreePX4Flow;
    ros::NodeHandle node_handlePX4Flow("~");
    std::string configContentPX4Flow;
    node_handlePX4Flow.getParam(filenamePX4Flow, configContentPX4Flow);
    std::stringstream ssPX4Flow;
    ssPX4Flow << configContentPX4Flow;
    boost::property_tree::read_xml(ssPX4Flow, configTreePX4Flow);

    BOOST_FOREACH(const boost::property_tree::ptree::value_type & v, configTreePX4Flow.get_child("config")) {
      if (v.first.compare("px4flow") == 0) {
        usePX4Flow = true;
        int useFH = v.second.get<int>("<xmlattr>.useFixedSensorHeight", 0);
        useFixedHeightPX4Flow = (useFH == 1);

        fixedHeightPX4Flow = v.second.get<double>("<xmlattr>.sensorHeight", 0);
        armP0PX4Flow = v.second.get<double>("<xmlattr>.armP0", 0);
        armP1PX4Flow = v.second.get<double>("<xmlattr>.armP1", 0);
        initAnglePX4Flow = v.second.get<double>("<xmlattr>.sensorInitAngle", 0);

        covarianceVelocityPX4Flow = v.second.get<double>("<xmlattr>.covarianceVelocity", 0);
        covarianceGyroZPX4Flow = v.second.get<double>("<xmlattr>.covarianceGyroZ", 0);
      }
    }

    //************************************
    // UWB
    //************************************

    boost::property_tree::ptree configTreeTag;
    ros::NodeHandle node_handleTag("~");
    std::string configContentTag;
    node_handleTag.getParam(filenameTag, configContentTag);
    std::stringstream ssTag;
    ssTag << configContentTag;
    boost::property_tree::read_xml(ssTag, configTreeTag);

    BOOST_FOREACH(const boost::property_tree::ptree::value_type & v, configTreeTag.get_child("config")) {
      if (v.first.compare("uwb") == 0) {
        int use = v.second.get<int>("<xmlattr>.useFixedHeight", 0);
        useFixedHeightUWB = (use == 1);
        fixedHeightUWB = v.second.get<double>("<xmlattr>.fixedHeight", 0);
      }
    }


    //************************************
    // IMU
    //************************************

    boost::property_tree::ptree configTreeImu;
    ros::NodeHandle node_handleImu("~");
    std::string configContentImu;
    node_handleImu.getParam(filenameImu, configContentImu);
    std::stringstream ssImu;
    ssImu << configContentImu;
    boost::property_tree::read_xml(ssImu, configTreeImu);

    BOOST_FOREACH(const boost::property_tree::ptree::value_type & v, configTreeImu.get_child("config")) {
      if (v.first.compare("imu") == 0) {
        //use="1" covarianceAcceleration="0.01"  useFixedCovarianceAngularVelocityZ="1" covarianceAngularVelocityZ="0.001" angleOffset="-0.4" covarianceOrientationZ="0.01"
        //int use = v.second.get<int>("<xmlattr>.use", 0);
        useImu = true;

        int useFCA = v.second.get<int>("<xmlattr>.useFixedCovarianceAcceleration", 0);
        useImuFixedCovarianceAcceleration = (useFCA == 1);
        imuCovarianceAcceleration = v.second.get<double>("<xmlattr>.covarianceAcceleration", 0);
        int useFCAVZ = v.second.get<int>("<xmlattr>.useFixedCovarianceAngularVelocityZ", 0);
        useImuFixedCovarianceAngularVelocityZ = (useFCAVZ == 1);
        imuCovarianceAngularVelocityZ = v.second.get<double>("<xmlattr>.covarianceAngularVelocityZ", 0);
      }
    }


    //************************************
    // Mag
    //************************************

    boost::property_tree::ptree configTreeMag;
    ros::NodeHandle node_handleMag("~");
    std::string configContentMag;
    node_handleMag.getParam(filenameMag, configContentMag);
    std::stringstream ssMag;
    ssMag << configContentMag;
    boost::property_tree::read_xml(ssMag, configTreeMag);

    BOOST_FOREACH(const boost::property_tree::ptree::value_type & v, configTreeMag.get_child("config")) {
      if (v.first.compare("mag") == 0) {
        //use="1" covarianceAcceleration="0.01"  useFixedCovarianceAngularVelocityZ="1" covarianceAngularVelocityZ="0.001" angleOffset="-0.4" covarianceOrientationZ="0.01"
        //int use = v.second.get<int>("<xmlattr>.use", 0);
        useMag = true;
        magAngleOffset = v.second.get<double>("<xmlattr>.angleOffset", 0);
        covarianceMag = v.second.get<double>("<xmlattr>.covarianceMag", 0);
      }
    }


    //************************************
    // Position
    //************************************

    ROS_INFO("POSGEN: Intentando cargar propiedad %s", filenamePos.c_str());
    boost::property_tree::ptree configTreePos;
    ros::NodeHandle node_handlePos("~");
    std::string configContentPos;
    node_handlePos.getParam(filenamePos, configContentPos);
    std::stringstream ssPos;
    ssPos << configContentPos;
    boost::property_tree::read_xml(ssPos, configTreePos);


    BOOST_FOREACH(const boost::property_tree::ptree::value_type & v, configTreePos.get_child("config")) {
      if (v.first.compare("algorithm") == 0) {

        int type = v.second.get<int>("<xmlattr>.type", 0);
        int variant = v.second.get<int>("<xmlattr>.variant", 0);
        int numIgnoredRangings = v.second.get<int>("<xmlattr>.numIgnoredRangings", 0);
        int bestMode = v.second.get<int>("<xmlattr>.bestMode", 0);
        double minZ = v.second.get<double>("<xmlattr>.minZ", 0);
        double maxZ = v.second.get<double>("<xmlattr>.maxZ", 0);

        useInitPosition = 0;

        int hasInitPosition = v.second.get<int>("<xmlattr>.useInitPosition", 0);
        useInitPosition = (hasInitPosition == 1);
        if (useInitPosition) {
          initPosition.x = v.second.get<double>("<xmlattr>.initX", 0);
          initPosition.y = v.second.get<double>("<xmlattr>.initY", 0);
          initPosition.z = v.second.get<double>("<xmlattr>.initZ", 0);
        }

        if (variant == 0) {
          setVariantNormal();
          ROS_INFO("POSGEN: Variant NORMAL");
        } else if (variant == 1) {
          setVariantIgnoreN(numIgnoredRangings);
          ROS_INFO("POSGEN: Variant IGNORE N. Ignored: %d", numIgnoredRangings);
        } else  if (variant == 2) {
          setVariantOnlyBest(bestMode, minZ, maxZ);
          ROS_INFO("POSGEN: Variant Best N:. BestMode: %d minZ: %f mazZ: %f", bestMode, minZ, maxZ);
        }

      }
    }

    return true;

  } catch (boost::exception const &ex) {
    return false;
  }


  return false;
}


void PosGenerator::reset() {
  _tagList.resize(0);
  _tagList.clear();
}


void PosGenerator::newUWBMeasurement(const gtec_msgs::Ranging::ConstPtr& uwbRanging) {
  if (anchorsSet){
     processRangingNow(uwbRanging->anchorId, uwbRanging->tagId, uwbRanging->range, uwbRanging->range, uwbRanging->errorEstimation, uwbRanging->seq, (uwbRanging->errorEstimation > 0.0));
  }

}


void PosGenerator::newPX4FlowMeasurement(const mavros_msgs::OpticalFlowRad::ConstPtr& px4FlowMeasurement) {

  if (usePX4Flow) {
    if (px4FlowMeasurement->integration_time_us > 0 && px4FlowMeasurement->quality >0 ) {
      kalmanFilter->newPX4FlowMeasurement(px4FlowMeasurement->integrated_x, px4FlowMeasurement->integrated_y, px4FlowMeasurement->integrated_zgyro, px4FlowMeasurement->integration_time_us, covarianceVelocityPX4Flow, covarianceGyroZPX4Flow, px4FlowMeasurement->quality);
    }
  }
}


void PosGenerator::newMAGMeasurement(const sensor_msgs::MagneticField::ConstPtr& magMeasurement) {
  if (useMag) {
    kalmanFilter->newMAGMeasurement(magMeasurement->magnetic_field.x, magMeasurement->magnetic_field.y, covarianceMag);
  }

}


void PosGenerator::newCompassMeasurement(const std_msgs::Float64 compasMeasurement) {
  if (useMag) {
    kalmanFilter->newCompassMeasurement(compasMeasurement.data, covarianceMag);
  }
}


void PosGenerator::newIMUMeasurement(const sensor_msgs::Imu::ConstPtr& imuMeasurement) {

  if (useImu) {
    double covarianceAccelerationXY[4];

    if (useImuFixedCovarianceAcceleration) {
      covarianceAccelerationXY[0] = imuCovarianceAcceleration;
      covarianceAccelerationXY[1] = imuMeasurement->linear_acceleration_covariance[1];
      covarianceAccelerationXY[2] = imuMeasurement->linear_acceleration_covariance[3];
      covarianceAccelerationXY[3] = imuCovarianceAcceleration;
    } else {
      covarianceAccelerationXY[0] = imuMeasurement->linear_acceleration_covariance[0];
      covarianceAccelerationXY[1] = imuMeasurement->linear_acceleration_covariance[1];
      covarianceAccelerationXY[2] = imuMeasurement->linear_acceleration_covariance[3];
      covarianceAccelerationXY[3] = imuMeasurement->linear_acceleration_covariance[4];
    }

    double covAngularVelocityZ = imuMeasurement->angular_velocity_covariance[8];
    if (useImuFixedCovarianceAngularVelocityZ) {
      covAngularVelocityZ = imuCovarianceAngularVelocityZ;
    }

    kalmanFilter->newIMUMeasurement(imuMeasurement->angular_velocity.z, covAngularVelocityZ, imuMeasurement->linear_acceleration.x, imuMeasurement->linear_acceleration.y, covarianceAccelerationXY);
  }
}

void PosGenerator::timerRangingCallback(const ros::TimerEvent& event){
    rangingTimeout = true;

    if (_tagList.size()>0){

     int tagId = 0;
      tag_reports_t rp = _tagList.at(tagId);
      sendRangingMeasurementIfAvailable(rp);
    }


}



void PosGenerator::sendRangingMeasurementIfAvailable(tag_reports_t tagReport) {

//Suponemos tagID 0
  int tagId = 0;
  bool canSend = false;
  int minRangingsToSend = mode3d ? 1 : 1;
  
tag_reports_t rp = tagReport;
 
  if (rp.rangeSeq != -1) {

    if (rp.rangeCount[rp.rangeSeq] >= minRangingsToSend) {
      canSend = true;
    } 

/*    else if ((rp.rangeCount[lastRangingSeq] >= minRangingsToSend) && (rangingTimeout)) {
      //Si hay timeout y tenemos un minimo de ranging, podemos usar los que haya
      canSend = true;
    }*/

  


    if (canSend) {
      ROS_INFO("Can send measurements"); 
      //ROS_INFO("Sending %d ranging measurements",rp.rangeCount[lastRangingSeq]); 

      //Paramos el timer de ranging
      timerRanging.stop();


      int count = rp.rangeCount[rp.rangeSeq];
      ROS_INFO("Sending %d rangings", count);
      double timeLag;
      auto now = std::chrono::steady_clock::now();

      if (mLastRangingPositionTimestamp == std::chrono::steady_clock::time_point::min()) {
        //Es la primera estimacion
        timeLag = 0;
      } else {

        std::chrono::duration<double> diff = now -  mLastRangingPositionTimestamp;
        timeLag = diff.count();

      }
      mLastRangingPositionTimestamp = now;

      Vector3 report;
      int result = calculateTagLocationWithRangings(&report, count, &rp.rangeValue[rp.rangeSeq][0], &rp.errorEstimation[rp.rangeSeq][0], timeLag);
      //publishPositionReport(report);
      //lastRangingSeq = -1;
    } else {
      //ROS_INFO("Can NOT send measurements"); 
    }
  }
}


void PosGenerator::processRangingNow(int anchorId, int tagId, double range, double rawrange, double errorEstimation, int seq, bool withErrorEstimation) {

  int idx = 0, lastSeq = 0, count = 0;
  bool trilaterate = false, canCalculatePosition = false;

  arma::mat resultCovarianceMatrix;
  int range_corrected = 0;
  int tagOffset = 0;

  switch (anchorId) {
  case 0:
    tagOffset = tagOffset0;
    break;
  case 1:
    tagOffset = tagOffset1;
    break;
  case 2:
    tagOffset = tagOffset2;
    break;
  case 3:
    tagOffset = tagOffset3;
    break;
  }


  if (useRawRange) {
    range_corrected = floor(rawrange) + (tagOffset * 10);
  } else {
    range_corrected = floor(range) + (tagOffset * 10);
  }


  for (idx = 0; idx < _tagList.size(); idx++) {
    if (_tagList.at(idx).id == tagId)
      break;
  }

  if (idx == _tagList.size()) {
    initialiseTagList(tagId);
  }

  tag_reports_t rp = _tagList.at(idx);

  //Ponemos la variable global apuntando al ultimo valor
  
  timestampLastRanging = std::chrono::steady_clock::now();

  if (rp.rangeSeq == seq)
  {
    //We are receiving a range with the current seq number
    rp.rangeCount[seq]++;
    rp.rangeSeq = seq;
    rp.rangeValue[seq][anchorId] = range_corrected;

    if (withErrorEstimation) {
      rp.errorEstimation[seq][anchorId] = errorEstimation;
    }
  } else
  {
    //Its a new sequence number, we process the previous report

    tag_reports_t newReport = rp;
    sendRangingMeasurementIfAvailable(newReport);

    //lastRangingSeq = seq;

    for (int i = 0; i < MAX_NUM_ANCS; ++i)
    {
      rp.rangeValue[seq][0]  = -1;
      rp.errorEstimation[seq][0]  = 0.0;
    }

    if (rp.rangeSeq == -1) {
      //Es la primera vez
      lastSeq = seq;
    } else {
      lastSeq = rp.rangeSeq;
    }

    rp.rangeCount[seq] = 1;
    rp.rangeSeq = seq;
    rp.rangeValue[seq][anchorId] = range_corrected;
    rp.errorEstimation[seq][anchorId] = errorEstimation;
  }

  _tagList.at(idx) = rp;

  //Paramos el timer de ranging
  timerRanging.stop();

  //Lo iniciamos de nuevo
  timerRanging.start();

  rangingTimeout = false;

  


}







void PosGenerator::processRanging(int anchorId, int tagId, double range, double rawrange, double errorEstimation, int seq, bool withErrorEstimation) {

  int idx = 0, lastSeq = 0, count = 0;
  bool trilaterate = false, canCalculatePosition = false;

  arma::mat resultCovarianceMatrix;
  int range_corrected = 0;
  int tagOffset = 0;

  switch (anchorId) {
  case 0:
    tagOffset = tagOffset0;
    break;
  case 1:
    tagOffset = tagOffset1;
    break;
  case 2:
    tagOffset = tagOffset2;
    break;
  case 3:
    tagOffset = tagOffset3;
    break;
  }


  if (useRawRange) {
    range_corrected = floor(rawrange) + (tagOffset * 10);
  } else {
    range_corrected = floor(range) + (tagOffset * 10);
  }


  for (idx = 0; idx < _tagList.size(); idx++) {
    if (_tagList.at(idx).id == tagId)
      break;
  }

  if (idx == _tagList.size()) {
    initialiseTagList(tagId);
  }

  tag_reports_t rp = _tagList.at(idx);

  if (rp.rangeSeq == seq)
  {
    rp.rangeCount[seq]++;
    rp.rangeSeq = seq;
    rp.rangeValue[seq][anchorId] = range_corrected;

    if (withErrorEstimation) {
      rp.errorEstimation[seq][anchorId] = errorEstimation;
    }
  }
  else
  {
    for (int i = 0; i < MAX_NUM_ANCS; ++i)
    {
      rp.rangeValue[seq][0]  = -1;
      rp.errorEstimation[seq][0]  = 0.0;
    }

    if (rp.rangeSeq == -1) {
      //Es la primera vez
      lastSeq = seq;
    } else {
      lastSeq = rp.rangeSeq;
    }


    //Es el primer ranging de una nueva secuencia
    rp.rangeCount[seq] = 1;
    rp.rangeSeq = seq;
    rp.rangeValue[seq][anchorId] = range_corrected;
    rp.errorEstimation[seq][anchorId] = errorEstimation;

    //Activamos esto para procesar la secuencia anterior
    trilaterate = true;
  }

  /***********************************
   * Esto cuando se usa solo el Tag 0
   *********************************/

  if (trilaterate) {
    //lastSeq = (seq - 1);
    count = rp.rangeCount[lastSeq];

    //Comprobamos cuantas medidas teniamos de ranging, para KF no ponemos limite minimo, para el resto de algoritmo necesitamos 3
    canCalculatePosition = false;

    if (count >= 3) {
      canCalculatePosition = true;
    } else if (algorithm == ALGORITHM_KF) {
      canCalculatePosition = true;
    }

    if (canCalculatePosition) {
      //Buscamos el timestamp desde la ultima medida o 0 si es la primera

      double timeLag;

      auto now = std::chrono::steady_clock::now();

      if (mLastRangingPositionTimestamp == std::chrono::steady_clock::time_point::min()) {
        //Es la primera estimacion
        timeLag = 0;
      } else {

        std::chrono::duration<double> diff = now -  mLastRangingPositionTimestamp;
        timeLag = diff.count();

      }
      mLastRangingPositionTimestamp = now;

      Vector3 report;
      int result = calculateTagLocationWithRangings(&report, count, &rp.rangeValue[lastSeq][0], &rp.errorEstimation[lastSeq][0], timeLag);
    }
    rp.rangeCount[lastSeq] = 0;
  }

  //update the list entry
  _tagList.at(idx) = rp;

}

void PosGenerator::publishPositionReport(Vector3 report) {

  if (!std::isnan(report.x)) {
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.pose.pose.position.x = report.x;
    msg.pose.pose.position.y = report.y;
    msg.pose.pose.position.z = report.z;
    msg.pose.pose.orientation.x = report.rotX;
    msg.pose.pose.orientation.y = report.rotY;
    msg.pose.pose.orientation.z = report.rotZ;
    msg.pose.pose.orientation.w = report.rotW;

    for (int i = 0; i < 36; i++) {
      msg.pose.covariance[i] = report.covarianceMatrix(i);
    }

    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();
    ros_pub.publish(msg);

    //PATH
    nav_msgs::Path pathMsg;

    geometry_msgs::PoseStamped newPos;
    newPos.pose.position.x = report.x;
    newPos.pose.position.y = report.y;
    newPos.pose.position.z = report.z;
    newPos.pose.orientation.x = report.rotX;
    newPos.pose.orientation.y = report.rotY;
    newPos.pose.orientation.z = report.rotZ;
    newPos.pose.orientation.w = report.rotW;

    newPos.header.frame_id = "world";
    newPos.header.stamp = ros::Time::now();

    path.push_back(newPos);
    if (path.size() > maxPathSize) {
      //Borramos el primer elemento q sera el mas antiguo
      path.erase(path.begin());
    }

//copy(path.begin(), path.end(), pathMsg.poses.begin());
    pathMsg.poses = path;
    pathMsg.header.frame_id = "world";
    pathMsg.header.stamp =  newPos.header.stamp;

    ros_pub_path.publish(pathMsg);


  //Odometry
  nav_msgs::Odometry odomMsg;

  odomMsg.pose.pose.position.x = report.x;
  odomMsg.pose.pose.position.y = report.y;
  odomMsg.pose.pose.position.z = report.z;
  odomMsg.pose.pose.orientation.x = report.rotX;
  odomMsg.pose.pose.orientation.y = report.rotY;
  odomMsg.pose.pose.orientation.z = report.rotZ;
  odomMsg.pose.pose.orientation.w = report.rotW;

  for (int i = 0; i < 36; i++) {
    odomMsg.pose.covariance[i] = report.covarianceMatrix(i);
    odomMsg.twist.covariance[i] = report.covarianceMatrix(i);
  }

  odomMsg.twist.twist.linear.x = report.linearSpeedX;
  odomMsg.twist.twist.linear.y = report.linearSpeedY;
  odomMsg.twist.twist.linear.z = report.linearSpeedZ;
  odomMsg.twist.twist.angular.x = report.angularSpeedX;
  odomMsg.twist.twist.angular.y = report.angularSpeedY;
  odomMsg.twist.twist.angular.z = report.angularSpeedZ;

  odomMsg.header.frame_id = "odom";
  odomMsg.header.stamp =  newPos.header.stamp;

  odomMsg.child_frame_id = "pioneer3at::chassis";

  ros_pub_odom.publish(odomMsg);

  mBroadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(report.rotX, report.rotY, report.rotZ, report.rotW), tf::Vector3(report.x, report.y, report.z)),
        newPos.header.stamp,"odom", "pioneer3at::chassis"));

  }

}


int PosGenerator::calculateTagLocationWithRangings(Vector3 *report, int count, int *ranges, double *errorEstimation, double timeLag) {
  int result = 0;

  std::vector<double> rangesdb, errorEstimationsdb;
  std::vector<Beacon> selectedBeacons;

  for (int i = 0; i < MAX_NUM_ANCS; i++) {
    if (ranges[i]>0){
      rangesdb.push_back((double) ranges[i] / 1000);
      errorEstimationsdb.push_back(errorEstimation[i]);
      Beacon beacon = beacons[i];
      selectedBeacons.push_back(beacon);
    }
  }

  Vector3 position;
  kalmanFilter->newUWBMeasurement(rangesdb, selectedBeacons, errorEstimationsdb, timeLag);
  result = RESULT_KF;

  return result;
}


void PosGenerator::initialiseTagList(int id) {
  tag_reports_t r;
  memset(&r, 0, sizeof(tag_reports_t));
  r.id = id;
  r.ready = false;
  r.rangeSeq = -1;
  memset(&r.rangeValue[0][0], -1, sizeof(r.rangeValue));
  _tagList.push_back(r);
}


void PosGenerator::setUseRawRange(bool useRawRange) {
  this->useRawRange = useRawRange;

}

void PosGenerator::setObservationCovariance(std::vector<double> obsCov) {
  this->observationCovariance = std::vector<double>(obsCov);
}


void PosGenerator::setTagOffset(int tagOffset0, int tagOffset1, int tagOffset2, int tagOffset3) {
  this->tagOffset0 = tagOffset0;
  this->tagOffset1 = tagOffset1;
  this->tagOffset2 = tagOffset2;
  this->tagOffset3 = tagOffset3;

}

void PosGenerator::setAlgorithm(int algorithm, double accelerationNoise, bool ignoreWorstAnchorMode, double ignoreCostThreshold, bool mode3d) {
  this->algorithm = algorithm;
  this->mode3d = mode3d;

  if (useInitPosition) {
    this->kalmanFilter.reset(new KalmanFilter(accelerationNoise, fixedHeightUWB, armP0PX4Flow, armP1PX4Flow, initPosition, fixedHeightPX4Flow, initAnglePX4Flow, magAngleOffset, mJolt));
  } else {
    this->kalmanFilter.reset(new KalmanFilter(accelerationNoise, fixedHeightUWB, armP0PX4Flow, armP1PX4Flow, fixedHeightPX4Flow, initAnglePX4Flow, magAngleOffset, mJolt));
  }
}

void PosGenerator::setTwoTagsMode(bool twoTagsMode) {
  this->useTwoTags = twoTagsMode;
}

void PosGenerator::setTag0RelativePosition(double x, double y, double z) {
  this->tag0RelativePosition.x = x;
  this->tag0RelativePosition.y = y;
  this->tag0RelativePosition.z = z;
}
void PosGenerator::setTag1RelativePosition(double x, double y, double z) {
  this->tag1RelativePosition.x = x;
  this->tag1RelativePosition.y = y;
  this->tag1RelativePosition.z = z;
}


void PosGenerator::setVariantIgnoreN(int numIgnoredRangings) {
  this->numIgnoredRangings = numIgnoredRangings;
  this->variant = ML_VARIANT_IGNORE_N;
}

void PosGenerator::setVariantOnlyBest(int bestMode, double minZ, double maxZ) {
  this->bestMode = bestMode;
  this->minZ = minZ;
  this->maxZ = maxZ;
  this->variant = ML_VARIANT_BEST;
}

void PosGenerator::setVariantNormal() {
  this->variant = ML_VARIANT_NORMAL;
}



void PosGenerator::setImuPublisher(ros::Publisher anImuPub, bool enabled) {
  publishImu = enabled;
  ros_pub_imu = anImuPub;

}
void PosGenerator::setCompassPublisher(ros::Publisher aCompassPub, bool enabled) {
  publishCompass = enabled;
  ros_pub_compass = aCompassPub;
}
void PosGenerator::setAnchorsPublisher(ros::Publisher anAnchorsPub, bool enabled) {
  publishAnchors = enabled;
  ros_pub_anchors = anAnchorsPub;
}

void PosGenerator::publishFixedRateReport() {
  Vector3 pose;
  bool canSendReport = false;

  canSendReport = this->kalmanFilter->getPose(pose);

  if (canSendReport){
    publishPositionReport(pose);
  }
}