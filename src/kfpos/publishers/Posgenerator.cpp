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
  }
  
}


void PosGenerator::start(std::string configFilenamePos, std::string configFilenamePX4Flow, std::string configFilenameTag, std::string configFilenameImu, std::string configFilenameMag, ros::Publisher aPub, ros::Publisher aPathPub) {

  ROS_INFO("POSGEN: Iniciando POS GENERATOR");
  //Cargamos la configuracion
  bool configLoaded = init(configFilenamePos, configFilenamePX4Flow, configFilenameTag, configFilenameImu, configFilenameMag);
  ros_pub = aPub;
  ros_pub_path = aPathPub;

  if (configLoaded) {
    ROS_INFO("POSGEN: Configuracion cargada");
  }
}


bool PosGenerator::init(std::string filenamePos, std::string filenamePX4Flow, std::string filenameTag, std::string filenameImu, std::string filenameMag) {

  try {

    //************************************
    //Cargamos la configuracion de PX4Flow
    //************************************

    ROS_INFO("Intentando cargar propiedad %s", filenamePX4Flow.c_str());
    boost::property_tree::ptree configTreePX4Flow;
    ros::NodeHandle node_handlePX4Flow("~");
    std::string configContentPX4Flow;
    node_handlePX4Flow.getParam(filenamePX4Flow, configContentPX4Flow);
    std::stringstream ssPX4Flow;
    ssPX4Flow << configContentPX4Flow;
    boost::property_tree::read_xml(ssPX4Flow, configTreePX4Flow);

    BOOST_FOREACH(const boost::property_tree::ptree::value_type & v, configTreePX4Flow.get_child("config")) {
      if (v.first.compare("px4flow") == 0) {
        //int use = v.second.get<int>("<xmlattr>.use", 0);
        usePX4Flow = true;

        int useFH = v.second.get<int>("<xmlattr>.useFixedSensorHeight", 0);
        useFixedHeightPX4Flow = (useFH == 1);

        fixedHeightPX4Flow = v.second.get<double>("<xmlattr>.sensorHeight", 0);
        armP0PX4Flow = v.second.get<double>("<xmlattr>.armP0", 0);
        armP1PX4Flow = v.second.get<double>("<xmlattr>.armP1", 0);
        initAnglePX4Flow = v.second.get<double>("<xmlattr>.sensorInitAngle", 0);

        //covarianceVelocity="0.0001" covarianceGyroZ="0.001"
        covarianceVelocityPX4Flow = v.second.get<double>("<xmlattr>.covarianceVelocity", 0);
        covarianceGyroZPX4Flow = v.second.get<double>("<xmlattr>.covarianceGyroZ", 0);
      }
    }

    //************************************
    //Cargamos la configuracion de tag UWB
    //************************************

    ROS_INFO("Intentando cargar propiedad %s", filenameTag.c_str());
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
    //Cargamos la configuracion de Erle IMU
    //************************************

    ROS_INFO("Intentando cargar propiedad %s", filenameImu.c_str());
    boost::property_tree::ptree configTreeImu;
    ros::NodeHandle node_handleImu("~");
    std::string configContentImu;
    node_handleImu.getParam(filenameImu, configContentImu);
    std::stringstream ssImu;
    ssImu << configContentImu;
    boost::property_tree::read_xml(ssImu, configTreeImu);

    BOOST_FOREACH(const boost::property_tree::ptree::value_type & v, configTreeImu.get_child("config")) {
      if (v.first.compare("erleimu") == 0) {
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
    //Cargamos la configuracion de Erle Mag
    //************************************

    ROS_INFO("Intentando cargar propiedad %s", filenameMag.c_str());
    boost::property_tree::ptree configTreeMag;
    ros::NodeHandle node_handleMag("~");
    std::string configContentMag;
    node_handleMag.getParam(filenameMag, configContentMag);
    std::stringstream ssMag;
    ssMag << configContentMag;
    boost::property_tree::read_xml(ssMag, configTreeMag);

    BOOST_FOREACH(const boost::property_tree::ptree::value_type & v, configTreeMag.get_child("config")) {
      if (v.first.compare("erlemag") == 0) {
        //use="1" covarianceAcceleration="0.01"  useFixedCovarianceAngularVelocityZ="1" covarianceAngularVelocityZ="0.001" angleOffset="-0.4" covarianceOrientationZ="0.01"
        //int use = v.second.get<int>("<xmlattr>.use", 0);
        useMag = true;
        magAngleOffset = v.second.get<double>("<xmlattr>.angleOffset", 0);
        covarianceMag = v.second.get<double>("<xmlattr>.covarianceMag", 0);
      }
    }


    //************************************
    //Cargamos la configuracion de Posicion
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

        // if (type == 0) {
        //   setAlgorithm(ALGORITHM_ML);
        //   ROS_INFO("POSGEN: ALGORITHM_ML");
        // } else if (type == 1) {
        //   setAlgorithm(ALGORITHM_KF);
        //   ROS_INFO("POSGEN: ALGORITHM_KF");
        // }

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

      } else if (v.first.compare("twotags") == 0) {
        int useTwoTags = v.second.get<int>("<xmlattr>.use", 0);

        if (useTwoTags == 1) {
          ROS_INFO("POSGEN: TWOTAGS ON");
          setTwoTagsMode(true);
          double t0x = v.second.get<double>("<xmlattr>.t0x", 0);
          double t0y = v.second.get<double>("<xmlattr>.t0y", 0);
          double t0z = v.second.get<double>("<xmlattr>.t0z", 0);
          double t1x = v.second.get<double>("<xmlattr>.t1x", 0);
          double t1y = v.second.get<double>("<xmlattr>.t1y", 0);
          double t1z = v.second.get<double>("<xmlattr>.t1z", 0);

          setTag0RelativePosition(t0x, t0y, t0z);
          setTag1RelativePosition(t1x, t1y, t1z);

        } else {
          ROS_INFO("POSGEN: TWOTAGS OFF");
          setTwoTagsMode(false);
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


void PosGenerator::newRanging(const gtec_msgs::Ranging::ConstPtr& uwbRanging) {
  if (anchorsSet){
    //TODO MARCAR TEMPORALMENTE CUANDO SE LANZA EL ALGORITMO PARA PODER DAR UN TIEMPO ENTRE RANGINGS
    //ROS_INFO("RRPOS :[AnchorId:%d, TagId:%d, Range:%d, ErrorEstimation:%f, SEQ:%d]", uwbRanging->anchorId, uwbRanging->tagId, uwbRanging->range, uwbRanging->errorEstimation, uwbRanging->seq);
    processRangingNow(uwbRanging->anchorId, uwbRanging->tagId, uwbRanging->range, uwbRanging->range, uwbRanging->errorEstimation, uwbRanging->seq, (uwbRanging->errorEstimation > 0.0));
  }

}


void PosGenerator::newPX4FlowMeasurement(const mavros_msgs::OpticalFlowRad::ConstPtr& px4FlowMeasurement) {

  if (usePX4Flow) {
    //if (px4FlowMeasurement->quality > 0) {
    if (px4FlowMeasurement->integration_time_us > 0 && px4FlowMeasurement->quality >0 ) {
      kalmanFilter->newPX4FlowMeasurement(px4FlowMeasurement->integrated_x, px4FlowMeasurement->integrated_y, px4FlowMeasurement->integrated_zgyro, px4FlowMeasurement->integration_time_us, covarianceVelocityPX4Flow, covarianceGyroZPX4Flow, px4FlowMeasurement->quality);
    }
    //}
  }
}


void PosGenerator::newErleMagMeasurement(const sensor_msgs::MagneticField::ConstPtr& erleMagMeasurement) {
  if (useMag) {
    kalmanFilter->newErleMagMeasurement(erleMagMeasurement->magnetic_field.x, erleMagMeasurement->magnetic_field.y, covarianceMag);
  }

}


void PosGenerator::newErleCompassMeasurement(const std_msgs::Float64 compasMeasurement) {
  // Solo para logs, descomentar para funcionamiento normal.

  // if (publishCompass){
  //  ros_pub_compass.publish(compasMeasurement);
  // }

  if (useMag) {
    kalmanFilter->newErleCompassMeasurement(compasMeasurement.data, covarianceMag);
  }
}


void PosGenerator::newErleImuMeasurement(const sensor_msgs::Imu::ConstPtr& erleImuMeasurement) {

  //Lo siguiente solo para log, comentar en comportamiento normal
  // if (publishImu){
  //   ros_pub_imu.publish(erleImuMeasurement);
  // }

  if (useImu) {
    double covarianceAccelerationXY[4];


    if (useImuFixedCovarianceAcceleration) {
      covarianceAccelerationXY[0] = imuCovarianceAcceleration;
      covarianceAccelerationXY[1] = erleImuMeasurement->linear_acceleration_covariance[1];
      covarianceAccelerationXY[2] = erleImuMeasurement->linear_acceleration_covariance[3];
      covarianceAccelerationXY[3] = imuCovarianceAcceleration;
    } else {
      covarianceAccelerationXY[0] = erleImuMeasurement->linear_acceleration_covariance[0];
      covarianceAccelerationXY[1] = erleImuMeasurement->linear_acceleration_covariance[1];
      covarianceAccelerationXY[2] = erleImuMeasurement->linear_acceleration_covariance[3];
      covarianceAccelerationXY[3] = erleImuMeasurement->linear_acceleration_covariance[4];
    }

    double covAngularVelocityZ = erleImuMeasurement->angular_velocity_covariance[8];
    if (useImuFixedCovarianceAngularVelocityZ) {
      covAngularVelocityZ = imuCovarianceAngularVelocityZ;
    }


    kalmanFilter->newErleImuMeasurement(erleImuMeasurement->angular_velocity.z, covAngularVelocityZ, erleImuMeasurement->linear_acceleration.x, erleImuMeasurement->linear_acceleration.y, covarianceAccelerationXY);
  }
}



void PosGenerator::timerRangingCallback(const ros::TimerEvent& event){
    rangingTimeout = true;
    sendRangingMeasurementIfAvailable();
}



void PosGenerator::sendRangingMeasurementIfAvailable() {

//Suponemos tagID 0
  int tagId = 0;
  bool canSend = false;

  if (lastRangingSeq != -1) {
    //Solo hacemos algo si tenemos un ultimo numero de secuencia valido
    tag_reports_t rp = _tagList.at(tagId);

    //TODO: esto revisar, solo si son maximo 4 rangins
    //Si tenemos 4 o mas, podemos enviarlos ya
    if (rp.rangeCount[lastRangingSeq] >= beacons.size()) {
      canSend = true;

    } else if ((rp.rangeCount[lastRangingSeq] >= 3) && (rangingTimeout)) {

      canSend = true;
    }




    if (canSend) {
      //ROS_INFO("Sending %d ranging measurements",rp.rangeCount[lastRangingSeq]); 

      //Paramos el timer de ranging
      timerRanging.stop();


      int count = rp.rangeCount[lastRangingSeq];
      //ROS_INFO("Sending %d rangings", count);
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
      int result = calculateTagLocationWithRangings(&report, count, &rp.rangeValue[lastRangingSeq][0], &rp.errorEstimation[lastRangingSeq][0], timeLag);
      //publishPositionReport(report);
      lastRangingSeq = -1;
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
  lastRangingSeq = seq;
  timestampLastRanging = std::chrono::steady_clock::now();

  if (rp.rangeSeq == seq)
  {
    rp.rangeCount[seq]++;
    rp.rangeSeq = seq;
    rp.rangeValue[seq][anchorId] = range_corrected;

    if (withErrorEstimation) {
      rp.errorEstimation[seq][anchorId] = errorEstimation;
    }
  } else
  {
    //TODO lo siguiente es temporal, habria que buscar otra forma

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

  sendRangingMeasurementIfAvailable();


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
    } else if ((algorithm == ALGORITHM_KF) && (count >= 3)) {
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
      //publishPositionReport(report);
    }
    rp.rangeCount[lastSeq] = 0;
  }

  //update the list entry
  _tagList.at(idx) = rp;

}

void PosGenerator::publishPositionReport(Vector3 report) {
  //PoseWithCovariance

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
    //ROS_INFO("POS: [%f, %f, %f] Angle: %f", msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, msg.pose.pose.orientation.w);


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

  }

}


int PosGenerator::calculateTagLocationWithRangings(Vector3 *report, int count, int *ranges, double *errorEstimation, double timeLag) {
  int result = 0;

  std::vector<double> rangesdb, errorEstimationsdb;
  for (int i = 0; i < MAX_NUM_ANCS; i++) {
    rangesdb.push_back((double) ranges[i] / 1000);
    errorEstimationsdb.push_back(errorEstimation[i]);
  }

  Vector3 position;

  kalmanFilter->newUWBMeasurement(rangesdb, beacons, errorEstimationsdb, timeLag);
  result = RESULT_KF;

  // report->x = position.x;
  // report->y = position.y;
  // report->z = position.z;

  // report->rotX = position.rotX;
  // report->rotY = position.rotY;
  // report->rotZ = position.rotZ;
  // report->rotW = position.rotW;

  // report->covarianceMatrix = position.covarianceMatrix;

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

void PosGenerator::setAlgorithm(int algorithm) {
  this->algorithm = algorithm;
  if (useInitPosition) {
    this->kalmanFilter.reset(new KalmanFilter(1, fixedHeightUWB, armP0PX4Flow, armP1PX4Flow, initPosition, fixedHeightPX4Flow, initAnglePX4Flow, magAngleOffset, 0.05));
  } else {
    this->kalmanFilter.reset(new KalmanFilter(1, fixedHeightUWB, armP0PX4Flow, armP1PX4Flow, fixedHeightPX4Flow, initAnglePX4Flow, magAngleOffset, 0.05));
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
  bool canSendReport = this->kalmanFilter->getPose(pose);

  if (canSendReport){
    publishPositionReport(pose);
  }
}
