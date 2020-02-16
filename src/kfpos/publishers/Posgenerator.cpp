#include "Posgenerator.h"

PosGenerator::PosGenerator() {
  this->mLastRangingPositionTimestamp =  std::chrono::steady_clock::time_point::min();
  this->timestampLastRanging = std::chrono::steady_clock::time_point::min();
  this->lastRangingSeq = -1;

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
      _ancArray[i].id = marker.id;
      _ancArray[i].label = "";
      _ancArray[i].x = marker.pose.position.x;
      _ancArray[i].y = marker.pose.position.y;
      _ancArray[i].z = marker.pose.position.z;
      beacons.push_back({ marker.id, i, { _ancArray[i].x, _ancArray[i].y, _ancArray[i].z } });
    }
    anchorsSet = true;
    ROS_INFO("Anchors set");

    for (int i = 0; i < beacons.size(); ++i)
    {
      Beacon beacon= beacons[i];
      _anchorIndexById[beacon.id] = beacon.index;
      ROS_INFO("Anchors: %d [%d]  (%f, %f, %f)", beacon.index, beacon.id,  beacon.position.x, beacon.position.y, beacon.position.z);
    }
  }
  
}


void PosGenerator::setPublishers(ros::Publisher aPub, ros::Publisher aPathPub, ros::Publisher aOdomPub){
  ros_pub = aPub;
  ros_pub_path = aPathPub;
  ros_pub_odom = aOdomPub;
}

void PosGenerator::setDynamicParameters(double accelerationNoise, double jolt){
  mJolt = jolt;
  mAccelerationNoise = accelerationNoise;
}

void PosGenerator::setExternalFilesParameters(std::string configFilenamePos, std::string configFilenamePX4Flow, std::string configFilenameTag, std::string configFilenameImu, std::string configFilenameMag){
    mFilenamePos =configFilenamePos;
    mFilenamePX4Flow=configFilenamePX4Flow;
    mFilenameTag=configFilenameTag; 
    mFilenameImu=configFilenameImu; 
    mFilenameMag=configFilenameMag;
}

void PosGenerator::setStartParameters(bool useStartPosition, double startPositionX, double startPositionY,double startPositionZ,double  startAngle){
  mUseInitPosition = useStartPosition;
  mInitPosition = {startPositionX,startPositionY,startPositionZ };
  mInitAngle = startAngle;
}


void PosGenerator::setHeuristicIgnore(bool ignoreWorstAnchorMode, double ignoreCostThreshold){
  mIgnoreWorstAnchorMode = ignoreWorstAnchorMode;
  mIgnoreCostThreshold = ignoreCostThreshold;

}

void PosGenerator::setDeviceIdentifiers(int toaTagId){
  mTOATagId = toaTagId;
}


void PosGenerator::setHeuristicML(bool use2d, int variant, int numRangingsToIgnore){
  mUse2d = use2d;
  mVariant = variant;
  mNumRangingsToIgnore = numRangingsToIgnore;
}


void PosGenerator::reset() {
  _tagList.resize(0);
  _tagList.clear();
}


void PosGenerator::newTOAMeasurement(const gtec_msgs::Ranging::ConstPtr& uwbRanging) {
  if (anchorsSet){
     processRangingNow(uwbRanging->anchorId, uwbRanging->tagId, uwbRanging->range, uwbRanging->range, uwbRanging->errorEstimation, uwbRanging->seq, (uwbRanging->errorEstimation > 0.0));
  }
}


void PosGenerator::newPX4FlowMeasurement(const mavros_msgs::OpticalFlowRad::ConstPtr& px4FlowMeasurement) {
    if (px4FlowMeasurement->integration_time_us > 0 && px4FlowMeasurement->quality >0 ) {
      mPositionAlgorithm->newPX4FlowMeasurement(px4FlowMeasurement->integrated_x, px4FlowMeasurement->integrated_y, px4FlowMeasurement->integrated_zgyro, px4FlowMeasurement->integration_time_us, px4FlowMeasurement->quality);
    }
}


void PosGenerator::newMAGMeasurement(const sensor_msgs::MagneticField::ConstPtr& magMeasurement) {

  double covarianceMag[9];
  VectorDim3 mag = {magMeasurement->magnetic_field.x, magMeasurement->magnetic_field.y, magMeasurement->magnetic_field.z};

  for (int i = 0; i < 9; ++i)
    {
      covarianceMag[i] = magMeasurement->magnetic_field_covariance[i];
    }

  mPositionAlgorithm->newMAGMeasurement(mag, covarianceMag);

}


void PosGenerator::newCompassMeasurement(const std_msgs::Float64 compasMeasurement) {
    mPositionAlgorithm->newCompassMeasurement(compasMeasurement.data);
}


void PosGenerator::newIMUMeasurement(const sensor_msgs::Imu::ConstPtr& imuMeasurement) {
  double covarianceAngularVelocity[9];
  double covarianceAcceleration[9];
  VectorDim3 angularVelocity = {imuMeasurement->angular_velocity.x, imuMeasurement->angular_velocity.y, imuMeasurement->angular_velocity.z};
  VectorDim3 linearAcceleration = {imuMeasurement->linear_acceleration.x, imuMeasurement->linear_acceleration.y, imuMeasurement->linear_acceleration.z};
    

  for (int i = 0; i < 9; ++i)
  {
    covarianceAngularVelocity[i] = imuMeasurement->angular_velocity_covariance[i];
    covarianceAcceleration[i] = imuMeasurement->linear_acceleration_covariance[i];
  }

  mPositionAlgorithm->newIMUMeasurement(angularVelocity, covarianceAngularVelocity, linearAcceleration, covarianceAcceleration);

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

  bool canSend = false;
  int minRangingsToSend = 1;
  
  tag_reports_t rp = tagReport;
 
  if (rp.rangeSeq != -1) {

    if (rp.rangeCount[rp.rangeSeq] >= minRangingsToSend) {
      canSend = true;
    } 


    if (canSend) {
      ROS_DEBUG("Can send measurements"); 
      //ROS_INFO("Sending %d ranging measurements",rp.rangeCount[lastRangingSeq]); 

      //Paramos el timer de ranging
      timerRanging.stop();


      int count = rp.rangeCount[rp.rangeSeq];
      ROS_DEBUG("Sending %d rangings", count);
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

    } 
  }
}


void PosGenerator::processRangingNow(int anchorId, int tagId, double range, double rawrange, double errorEstimation, int seq, bool withErrorEstimation) {

  if (tagId!=mTOATagId){
    return;
  }

 ROS_DEBUG("Process Ranging Now. TagId: %d, anchorId: %d", tagId, anchorId);

  int idx = 0, lastSeq = 0, count = 0;
  bool trilaterate = false, canCalculatePosition = false;

  arma::mat resultCovarianceMatrix;
  int range_corrected = floor(rawrange);

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

  int anchorIndex = _anchorIndexById[anchorId];

  if (rp.rangeSeq == seq)
  {
    //We are receiving a range with the current seq number
    rp.rangeCount[seq]++;
    rp.rangeSeq = seq;
    rp.rangeValue[seq][anchorIndex] = range_corrected;

    if (withErrorEstimation) {
      rp.errorEstimation[seq][anchorIndex] = errorEstimation;
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
    rp.rangeValue[seq][anchorIndex] = range_corrected;
    rp.errorEstimation[seq][anchorIndex] = errorEstimation;
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
  int range_corrected = floor(range);



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

    canCalculatePosition = false;
/*
    if (count >= 3) {
      canCalculatePosition = true;
    } else if (algorithm == ALGORITHM_KF) {
      canCalculatePosition = true;
    }
*/
    if (canCalculatePosition) {

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

    printf("(%f, %f, %f)\r", report.x, report.y, report.z);

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

  mPositionAlgorithm->newTOAMeasurement(rangesdb, selectedBeacons, errorEstimationsdb, timeLag);
 
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


void PosGenerator::setAlgorithm(int algorithm) {
  if (algorithm==ALGORITHM_KF_TOA){
      if (!mUseInitPosition) {
        mPositionAlgorithm.reset(new KalmanFilterTOA(mAccelerationNoise, mIgnoreWorstAnchorMode, mIgnoreCostThreshold, mInitPosition));
      } else {
        mPositionAlgorithm.reset(new KalmanFilterTOA(mAccelerationNoise ,mIgnoreWorstAnchorMode, mIgnoreCostThreshold));
      }
  } else if (algorithm==ALGORITHM_KF){
      if (!mUseInitPosition) {
        mPositionAlgorithm.reset(new KalmanFilter( mAccelerationNoise, mInitAngle, mJolt, mFilenamePos, mFilenamePX4Flow, mFilenameTag, mFilenameImu, mFilenameMag));
      } else {
        mPositionAlgorithm.reset(new KalmanFilter( mAccelerationNoise, mInitAngle, mJolt, mFilenamePos, mFilenamePX4Flow, mFilenameTag, mFilenameImu, mFilenameMag, mInitPosition));
      }
  } else if (algorithm==ALGORITHM_KF_TOA_IMU){
      if (!mUseInitPosition) {
        mPositionAlgorithm.reset(new KalmanFilterTOAIMU( mAccelerationNoise, mJolt));
      } else {
        mPositionAlgorithm.reset(new KalmanFilterTOAIMU( mAccelerationNoise, mJolt, mInitPosition));
      }
  } else if (algorithm==ALGORITHM_ML){
      if (!mUseInitPosition) {
        mPositionAlgorithm.reset(new MLLocation(mUse2d, mVariant, mNumRangingsToIgnore, {1,1,4}));
      } else {
        mPositionAlgorithm.reset(new MLLocation(mUse2d, mVariant, mNumRangingsToIgnore, mInitPosition));
      }
  }

  mPositionAlgorithm->init();
}


void PosGenerator::publishFixedRateReport() {
  Vector3 pose = {NAN, NAN, NAN};
  bool canSendReport = mPositionAlgorithm->getPose(pose);

  if (canSendReport){
    publishPositionReport(pose);
  }
}