#!/usr/bin/env python


""" MIT License

Copyright (c) 2020 Group of Electronic Technology and Communications. University of A Coruna.

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
SOFTWARE. """


import rospy, time, serial, os, tf2_ros
from time import sleep
from multilateration import Engine

from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import MarkerArray
from gtec_msgs.msg import Ranging
from geometry_msgs.msg import TransformStamped

class DiscreteLocalizator(object):
    def __init__(self, ros_rate, publisher_loc, targetDeviceId, height=None):
        self.targetDeviceId = targetDeviceId
        self.publisherPos = publisher_loc
        self.rate = ros_rate
        self.engine = Engine(goal=[None, None, height])
        self.hasAnchors = False
        self.currentSeqNumber = 0

    def setAnchors(self,anchors):
        for anchor in anchors:
            self.engine.add_anchor(anchor.id,(anchor.pose.position.x,anchor.pose.position.y, anchor.pose.position.z))
        while not rospy.is_shutdown():
            self.rate.sleep()
            #self.publishPosition()

    def callbackRangingMessage(self, rangingMsg):
        self.engine.add_measure_id(rangingMsg.anchorId,rangingMsg.range/1000.0)
        #print('Ranging: {0} Anchor: {1}'.format(rangingMsg.range, rangingMsg.anchorId))
        if rangingMsg.seq!=self.currentSeqNumber:
            #print('Ranging: {0} Anchor: {1}'.format(rangingMsg.range, rangingMsg.anchorId))
            #print('Seq: {0}'.format(rangingMsg.seq))
            self.currentSeqNumber = rangingMsg.seq
            self.publishPosition()

    def callbackAnchorsMessage(self, anchorsMsg):
        if not self.hasAnchors:
            print("Anchors Msg received")
            anchors = []
            for anchor in anchorsMsg.markers:
                anchors.append(anchor)
            self.setAnchors(anchors)
            self.hasAnchors = True

    def publishPosition(self):
        """Publish a new position"""
        try:
            loc=self.engine.solve()
        except:
            print("Solving error")
            return
        #print(loc)

        # network_id = self.remote_id
        # if network_id is None:
        #     network_id = 0

        p = PoseWithCovarianceStamped()
        p.pose.pose.position.x = loc.x
        p.pose.pose.position.y = loc.y
        p.pose.pose.position.z = loc.z
        # Make sure the quaternion is valid and normalized
        p.pose.pose.orientation.x = 0.0
        p.pose.pose.orientation.y = 0.0
        p.pose.pose.orientation.z = 0.0
        p.pose.pose.orientation.w = 1.0
        
        for i in range(36):
            p.pose.covariance[i] = 0.0
        
        p.header.frame_id = "map"
        p.header.stamp = rospy.Time.now()

        self.publisherPos.publish(p)

        tf2Broadcast = tf2_ros.TransformBroadcaster()
        tf2Stamp = TransformStamped()
        tf2Stamp.header.stamp = rospy.Time.now()
        tf2Stamp.header.frame_id = 'map'
        tf2Stamp.child_frame_id = 'odom'
        tf2Stamp.transform.translation.x = loc.x
        tf2Stamp.transform.translation.y = loc.y
        tf2Stamp.transform.translation.z = loc.z
        tf2Stamp.transform.rotation.x = 0
        tf2Stamp.transform.rotation.y = 0
        tf2Stamp.transform.rotation.z = 0
        tf2Stamp.transform.rotation.w = 1

        tf2Broadcast.sendTransform(tf2Stamp)


        tf2StampWorld = TransformStamped()
        tf2StampWorld.header.stamp = rospy.Time.now()
        tf2StampWorld.header.frame_id = 'world'
        tf2StampWorld.child_frame_id = 'map'
        tf2StampWorld.transform.translation.x = 0
        tf2StampWorld.transform.translation.y = 0
        tf2StampWorld.transform.translation.z = 0
        tf2StampWorld.transform.rotation.x = 0
        tf2StampWorld.transform.rotation.y = 0
        tf2StampWorld.transform.rotation.z = 0
        tf2StampWorld.transform.rotation.w = 1

        tf2Broadcast.sendTransform(tf2StampWorld)

        # #print("POS ID {}, x(mm): {pos.x} y(mm): {pos.y} z(mm): {pos.z}".format(
        # #    "0x%0.4x" % network_id, pos=position))


# class ReadyToLocalize(object):
#     """Continuously calls the Pozyx positioning function and prints its position."""

#     def __init__(self, pozyx, pose_pub, ranging_pub, imu_pub, algorithm=POZYX_POS_ALG_UWB_ONLY, dimension=POZYX_3D, height=1000, remote_id=None):
#         self.pozyx = pozyx
#         # self.anchors = anchors
#         self.algorithm = algorithm
#         self.dimension = dimension
#         self.height = height
#         self.remote_id = remote_id
#         self.pose_pub = pose_pub
#         self.ranging_pub = ranging_pub
#         self.imu_pub = imu_pub
#         self.seq = -1


#     def setAnchors(self, anchors):
#         self.anchors = anchors

#     def setup(self):
#         """Sets up the Pozyx for positioning by calibrating its anchor list."""
#         print("------------POZYX POSITIONING V{} -------------".format(version))
#         print("")
#         print("- System will manually configure tag")
#         print("")
#         print("- System will auto start positioning")
#         print("")
#         # if self.remote_id is None:
#         #     self.pozyx.printDeviceInfo(self.remote_id)
#         # else:
#         #     for device_id in [None, self.remote_id]:
#         #         self.pozyx.printDeviceInfo(device_id)
#         print("")
#         print("------------POZYX POSITIONING V{} -------------".format(version))
#         print("")

#         self.setAnchorsManual(save_to_flash=False)
#         self.printPublishConfigurationResult()

#     def loop(self):
#         """Performs positioning and displays/exports the results."""
#         position = Coordinates()
#         status = self.pozyx.doPositioning(
#             position, self.dimension, self.height, self.algorithm, remote_id=self.remote_id)
#         if status == POZYX_SUCCESS:
#             self.printPublishPosition(position)
#             self.seq += 1
#             if self.seq>255:
#                 self.seq=0
#             current_seq = self.seq
#             for anchor in self.anchors:
#                 range = DeviceRange()
#                 status = self.pozyx.getDeviceRangeInfo(anchor.network_id,range,self.remote_id)
#                 if status == POZYX_SUCCESS:
#                     self.publishRanging(anchor.network_id, range, current_seq)
#                     #print("Anchor {}, Range (mm): {ran.distance} Rss: {ran.RSS} ".format("0x%0.4x" % anchor.network_id, ran=range))
            

#         # else:
#         #     self.printPublishErrorCode("positioning")
    
#     def publishImu(self):
#         quaternion = Quaternion()
#         acceleration = Acceleration()
#         angular_velocity = AngularVelocity()
#         self.pozyx.getQuaternion(quaternion, remote_id=self.remote_id)
#         self.pozyx.getLinearAcceleration_mg(acceleration, remote_id=self.remote_id)
#         self.pozyx.getAngularVelocity_dps(angular_velocity, remote_id=self.remote_id)
#         imu = Imu()
#         imu.header.frame_id = "base_link"
#         imu.header.stamp = rospy.Time.now()
#         imu.linear_acceleration = acceleration
#         imu.orientation.x = quaternion.x
#         imu.orientation.y = quaternion.y
#         imu.orientation.z = quaternion.z
#         imu.orientation.w = quaternion.w
#         imu.linear_acceleration.x = acceleration.x/1000 * 9.80664999999998
#         imu.linear_acceleration.y = acceleration.y/1000 * 9.80664999999998
#         imu.linear_acceleration.z = acceleration.z/1000 * 9.80664999999998
#         imu.angular_velocity.x = angular_velocity.x/57.2957795130824
#         imu.angular_velocity.y = angular_velocity.y/57.2957795130824
#         imu.angular_velocity.z = angular_velocity.z/57.2957795130824
        
#         for i in range(9):
#             imu.angular_velocity_covariance[i] = 0.0
#             imu.linear_acceleration_covariance[i] = 0.0
#             imu.orientation_covariance[i] = 0.0

        

#         for i in [0,4,8]:
#             imu.angular_velocity_covariance[i] = 0.001
#             imu.linear_acceleration_covariance[i] = 0.001
#             imu.orientation_covariance[i] = 0.001

        
#         self.imu_pub.publish(imu)




#     def publishRanging(self, anchorId, range, seq):
#         ranging = Ranging()
#         ranging.anchorId = anchorId
#         ranging.tagId = self.remote_id
#         ranging.range = range.distance
#         ranging.rss = range.RSS
#         ranging.seq = seq
#         ranging.errorEstimation = 0.00393973
#         self.ranging_pub.publish(ranging)


#     def printPublishPosition(self, position):
#         """Prints the Pozyx's position and possibly sends it as a OSC packet"""

#         network_id = self.remote_id
#         if network_id is None:
#             network_id = 0

#         p = PoseWithCovarianceStamped()
#         p.pose.pose.position.x = position.x/1000.0
#         p.pose.pose.position.y = position.y/1000.0
#         p.pose.pose.position.z = position.z/1000.0
#         # Make sure the quaternion is valid and normalized
#         p.pose.pose.orientation.x = 0.0
#         p.pose.pose.orientation.y = 0.0
#         p.pose.pose.orientation.z = 0.0
#         p.pose.pose.orientation.w = 1.0
        
#         for i in range(36):
#             p.pose.covariance[i] = 0.0
        
#         p.header.frame_id = "map"
#         p.header.stamp = rospy.Time.now()

#         self.pose_pub.publish(p)
#         self.publishImu()

#         # tf2Broadcast = tf2_ros.TransformBroadcaster()
#         # tf2Stamp = TransformStamped()
#         # tf2Stamp.header.stamp = rospy.Time.now()
#         # tf2Stamp.header.frame_id = 'map'
#         # tf2Stamp.child_frame_id = 'odom'
#         # tf2Stamp.transform.translation.x = position.x/1000.0
#         # tf2Stamp.transform.translation.y = position.y/1000.0
#         # tf2Stamp.transform.translation.z = position.z/1000.0
#         # tf2Stamp.transform.rotation.x = 0
#         # tf2Stamp.transform.rotation.y = 0
#         # tf2Stamp.transform.rotation.z = 0
#         # tf2Stamp.transform.rotation.w = 1

#         # tf2Broadcast.sendTransform(tf2Stamp)

#         #print("POS ID {}, x(mm): {pos.x} y(mm): {pos.y} z(mm): {pos.z}".format(
#         #    "0x%0.4x" % network_id, pos=position))

#     def printPublishErrorCode(self, operation):
#         """Prints the Pozyx's error and possibly sends it as a OSC packet"""
#         error_code = SingleRegister()
#         network_id = self.remote_id
#         if network_id is None:
#             self.pozyx.getErrorCode(error_code)
#             print("LOCAL ERROR %s, %s" % (operation, self.pozyx.getErrorMessage(error_code)))
#         status = self.pozyx.getErrorCode(error_code, self.remote_id)
#         if status == POZYX_SUCCESS:
#             print("ERROR %s on ID %s, %s" %
#                   (operation, "0x%0.4x" % network_id, self.pozyx.getErrorMessage(error_code)))
#         else:
#             self.pozyx.getErrorCode(error_code)
#             print("ERROR %s, couldn't retrieve remote error code, LOCAL ERROR %s" %
#                   (operation, self.pozyx.getErrorMessage(error_code)))
#             # should only happen when not being able to communicate with a remote Pozyx.

#     def setAnchorsManual(self, save_to_flash=False):
#         """Adds the manually measured anchors to the Pozyx's device list one for one."""
#         status = self.pozyx.clearDevices(remote_id=self.remote_id)
#         for anchor in self.anchors:
#             status &= self.pozyx.addDevice(anchor, remote_id=self.remote_id)
#         if len(self.anchors) > 4:
#             status &= self.pozyx.setSelectionOfAnchors(PozyxConstants.ANCHOR_SELECT_AUTO, len(self.anchors),
#                                                        remote_id=self.remote_id)

#         if save_to_flash:
#             self.pozyx.saveAnchorIds(remote_id=self.remote_id)
#             self.pozyx.saveRegisters([PozyxRegisters.POSITIONING_NUMBER_OF_ANCHORS], remote_id=self.remote_id)
#         return status

#     def printPublishConfigurationResult(self):
#         """Prints and potentially publishes the anchor configuration result in a human-readable way."""
#         list_size = SingleRegister()

#         self.pozyx.getDeviceListSize(list_size, self.remote_id)
#         print("List size: {0}".format(list_size[0]))
#         if list_size[0] != len(self.anchors):
#             self.printPublishErrorCode("configuration")
#             return
#         device_list = DeviceList(list_size=list_size[0])
#         self.pozyx.getDeviceIds(device_list, self.remote_id)
#         print("Calibration result:")
#         print("Anchors found: {0}".format(list_size[0]))
#         print("Anchor IDs: ", device_list)

#         for i in range(list_size[0]):
#             anchor_coordinates = Coordinates()
#             self.pozyx.getDeviceCoordinates(device_list[i], anchor_coordinates, self.remote_id)
#             print("ANCHOR, 0x%0.4x, %s" % (device_list[i], str(anchor_coordinates)))
#             sleep(0.025)

#     def printPublishAnchorConfiguration(self):
#         """Prints and potentially publishes the anchor configuration"""
#         for anchor in self.anchors:
#             print("ANCHOR,0x%0.4x,%s" % (anchor.network_id, str(anchor.coordinates)))
#             sleep(0.025)


    

if __name__ == "__main__":

    rospy.init_node('DiscreteLocalization', anonymous=True)

    # Read parameters
    targetDeviceId= rospy.get_param('~targetDeviceId')
    algorithm= rospy.get_param('~algorithm')
    dimension = rospy.get_param('~dimension')
    rangingTopic= rospy.get_param('~rangingTopic')
    anchorsTopic = rospy.get_param('~anchorsTopic')
    positionTopic = rospy.get_param('~positionTopic')
    height = None
    if rospy.has_param('~fixedHeight'):
        height = rospy.get_param('~fixedHeight')


    pub_localization = rospy.Publisher(positionTopic, PoseWithCovarianceStamped, queue_size=100)
    rate = rospy.Rate(30) 


    print("=========== Discrete Localizator ============")
    print("targetDeviceId: " + targetDeviceId)
    print("algorithm: " + algorithm)
    print("dimension: " + dimension)
    print("=========== [---------------] ============")


    discreteLocalizator = DiscreteLocalizator(rate, pub_localization, targetDeviceId, height)


    rospy.Subscriber(anchorsTopic, MarkerArray, discreteLocalizator.callbackAnchorsMessage)
    rospy.Subscriber(rangingTopic, Ranging, discreteLocalizator.callbackRangingMessage)

    print("Waiting anchors positions...")
    rospy.spin()


