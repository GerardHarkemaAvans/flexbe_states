#!/usr/bin/env python


import rospy
import rostopic

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from geometry_msgs.msg import PoseStamped, Pose


import math
import sys
import copy
import time

from tf.transformations import *

import geometry_msgs
import tf2_ros
import tf2_geometry_msgs
import actionlib

from flexbe_core.proxy import ProxyActionClient
from open_manipulator_msgs.srv import *
from open_manipulator_msgs.msg import *
from flexbe_core.proxy import ProxySubscriberCached


'''
Created on Nov 8 2021
@author: Garard Harkema
This state sets the joints of a open manipultor joints to a specific kinimatic pose
'''

class setTaskSpacePathState(EventState):
  '''
  Computes the joint configuration needed to grasp the part given its pose.
  -- end_effector_name    string
  -- path_time            float64
  -- robot_base_link      string
  ># pose_stamped         geometry_msgs/PoseStamped

  '''
  def __init__(self, end_effector_name='gripper', rotation=0.0, path_time=2.0, robot_base_link='link1', offset_z = 0.0):
    # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
    super(setTaskSpacePathState, self).__init__(outcomes = ['done', 'failed'], input_keys = ['pose_stamped'])

    self._end_effector_name = end_effector_name
    self._path_time = path_time
    self._robot_base_link = robot_base_link
    self._open_manipulator_moving_state = 'STOPPED'
    self._connected_to_status_topic = False

    self._service_timeout = 3
    self._service_up = True
    self._path_time = path_time
    self._is_planned = False
    self._states_topic = '/states'
    self._connected_to_status_topic = False
    self._moveing = False
    self._offset_z = offset_z
    self._rotation = rotation

    Logger.loginfo('Connect to open maniplator states topic...')
    self.state_sub = rospy.Subscriber(self._states_topic, OpenManipulatorState, self.state_callback)
    #Logger.loginfo('Successfully subscribed to previously unavailable topic %s' % self._states_topic)

    ''' this doe not work
    if self.state_sub:
        Logger.loginfo('Successfully subscribed to previously unavailable topic %s' % self._states_topic)
    else:
        Logger.logwarn('Topic %s still not available, giving up.' % self._states_topic)
        return
    '''
    self._connected_to_status_topic = True

    Logger.loginfo('Waiting for open manipulator goal_task_space_path service...')
    try:
      rospy.wait_for_service('goal_task_space_path', self._service_timeout)
    except rospy.ROSException, e:
      Logger.logwarn('Open manipulator: goal_joint_space_path service not up')
      self._service_up = False
      return

    self.set_task_space_path_srv = rospy.ServiceProxy('goal_task_space_path', SetKinematicsPose)

    # tf to transfor the object pose
    self._tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0)) #tf buffer length
    self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

  def execute(self, userdata):
    # This method is called periodically while the state is active.
    # Main purpose is to check state conditions and trigger a corresponding outcome.
    # If no outcome is returned, the state will stay active.
    if not (self._service_up or self._connected_to_status_topic):
      Logger.logwarn('Not connected')
      return 'failed'

    if not self._is_planned:
      Logger.logwarn('Not planned')
      return 'failed'

    if not self._failed:
      return 'failed'

    if not self._moveing:
      return 'done'

  def on_enter(self, userdata):

    self._is_planned = False
    self._failed = False

    if not (self._service_up or self._connected_to_status_topic):
      return

    Logger.loginfo('input_pose %s' % userdata.pose_stamped)
    rospy.logerr(userdata.pose_stamped)

    # Get transform between camera and robot1_base
    while True:
      #rospy.loginfo('.')
      rospy.sleep(0.1)
      try:
        target_pose = self._tf_buffer.transform(userdata.pose_stamped, self._robot_base_link)
        break
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("setTaskSpacePathState::on_enter - Failed to transform to world")
        self._failed = True
        pass

    #Logger.loginfo('target_pose %s' % target_pose)

    set_kinematics_position = SetKinematicsPoseRequest()
    set_kinematics_position.end_effector_name = self._end_effector_name;
    set_kinematics_position.kinematics_pose.pose = target_pose.pose


    orientation = quaternion_from_euler(0, self._rotation, 0)
    #orientation = quaternion_from_euler(0, 0, 0)
    #rospy.loginfo(orientation)

    set_kinematics_position.kinematics_pose.pose.position.z += self._offset_z

    if 1:
        set_kinematics_position.kinematics_pose.pose.orientation.x = orientation[0]
        set_kinematics_position.kinematics_pose.pose.orientation.y = orientation[1]
        set_kinematics_position.kinematics_pose.pose.orientation.z = orientation[2]
        set_kinematics_position.kinematics_pose.pose.orientation.w = orientation[3]
    else:
        set_kinematics_position.kinematics_pose.pose.orientation.x = 0
        set_kinematics_position.kinematics_pose.pose.orientation.y = 0.66
        set_kinematics_position.kinematics_pose.pose.orientation.z = 0
        set_kinematics_position.kinematics_pose.pose.orientation.w = 0.74


    set_kinematics_position.path_time = self._path_time

    #Logger.loginfo('set_kinematics_position = %s' % set_kinematics_position)

    response = self.set_task_space_path_srv(set_kinematics_position)

    self._is_planned = response.is_planned
    time.sleep(0.5) # wait a while for the actual start

  def on_stop(self):
    pass

  def on_pause(self):
    pass

  def on_resume(self, userdata):
    pass

  def state_callback(self, state_message):
    #Logger.logwarn('%s' % state_message.open_manipulator_moving_state)
    if state_message.open_manipulator_moving_state == state_message.IS_MOVING:#"IS_MOVING":
        self._moveing = True;
    else:
        self._moveing = False;
