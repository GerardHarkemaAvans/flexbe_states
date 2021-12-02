#!/usr/bin/env python

import rospy
import rostopic

import xml.etree.ElementTree as ET
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient
from open_manipulator_msgs.srv import *
from open_manipulator_msgs.msg import *
from flexbe_core.proxy import ProxySubscriberCached

import time

'''
Created on november 11, 2021


@authour: Gerard Harkema
'''

class srdfSetJointSpacePathState(EventState):
  '''
  State to look up a pre-defined joint configuration from the SRDF file loaded in the parameter server (/robot_description_semantic)
  and send it to open manipulator to plan and move.

  ># config_name          string              Name of the joint configuration of interest.

  ># group           string              Name of the group to be used for planning.

  ># robot_name           string              Optional name of the robot to be used.
                                                          If left empty, the first one found will be used
                                                          (only required if multiple robots are specified in the same file).

  #> joint_names		string[]	    Names of the joints to set.
					  Does not need to specify all joints.
  #> joint_values		float[]		    Target configuration of the joints.
					  Same order as their corresponding names in joint_names.
  <= done                                  Target joint configuration has been reached.
  <= planning_failed                          Failed to find a plan to the given joint configuration.
  <= control_failed                           Failed to move the arm along the planned trajectory.

  '''

  def __init__(self, path_time=2.0):
    '''
    Constructor
    '''
    super(srdfSetJointSpacePathState, self).__init__(outcomes=['done', 'failed'],
    input_keys = ['config_name', 'group', 'robot_name'],
                  output_keys=['joint_values', 'joint_names'])


    self._param_error = False
    self._srdf = None
    self._service_timeout = 3
    self._service_up = True
    self._path_time = path_time
    self._is_planned = False
    self._states_topic = '/states'
    self._connected_to_status_topic = False
    self._moveing = False

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

    Logger.loginfo('Waiting for open manipulator goal_joint_space_path service...')
    try:
      rospy.wait_for_service('goal_joint_space_path', self._service_timeout)
    except rospy.ROSException, e:
      Logger.logwarn('Open manipulator: goal_joint_space_path service not up')
      self._service_up = False
      return

    self.set_joint_position_srv = rospy.ServiceProxy('goal_joint_space_path', SetJointPosition)

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

    if not self._moveing:
      return 'done'

  def on_enter(self, userdata):

    self._is_planned = False

    if not (self._service_up or self._connected_to_status_topic):
      return

    self._config_name  = userdata.config_name
    self._group   = userdata.group
    self._robot_name   = userdata.robot_name

    self._srdf_param = None
    if rospy.has_param('/robot_description_semantic'):
      self._srdf_param = rospy.get_param('/robot_description_semantic')
    else:
      Logger.logerr('Unable to get parameter: %s' % srdf_param)
      return

    self._param_error = False
    self._srdf = None

    try:
      self._srdf = ET.fromstring(self._srdf_param)
    except Exception as e:
      Logger.logwarn('Unable to parse given SRDF parameter: /robot_description_semantic')
      return

    if not self._param_error:
      robot = None
      for r in self._srdf.iter('robot'):
        if self._robot_name == '' or self._robot_name == r.attrib['name']:
          robot = r
          break
      if robot is None:
        Logger.logwarn('Did not find robot name in SRDF: %s' % self._robot_name)
        return

      config = None
      for c in robot.iter('group_state'):
        if (self._group == c.attrib['group']) and c.attrib['name'] == self._config_name:
          config = c
          break

      if config is None:
        Logger.logwarn('Did not find group %s and config %s in SRDF' % self._group, self._config_name)
        return

      # get joint values from specific jount in group_state
      self._joint_config=[]
      self._joint_names=[]
      for j in config.iter('joint'):
        self._joint_config.append(float(j.attrib['value']))
        self._joint_names.append(j.attrib['name'])

      if len(self._joint_names) == 0:
        Logger.logwarn('No joints found')
        return

      if 0: # only for testing
        for i in range(len(self._joint_names)):
          Logger.logwarn('Jointname: %s' % self._joint_names[i])
          Logger.logwarn('Jointconfig: %f' % self._joint_config[i])

      userdata.joint_values = self._joint_config  # Save joint configuration to output key
      userdata.joint_names  = self._joint_names  # Save joint names to output key

      set_joint_position = SetJointPositionRequest()

      set_joint_position.joint_position.position = self._joint_config
      set_joint_position.joint_position.joint_name = self._joint_names
      set_joint_position.path_time = self._path_time

      response = self.set_joint_position_srv(set_joint_position)
      #Logger.logwarn('starting maniplulator')
      #Logger.logwarn('%s' % response)

      self._is_planned = response.is_planned
      time.sleep(0.5) # wait a while for the actual start

    else:
      self._is_planned = False


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
