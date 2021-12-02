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

class SetToolControlState(EventState):
  '''

  ># tool          string              Name of the tool

  #> joint_angle		float		    Target configuration of the joints.
					  Same order as their corresponding names in joint_names.
  <= done                                  Target joint configuration has been reached.
  <= failed                          Failed to find a plan to the given joint configuration.

  '''

  def __init__(self, tool='gripper'):
    '''
    Constructor
    '''
    super(SetToolControlState, self).__init__(outcomes=['done', 'failed'],
    input_keys = ['joint_angle'])


    self._tool = tool
    self._service_timeout = 3
    self._service_up = True
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

    Logger.loginfo('Waiting for open manipulator goal_tool_control service...')
    try:
      rospy.wait_for_service('goal_tool_control', self._service_timeout)
    except rospy.ROSException, e:
      Logger.logwarn('Open manipulator: goal_joint_space_path service not up')
      self._service_up = False
      return

    self.set_tool_control_srv = rospy.ServiceProxy('goal_tool_control', SetJointPosition)

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
      Logger.logwarn('Not connected')
      return


    set_tool_control = SetJointPositionRequest()

    set_tool_control.joint_position.position.append(userdata.joint_angle)
    set_tool_control.joint_position.joint_name.append(self._tool);

    response = self.set_tool_control_srv(set_tool_control)

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
