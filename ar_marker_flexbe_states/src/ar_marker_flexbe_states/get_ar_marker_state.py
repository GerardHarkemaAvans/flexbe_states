#!/usr/bin/env python

import rospy
import rostopic

from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient
from ar_track_alvar_msgs.msg import *
from flexbe_core.proxy import ProxySubscriberCached

import time

'''
Created on november 11, 2021


@authour: Gerard Harkema
'''

class GetArMarkerState(EventState):
  '''
  Returns list of detected ar_markers

  #> ar_markers		ar_track_alvar_msgs/AlvarMarkers		    List of ar_markers
  <= done                                  Function found list of marker, note: list can be empty.
  <= failed                                Failed to get list of markers.

  '''

  def __init__(self):
    '''
    Constructor
    '''
    super(GetArMarkerState, self).__init__(outcomes=['done', 'failed'],
    output_keys = ['ar_markers'])


    self._ar_marker_topic= '/ar_pose_marker'
    self._connected_to_ar_marker_topic = False
    self.ar_marker_message=None

    Logger.loginfo('Connect to ar marker topic...')
    self.state_sub = rospy.Subscriber(self._ar_marker_topic, AlvarMarkers, self.ar_marker_callback)
    #Logger.loginfo('Successfully subscribed to previously unavailable topic %s' % self._ar_marker_topic)

    ''' this does not work
    if self.state_sub:
        Logger.loginfo('Successfully subscribed to previously unavailable topic %s' % self.ar_marker_topic)
    else:
        Logger.logwarn('Topic %s still not available, giving up.' % self.ar_marker_topic)
        return
    '''
    self._connected_to_ar_marker_topic = True


  def execute(self, userdata):
    # This method is called periodically while the state is active.
    # Main purpose is to check state conditions and trigger a corresponding outcome.
    # If no outcome is returned, the state will stay active.

    if not self._connected_to_ar_marker_topic:
      return 'failed'

    userdata.ar_markers = self.ar_marker_message
    return 'done'

  def on_enter(self, userdata):
    pass

  def on_stop(self):
    pass

  def on_pause(self):
    pass

  def on_resume(self, userdata):
    pass

  def ar_marker_callback(self, ar_marker_message):
    self.ar_marker_message = ar_marker_message
