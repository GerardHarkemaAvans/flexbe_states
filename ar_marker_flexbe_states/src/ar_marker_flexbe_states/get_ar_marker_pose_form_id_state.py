#!/usr/bin/env python

import rospy
import rostopic

from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient
from ar_track_alvar_msgs.msg import *
from flexbe_core.proxy import ProxySubscriberCached

'''
Created on november 11, 2021


@authour: Gerard Harkema
'''

class GetArMarkerPoseFromId(EventState):
  '''
  Returns the pose of a specific ar_markers, by id

  #> id     uint32                        Identification of the marker
  #> pose		geometry_msgs/PoseStamped		    Pose if the marker
  <= done                                  Function found list of marker, note: list can be empty.
  <= failed                                Failed to get list of markers.

  '''

  def __init__(self):
    '''
    Constructor
    '''
    super(GetArMarkerPoseFromId, self).__init__(outcomes=['done', 'not_found', 'failed'],
        input_keys = ['id'], output_keys = ['pose'])

    self._ar_marker_topic= '/ar_pose_marker'
    self._connected_to_ar_marker_topic = False
    self._ar_marker_message=None
    self._pose = None

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

    if self._not_found == True:
        return 'not_found'

    if not self._connected_to_ar_marker_topic:
      return 'failed'

    userdata.pose = self._pose
    return 'done'

  def on_enter(self, userdata):

    self._not_found = False
    self._pose = None

    if not self._connected_to_ar_marker_topic:
      return

    if self._ar_marker_message == None:
        self._not_found = True
        return

    markers = self._ar_marker_message.markers

    if len(markers) == 0:
        self._not_found = True
        Logger.logwarn('No markers found')
        return

    for i in range(len(markers)):
        if markers[i].id == userdata.id:
            self._pose = markers[i].pose
            self._pose.header = markers[i].header
            break

  def on_stop(self):
    pass

  def on_pause(self):
    pass

  def on_resume(self, userdata):
    pass

  def ar_marker_callback(self, ar_marker_message):
    self._ar_marker_message = ar_marker_message
