
#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import PoseStamped

from flexbe_core import EventState, Logger
from tf.transformations import *


'''
Created on 06/05/2022

@author: Gerard Harkema
'''
class TFTransformState(EventState):
  '''
  Transforms a pose to an new frame_id

  ># input_pose
  ># new_frame_id

  #> transformd_pose 	PoseStamped	The current transformation between the frames

  <= done 					Transformation has been retrieved.
  <= failed 				Failed to retrieve transformation.

  '''

  def __init__(self, new_frame_id = 'world'):
    '''Constructor'''
    super(TFTransformState, self).__init__(outcomes = ['done', 'failed'], input_keys = ['input_pose', 'transformd_pose'])

    self._new_frame_id = new_frame_id
    self._failed = False
    # tf to transfor the object pose
    self._tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0)) #tf buffer length
    self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)


  def execute(self, userdata):

    if self._failed:
      return 'failed'
    else:
      return 'done'

  def on_enter(self, userdata):
    self._failed = False

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
      try:
        trans = self._tf_buffer.lookup_transform(self._new_frame_id, userdata.input_pose, rospy.Time())
        #Logger.loginfo('Getting transform %s' % (trans))
        break
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rate.sleep()
        continue
    br = tf2_ros.StaticTransformBroadcaster()
    Logger.loginfo('sending transform')
    trans.child_frame_id = userdata.transformd_pose
    trans.header.stamp = rospy.Time.now()

    #Logger.loginfo('Ouput transform %s' % (trans))
    
    br.sendTransform(trans)


