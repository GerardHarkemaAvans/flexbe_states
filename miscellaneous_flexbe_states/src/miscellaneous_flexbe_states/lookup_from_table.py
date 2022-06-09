#!/usr/bin/env python

import rospy
import xml.etree.ElementTree as ET
from flexbe_core import EventState, Logger


'''
Created on 2021-04-27

@author: Gerard Harkema
'''

class LookupFromTableState(EventState):
    '''
    Query's a value from a table on the parameter server

        -- parameter_name    string      name of the parameter on the parameters server
        -- table_name        string      name of the table to get values from
        -- index_title       string      name of the first column of the table
        -- column_title      string      name of the column to query the desired value

        ># index_value       string      value of the index to be quered
        <# column_value      string      quered value from the table

        <= found                        succes the value is found
        <= not_found                    failed the value is not found
    '''

    def __init__(self, parameter_name, table_name, index_title, column_title):
        '''
        Constructor
        '''
        super(LookupFromTableState, self).__init__(outcomes=['found', 'not_found'],
            input_keys = ['index_value'],
            output_keys=['column_value'])


        self._param_error = False
        self._table = None
        self._parameter_name = parameter_name
        self._table_name = table_name
        self._index_title = index_title
        self._column_title = column_title


    def execute(self, userdata):
        if self._param_error :
            return 'not_found'
        return 'found'


    def on_enter(self, userdata):

        self._success         = False

        self._tables_param = None
        if rospy.has_param(self._parameter_name):
            self._tables_param = rospy.get_param(self._parameter_name)
        else:
            Logger.logerr('Unable to get parameters: %s' % self._parameter_name)
            self._param_error     = True
            return

        self._table = None


        try:
            self._tables = ET.fromstring(self._tables_param)
        except Exception as e:
            Logger.logwarn('Unable to parse given table parameter: %s' % self._parameter_name)
            self._param_error = True
            return

        userdata.column_value = None
        break_flag = False
        if not self._param_error:
            for tables in self._tables.iter('tables'):
                for table in tables.iter('table'):
                    #Logger.logwarn(table.attrib['name'])
                    if self._table_name == table.attrib['name']:
                        for index_column in table.iter(self._index_title):
                            #Logger.logwarn(index_column.attrib['name'])
                            if userdata.index_value == index_column.attrib['name']:
                                #Logger.logwarn("Found")
                                for column in index_column.iter(self._column_title):
                                    #Logger.logwarn(column.attrib['value'])
                                    userdata.column_value = column.attrib['value']
                                    braek_flag = True
                                    break
                            if break_flag:
                                break
                    if break_flag:
                        break

                break # only one tables-section                        
                
						
        if not userdata.column_value:
            Logger.logwarn('Did not found item in lookup table')
            self._param_error     = True
            return

    def on_stop(self):
        pass

    def on_pause(self):
        pass

    def on_resume(self, userdata):
        pass
