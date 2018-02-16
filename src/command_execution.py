# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Dorian Scholz
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys
import rospy

import dynamic_graph_bridge.srv
import dynamic_graph_bridge_msgs.srv

# rospy.init_node ("sot_gepetto_viewer_plugin", anonymous=True)

class CommandExecution(object):
    # _localsGetter  = None
    # _globalsGetter = None
    # _locals  = None
    # _globals = None
    _local = False

    def __init__(self):
        self._client = rospy.ServiceProxy(
            'run_command', dynamic_graph_bridge_msgs.srv.RunCommand, True)
        self._locals  = {}
        self._globals = {}
        self._init()

    def _init(self):
        self.run ("import dynamic_graph as dg", False)

    # @staticmethod
    # def localsGetter(g):
      # CommandExecution._localsGetter = g
    # @staticmethod
    # def globalsGetter(g):
      # CommandExecution._globalsGetter = g

    # @staticmethod
    # def locals(l):
      # CommandExecution._locals = l
    # @staticmethod
    # def globals(g):
      # CommandExecution._globals = g
    @staticmethod
    def dgIsLocal():
      CommandExecution._local = True

    def run(self, code, retValue = True):
        # if CommandExecution._localsGetter is not None:
        # if CommandExecution._locals is not None:
        if CommandExecution._local:
          return self.runLocalCode (code, retValue)
        else:
          return self.runRemoteCode (code, retValue, True)

    def runLocalCode(self, code, retValue):
        # print ("Local")
        # l = CommandExecution._localsGetter()
        # g = CommandExecution._globalsGetter() if CommandExecution._globalsGetter is not None else {}
        # l = CommandExecution._locals
        # g = CommandExecution._globals
        # l = self._locals
        l = None
        g = self._globals
        if retValue:
          exec("_returnValue = " + code, g, l)
          return eval("_returnValue", g, l)
        else:
          exec(code, g, l)

    def runRemoteCode(self, code, retValue, retry = True):
        # print ("Remote" + str(retValue))
        try:
            if not self._client:
                if not retry:
                    print("Connection to remote server lost. Reconnecting...")
                self._client = rospy.ServiceProxy(
                    'run_command', dynamic_graph_bridge_msgs.srv.RunCommand, True)
            response = self._client(str(code))
            if response.standardoutput != "":
                print(response.standardoutput[:-1])
            if response.standarderror != "":
                print(response.standarderror[:-1])
            elif response.result != "None":
                if not retValue: return
                res = eval(response.result)
                # print(response.result)
                return res
        except rospy.ServiceException, e:
            print("Connection to remote server lost. Reconnecting...")
            self._client = rospy.ServiceProxy(
                'run_command', dynamic_graph_bridge_msgs.srv.RunCommand, True)
            if retry:
                self.runRemoteCode ("import dynamic_graph as dg", False, False)
                return self.runRemoteCode(code, retValue, False)
            else:
                print("Failed to connect. Is Stack of Tasks running?")
                raise IOError("Failed to connect. Is Stack of Tasks running?")
