from PythonQt import QtGui, Qt
from graph import Graph
from plot import Plot

hookRegistration = "from sot_gepetto_viewer.callback_after_robot_increment import CallbackRobotAfterIncrement"

class Plugin(QtGui.QDockWidget):
    def __init__(self, main):
        super(Plugin, self).__init__ ("Stack Of Tasks plugin", main)
        self.setObjectName("Stack Of Tasks plugin")
        self.main = main
        self.graph = Graph (self)
        self.plot = Plot (self)

        self.tabWidget = QtGui.QTabWidget(self)
        self.setWidget (self.tabWidget)
        self.tabWidget.addTab (self.graph.view, "SoT graph")
        self.tabWidget.addTab (self.plot, "Plot")

        toolBar = QtGui.QToolBar ("SoT buttons")
        toolBar.addAction(QtGui.QIcon.fromTheme("view-refresh"), "Create entire graph", self.graph.createAllGraph)
        toolBar.addSeparator()
        toolBar.addAction(QtGui.QIcon.fromTheme("media-playback-stop"), "Stop fetching data", self.stopAnimation)
        toolBar.addSeparator()
        toolBar.addAction(QtGui.QIcon.fromTheme("window-new"), "Create viewer", self.createRobotView)
        main.addToolBar (toolBar)

        self.displaySignals = []
        self.hookRegistered = False
        self.displaySignalValuesStarted = False

    def createRobotView (self):
        from pinocchio import RobotWrapper, se3
        import os
        file = str(QtGui.QFileDialog.getOpenFileName(self, "Robot description file"))
        print file
        # file = "/local/jmirabel/devel/openrobots/install/share/talos_data/robots/talos_reduced.urdf"
        self.robot = RobotWrapper (
                filename = file,
                package_dirs = os.getenv("ROS_PACKAGE_PATH", None),
                # package_dirs = [ "/local/jmirabel/devel/openrobots/install/share", ],
                root_joint = se3.JointModelFreeFlyer())
        self.robot.initDisplay()
        self.robot.loadDisplayModel("world/pinocchio")
        cmd = self.graph.cmd
        q = cmd.run("robot.dynamic.position.value")
        q = self._sotToPin (q)
        self.robot.display(q)

    def toggleDisplaySignalValue (self, entity, signal):
        print "Toggle", entity, signal
        k = (entity, signal)
        try:
            idx = self.displaySignals.index(k)
            self.plot.stopAnimation()
            self._dehookSignal(entity, signal)
            self.displaySignals.pop(idx)
        except ValueError:
            if not self.hookRegistered:
                self._registerHook()
                self.hookRegistered = True
            self._hookSignal(entity, signal)
            self.displaySignals.append(k)
        self.plot.initCurves (self.displaySignals)

    def stopAnimation (self):
        self.plot.stopAnimation()
        for k in self.displaySignals:
            self._dehookSignal (k[0], k[1])
        self.displaySignals = []

    def _sotToPin(self, q):
        # Handle the very annoying problem of RPY->quaternion
        # with the good convention...
        from dynamic_graph.sot.tools.quaternion import Quaternion
        import numpy as np
        quat = Quaternion()
        quat = tuple(quat.fromRPY(q[3], q[4], q[5]).array.tolist())
        return np.matrix(q[0:3] + quat[1:] + (quat[0],) + q[6:])

    def _createView (self, name):
        osg = self.main.createView (name)
        return osg.wid()

    def _registerHook (self):
        self.graph.cmd.run (hookRegistration,False)
        self.graph.cmd.run ("hook = CallbackRobotAfterIncrement()", False)
        self.graph.cmd.run ("hook.register()", False)

    def _hookSignal(self, entity, signal):
        self.graph.cmd.run ("hook.watchSignal('"+entity+"', '"+signal+"')", False)
    def _dehookSignal(self, entity, signal):
        self.graph.cmd.run ("hook.unwatchSignal('"+entity+"', '"+signal+"')", False)

    def _fetchNewSignalValues(self):
        values = self.graph.cmd.run ("hook.fetch()")
        return values

    def refreshInterface(self):
        self.graph.createAllGraph()
