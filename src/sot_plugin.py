from PythonQt import QGraphViz, QtGui, Qt
from command_execution import CommandExecution
import rospy

hookRegistration = """\
class CallbackRobotAfterIncrement(object):
  def __init__(self):
    self.hook = dg.signal_base.SignalWrapper ("hook_after_increment", "bool", self.callback)
    self.bMaxLen = 1000
    self.signals = []
    self.buffers = []
    self.robotSimu = None
    for e in dg.entity.Entity.entities.values():
      if e.className == "RobotSimu":
        self.robotSimu = e
        break
  def register(self):
    self.robotSimu.after.rmSignal("python_signals.hook_after_increment")
    self.robotSimu.after.addSignal("python_signals.hook_after_increment")
  def callback(self, time):
    # print("Called at time " + str(time))
    for s,b in zip(self.signals, self.buffers):
      if s.time == time:
        b.append(s.value)
      else:
        b.append(None)
  def watchSignal(self,entity,signal):
    from collections import deque
    e = dg.entity.Entity.entities[entity]
    s = e.signal(signal)
    self.signals.append(s)
    self.buffers.append(deque (maxlen=self.bMaxLen))
  def unwatchSignal(self,entity,signal):
    e = dg.entity.Entity.entities[entity]
    s = e.signal(signal)
    idx = self.signals.index(s)
    self.signals.pop(idx)
    self.buffers.pop(idx)
  def fetch(self):
    ret = []
    if len(self.buffers) == 0: return ret
    c = len(self.buffers[-1])
    for b in self.buffers: ret.append([None,] * c)
    for i in range(c):
        for j,b in enumerate(self.buffers):
            ret[j][i] = b.popleft()
    return ret
"""

class Graph:
    def __init__(self, plugin):
        self.plugin = plugin
        self.graph = QGraphViz.QGVScene("graph")
        self.view = QtGui.QGraphicsView (self.graph)
        self.layoutShouldBeFreed = False
        self.graph.connect(Qt.SIGNAL("nodeMouseRelease(QGVNode*)"), self.updateLayout)
        self.graph.connect(Qt.SIGNAL("nodeContextMenu(QGVNode*)" ), self._nodeContextMenu)
        self.graph.connect(Qt.SIGNAL("edgeContextMenu(QGVEdge*)" ), self._signalContextMenu)

        self.typeCallbacks = {
                "Task": ( self._nodeEntityTask, self._edgeEntityTask ),
                "SOT" : ( self._nodeEntitySOT , self._edgeEntitySOT  )
                }

        self.initCmd()
        self.createAllGraph()

    def clear (self):
        if self.layoutShouldBeFreed:
            self.graph.freeLayout()
            self.layoutShouldBeFreed = False
        self.nodes = {}
        self.types = {}
        self.edges = {}
        self.edgesBack = {}
        self.subgraphs = {}
        self.graph.clear()


    def initLayout (self):
        if self.layoutShouldBeFreed:
            self.graph.freeLayout()
        self.graph.applyLayout("dot")
        self.graph.setNodePositionAttribute()
        self.graph.setGraphAttribute("splines","spline")
        self.graph.setGraphAttribute("overlap","false")
        self.graph.setNodeAttribute ("pin","true")
        self.layoutShouldBeFreed = True

    def updateLayout (self):
        if self.layoutShouldBeFreed:
            self.graph.freeLayout()
        self.graph.applyLayout("nop2")
        self.layoutShouldBeFreed = True

    def initCmd (self):
        self.cmd = CommandExecution()

    def createAllGraph (self):
        entities = self.cmd.run ("dg.entity.Entity.entities.keys()")
        self.clear()
        for e in entities:
            etype = self.cmd.run("dg.entity.Entity.entities['"+e+"'].className")
            self.types[e] = etype
            if self.typeCallbacks.has_key(etype):
                self.typeCallbacks[etype][0] (e)
            else:
                self._nodeEntity(e)
        for e in entities:
            etype = self.types[e]
            if self.typeCallbacks.has_key(etype):
                self.typeCallbacks[etype][1] (e)
            else:
                self._edgeEntitySignals (e)
        self.initLayout()

    def createGraphBackwardFromEntity (self, e):
        ok = self.cmd.run("dg.entity.Entity.entities.has_key('"+e+"')")
        if not ok:
            raise ValueError ("Entity " + e + " does not exist")
        self.clear()
        self._createGraphBackwardFromEntity (e)
        self.initLayout()

    def _createGraphBackwardFromEntity (self, e):
        etype = self.cmd.run("dg.entity.Entity.entities['"+e+"'].className")
        self.types[e] = etype
        if self.typeCallbacks.has_key(etype):
            self.typeCallbacks[etype][0] (e)
            self.typeCallbacks[etype][1] (e)
        else:
            self._nodeEntity(e)
            self._edgeEntitySignals (e)

    def _nodeEntitySOT(self, s):
        subgraph = self.graph.addSubGraph("Tasks in " + s, True) # True means "cluster"
        # subgraph.setAttribute("rank", "same")
        # subgraph.setAttribute("rank", "max")
        # subgraph.setAttribute("rankdir", "LR")
        subgraph.setAttribute("ranksep", "0.1")
        # print self.cmd.run("dg.entity.Entity.entities['"+s+"'].display()")
        tasks = eval(self.cmd.run("dg.entity.Entity.entities['"+s+"'].list()"))
        nodes = []
        for i, t in enumerate(tasks):
            node = subgraph.addNode (str(i))
            #TODO set properties
            nodes.append(node)
        # node = self.graph.addNode(s)
        node = subgraph.addNode(s)
        # node.setAttribute("rank", "max")
        self.nodes[s] = node
        for prev, node in zip(nodes,nodes[1:] + [node,]):
            edge = self.graph.addEdge (prev,node)
            # edge.setAttribute("constraint","false")
            #TODO set properties
        self.subgraphs[s] = (subgraph, tasks, nodes)
        # self._nodeEntity(s)

    def _edgeEntitySOT(self, s):
        subgraph, tasks, nodes = self.subgraphs[s]
        for t,n in zip(tasks, nodes):
            if not self.nodes.has_key(t):
                self._createGraphBackwardFromEntity(t)
            self.graph.addEdge (self.nodes[t], n)

    def _nodeEntityTask(self, t):
        self._nodeEntity(t)

    def _edgeEntityTask(self, t):
        # Build features list
        features = eval(self.cmd.run("dg.entity.Entity.entities['"+t+"'].list()"))
        for f in features:
            if not self.nodes.has_key(f):
                self._createGraphBackwardFromEntity(f)
            edge = self.graph.addEdge (self.nodes[f], self.nodes[t])
            # TODO set edge properties
            edge.setAttribute("color", "red")
        self._edgeEntitySignals (t)
        pass

    def _nodeEntity(self, e):
        self.nodes[e] = self.graph.addNode (e)

    def _edgeEntitySignals(self, e):
        signals = self.cmd.run("[ s.name for s in dg.entity.Entity.entities['"+e+"'].signals() ]")
        for s in signals:
            ss = s.split("::")
            if len(ss) != 3:
                print "Cannot handle", s
            elif ss[1].startswith("in"):
                plugged = self.cmd.run("dg.entity.Entity.entities['"+e+"'].signal('"+ss[-1]+"').isPlugged()")
                other_s = self.cmd.run("dg.entity.Entity.entities['"+e+"'].signal('"+ss[-1]+"').getPlugged().name if dg.entity.Entity.entities['"+e+"'].signal('"+ss[-1]+"').isPlugged() else None")
                if other_s is not None and s != other_s:
                    idx = other_s.index('(')+1
                    other_e = other_s[idx:other_s.index(')',idx)]
                    if not self.nodes.has_key(other_e):
                        self._createGraphBackwardFromEntity(other_e)
                    self.edges[s] = (other_e, e,
                            self.graph.addEdge (self.nodes[other_e], self.nodes[e], ss[2]))
                    self.edgesBack[self.edges[s][2]] = s
            elif ss[1].startswith("out"):
                pass
            else:
                print "unknown", s

    def _nodeContextMenu (self, node):
        e = node.getAttribute("label")
        if self.nodes.has_key(e):
            menu = QtGui.QMenu("Entity " + e, self.view)
            a = menu.addAction("Show graph backward")
            a.connect(Qt.SIGNAL("triggered()"), lambda: self.createGraphBackwardFromEntity(e))
            menu.popup(QtGui.QCursor.pos())

    def _signalContextMenu (self, edge):
        s = edge.getAttribute("xlabel")
        if self.edgesBack.has_key(edge):
            e = self.edges[self.edgesBack[edge]][1]
            menu = QtGui.QMenu("Signal " + e, self.view)
            a = menu.addAction("Toggle display value")
            a.connect(Qt.SIGNAL("triggered()"), lambda: self.plugin.toggleDisplaySignalValue(e, s))
            menu.popup(QtGui.QCursor.pos())

class Plugin(QtGui.QDockWidget):
    def __init__(self, main):
        super(Plugin, self).__init__ ("Stack Of Tasks plugin", main)
        self.setObjectName("Stack Of Tasks plugin")
        self.main = main
        self.graph = Graph (self)

        cw = QtGui.QWidget (self)
        layout = QtGui.QVBoxLayout (cw)
        layout.addWidget (self.graph.view)
        # cw.setLayout(layout)
        self.setWidget (cw)

        toolBar = QtGui.QToolBar ("SoT buttons")
        toolBar.addAction(QtGui.QIcon.fromTheme("view-refresh"), "Create entire graph", self.graph.createAllGraph)
        toolBar.addAction(QtGui.QIcon.fromTheme("window-new"), "Create viewer", self.createRobotView)
        main.addToolBar (toolBar)

        self.displaySignals = []
        self.timerDisplaySignalValues = None

    def createRobotView (self):
        from pinocchio import RobotWrapper, se3
        import os
        # file = str(QtGui.QFileDialog.getOpenFileName(self, "Robot description file"))
        # print file
        file = "/local/jmirabel/devel/openrobots/install/share/talos_data/robots/talos_reduced.urdf"
        self.robot = RobotWrapper (
                filename = file,
                package_dirs = os.getenv("ROS_PACKAGE_PATH", None),
                # package_dirs = [ "/local/jmirabel/devel/openrobots/install/share", ],
                root_joint = se3.JointModelFreeFlyer())
        # self.robot.initDisplay(loadModel=True)
        self.robot.initDisplay()
        # FIXME Hack because for some reason, creating the first OSGWidget with
        # CORBA creates a SEGV...
        # The second OSGWidget does not cause a crash and the first through
        # self.main.createView does not neither.
        self.robot.viewer.gui.createWindow = self._createView
        self.robot.loadDisplayModel("world/pinocchio")
        cmd = self.graph.cmd
        q = cmd.run("robot.dynamic.position.value")
        q = self._sotToPin (q)
        self.robot.display(q)

    def toggleDisplaySignalValue (self, entity, signal):
        k = (entity, signal)
        try:
            idx = self.displaySignals.index(k)
            self.displaySignals.pop(idx)
            self._dehookSignal(entity, signal)
            if len(self.displaySignals) == 0:
                self.timerDisplaySignalValues.stop()
        except ValueError:
            if self.timerDisplaySignalValues is None:
                self._registerHook()
                self.timerDisplaySignalValues = Qt.QTimer(self)
                self.timerDisplaySignalValues.setInterval(50)
                self.timerDisplaySignalValues.connect(Qt.SIGNAL("timeout()"), self._fetchNewSignalValues)
            self._hookSignal(entity, signal)
            self.displaySignals.append(k)
            self.timerDisplaySignalValues.setSingleShot(False)
            self.timerDisplaySignalValues.start()


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
        self.graph.cmd.run ("hook.unwatchSignal('"+entity+"', '"+signal+")", False)

    def _fetchNewSignalValues(self):
        values = self.graph.cmd.run ("hook.fetch()")
        if len(values) == len(self.displaySignals):
            if len(values) > 0:
                ntimes = len(values[0])
                if ntimes > 0:
                    print values
        else:
            print "Expected a list of size", len(self.displaySignals)
            print "Got a list of size", len(values)

    def refreshInterface(self):
        self.graph.createAllGraph()
