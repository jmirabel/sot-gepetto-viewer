from PythonQt import QGraphViz, QtGui, Qt
from command_execution import CommandExecution
import rospy

class Graph:
    def __init__(self, plugin):
        self.plugin = plugin
        self.graph = QGraphViz.QGVScene("graph")
        self.view = QtGui.QGraphicsView (self.graph)
        self.layoutShouldBeFreed = False
        self.graph.connect("nodeMouseRelease(QGVNode*)", self.updateLayout)
        self.graph.connect("nodeContextMenu(QGVNode*)", self._nodeContextMenu)

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
        self.cmd.run ("import dynamic_graph as dg", False)

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
            if ss[1].startswith("in"):
                plugged = self.cmd.run("dg.entity.Entity.entities['"+e+"'].signal('"+ss[-1]+"').isPlugged()")
                other_s = self.cmd.run("dg.entity.Entity.entities['"+e+"'].signal('"+ss[-1]+"').getPlugged().name if dg.entity.Entity.entities['"+e+"'].signal('"+ss[-1]+"').isPlugged() else None")
                if other_s is not None and s != other_s:
                    idx = other_s.index('(')+1
                    other_e = other_s[idx:other_s.index(')',idx)]
                    if not self.nodes.has_key(other_e):
                        self._createGraphBackwardFromEntity(other_e)
                    self.edges[s] = self.graph.addEdge (self.nodes[other_e], self.nodes[e], ss[2])
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

class Plugin(QtGui.QDockWidget):
    def __init__(self, main):
        super(Plugin, self).__init__ ("Stack Of Tasks plugin", main)
        self.setObjectName("Stack Of Tasks plugin")
        self.main = main
        self.graph = Graph (self)

        self.setWidget (self.graph.view)
