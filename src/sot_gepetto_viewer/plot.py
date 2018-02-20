from PythonQt import QtGui, Qt
from PythonQt.QCustomPlot import QCustomPlot

pens = (
    Qt.QPen (Qt.Qt.black),
    Qt.QPen (Qt.Qt.red),
    Qt.QPen (Qt.Qt.green),
    Qt.QPen (Qt.Qt.blue),
    Qt.QPen (Qt.Qt.magenta),
    Qt.QPen (Qt.Qt.yellow),
    Qt.QPen (Qt.Qt.cyan),
    Qt.QPen (Qt.Qt.gray),
    )

class Plot (QtGui.QWidget):
  def __init__ (self, plugin):
    super(Plot, self).__init__ (self)
    self.plugin = plugin

    self.qcp = QCustomPlot ()
    layout = QtGui.QVBoxLayout (self)
    layout.addWidget (self.qcp)
    self.qcp.setInteraction(1, True) # iRangeDrag
    self.qcp.setInteraction(2, True) # iRangeZoom
    self.qcp.setAutoAddPlottableToLegend(True)
    self.qcp.legend().setVisible(True)

    self.timer = Qt.QTimer(self)
    self.timer.setSingleShot(False)
    self.timer.setInterval(100)
    self.timer.connect(Qt.SIGNAL("timeout()"), self._step)

  def initCurves(self, signals):
    self.qcp.clearGraphs()

    self.signals = signals
    self.times = []
    for i,s in enumerate(signals):
      graph = self.qcp.addGraph()
      graph.setName(s[0] + '.' + s[1])
      graph.setPen(pens[i])
    if len(signals) > 0:
      self.timer.start()

  def stopAnimation (self):
    print "Stop animation"
    self.timer.stop()

  def _step(self):
    # print "step", frame
    nys = self.plugin._fetchNewSignalValues ()
    if nys is not None and len(nys) == len(self.signals):
      if len(nys) > 0:
        l = len(nys[0])
        # l = len(nys[0][1])
        if l > 0:
          # ntimes = [ ny[0] for ny in nys
          # print nys
          # start = (self.times[-1] if len(self.times) > 0 else 0) + 1
          # ntimes = range(start,start+l)
          # self.times.extend(ntimes)
          for i,ny in enumerate(nys):
            nnt = [ n[0]    for n in ny ]
            nny = [ n[1][0] for n in ny ]
            # print ntimes,nny
            self.qcp.addData(i,nnt,nny)

          # self.qcp.xAxis_setRange(0, self.times[-1])
          self.qcp.rescaleAxes()
          self.qcp.replot()
