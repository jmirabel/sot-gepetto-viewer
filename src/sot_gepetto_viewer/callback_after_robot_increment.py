import dynamic_graph as dg
from collections import deque
from threading import Lock

class CallbackRobotAfterIncrement(object):
  def __init__(self):
    sig_container = dg.create_entity("PythonSignalContainer", "python_signals")
    if dg.entity_has_signal (sig_container, "hook_after_increment"):
      dg.entity_execute_command (sig_container, "rmSignal", ("hook_after_increment",))
    self.hook_sig = dg.signal_base.SignalWrapper ("hook_after_increment", "bool", self.callback)
    self.bMaxLen = 1000
    self.signals = []
    self.buffers = []
    self.robotSimu = None
    self.commands = deque()
    self.mutex = Lock()
    for e in dg.entity.Entity.entities.values():
      if e.className == "RobotSimu":
        self.robotSimu = e
        break
      elif e.className.lower().find("device") >= 0:
        print("Using entity " + e.name + " as device")
        self.robotSimu = e
    if self.robotSimu is None:
        print("Robot simu not found. You must manually set member 'robotSimu' to the device entity")

  def register(self):
    self.robotSimu.after.rmSignal("python_signals.hook_after_increment")
    self.robotSimu.after.addSignal("python_signals.hook_after_increment")
  def callback(self, time):
    if not self.mutex.acquire (False):
      print("Not called at time " + str(time))
      return
    while len(self.commands) > 0:
      add, signal = self.commands.popleft()
      if add:
        self.signals.append(signal)
        self.buffers.append(deque (maxlen=self.bMaxLen))
      else:
        idx = self.signals.index(signal)
        self.signals.pop(idx)
        self.buffers.pop(idx)
    # print("Called at time " + str(time))
    for s,b in zip(self.signals, self.buffers):
      if s.time == time:
        b.append((time,s.value))
      elif s.time == time - 1:
        # print "One period late:", s.name, s.time
        b.append((s.time,s.value))
      else:
        print "Too late:", s.name, s.time, time
        # b.append(None)
    self.mutex.release()
  def watchSignal(self,entity,signal):
    e = dg.entity.Entity.entities[entity]
    s = e.signal(signal)
    # self.signals.append(s)
    # self.buffers.append(deque (maxlen=self.bMaxLen))
    self.commands.append ( (True, s) )
  def unwatchSignal(self,entity,signal):
    e = dg.entity.Entity.entities[entity]
    s = e.signal(signal)
    # idx = self.signals.index(s)
    # self.signals.pop(idx)
    # self.buffers.pop(idx)
    self.commands.append ( (False, s) )
  def fetch(self):
    ret = []
    if len(self.buffers) == 0: return ret
    self.mutex.acquire ()
    c = len(self.buffers[-1])
    for b in self.buffers: ret.append([None,] * c)
    for i in range(c):
        for j,b in enumerate(self.buffers):
            assert len(b) > 0
            ret[j][i] = b.popleft()
    self.mutex.release()
    return ret
