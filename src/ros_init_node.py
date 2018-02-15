import rospy

class Plugin:
    def __init__(self, main):
        rospy.init_node ("gepetto_gui", anonymous=True, disable_signals=True)
    def __del__ (self):
        rospy.signal_shutdown ("Deleting the ros_init_node plugin.")
