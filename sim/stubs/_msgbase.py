import rospy
class V3(object):
    def __init__(self): self.x = self.y = self.z = 0.0
class Q4(object):
    def __init__(self): self.x = self.y = self.z = 0.0; self.w = 1.0
class Header(object):
    def __init__(self): self.stamp = rospy.Time(0); self.frame_id = ""
