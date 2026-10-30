from _msgbase import Header
from geometry_msgs.msg import _Pose, _Tw
class _PC(object):
    def __init__(self): self.pose = _Pose()
class _TC(object):
    def __init__(self): self.twist = _Tw()
class Odometry(object):
    def __init__(self): self.header = Header(); self.pose = _PC(); self.twist = _TC()
