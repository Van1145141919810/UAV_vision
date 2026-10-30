from _msgbase import V3, Q4, Header
class _Pose(object):
    def __init__(self): self.position = V3(); self.orientation = Q4()
class PoseStamped(object):
    def __init__(self): self.header = Header(); self.pose = _Pose()
class _Tw(object):
    def __init__(self): self.linear = V3(); self.angular = V3()
class TwistStamped(object):
    def __init__(self): self.header = Header(); self.twist = _Tw()
class PointStamped(object):
    def __init__(self): self.header = Header(); self.point = V3()
class _PoseCov(object):
    def __init__(self): self.pose = _Pose(); self.covariance = [0.0] * 36
class PoseWithCovarianceStamped(object):
    def __init__(self): self.header = Header(); self.pose = _PoseCov()
