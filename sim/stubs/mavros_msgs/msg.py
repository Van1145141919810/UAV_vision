from _msgbase import V3, Header
class PositionTarget(object):
    FRAME_LOCAL_NED = 1
    IGNORE_PX, IGNORE_PY, IGNORE_PZ = 1, 2, 4
    IGNORE_VX, IGNORE_VY, IGNORE_VZ = 8, 16, 32
    IGNORE_AFX, IGNORE_AFY, IGNORE_AFZ = 64, 128, 256
    FORCE, IGNORE_YAW, IGNORE_YAW_RATE = 512, 1024, 2048
    def __init__(self):
        self.header = Header(); self.coordinate_frame = 0; self.type_mask = 0
        self.position = V3(); self.velocity = V3(); self.acceleration_or_force = V3()
        self.yaw = 0.0; self.yaw_rate = 0.0
class State(object):
    def __init__(self): self.connected = True; self.armed = False; self.mode = "POSCTL"
class ExtendedState(object):
    def __init__(self): self.landed_state = 1; self.vtol_state = 0
class EstimatorStatus(object):
    pass
class StatusText(object):
    def __init__(self): self.severity = 6; self.text = ""
class ActuatorControl(object):
    def __init__(self): self.controls = [0.0] * 8
