"""最小 rospy 桩: 进程内话题总线 + 墙钟时间。只给 stage12 离线仿真用。"""
import logging, threading, time as _t
_log = logging.getLogger("rosout")
_bus = {}            # topic -> [cb]
_services = {}       # name -> fn
_shutdown = [False]
_lock = threading.Lock()
PRINT = [True]

class ROSInterruptException(Exception):
    pass

class Duration(object):
    def __init__(self, secs=0.0):
        self.s = float(secs)
    def to_sec(self):
        return self.s

class Time(object):
    def __init__(self, secs=0.0, nsecs=0):
        self.s = float(secs) + nsecs * 1e-9
    @staticmethod
    def now():
        return Time(_t.time())
    @staticmethod
    def from_sec(s):
        return Time(s)
    def to_sec(self):
        return self.s
    def __add__(self, d):
        return Time(self.s + (d.s if isinstance(d, Duration) else d))
    def __sub__(self, o):
        if isinstance(o, Time):
            return Duration(self.s - o.s)
        return Time(self.s - o.s)
    def __lt__(self, o): return self.s < o.s
    def __le__(self, o): return self.s <= o.s
    def __gt__(self, o): return self.s > o.s
    def __ge__(self, o): return self.s >= o.s
    def __eq__(self, o): return isinstance(o, Time) and self.s == o.s
    def __hash__(self): return hash(self.s)

class Rate(object):
    def __init__(self, hz):
        self.dt = 1.0 / hz
        self.last = _t.time()
    def sleep(self):
        nxt = self.last + self.dt
        d = nxt - _t.time()
        if d > 0:
            _t.sleep(d)
            self.last = nxt
        else:
            self.last = _t.time()

def _pub(topic, msg):
    with _lock:
        cbs = list(_bus.get(topic, []))
    for cb in cbs:
        try:
            cb(msg)
        except Exception as e:
            print("[stub] callback error on", topic, repr(e))

class Publisher(object):
    def __init__(self, topic, mtype=None, queue_size=1, latch=False):
        self.topic = topic
    def publish(self, msg):
        _pub(self.topic, msg)

class Subscriber(object):
    def __init__(self, topic, mtype=None, cb=None, queue_size=1):
        with _lock:
            _bus.setdefault(topic, []).append(cb)

def ServiceProxy(name, stype=None):
    return lambda *a, **k: _services[name](*a, **k)

def wait_for_service(name, timeout=None):
    return True

def init_node(name, **kw):
    pass

def get_name():
    return "/flight_ctrl"

def is_shutdown():
    return _shutdown[0]

def signal_shutdown(reason=""):
    _shutdown[0] = True

def sleep(d):
    _t.sleep(d.s if isinstance(d, Duration) else d)

def _mk(level, pfx):
    def f(msg, *a):
        _log.log(level, msg)
        if PRINT[0]:
            print("%.2f %s %s" % (_t.time() % 1000, pfx, msg), flush=True)
    return f
loginfo = _mk(logging.INFO, "I")
logwarn = _mk(logging.WARNING, "W")
logerr = _mk(logging.ERROR, "E")
logfatal = _mk(logging.CRITICAL, "F")
_thr = {}
def _throttle(fn):
    def f(period, msg, *a):
        k = msg[:30]
        now = _t.time()
        if now - _thr.get(k, 0) >= period:
            _thr[k] = now
            fn(msg)
    return f
logwarn_throttle = _throttle(logwarn)
loginfo_throttle = _throttle(loginfo)
logerr_throttle = _throttle(logerr)
