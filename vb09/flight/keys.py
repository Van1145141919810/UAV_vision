# -*- coding: utf-8 -*-
"""
vb09/flight/keys.py —— 终端键盘 (cbreak, 读空缓冲, EOF = SSH 断连)
===============================================================================
2026-10-30 从 stage12_gate.py 原样搬来 (第 4.2 步, tools/dev/split_stage12.py 生成);
共享状态改成 F.名字 (vb09/flight/ctx.py 的 FlightContext 单例), 参数是 C.名字。
"""

import sys

import rospy


# =============================================================================
# ============================   键盘输入处理   ===============================
# =============================================================================
class KeyReader:
    """
    cbreak 模式读键盘。两条要点:

    1) 每轮读空缓冲区, 方向键只保留最后一个。终端按键重复约 30-40 字符/秒,
       和 30Hz 控制环基本持平; 每轮只读 1 字节的话一次调度打嗝就开始积压,
       松开 W 之后飞机还会按 W 飞几秒, dead-man 也永远不触发。

    2) SSH 断连后 read(1) 返回**空串**而不是抛异常 → 识别为 EOF, 调用方立即降落。
       RC 的 SWC/SWD 是坏的, 键盘是唯一的人工急停通道。
    """

    MOVE_KEYS = ('w', 's', 'a', 'd', 'r', 'f', 'q', 'e', ' ')

    def __init__(self):
        import termios
        import tty
        import select
        self._termios = termios
        self._tty = tty
        self._select = select
        self.fd = sys.stdin.fileno()
        self._old = None

    def __enter__(self):
        self._old = self._termios.tcgetattr(self.fd)
        self._tty.setcbreak(self.fd)
        self.drain()
        return self

    def __exit__(self, *exc):
        try:
            self._termios.tcsetattr(self.fd, self._termios.TCSADRAIN, self._old)
        except Exception as e:
            rospy.logwarn(f"[key] 恢复 tty 失败: {e}")
        return False

    def drain(self):
        while self._select.select([sys.stdin], [], [], 0.0)[0]:
            try:
                if sys.stdin.read(1) == "":
                    return
            except Exception:
                return

    def poll(self):
        """
        读空缓冲区。返回 (move_key, flags, eof, n_read)
          move_key: 本批里最后一个方向/悬停键，没有则 None
          flags:    集合，含本批出现过的 'x' / 'l' / 't' / 'g' / 'm'
          eof:      stdin 已关闭 (SSH 断连)
        """
        move = None
        flags = set()
        eof = False
        n = 0
        while self._select.select([sys.stdin], [], [], 0.0)[0]:
            try:
                ch = sys.stdin.read(1)
            except Exception:
                break
            if ch == "":
                eof = True
                break
            n += 1
            k = ch.lower()
            if k in self.MOVE_KEYS:
                move = k
            elif k in ('x', 'l', 't', 'g', 'm', 'y'):
                flags.add(k)
            if n > 256:          # 极端积压，别把这一轮卡死
                break
        return move, flags, eof, n


def _flush_stdin():
    """丢掉终端里积压的按键, 免得之前误按的回车直接把确认提示答掉。"""
    try:
        import termios
        termios.tcflush(sys.stdin, termios.TCIFLUSH)
    except Exception:
        pass
