# -*- coding: utf-8 -*-
"""重构辅助: 精确替换, 每条必须恰好命中 count 次 (默认 1), 否则整个文件不写。
   from rep import R; R(path, [(old, new[, count]), ...])"""
import sys


def R(path, pairs):
    s = open(path, encoding="utf-8").read()
    for item in pairs:
        old, new = item[0], item[1]
        cnt = item[2] if len(item) > 2 else 1
        n = s.count(old)
        if n != cnt:
            sys.exit("%s: expected %d match(es), got %d for:\n%s" % (path, cnt, n, old[:300]))
        s = s.replace(old, new)
    open(path, "w", encoding="utf-8").write(s)
    print("%s: %d edit(s)" % (path, len(pairs)))


def cut(path, start, end, keep_end=False):
    """删掉从 start 标记 (含) 到 end 标记 (不含, keep_end) 之间的文本。各标记必须唯一。"""
    s = open(path, encoding="utf-8").read()
    for m in (start, end):
        if s.count(m) != 1:
            sys.exit("%s: marker count %d: %s" % (path, s.count(m), m[:200]))
    a, b = s.index(start), s.index(end)
    if b < a:
        sys.exit("end before start")
    s = s[:a] + s[b:]
    open(path, "w", encoding="utf-8").write(s)
    print("%s: cut %d chars" % (path, b - a))
