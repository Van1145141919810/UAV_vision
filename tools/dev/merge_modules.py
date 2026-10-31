# -*- coding: utf-8 -*-
"""
一次性重构辅助 (2026-11-01): 把几个模块整文件拼成一个。
    python3 merge_modules.py <目标.py> <文档首行> <源1.py> <源2.py> ...
每个源: 去掉文件头 docstring 和顶层 import 区, 其余 (含注释) 原样按顺序拼接; import 去重后放最前。
源文件之间互相 import 的行 (from <源模块> import ...) 去掉。不删源文件 (调用方自己 git rm)。
"""
import ast
import os
import sys


def split(path):
    s = open(path, encoding="utf-8").read()
    t = ast.parse(s)
    lines = s.splitlines(keepends=True)
    imps, last = [], 0
    for n in t.body:
        if isinstance(n, ast.Expr) and isinstance(n.value, ast.Constant) and n is t.body[0]:
            last = n.end_lineno
            continue
        if isinstance(n, (ast.Import, ast.ImportFrom)):
            imps.append("".join(lines[n.lineno - 1:n.end_lineno]).strip())
            last = n.end_lineno
            continue
        break
    return imps, "".join(lines[last:]).strip("\n") + "\n"


def main():
    dst, doc, srcs = sys.argv[1], sys.argv[2], sys.argv[3:]
    mods = {os.path.relpath(p, os.getcwd())[:-3].replace(os.sep, ".") for p in srcs}
    imps, bodies, seen = [], [], set()
    for p in srcs:
        im, body = split(p)
        for i in im:
            if any(i.startswith("from %s import" % m) for m in mods) or i in seen:
                continue
            seen.add(i)
            imps.append(i)
        bodies.append(body)
    head = '# -*- coding: utf-8 -*-\n"""\n%s\n"""\n' % doc.replace("\\n", "\n")
    out = head + "\n".join(imps) + "\n\n\n" + "\n\n".join(bodies)
    ast.parse(out)
    open(dst, "w", encoding="utf-8").write(out)
    print(dst, len(out.splitlines()), "lines")


if __name__ == "__main__":
    main()
