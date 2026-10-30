# -*- coding: utf-8 -*-
"""
qualify.py —— 重构辅助: 把源码里对一组模块级名字的引用改成 `前缀.名字` (按 AST 位置改, f-string 里也改)
    python3 qualify.py <file.py> <prefix> <names_file> [--skip-defs NAME,NAME]
  · 只改 Name 节点 (Load / Store / Del); 属性名、关键字参数名、字符串里的字面文字都不碰
  · 函数参数或函数内局部赋值和这些名字重名 → 报错退出 (要人看)
  · `global X` 声明里的这些名字会被删掉 (整句只剩它们就删整句)
  · --skip-defs: 不改这些顶层定义 (比如 CONFIG 区本身, 之后整块搬走)
需要 Python ≥ 3.12 (f-string 内表达式的列号才准)。
"""
import ast
import sys


def qualify(src, names, prefix, skip_lines=None):
    """names: 可迭代的名字, 或 {旧名: 新属性名} 字典 (换名)"""
    t = ast.parse(src)
    rename = dict(names) if isinstance(names, dict) else {n: n for n in names}
    names = set(rename)
    edits = []          # (lineno, col, end_col, text)
    problems = []
    lines = src.splitlines(keepends=True)

    for fn in ast.walk(t):
        if isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda)):
            args = fn.args
            params = {a.arg for a in args.args + args.kwonlyargs + args.posonlyargs}
            if args.vararg:
                params.add(args.vararg.arg)
            if args.kwarg:
                params.add(args.kwarg.arg)
            bad = params & names
            if bad:
                problems.append("line %d: parameter shadows %s" % (fn.lineno, sorted(bad)))
            if isinstance(fn, ast.Lambda):
                continue
            glob = {x for st in ast.walk(fn) if isinstance(st, ast.Global) for x in st.names}
            for st in ast.walk(fn):
                if isinstance(st, ast.Name) and isinstance(st.ctx, ast.Store) and st.id in names \
                        and st.id not in glob:
                    problems.append("line %d: local assignment shadows %s" % (st.lineno, st.id))
    if problems:
        return None, problems

    for node in ast.walk(t):
        if isinstance(node, ast.Name) and node.id in names:
            if skip_lines and node.lineno in skip_lines:
                continue
            edits.append((node.lineno, node.col_offset, node.end_col_offset, prefix + "." + rename[node.id]))
        elif isinstance(node, ast.Global):
            keep = [x for x in node.names if x not in names]
            if len(keep) != len(node.names):
                ln = lines[node.lineno - 1]
                seg = ln[node.col_offset:node.end_col_offset]
                new = ("global " + ", ".join(keep)) if keep else "pass"
                edits.append((node.lineno, node.col_offset, node.end_col_offset, new))
                assert seg.startswith("global")
    # 行内列号是 utf-8 字节偏移 → 按字节改
    blines = [ln.encode("utf-8") for ln in lines]
    for ln, c0, c1, txt in sorted(edits, key=lambda e: (e[0], e[1]), reverse=True):
        b = blines[ln - 1]
        blines[ln - 1] = b[:c0] + txt.encode("utf-8") + b[c1:]
    out = b"".join(blines).decode("utf-8")
    ast.parse(out)
    return out, len(edits)


if __name__ == "__main__":
    path, prefix, nf = sys.argv[1:4]
    names = {}
    for x in open(nf):
        p = x.split()
        if p:
            names[p[0]] = p[1] if len(p) > 1 else p[0]
    skip = set()
    if "--skip-lines" in sys.argv:
        a, b = sys.argv[sys.argv.index("--skip-lines") + 1].split("-")
        skip = set(range(int(a), int(b) + 1))
    out, n = qualify(open(path, encoding="utf-8").read(), names, prefix, skip)
    if out is None:
        print("\n".join(n))
        sys.exit(1)
    open(path, "w", encoding="utf-8").write(out)
    print("%s: %d edit(s)" % (path, n))
