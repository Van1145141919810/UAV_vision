# -*- coding: utf-8 -*-
"""静态检查：嵌套函数里给外层变量赋值却没写 nonlocal（会静默失效）

★ 修过一次：原先 V().visit(node) 会直接命中 visit_FunctionDef，
  只记录函数名、不进函数体，导致永远返回 OK（假阴性）。
  现在改成只遍历 node.body 里的语句。
"""
import ast
import sys
from pathlib import Path

TARGET = Path(sys.argv[1] if len(sys.argv) > 1 else
              r"D:\Desktop\无人系统\MiniCPM-V\scripts\breach_run.py")
src = TARGET.read_text(encoding="utf-8")
tree = ast.parse(src)


def names_assigned(fn):
    """fn 函数体内直接被赋值的名字（不递归进它内部的嵌套函数）。"""
    out = set()

    class V(ast.NodeVisitor):
        def visit_FunctionDef(self, n):
            out.add(n.name)          # 只记名字，不深入

        visit_AsyncFunctionDef = visit_FunctionDef

        def visit_Assign(self, n):
            for t in n.targets:          # ★ 只认裸名字；x[0]= 属于就地修改
                if isinstance(t, ast.Name):
                    out.add(t.id)
            self.generic_visit(n)

        def visit_AugAssign(self, n):
            if isinstance(n.target, ast.Name):
                out.add(n.target.id)
            self.generic_visit(n)

        def visit_AnnAssign(self, n):
            if isinstance(n.target, ast.Name):
                out.add(n.target.id)
            self.generic_visit(n)

        def visit_For(self, n):
            if isinstance(n.target, ast.Name):
                out.add(n.target.id)
            self.generic_visit(n)

        def visit_With(self, n):
            for item in n.items:
                if isinstance(item.optional_vars, ast.Name):
                    out.add(item.optional_vars.id)
            self.generic_visit(n)

        def visit_ExceptHandler(self, n):
            if n.name:
                out.add(n.name)
            self.generic_visit(n)

    v = V()
    for stmt in fn.body:             # ★ 只遍历函数体语句（关键修复）
        v.visit(stmt)
    return out


def nonlocals_of(fn):
    return {n for node in fn.body if isinstance(node, ast.Nonlocal)
            for n in node.names}


def params_of(fn):
    a = fn.args
    return ({x.arg for x in a.args} | {x.arg for x in a.kwonlyargs}
            | ({a.vararg.arg} if a.vararg else set())
            | ({a.kwarg.arg} if a.kwarg else set()))


problems = []
checked = 0


def walk(fn, outer, depth=0):
    global checked
    for node in ast.walk(fn):
        if not isinstance(node, ast.FunctionDef) or node is fn:
            continue
        inner = names_assigned(node)
        declared = nonlocals_of(node)
        clash = (inner & outer) - declared - params_of(node)
        checked += 1
        if clash:
            problems.append((node.name, node.lineno, sorted(clash)))
        walk(node, outer | inner, depth + 1)


for node in ast.walk(tree):
    if isinstance(node, ast.FunctionDef):
        walk(node, names_assigned(node))

print(f"检查了 {checked} 个嵌套函数")
if problems:
    print("发现漏写 nonlocal：")
    for name, line, names in problems:
        print(f"  L{line}  {name}() 赋值了外层变量 {names} 但没声明 nonlocal")
    sys.exit(1)
print("OK：没有漏写 nonlocal 的嵌套赋值")
