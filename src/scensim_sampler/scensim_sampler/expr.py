"""A restricted expression: arithmetic, comparisons, `and`/`or`/`not`, `a if c else b`, `in`
over a literal list, a few math functions, and names bound to the values drawn so far (and the
world's knobs). Parsed with `ast` and walked; nothing is passed to `eval`.

    Expr("ego_speed * ttc_entry")({"ego_speed": 8.0, "ttc_entry": 2.0})   # 16.0
    Expr("'night' if hour < 6 else 'day'")({"hour": 3})                   # 'night'
"""
import ast
import math
import operator

FUNCTIONS = {
    "abs": abs, "min": min, "max": max, "round": round,
    "sqrt": math.sqrt, "exp": math.exp, "log": math.log, "floor": math.floor, "ceil": math.ceil,
    "sin": math.sin, "cos": math.cos, "tan": math.tan, "atan2": math.atan2,
    "int": int, "float": float, "str": str,
}
CONSTANTS = {"pi": math.pi, "g": 9.80665, "true": True, "false": False}

BINARY = {
    ast.Add: operator.add, ast.Sub: operator.sub, ast.Mult: operator.mul,
    ast.Div: operator.truediv, ast.FloorDiv: operator.floordiv, ast.Mod: operator.mod,
    ast.Pow: operator.pow,
}
UNARY = {ast.UAdd: operator.pos, ast.USub: operator.neg, ast.Not: operator.not_}
COMPARE = {
    ast.Eq: operator.eq, ast.NotEq: operator.ne, ast.Lt: operator.lt, ast.LtE: operator.le,
    ast.Gt: operator.gt, ast.GtE: operator.ge,
    ast.In: lambda a, b: a in b, ast.NotIn: lambda a, b: a not in b,
}


class ExprError(ValueError):
    pass


class Expr:
    def __init__(self, text, where="expression"):
        if not isinstance(text, str):
            raise ExprError(f"{where}: an expression is a string, not {text!r}")
        self.text, self.where = text, where
        try:
            self.tree = ast.parse(text.strip(), mode="eval").body
        except SyntaxError as e:
            raise ExprError(f"{where}: {text!r} does not parse ({e.msg})") from None
        self.names = set()
        self._check(self.tree)

    def _check(self, n):
        if isinstance(n, ast.Constant):
            if not isinstance(n.value, (int, float, str, bool)):
                raise ExprError(f"{self.where}: {self.text!r}: constant {n.value!r} not allowed")
        elif isinstance(n, ast.Name):
            if n.id not in CONSTANTS:
                self.names.add(n.id)
        elif isinstance(n, ast.BinOp) and type(n.op) in BINARY:
            self._check(n.left), self._check(n.right)
        elif isinstance(n, ast.UnaryOp) and type(n.op) in UNARY:
            self._check(n.operand)
        elif isinstance(n, ast.BoolOp):
            for v in n.values:
                self._check(v)
        elif isinstance(n, ast.Compare) and all(type(o) in COMPARE for o in n.ops):
            self._check(n.left)
            for c in n.comparators:
                self._check(c)
        elif isinstance(n, ast.IfExp):
            self._check(n.test), self._check(n.body), self._check(n.orelse)
        elif isinstance(n, (ast.Tuple, ast.List)):
            for e in n.elts:
                self._check(e)
        elif (isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
              and n.func.id in FUNCTIONS and not n.keywords):
            for a in n.args:
                self._check(a)
        else:
            raise ExprError(f"{self.where}: {self.text!r}: {type(n).__name__} is not allowed "
                            f"(arithmetic, comparisons, and/or/not, if-else, in, and "
                            f"{', '.join(sorted(FUNCTIONS))})")

    def __call__(self, env):
        try:
            return self._eval(self.tree, env)
        except ExprError:
            raise
        except Exception as e:  # noqa: BLE001 - a user's expression: say which and why
            raise ExprError(f"{self.where}: {self.text!r}: {type(e).__name__}: {e}") from None

    def _eval(self, n, env):
        if isinstance(n, ast.Constant):
            return n.value
        if isinstance(n, ast.Name):
            if n.id in env:
                return env[n.id]
            if n.id in CONSTANTS:
                return CONSTANTS[n.id]
            raise ExprError(f"{self.where}: {self.text!r}: {n.id} is not known here "
                            f"(known: {', '.join(sorted(env)) or 'nothing'})")
        if isinstance(n, ast.BinOp):
            return BINARY[type(n.op)](self._eval(n.left, env), self._eval(n.right, env))
        if isinstance(n, ast.UnaryOp):
            return UNARY[type(n.op)](self._eval(n.operand, env))
        if isinstance(n, ast.BoolOp):
            if isinstance(n.op, ast.And):
                v = True
                for e in n.values:
                    v = self._eval(e, env)
                    if not v:
                        return v
                return v
            v = False
            for e in n.values:
                v = self._eval(e, env)
                if v:
                    return v
            return v
        if isinstance(n, ast.Compare):
            left = self._eval(n.left, env)
            for op, c in zip(n.ops, n.comparators):
                right = self._eval(c, env)
                if not COMPARE[type(op)](left, right):
                    return False
                left = right
            return True
        if isinstance(n, ast.IfExp):
            return self._eval(n.body if self._eval(n.test, env) else n.orelse, env)
        if isinstance(n, (ast.Tuple, ast.List)):
            return tuple(self._eval(e, env) for e in n.elts)
        return FUNCTIONS[n.func.id](*(self._eval(a, env) for a in n.args))

    def __repr__(self):
        return f"Expr({self.text!r})"
