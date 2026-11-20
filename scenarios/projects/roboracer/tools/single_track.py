"""Roadmap 091 W8: a steady-state single-track model, for M5 (steady circles) and M6 (grip limit), with a
tyre given as F_y / F_z against slip angle per axle. No load transfer (ForzaETH neglect it too)."""
import math
import numpy as np
from scipy.optimize import fsolve

G = 9.81


def pacejka(B, C, D, E, mu):
    def f(a):
        x = B * a
        return mu * D * math.sin(C * math.atan(x - E * (x - math.atan(x))))
    return f


def table(points):
    xs, ys = zip(*points)
    def f(a):
        s = 1 if a >= 0 else -1
        return s * float(np.interp(abs(a), xs, ys))
    return f


def steady(car, tf, tr, delta, v, guess=None):
    m, lf, lr = car["m"], car["lf"], car["lr"]
    L = lf + lr
    fzf, fzr = m * G * lr / L, m * G * lf / L
    def eq(x):
        beta, r = x
        af = delta - math.atan2(v * math.sin(beta) + lf * r, v * math.cos(beta))
        ar = -math.atan2(v * math.sin(beta) - lr * r, v * math.cos(beta))
        Ff, Fr = tf(af) * fzf, tr(ar) * fzr
        return [Ff * math.cos(delta) + Fr - m * v * r, lf * Ff * math.cos(delta) - lr * Fr]
    b0 = math.atan(lr * math.tan(delta) / L)  # the kinematic state: no slip angles
    r0 = v * math.cos(b0) * math.tan(delta) / L
    sol, info, ok, msg = fsolve(eq, guess or [b0, r0], full_output=True)
    if ok != 1 or abs(sol[1]) < 1e-9 or max(abs(c) for c in eq(sol)) > 1e-6:
        return None
    beta, r = sol
    return {"radius": v / r, "ay": v * r, "beta": beta, "x": [float(beta), float(r)]}


def m6(car, tf, tr, delta, vmax=10.0):
    """The highest steady lateral acceleration at a tire angle as the speed rises: the last
    steady solution, continued from the one before, before none exists (the slide)."""
    best, guess = None, None
    for v in np.arange(0.3, vmax, 0.01):
        s = steady(car, tf, tr, delta, float(v), guess)
        if s is None or s["ay"] <= 0:
            if best is None:
                continue
            break
        guess = [s["x"][0], s["x"][1] * (v + 0.01) / v]
        if best is None or s["ay"] > best["ay"]:
            best = dict(s, v=float(v))
    return best


def steady_cont(car, tf, tr, delta, v_target, dv=0.01):
    """The steady state at v_target on the branch that starts from the kinematic one at low
    speed, continued in speed; None when that branch ends first (the car slides)."""
    guess, s = None, None
    v = 0.1
    while v <= v_target + 1e-9:
        s = steady(car, tf, tr, delta, v, guess)
        if s is None:
            return None
        guess = [s["x"][0], s["x"][1] * (v + dv) / v]
        if v == v_target:
            break
        v = min(v + dv, v_target)
    return s
