"""Тест алгоритму наведення на спрощеній моделі (без ROS/Gazebo).

Модель: швидкість дрона наздоганяє команду з лагом (перший порядок),
крок керування 0.2 с, як у вузлі. Перевіряє: досягнення цілі варіанта 9,
відсутність прольоту повз ціль, спрацювання захисту при хибному знаку.
"""
import math

from flight_test.navigator import TargetNavigator


def simulate(x0, y0, tx, ty, sign=1.0, tau=0.6, dt=0.2, t_max=120.0,
             noise=0.0):
    nav = TargetNavigator(tx, ty)
    x, y, vx, vy = x0, y0, 0.0, 0.0
    t = 0.0
    min_dist = math.hypot(tx - x, ty - y)
    max_overshoot = 0.0
    import random
    rnd = random.Random(1)
    while t < t_max:
        mx = x + rnd.gauss(0, noise)
        my = y + rnd.gauss(0, noise)
        cmd = nav.update(mx, my, t)
        if cmd.reached:
            return dict(ok=True, t=t, x=x, y=y,
                        err=math.hypot(x - tx, y - ty),
                        max_overshoot=max_overshoot)
        if cmd.diverging:
            return dict(ok=False, diverged=True, t=t,
                        err=math.hypot(x - tx, y - ty))
        cx, cy = sign * cmd.vx, sign * cmd.vy
        a = dt / (tau + dt)
        vx += a * (cx - vx)
        vy += a * (cy - vy)
        x += vx * dt
        y += vy * dt
        t += dt
        d = math.hypot(tx - x, ty - y)
        if d < min_dist:
            min_dist = d
        max_overshoot = max(max_overshoot, d - min_dist)
    return dict(ok=False, diverged=False, t=t,
                err=math.hypot(x - tx, y - ty))


def test_variant9_reaches_target():
    x0, y0 = 0.0, 0.0
    tx, ty = x0 - 5.2, y0 - (-0.1)  # варіант 9
    r = simulate(x0, y0, tx, ty)
    assert r["ok"], r
    assert r["err"] <= 0.2, r
    assert r["max_overshoot"] < 0.2, r


def test_reaches_with_sensor_noise():
    r = simulate(0.0, 0.0, -5.2, 0.1, noise=0.03)
    assert r["ok"], r
    assert r["err"] <= 0.3, r


def test_all_variants_distance_at_least_4m():
    # зміщення з таблиці 2.3, перші 10 варіантів
    offsets = [(7.0, 1.4), (6.4, -0.4), (6.4, -0.1), (7.6, 1.1),
               (7.0, -0.1), (8.2, 0.2), (5.8, 0.8), (4.6, 1.4),
               (5.2, -0.1), (4.6, 0.8)]
    for dx, dy in offsets:
        r = simulate(0.0, 0.0, -dx, -dy)
        assert r["ok"] and r["err"] <= 0.25, (dx, dy, r)


def test_wrong_sign_triggers_divergence_guard():
    r = simulate(0.0, 0.0, -5.2, 0.1, sign=-1.0)
    assert r.get("diverged"), r


if __name__ == "__main__":
    import sys
    import pytest
    sys.exit(pytest.main([__file__, "-v"]))
