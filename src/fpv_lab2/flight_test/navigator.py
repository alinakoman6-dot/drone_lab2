"""Алгоритм наведення на цільову точку (без залежностей від ROS).

Пропорційний регулятор по відстані з обмеженням максимальної швидкості:
    v = min(v_max, kp * d),  напрямок - одиничний вектор на ціль.
Біля цілі швидкість лінійно спадає до нуля, тому дрон не "пролітає" ціль.
Ціль вважається досягнутою, коли відстань < tol протягом hold_time секунд.
"""
import math
from dataclasses import dataclass


@dataclass
class NavigationCommand:
    vx: float
    vy: float
    distance: float
    reached: bool
    diverging: bool


class TargetNavigator:
    def __init__(
        self,
        target_x: float,
        target_y: float,
        v_max: float = 1.0,
        kp: float = 0.6,
        tolerance: float = 0.2,
        hold_time: float = 1.0,
        v_min: float = 0.08,
        diverge_margin: float = 0.7,
        diverge_time: float = 4.0,
    ):
        self.target_x = target_x
        self.target_y = target_y
        self.v_max = v_max
        self.kp = kp
        self.tolerance = tolerance
        self.hold_time = hold_time
        self.v_min = v_min
        self.diverge_margin = diverge_margin
        self.diverge_time = diverge_time

        self._inside_since = None
        self._best_distance = None
        self._best_distance_time = None

    @staticmethod
    def distance_between(x1, y1, x2, y2) -> float:
        return math.hypot(x1 - x2, y1 - y2)

    def update(self, x: float, y: float, now: float) -> NavigationCommand:
        dx = self.target_x - x
        dy = self.target_y - y
        distance = math.hypot(dx, dy)

        # --- перевірка досягнення цілі (утримання в зоні допуску) ---
        if distance < self.tolerance:
            if self._inside_since is None:
                self._inside_since = now
        else:
            self._inside_since = None
        reached = (
            self._inside_since is not None
            and now - self._inside_since >= self.hold_time
        )

        # --- захист: відстань зростає замість зменшення (невірна система
        # координат/знак) ---
        if self._best_distance is None or distance < self._best_distance:
            self._best_distance = distance
            self._best_distance_time = now
        diverging = (
            distance > self._best_distance + self.diverge_margin
            and now - self._best_distance_time > self.diverge_time
        )

        # --- команда швидкості ---
        if distance < 1e-6:
            return NavigationCommand(0.0, 0.0, distance, reached, diverging)

        speed = min(self.v_max, self.kp * distance)
        if distance > self.tolerance / 2:
            speed = max(speed, self.v_min)  # щоб не зависнути за мілиною
        else:
            speed = 0.0
        vx = speed * dx / distance
        vy = speed * dy / distance
        return NavigationCommand(vx, vy, distance, reached, diverging)
