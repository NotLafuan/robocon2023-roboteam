from dataclasses import dataclass
import time


@dataclass
class Position:
    x: float
    y: float


@dataclass
class PID:
    kp: float
    ki: float
    kd: float
    value: float = 0
    target: float = 0
    _i: float = 0
    error_prev: float = 0
    time_prev: float = time.time()

    @property
    def time_diff(self):
        return time.time() - self.time_prev

    @property
    def error(self):
        return self.target - self.value

    @property
    def p(self):
        return self.kp * self.error

    @property
    def i(self):
        self._i = self._i + self.ki * self.error
        return self._i

    @property
    def d(self):
        return self.kd*((self.error - self.error_prev)/self.time_diff)

    @property
    def total(self):
        pid = self.p + self.i + self.d
        self.error_prev = self.error
        self.time_prev = time.time()
        return pid


def map_value(x: float,  in_min: float,  in_max: float,  out_min: float,  out_max: float):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def clamp(num, min_value, max_value):
    return max(min(num, max_value), min_value)
