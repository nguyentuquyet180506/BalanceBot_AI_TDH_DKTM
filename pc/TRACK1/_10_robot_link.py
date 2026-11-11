import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from balancebot_udp import BalanceBotUDP, MAX_V, MAX_W, PING_S   # noqa: E402

import _0_config as cfg   # noqa: E402


def _approach(cur, goal, up, down):
    if cur * goal < 0:
        goal = 0.0
    step = up if abs(goal) > abs(cur) else down
    if cur < goal:
        return min(cur + step, goal)
    return max(cur - step, goal)


def turn_duration(deg, w, accel=None, decel=None):
    a = accel or cfg.ROBOT_YAW_ACCEL
    d = decel or cfg.ROBOT_YAW_DECEL
    w = abs(w)
    deg = abs(deg)
    t = (deg + w * w / (2.0 * a) - w * w / (2.0 * d)) / w
    if t >= w / a:
        return t
    return math.sqrt(deg / (a / 2.0 + a * a / (2.0 * d)))


class RobotLink:
    def __init__(self, ip_provider=None, default_ip=None):
        self.bot = BalanceBotUDP(default_ip or cfg.ROBOT_DEFAULT_IP)
        self.bot.connect(claim=False)
        self._ip_provider = ip_provider
        self._lock = threading.Lock()
        self._target = (0.0, 0.0)
        self._target_t = 0.0
        self._cur = [0.0, 0.0]
        self._action = None
        self._driving = False
        self._last_tx = 0.0
        self._stop_evt = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    # ── API cho main.py ──
    @property
    def claimed(self):
        return self.bot.claimed

    @property
    def telemetry(self):
        return self.bot.telemetry

    @property
    def command(self):
        with self._lock:
            return tuple(self._cur)

    @property
    def busy(self):
        with self._lock:
            return self._action is not None

    def is_still(self, v_thr=120.0):
        t = self.bot.telemetry
        if self.busy or not t.get("connected"):
            return False
        return (not t.get("moving")) and abs(float(t.get("v", 0.0))) < v_thr

    def claim(self):
        with self._lock:
            self._cur = [0.0, 0.0]
            self._target = (0.0, 0.0)
            self._action = None
            self._driving = False
        self._refresh_ip()
        self.bot.claim()

    def release(self):
        with self._lock:
            self._cur = [0.0, 0.0]
            self._target = (0.0, 0.0)
            self._action = None
            self._driving = False
        self.bot.release()

    def set_target(self, v, w):
        v = max(-MAX_V, min(MAX_V, float(v)))
        w = max(-MAX_W, min(MAX_W, float(w)))
        with self._lock:
            self._target = (v, w)
            self._target_t = time.monotonic()
            if self._action is not None and (v != 0 or w != 0):
                self._action = None

    def stop(self):
        self.set_target(0, 0)

    def run_for(self, v, w, seconds):
        v = max(-MAX_V, min(MAX_V, float(v)))
        w = max(-MAX_W, min(MAX_W, float(w)))
        with self._lock:
            self._cur = [0.0, 0.0]
            self._action = (v, w, time.monotonic() + max(0.0, float(seconds)))

    def turn_by(self, deg, w_abs, direction):
        w = max(1.0, abs(w_abs)) * (1 if direction >= 0 else -1)
        self.run_for(0.0, w, turn_duration(deg, w))

    def cancel_action(self):
        with self._lock:
            self._action = None

    def arm(self):
        self.bot.arm()

    def disarm(self):
        self.bot.disarm()

    def close(self):
        self._stop_evt.set()
        self._thread.join(timeout=0.5)
        self.bot.close()

    # ── Luồng gửi lệnh 50 Hz ──
    def _refresh_ip(self):
        if self._ip_provider is None:
            return
        try:
            ip = self._ip_provider()
        except Exception:
            ip = None
        if ip:
            self.bot.set_ip(ip)

    def _loop(self):
        period = 1.0 / cfg.ROBOT_SEND_HZ
        nxt = time.monotonic()
        last_ip_check = 0.0
        while not self._stop_evt.is_set():
            now = time.monotonic()
            if now - last_ip_check > 1.0:
                last_ip_check = now
                self._refresh_ip()
            try:
                self._tick(now, period)
            except Exception as e:
                print(f"[ROBOT] Loi gui lenh: {e}")
            nxt += period
            delay = nxt - time.monotonic()
            if delay < -period:
                nxt = time.monotonic()
                delay = 0
            self._stop_evt.wait(max(0.0, delay))

    def _tick(self, now, dt):
        if not self.bot.claimed:
            return
        with self._lock:
            act = self._action
            if act is not None and now >= act[2]:
                self._action = act = None
                self._cur = [0.0, 0.0]
                self._driving = True
            if act is None:
                tv, tw = self._target if now - self._target_t <= cfg.ROBOT_CMD_TIMEOUT else (0.0, 0.0)
                self._cur[0] = _approach(self._cur[0], tv, cfg.STICK_RATE_UP * MAX_V * dt, cfg.STICK_RATE_DOWN * MAX_V * dt)
                self._cur[1] = _approach(self._cur[1], tw, cfg.STICK_RATE_UP * MAX_W * dt, cfg.STICK_RATE_DOWN * MAX_W * dt)
                v, w = self._cur
            else:
                v, w = act[0], act[1]
        if round(v) != 0 or round(w) != 0:
            self.bot.drive(v, w)
            self._driving = True
            self._last_tx = now
        elif self._driving:
            self.bot.stop()
            self.bot.stop()
            self._driving = False
            self._last_tx = now
        elif now - self._last_tx >= PING_S:
            self.bot.ping()
            self._last_tx = now
