import math
from collections import deque
import time
import numpy as np
import _0_config as cfg

class TrajectoryPredictor:
    def __init__(self, meas_std=None, acc_std=None):
        meas_std = cfg.KAL_MEAS_STD if meas_std is None else meas_std
        acc_std = cfg.KAL_ACC_STD if acc_std is None else acc_std

        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]], dtype=np.float64)
        self.R = np.eye(2, dtype=np.float64) * meas_std ** 2
        self._acc_var = acc_std ** 2

        self.max_dt = 0.30
        self.coast_after = 0.2
        self.coast_tau = 0.8
        self.max_coast_time = cfg.GATE_MAX_COAST

        self._h_win = deque(maxlen=14)
        self.reset()

    def reset(self):
        self._initialized = False
        self._last_time = None
        self._last_update_time = None
        self._upd_dt = None
        self._h_win.clear()
        self._h_kind = None
        self.state = np.zeros((4, 1), dtype=np.float64)
        self.P = np.diag([100.0, 100.0, 1e4, 1e4])

    # ── nội bộ ──
    def _init_from(self, m):
        self.state = np.array([[m[0]], [m[1]], [0.0], [0.0]], dtype=np.float64)
        self.P = np.diag([100.0, 100.0, 1e4, 1e4])
        self._initialized = True
        self._last_time = self._last_update_time = time.monotonic()

    def _Q(self, dt):
        q = self._acc_var
        a, b, c = dt ** 4 / 4.0, dt ** 3 / 2.0, dt ** 2
        return q * np.array([[a, 0, b, 0],
                             [0, a, 0, b],
                             [b, 0, c, 0],
                             [0, b, 0, c]], dtype=np.float64)

    # ── API ──
    def predict(self):
        if not self._initialized:
            return None
        now = time.monotonic()
        dt = min(max(now - self._last_time, 0.001), self.max_dt)
        self._last_time = now

        since = now - self._last_update_time
        if since > self.max_coast_time:
            return None

        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]], dtype=np.float64)
        coast_after = self.coast_after if self._upd_dt is None else min(max(self.coast_after, 2.5 * self._upd_dt), 1.0)
        if since > coast_after:
            damp = math.exp(-dt / self.coast_tau)
            F[2, 2] = F[3, 3] = damp

        self.state = F @ self.state
        self.P = F @ self.P @ F.T + self._Q(dt)
        return int(self.state[0, 0]), int(self.state[1, 0])

    def update(self, measurement, height=None, kind='h'):
        now = time.monotonic()
        if height is not None and height > 1:
            if kind != self._h_kind:
                self._h_win.clear()
                self._h_kind = kind
            self._h_win.append((now, math.log(float(height))))
        if (not self._initialized
                or now - self._last_update_time > self.max_coast_time):
            self._init_from(measurement)
            return

        gap = now - self._last_update_time
        self._upd_dt = gap if self._upd_dt is None else 0.8 * self._upd_dt + 0.2 * gap
        z = np.array(measurement, dtype=np.float64).reshape(2, 1)
        y = z - self.H @ self.state
        S = self.H @ self.P @ self.H.T + self.R
        K = np.linalg.solve(S, self.H @ self.P).T
        self.state = self.state + K @ y
        I_KH = np.eye(4) - K @ self.H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T
        self._last_update_time = now

    def position(self):
        if not self._initialized:
            return None
        return float(self.state[0, 0]), float(self.state[1, 0])

    def coast_time(self):
        if not self._initialized:
            return 0.0
        return time.monotonic() - self._last_update_time

    def approach_rate(self):
        now = time.monotonic()
        pts = [(t, v) for t, v in self._h_win if now - t <= 1.2]
        if len(pts) < 4 or pts[-1][0] - pts[0][0] < 0.4:
            return 0.0
        ts = np.array([t for t, _ in pts], dtype=np.float64)
        vs = np.array([v for _, v in pts], dtype=np.float64)
        ts -= ts.mean()
        den = float((ts ** 2).sum())
        return float((ts * (vs - vs.mean())).sum() / den) if den > 0 else 0.0

    def last_height(self):
        return math.exp(self._h_win[-1][1]) if self._h_win else None

    def motion_state(self, box_h):
        if not self._initialized or self.coast_time() > cfg.FACING_MAX_COAST:
            return {"known": False, "sideways": False, "dir": 0, "approach": 0.0}
        vx, vy = self.velocity
        bh = max(float(box_h), 1.0)
        side = abs(vx) / bh >= cfg.FACING_SIDE_SPEED and abs(vx) >= cfg.FACING_SIDE_RATIO * abs(vy)
        return {"known": True, "sideways": bool(side), "dir": (1 if vx > 0 else -1) if side else 0,
                "approach": self.approach_rate()}

    @property
    def velocity(self):
        return float(self.state[2, 0]), float(self.state[3, 0])

# ── Hỗ trợ chống tráo người ───────────────────────────────────
def box_overlap_ratio(a, b):
    ix = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    smaller = min((a[2] - a[0]) * (a[3] - a[1]), (b[2] - b[0]) * (b[3] - b[1]))
    return (ix * iy) / smaller if smaller > 0 else 0.0

def gate_radius(box_h, coast_time):
    grow = min(max(coast_time, 0.0), cfg.GATE_MAX_COAST)
    return box_h * (cfg.GATE_BASE + cfg.GATE_GROW * grow)