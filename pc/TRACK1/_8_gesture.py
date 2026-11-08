import math
from collections import deque

import numpy as np

import _0_config as cfg

NOSE, LS, RS, LE, RE, LW, RW, LH, RH = 0, 5, 6, 7, 8, 9, 10, 11, 12

REST, DOWN, SIDE, UP, MID, UNK = "rest", "down", "side", "up", "mid", "unk"


def _pt(kpts, idx, conf):
    if idx >= len(kpts):
        return None
    x, y, c = float(kpts[idx][0]), float(kpts[idx][1]), float(kpts[idx][2])
    return (x, y) if c >= conf else None


def _arm_state(kpts, idxs, outward, sw, conf):
    sh_i, el_i, wr_i = idxs
    sh = _pt(kpts, sh_i, conf)
    if sh is None:
        return UNK, None

    wr = _pt(kpts, wr_i, conf)
    el = _pt(kpts, el_i, conf)
    if wr is not None:
        end, min_len = wr, cfg.GESTURE_ARM_MIN_LEN_WRIST * sw
    elif el is not None:
        end, min_len = el, cfg.GESTURE_ARM_MIN_LEN_ELBOW * sw
    else:
        return UNK, None

    dx, dy = end[0] - sh[0], sh[1] - end[1]
    out = outward * dx
    if end is wr and dy >= cfg.GESTURE_RAISED_DY * sw:
        return UP, math.degrees(math.atan2(dy, out))
    if math.hypot(out, dy) < min_len:
        return REST, None

    ang = math.degrees(math.atan2(dy, out))
    if cfg.GESTURE_UP_MIN <= ang <= cfg.GESTURE_UP_MAX:
        return UP, ang
    if -cfg.GESTURE_SIDE_TOL <= ang <= cfg.GESTURE_SIDE_TOL:
        return SIDE, ang
    if ang < cfg.GESTURE_DOWN_MAX:
        return DOWN, ang
    return MID, ang


def _arms_by_image(kpts, conf):
    rs, ls = _pt(kpts, RS, conf), _pt(kpts, LS, conf)
    if rs is None or ls is None:
        return None
    r, l = (RS, RE, RW), (LS, LE, LW)
    return (r, l) if rs[0] <= ls[0] else (l, r)


def _crossed_arms(kpts, sw, conf, img_left, img_right):
    rs, ls = _pt(kpts, RS, conf), _pt(kpts, LS, conf)
    wl, wr = _pt(kpts, img_left[2], conf), _pt(kpts, img_right[2], conf)
    if None in (rs, ls, wl, wr):
        return False
    if wl[0] <= wr[0]:
        return False
    y_top = min(rs[1], ls[1]) - 0.2 * sw
    y_bot = max(rs[1], ls[1]) + cfg.GESTURE_CHEST_DEPTH * sw
    return y_top <= wl[1] <= y_bot and y_top <= wr[1] <= y_bot


def classify_raw(kpts, facing=None, profile=False):
    conf = cfg.GESTURE_KPT_CONF
    facing = facing or "front"
    rs, ls = _pt(kpts, RS, conf), _pt(kpts, LS, conf)
    if rs is None or ls is None:
        return "DUNG", {"why": "thieu vai"}
    sw = math.hypot(rs[0] - ls[0], rs[1] - ls[1])
    if sw < cfg.GESTURE_MIN_SHOULDER_PX:
        return "DUNG", {"why": "nguoi qua xa/nghieng"}
    if abs(rs[1] - ls[1]) > cfg.GESTURE_SHOULDER_TILT * sw:
        return "DUNG", {"why": "vai nghieng"}
    if profile:
        return "DUNG", {"why": "quay ngang"}

    img_left, img_right = _arms_by_image(kpts, conf)
    if _crossed_arms(kpts, sw, conf, img_left, img_right):
        return "DUNG TAI VI TRI", {"why": "tay cheo"}
    if getattr(cfg, "GESTURE_NEED_FACING", True) and facing not in ("front", "back"):
        return "DUNG", {"why": "huong chua ro"}

    if facing == "back":
        r_idx, l_idx, r_out = img_right, img_left, +1
    else:
        r_idx, l_idx, r_out = img_left, img_right, -1
    r_state, r_ang = _arm_state(kpts, r_idx, r_out, sw, conf)
    l_state, l_ang = _arm_state(kpts, l_idx, -r_out, sw, conf)
    dbg = {"R": (r_state, r_ang), "L": (l_state, l_ang), "facing": facing}

    idle = (REST, DOWN, UNK)
    if r_state == UP and l_state in idle:
        return "TIEN", dbg
    if l_state == UP and r_state in idle:
        return "LUI", dbg
    if r_state == SIDE and l_state in idle:
        return "DI SANG PHAI", dbg
    if l_state == SIDE and r_state in idle:
        return "DI SANG TRAI", dbg
    return "DUNG", dbg


class KeypointSmoother:
    IDX = (LS, RS, LE, RE, LW, RW)

    def __init__(self):
        self.last = {}
        self.age = {}

    def reset(self):
        self.last.clear()
        self.age.clear()

    def update(self, kpts):
        out = np.array(kpts, dtype=np.float64, copy=True)
        for i in self.IDX:
            if i >= len(out):
                continue
            x, y, c = out[i]
            prev = self.last.get(i)
            if c >= cfg.GESTURE_KPT_CONF:
                if prev is not None and self.age.get(i, 0) <= cfg.KPT_HOLD_RUNS:
                    a = cfg.KPT_SMOOTH_ALPHA
                    x, y = a * x + (1 - a) * prev[0], a * y + (1 - a) * prev[1]
                    out[i, 0], out[i, 1] = x, y
                self.last[i] = (x, y, c)
                self.age[i] = 0
            else:
                self.age[i] = self.age.get(i, 0) + 1
                if prev is not None and self.age[i] <= cfg.KPT_HOLD_RUNS:
                    out[i] = (prev[0], prev[1], max(c, cfg.GESTURE_KPT_CONF))
        return out


class GestureClassifier:

    def __init__(self):
        self.votes = deque(maxlen=cfg.GESTURE_VOTE_WINDOW)
        self.stable = "DUNG"
        self.last_raw = "DUNG"
        self.last_dbg = {}

    def reset(self):
        self.votes.clear()
        self.stable = "DUNG"
        self.last_raw = "DUNG"
        self.last_dbg = {}

    def update(self, kpts, facing=None, profile=False):
        raw, dbg = classify_raw(kpts, facing=facing, profile=profile)
        self.last_raw, self.last_dbg = raw, dbg
        self.votes.append(raw)

        need = cfg.GESTURE_VOTE_NEEDED
        keep = max(1, need - 1)
        count = {}
        for v in self.votes:
            count[v] = count.get(v, 0) + 1

        if self.stable != "DUNG" and count.get(self.stable, 0) >= keep and raw != "DUNG":
            return self.stable
        best = max((k for k in count if k != "DUNG"), key=lambda k: count[k], default=None)
        if best is not None and count[best] >= need:
            self.stable = best
        else:
            self.stable = "DUNG"
        return self.stable