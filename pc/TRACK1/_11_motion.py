import _0_config as cfg
from _9_facing import BACK


def _sign(x):
    return 1.0 if x > 0 else (-1.0 if x < 0 else 0.0)


class FollowController:
    def __init__(self):
        self.reset()

    def reset(self):
        self._ex = None
        self._hr = None
        self._turning = False
        self._moving = False
        self._err = 0.0
        self._allow_back = True
        self.last_side = 0.0
        self.last_v = 0.0
        self.last_hr = 0.0
        self.dbg = ""

    @property
    def settled(self):
        return not self._turning and (not self._moving or (not self._allow_back and self._err <= 0))

    def update(self, anchor_x, box_h, frame_w, frame_h, v_max=None, w_max=None, allow_back=True):
        v_max = cfg.FOLLOW_V_MAX if v_max is None else max(0.0, float(v_max))
        w_max = cfg.FOLLOW_W_MAX if w_max is None else max(0.0, float(w_max))
        v_back = min(cfg.FOLLOW_V_BACK_MAX, v_max) if allow_back else 0.0
        self._allow_back = allow_back
        ex = max(-1.0, min(1.0, (anchor_x - frame_w / 2.0) / (frame_w / 2.0)))
        hr = max(0.0, box_h / float(max(frame_h, 1)))
        a = cfg.FOLLOW_EMA
        self._ex = ex if self._ex is None else a * ex + (1 - a) * self._ex
        self._hr = hr if self._hr is None else a * hr + (1 - a) * self._hr
        ex, hr = self._ex, self._hr
        if abs(ex) > 0.05:
            self.last_side = _sign(ex)

        # ── Xoay: chỉ khi lệch vượt vùng chấp nhận, xoay tới khi về gần tâm (không cần đúng tâm) ──
        db = cfg.FOLLOW_X_DEADBAND
        if self._turning:
            if abs(ex) < db - cfg.FOLLOW_X_HYST:
                self._turning = False
        elif abs(ex) > db:
            self._turning = True
        w = 0.0
        if self._turning:
            base = db - cfg.FOLLOW_X_HYST
            mag = min(1.0, max(0.0, (abs(ex) - base) / (1.0 - base)))
            w = _sign(ex) * cfg.FOLLOW_TURN_SIGN * min(1.0, cfg.FOLLOW_KW * mag) * w_max

        # ── Tiến / lùi theo cỡ người (xa -> box thấp -> tiến; gần -> box cao -> lùi) ──
        err = (cfg.FOLLOW_TARGET_H - hr) / cfg.FOLLOW_TARGET_H
        self._err = err
        hdb = cfg.FOLLOW_H_DEADBAND
        if self._moving:
            if abs(err) < hdb - cfg.FOLLOW_H_HYST:
                self._moving = False
        elif abs(err) > hdb:
            self._moving = True
        v = 0.0
        if self._moving:
            base = hdb - cfg.FOLLOW_H_HYST
            e = _sign(err) * max(0.0, abs(err) - base)
            v = cfg.FOLLOW_KV * e * v_max
            v = max(-v_back, min(v_max, v))
        if hr >= cfg.FOLLOW_TOO_CLOSE:
            v = -v_back
        if v > 0:
            v *= max(0.0, 1.0 - cfg.FOLLOW_TURN_SLOWDOWN * abs(ex))
        self.dbg = f"lech {ex:+.2f} cao {hr:.2f}"
        self.last_v, self.last_hr = v, hr
        return v, w


GESTURE_MOVES = ("TIEN", "LUI", "DI SANG TRAI", "DI SANG PHAI")


class GestureManeuver:

    def __init__(self):
        self.follow = FollowController()
        self.steps = []
        self.i = 0
        self.name = ""
        self._latched = None
        self._t0 = 0.0
        self._ok = 0

    @property
    def active(self):
        return bool(self.steps)

    def abort(self, link):
        if self.steps:
            link.cancel_action()
            link.stop()
        self.steps = []
        self.name = ""

    def _plan(self, cmd, v, facing):
        if cmd == "TIEN":
            return [("drive", v, cfg.GESTURE_FWD_TIME)]
        if cmd == "LUI":
            return [("drive", -v, cfg.GESTURE_FWD_TIME)]
        side = 1 if cmd == "DI SANG TRAI" else -1
        if cfg.GESTURE_SIDE_FOLLOW_POINTING and facing != BACK:
            side = -side
        p = cfg.GESTURE_STEP_PAUSE
        return [("turn", side), ("wait", p), ("drive", v, cfg.GESTURE_SIDE_DRIVE_TIME), ("wait", p),
                ("turn", -side), ("wait", p), ("adjust", cfg.GESTURE_ADJUST_MAX_TIME)]

    def _next(self, now, link, w):
        self.i += 1
        if self.i >= len(self.steps):
            self.steps = []
            link.stop()
            return
        self._t0 = now
        st = self.steps[self.i]
        if st[0] == "turn":
            link.turn_by(cfg.GESTURE_SIDE_TURN_DEG, w, st[1])
        elif st[0] == "drive":
            link.run_for(st[1], 0.0, st[2])
        elif st[0] == "adjust":
            self.follow.reset()
            self._ok = 0

    def update(self, now, cmd, owner, link, v, w, facing=None):
        w = max(5.0, float(w))
        if not self.steps:
            if cmd == "DUNG":
                self._latched = None
            if cmd in GESTURE_MOVES and cmd != self._latched:
                self._latched = cmd
                self.name = cmd
                self.steps = self._plan(cmd, v, facing)
                self.i = -1
                self._next(now, link, w)
            else:
                link.stop()
                return "cho cu chi" + (f" (buong tay de lap lai {self._latched})" if self._latched else "")
        kind = self.steps[self.i][0] if self.steps else ""
        if kind in ("turn", "drive"):
            if not link.busy:
                self._next(now, link, w)
        elif kind == "wait":
            link.stop()
            if now - self._t0 >= self.steps[self.i][1]:
                self._next(now, link, w)
        elif kind == "adjust":
            if owner is None:
                link.stop()
                self._ok = 0
            else:
                vv, ww = self.follow.update(*owner, v_max=v, w_max=w, allow_back=False)
                link.set_target(vv, ww)
                self._ok = self._ok + 1 if self.follow.settled else 0
            if self._ok >= 3 or now - self._t0 >= self.steps[self.i][1]:
                self._next(now, link, w)
        if not self.steps:
            return f"xong {self.name}"
        names = {"turn": f"quay {cfg.GESTURE_SIDE_TURN_DEG} do", "drive": "chay", "wait": "on dinh", "adjust": "tu chinh vi tri"}
        return f"{self.name}: {names[self.steps[self.i][0]]} ({self.i + 1}/{len(self.steps)})"


class SearchPlanner:

    def __init__(self):
        self.reset()

    def reset(self):
        self._phase = "settle"
        self._t = None
        self.turns = 0
        self._looked = {}

    def _look(self, now, people):
        best = None
        for tid in people:
            left = cfg.SEARCH_LOOK_TIME - (now - self._looked.setdefault(tid, now))
            if left > 0 and (best is None or left > best[1]):
                best = (tid, left)
        return best

    def update(self, now, lost_since, side, link, w=None, people=()):
        waited = now - lost_since
        if waited < cfg.SEARCH_START_DELAY:
            return f"mat dau {waited:.1f}s"
        direction = -side * cfg.FOLLOW_TURN_SIGN if side else 1
        dname = "trai" if direction > 0 else "phai"
        looking = self._look(now, people)
        if self._phase == "turn":
            if link.busy:
                return f"xoay tai cho {cfg.SEARCH_TURN_DEG} do ({dname}) lan {self.turns}"
            self._phase, self._t = "settle", now
        look_msg = None if looking is None else f"THAY NGUOI (ID {looking[0]}) - dung yen nhin xac nhan, con {looking[1]:.1f}s"
        if self._phase == "settle":
            if self._t is None:
                self._t = now
            if not link.is_still(cfg.SEARCH_STILL_V) and now - self._t < cfg.SEARCH_STILL_MAX_WAIT:
                return look_msg or "cho xe dung han roi moi xoay tai cho"
            if self.turns == 0:
                self._phase = "go"
            else:
                self._phase, self._t = "pause", now
        if self._phase == "pause":
            left = cfg.SEARCH_PAUSE - (now - self._t)
            if left > 0:
                return look_msg or f"dung yen tim chu {left:.1f}s (da xoay {self.turns} lan x {cfg.SEARCH_TURN_DEG} do)"
            self._phase = "go"
        if look_msg:
            return look_msg
        link.turn_by(cfg.SEARCH_TURN_DEG, max(5.0, w or cfg.SEARCH_TURN_W), direction)
        self.turns += 1
        self._phase, self._t = "turn", now
        return f"xoay tai cho {cfg.SEARCH_TURN_DEG} do ({dname}) lan {self.turns}"
