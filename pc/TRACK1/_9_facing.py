import math

import _0_config as cfg

NOSE, LEYE, REYE, LEAR, REAR, LS, RS, LH, RH = 0, 1, 2, 3, 4, 5, 6, 11, 12

FRONT, BACK, SIDE_R, SIDE_L, UNK = "front", "back", "side_r", "side_l", "unk"

CONCLUSIVE = (FRONT, BACK, SIDE_R, SIDE_L) if cfg.SLEEVE_MODE else (FRONT, BACK, SIDE_R)

VIEW_TEXT = {
    FRONT: "MAT",
    BACK: "LUNG",
    SIDE_R: "NGHIENG (tay phai ve cam)",
    SIDE_L: "NGHIENG (tay phai bi che)",
    UNK: "CHUA RO",
}


def _c(kpts, idx):
    return float(kpts[idx][2]) if idx < len(kpts) else 0.0


def raw_view(kpts, vx=0.0):
    if kpts is None or len(kpts) < 13:
        return UNK, {"why": "thieu keypoint"}

    kc = cfg.FACING_KPT_CONF
    if _c(kpts, LS) < kc or _c(kpts, RS) < kc:
        return UNK, {"why": "thieu vai"}

    ls, rs = kpts[LS], kpts[RS]
    sw = math.hypot(ls[0] - rs[0], ls[1] - rs[1])
    mx, my = (ls[0] + rs[0]) / 2.0, (ls[1] + rs[1]) / 2.0

    hips = [kpts[i] for i in (LH, RH) if _c(kpts, i) >= kc]
    torso = None
    if hips:
        t = sum(p[1] for p in hips) / len(hips) - my
        if t >= 15:
            torso = t
    ratio = sw / torso if torso else None

    nose_c, le_c, re_c = _c(kpts, NOSE), _c(kpts, LEYE), _c(kpts, REYE)
    dbg = {"ratio": ratio, "nose": nose_c, "eyes": (le_c, re_c)}

    if cfg.FACING_USE_SHOULDERS and ratio is not None and ratio < cfg.FACING_PROFILE_RATIO:
        d = 0
        if nose_c >= kc:
            off = (kpts[NOSE][0] - mx) / torso
            if abs(off) > cfg.FACING_NOSE_OFFSET:
                d = 1 if off > 0 else -1
        if d == 0 and abs(vx) >= cfg.FACING_VX_THRESH:
            d = 1 if vx > 0 else -1
        if d == 0:
            dbg["why"] = "nghieng, khong ro mat quay ben nao"
            return UNK, dbg
        return (SIDE_R if d > 0 else SIDE_L), dbg

    if nose_c >= cfg.FACING_FACE_FRONT_CONF and max(le_c, re_c) >= cfg.FACING_FACE_FRONT_CONF - 0.05:
        return FRONT, dbg
    if max(nose_c, le_c, re_c) < cfg.FACING_FACE_BACK_CONF:
        return BACK, dbg
    dbg["why"] = "mat mo, khong chac"
    return UNK, dbg


def view_with_motion(view, motion_dir):
    if view == UNK and motion_dir != 0:
        return SIDE_R if motion_dir > 0 else SIDE_L
    return view


def arms_for(view):
    if cfg.SLEEVE_MODE:
        return "both"
    if view == FRONT and cfg.ARMBAND_SINGLE_WHEN_FRONT:
        return "R"
    return "both" if cfg.ARMBAND_EITHER_ARM else "R"


class FacingEstimator:

    def __init__(self):
        self.reset()

    def reset(self):
        self.stable = FRONT
        self._cand = None
        self._n = 0
        self.last_raw = UNK
        self.last_dbg = {}

    def update(self, kpts, vx=0.0, approach=0.0, motion_dir=0):
        raw, dbg = raw_view(kpts, vx)
        raw = view_with_motion(raw, motion_dir)
        self.last_raw, self.last_dbg = raw, dbg

        if raw in (SIDE_R, SIDE_L):
            cand = raw
        else:
            mot = FRONT if approach >= cfg.FACING_DEPTH_RATE else (BACK if approach <= -cfg.FACING_DEPTH_RATE else None)
            face = raw if raw in (FRONT, BACK) else None
            if mot and face and mot != face:
                cand = UNK
            else:
                cand = mot or face
            if cand is None and self.stable in (SIDE_R, SIDE_L):
                cand = UNK

        if cand is None or cand == self.stable:
            self._cand, self._n = None, 0
            return self.stable
        self._n = self._n + 1 if cand == self._cand else 1
        self._cand = cand
        if self._n >= cfg.FACING_CONFIRM:
            self.stable, self._cand, self._n = cand, None, 0
        return self.stable