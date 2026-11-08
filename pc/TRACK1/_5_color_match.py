import cv2
import numpy as np

from _0_config import (
    SHIRT_MIN_CORR, PANTS_MIN_CORR, ARMBAND_KPT_CONF,
    ARMBAND_ROI_PAD, ARMBAND_MIN_SAT, ARMBAND_MIN_VAL,
    ARMBAND_MASTER_BIN_FRAC, ARMBAND_MIN_AREA, ARMBAND_FULL_AREA,
    ARMBAND_REF_FRAC, ARMBAND_SCAN_MIN_AREA,
    ARMBAND_V_TOL_BINS, ARMBAND_SIDE_CHEST_ROI,
    FEATURE_KPT_CONF, UNMEASURABLE_SCORE,
)

HIST_BINS = [8, 4, 4]
HIST_RANGES = [0, 180, 0, 256, 0, 256]

_NONE3 = (None, None, None)

def _calc_hsv_hist(bgr_roi):
    if bgr_roi is None or bgr_roi.shape[0] < 8 or bgr_roi.shape[1] < 8:
        return None
    hsv = cv2.cvtColor(bgr_roi, cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1, 2], None, HIST_BINS, HIST_RANGES)
    return cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)

def _finish(frame, x1, y1, x2, y2, min_side=10):
    img_h, img_w = frame.shape[:2]
    x1, x2 = max(0, x1), min(img_w, x2)
    y1, y2 = max(0, y1), min(img_h, y2)
    if (x2 - x1) < min_side or (y2 - y1) < min_side:
        return _NONE3
    roi = frame[y1:y2, x1:x2]
    hist = _calc_hsv_hist(roi)
    if hist is None:
        return _NONE3
    return hist, roi, cv2.mean(roi)[:3]

# ── Trích đặc trưng ───────────────────────────────────────────
def extract_shirt_color(frame, kpts):
    ls, rs, lh, rh = kpts[5], kpts[6], kpts[11], kpts[12]
    sh = [p for p in (ls, rs) if p[2] >= FEATURE_KPT_CONF]
    hp = [p for p in (lh, rh) if p[2] >= FEATURE_KPT_CONF]
    if not sh or not hp:
        return _NONE3

    ymin = int(min(p[1] for p in sh))
    ymax = int(max(p[1] for p in hp))
    xs = [p[0] for p in sh + hp]
    xmin, xmax = int(min(xs)), int(max(xs))
    torso_h, torso_w = ymax - ymin, max(xmax - xmin, 10)
    if torso_h < 15:
        return _NONE3
    if torso_w < 0.45 * torso_h:
        cx = (xmin + xmax) / 2.0
        torso_w = 0.45 * torso_h
        xmin, xmax = int(cx - torso_w / 2), int(cx + torso_w / 2)

    return _finish(frame,
                   int(xmin + torso_w * 0.25), int(ymin + torso_h * 0.25),
                   int(xmax - torso_w * 0.25), int(ymax - torso_h * 0.25))

def extract_pants_color(frame, kpts):
    lh, rh, lk, rk = kpts[11], kpts[12], kpts[13], kpts[14]
    hp = [p for p in (lh, rh) if p[2] >= FEATURE_KPT_CONF]
    kn = [p for p in (lk, rk) if p[2] >= FEATURE_KPT_CONF]
    if not hp or not kn:
        return _NONE3

    ymin = int(max(p[1] for p in hp))
    ymax = int(max(p[1] for p in kn))
    xs = [p[0] for p in hp + kn]
    xmin, xmax = int(min(xs)), int(max(xs))
    leg_h, leg_w = ymax - ymin, max(xmax - xmin, 10)
    if leg_h < 15:
        return _NONE3
    if leg_w < 0.35 * leg_h:
        cx = (xmin + xmax) / 2.0
        leg_w = 0.35 * leg_h
        xmin, xmax = int(cx - leg_w / 2), int(cx + leg_w / 2)

    return _finish(frame,
                   int(xmin + leg_w * 0.25), int(ymin + leg_h * 0.15),
                   int(xmax - leg_w * 0.25), int(ymin + leg_h * 0.55))

def _strip_box(rs, re, t0, t1, half_w):
    x0, y0 = rs[0] + (re[0] - rs[0]) * t0, rs[1] + (re[1] - rs[1]) * t0
    x1, y1 = rs[0] + (re[0] - rs[0]) * t1, rs[1] + (re[1] - rs[1]) * t1
    return (int(min(x0, x1)) - half_w, int(min(y0, y1)) - half_w // 2,
            int(max(x0, x1)) + half_w, int(max(y0, y1)) + half_w // 2)

def _clamp_box(frame, box):
    img_h, img_w = frame.shape[:2]
    x1, y1, x2, y2 = box
    return max(0, x1), max(0, y1), min(img_w, x2), min(img_h, y2)

def _vivid_hist(hsv):
    mask = cv2.inRange(hsv, (0, ARMBAND_MIN_SAT, ARMBAND_MIN_VAL), (180, 255, 255))
    hist = cv2.calcHist([hsv], [0, 1, 2], mask, HIST_BINS, HIST_RANGES).astype(np.float32)
    return hist, mask

def extract_armband_color(frame, kpts):
    rs, re = kpts[6], kpts[8]
    if rs[2] < ARMBAND_KPT_CONF or re[2] < ARMBAND_KPT_CONF:
        return _NONE3

    arm_len = float(np.hypot(re[0] - rs[0], re[1] - rs[1]))
    if arm_len < 20:
        return _NONE3
    half_w = max(5, int(arm_len * 0.15))

    x1, y1, x2, y2 = _clamp_box(frame, _strip_box(rs, re, 0.55, 0.75, half_w))
    if (x2 - x1) < 6 or (y2 - y1) < 6:
        return _NONE3
    roi = frame[y1:y2, x1:x2]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    band, vivid = _vivid_hist(hsv)
    band /= float(roi.shape[0] * roi.shape[1])

    ref, ref_area = np.zeros(HIST_BINS, np.float32), 0
    for t0, t1 in ((0.12, 0.34), (0.92, 1.10)):
        rx1, ry1, rx2, ry2 = _clamp_box(frame, _strip_box(rs, re, t0, t1, half_w))
        if (rx2 - rx1) < 6 or (ry2 - ry1) < 6:
            continue
        rroi = frame[ry1:ry2, rx1:rx2]
        rh, _ = _vivid_hist(cv2.cvtColor(rroi, cv2.COLOR_BGR2HSV))
        ref += rh
        ref_area += rroi.shape[0] * rroi.shape[1]
    if ref_area:
        ref /= float(ref_area)

    distinct = (band >= 0.01) & (ref < ARMBAND_REF_FRAC * band)
    if float(band[distinct].sum()) < ARMBAND_SCAN_MIN_AREA:
        return _NONE3

    hist = np.where(distinct, band, 0).astype(np.float32)
    hist = cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)
    hi = (hsv[..., 0].astype(np.int32) * HIST_BINS[0]) // 180
    si = (hsv[..., 1].astype(np.int32) * HIST_BINS[1]) // 256
    vi = (hsv[..., 2].astype(np.int32) * HIST_BINS[2]) // 256
    pix = distinct[hi, si, vi] & (vivid > 0)
    mean_bgr = cv2.mean(roi, mask=pix.astype(np.uint8) * 255)[:3] if pix.any() else cv2.mean(roi)[:3]
    return hist, roi, mean_bgr

def _armband_track_one(frame, sh, el):
    if sh[2] < ARMBAND_KPT_CONF or el[2] < ARMBAND_KPT_CONF:
        return None

    arm_len = float(np.hypot(el[0] - sh[0], el[1] - sh[1]))
    if arm_len < 20:
        return None

    x0 = sh[0] + (el[0] - sh[0]) * 0.10
    y0 = sh[1] + (el[1] - sh[1]) * 0.10
    pad = arm_len * ARMBAND_ROI_PAD
    x1, x2 = int(min(x0, el[0]) - pad), int(max(x0, el[0]) + pad)
    y1, y2 = int(min(y0, el[1]) - pad), int(max(y0, el[1]) + pad)

    img_h, img_w = frame.shape[:2]
    x1, x2 = max(0, x1), min(img_w, x2)
    y1, y2 = max(0, y1), min(img_h, y2)
    if (x2 - x1) < 12 or (y2 - y1) < 12:
        return None
    roi = frame[y1:y2, x1:x2]

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, ARMBAND_MIN_SAT, ARMBAND_MIN_VAL), (180, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    hist = cv2.calcHist([hsv], [0, 1, 2], mask, HIST_BINS, HIST_RANGES).astype(np.float32)
    hist /= float(roi.shape[0] * roi.shape[1])
    mean_bgr = cv2.mean(roi, mask=mask)[:3] if mask.any() else cv2.mean(roi)[:3]
    return hist, roi, mean_bgr

def _vivid_roi_hist(frame, x1, y1, x2, y2):
    img_h, img_w = frame.shape[:2]
    x1, x2 = max(0, int(x1)), min(img_w, int(x2))
    y1, y2 = max(0, int(y1)), min(img_h, int(y2))
    if (x2 - x1) < 12 or (y2 - y1) < 12:
        return None
    roi = frame[y1:y2, x1:x2]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, ARMBAND_MIN_SAT, ARMBAND_MIN_VAL), (180, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    hist = cv2.calcHist([hsv], [0, 1, 2], mask, HIST_BINS, HIST_RANGES).astype(np.float32)
    hist /= float(roi.shape[0] * roi.shape[1])
    mean_bgr = cv2.mean(roi, mask=mask)[:3] if mask.any() else cv2.mean(roi)[:3]
    return hist, roi, mean_bgr

def _armband_chest_roi(frame, kpts):
    ls, rs, lh, rh = kpts[5], kpts[6], kpts[11], kpts[12]
    if ls[2] < ARMBAND_KPT_CONF or rs[2] < ARMBAND_KPT_CONF:
        return None
    mx, my = (ls[0] + rs[0]) / 2.0, (ls[1] + rs[1]) / 2.0
    sw = float(np.hypot(ls[0] - rs[0], ls[1] - rs[1]))
    if lh[2] >= 0.3 and rh[2] >= 0.3:
        torso = float(np.hypot(mx - (lh[0] + rh[0]) / 2.0, my - (lh[1] + rh[1]) / 2.0))
    else:
        torso = max(sw * 1.3, 30.0)
    half = 0.4 * torso
    return _vivid_roi_hist(frame, mx - half, my, mx + half, my + 0.8 * torso)

def extract_armband_track(frame, kpts, arms='R', sideways=False):
    pairs = [(kpts[6], kpts[8])]
    if arms == 'both' or sideways:
        pairs.append((kpts[5], kpts[7]))
    found = [r for r in (_armband_track_one(frame, sh, el) for sh, el in pairs) if r is not None]
    if sideways and ARMBAND_SIDE_CHEST_ROI:
        chest = _armband_chest_roi(frame, kpts)
        if chest is not None:
            found.append(chest)
    if not found:
        return _NONE3
    hist = found[0][0]
    for r in found[1:]:
        hist = np.maximum(hist, r[0])
    return hist, found[0][1], found[0][2]

_FEATURE_EXTRACTORS = {
    'shirt': extract_shirt_color,
    'pants': extract_pants_color,
    'armband': extract_armband_color,
}

def extract_all_features(frame, kpts, only=None, track=False, arms='R', sideways=False):
    keys = _FEATURE_EXTRACTORS.keys() if only is None else only
    out = {}
    for k in keys:
        if track and k == 'armband':
            out[k] = extract_armband_track(frame, kpts, arms, sideways)
        else:
            out[k] = _FEATURE_EXTRACTORS[k](frame, kpts)
    return out

# ── So khớp ───────────────────────────────────────────────────
def _corr(master, cand, key):
    m, c = master.get(key), cand.get(key)
    if m is None or c is None:
        return None
    return float(cv2.compareHist(m, c, cv2.HISTCMP_CORREL))

def _armband_presence(master, cand):
    m = master['armband']
    c = cand['armband']
    mx = float(m.max())
    if mx <= 0:
        return None
    sel = m >= ARMBAND_MASTER_BIN_FRAC * mx
    if ARMBAND_V_TOL_BINS > 0 and sel.ndim == 3:
        ext = sel.copy()
        for k in range(1, ARMBAND_V_TOL_BINS + 1):
            ext[:, :, k:] |= sel[:, :, :-k]
            ext[:, :, :-k] |= sel[:, :, k:]
        sel = ext
    return float(c[sel].sum())

def compute_pursuit_score(master_hists, cand_hists, armband_strict=True, relax=1.0):
    arm_c = None
    if master_hists.get('armband') is None:
        arm_state = 'none'
    elif cand_hists.get('armband') is None:
        arm_state = 'unseen'
    else:
        arm_c = _armband_presence(master_hists, cand_hists)
        if arm_c is None:
            arm_state = 'none'
        else:
            arm_state = 'match' if arm_c >= ARMBAND_MIN_AREA else 'mismatch'
            if arm_state == 'mismatch' and not armband_strict:
                arm_state, arm_c = 'unseen', None
            elif arm_c is not None:
                arm_c = min(1.0, arm_c / ARMBAND_FULL_AREA)

    if arm_state == 'mismatch':
        return -1.0, arm_state

    shirt_c = _corr(master_hists, cand_hists, 'shirt')
    if shirt_c is not None and shirt_c < SHIRT_MIN_CORR * relax:
        return -1.0, arm_state

    pants_c = _corr(master_hists, cand_hists, 'pants')
    if pants_c is not None and pants_c < PANTS_MIN_CORR * relax:
        return -1.0, arm_state

    if shirt_c is None:
        return UNMEASURABLE_SCORE, arm_state

    parts = [(0.5 if pants_c is not None else 0.8, shirt_c)]
    if pants_c is not None:
        parts.append((0.3, pants_c))
    if arm_c is not None:
        parts.append((0.2, arm_c))

    total_w = sum(w for w, _ in parts)
    score = sum(w * c for w, c in parts) / total_w
    return min(max(score, 0.0), 1.0), arm_state

# ── Tư thế đẹp để quét màu (giữ nguyên logic cũ) ──────────────
def is_good_pose_for_scan(kpts, feature_type='shirt'):
    if kpts is None or len(kpts) < 13:
        return False
    CONF = 0.40
    ls, rs, lh, rh = kpts[5], kpts[6], kpts[11], kpts[12]
    if not all(p[2] > CONF for p in (ls, rs, lh, rh)):
        return False

    torso_h = max(abs(lh[1] - ls[1]), abs(rh[1] - rs[1]), 1)
    if abs(ls[1] - rs[1]) / torso_h > 0.15:
        return False

    shoulder_w = abs(rs[0] - ls[0])
    hip_cx, shoulder_cx = (lh[0] + rh[0]) / 2.0, (ls[0] + rs[0]) / 2.0
    if shoulder_w > 5 and abs(hip_cx - shoulder_cx) / shoulder_w > 0.25:
        return False
    if torso_h < 40:
        return False

    if feature_type == 'pants':
        lk, rk = kpts[13], kpts[14]
        if not any(p[2] > CONF for p in (lk, rk)):
            return False
        hip_y = max(lh[1], rh[1])
        knee_y = max(lk[1] if lk[2] > CONF else 0, rk[1] if rk[2] > CONF else 0)
        if knee_y - hip_y < 30:
            return False
    elif feature_type == 'armband':
        re = kpts[8]
        if rs[2] < CONF or re[2] < CONF:
            return False
        if np.hypot(re[0] - rs[0], re[1] - rs[1]) < 20:
            return False
    return True