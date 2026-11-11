import cv2
import numpy as np

from _0_config import (
    SHIRT_MIN_CORR, PANTS_MIN_CORR, ARMBAND_KPT_CONF,
    ARMBAND_ROI_PAD, ARMBAND_MIN_SAT, ARMBAND_MIN_VAL,
    ARMBAND_MASTER_BIN_FRAC, ARMBAND_MIN_AREA, ARMBAND_FULL_AREA,
    ARMBAND_REF_FRAC, ARMBAND_SCAN_MIN_AREA,
    ARMBAND_SCAN_T0, ARMBAND_SCAN_T1, ARMBAND_SCAN_HALF_W,
    ARMBAND_SHIRT_HUE_TOL, ARMBAND_SHIRT_MIN_SAT, ARMBAND_SHIRT_HUE_FRAC, ARMBAND_HUE_SPREAD,
    ARMBAND_TRACK_SHIRT_EXCLUDE, ARMBAND_S_TOL_BINS, HUE_SHIFT,
    ARMBAND_V_TOL_BINS, ARMBAND_SIDE_CHEST_ROI,
    FEATURE_KPT_CONF, UNMEASURABLE_SCORE,
    SCAN_MIN_TORSO_PX,
    COLOR_SOFT_MATCH, COLOR_S_BRIDGE, COLOR_HUE_SMOOTH, COLOR_V_SMOOTH,
    ARMBAND_SCAN_BOTH_ARMS,
    SLEEVE_MODE, SLEEVE_TORSO_TOP,
)

HIST_BINS = [8, 4, 4]
HIST_RANGES = [0, 180, 0, 256, 0, 256]

_NONE3 = (None, None, None)

def _hsv(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    if HUE_SHIFT:
        hsv[..., 0] = ((hsv[..., 0].astype(np.uint16) + HUE_SHIFT) % 180).astype(np.uint8)
    return hsv

def _calc_hsv_hist(bgr_roi):
    if bgr_roi is None or bgr_roi.shape[0] < 8 or bgr_roi.shape[1] < 8:
        return None
    hsv = _hsv(bgr_roi)
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

def _strip_poly(p0, p1, t0, t1, half_w, offset=0.0):
    dx, dy = float(p1[0] - p0[0]), float(p1[1] - p0[1])
    ln = max(float(np.hypot(dx, dy)), 1e-6)
    nx, ny = -dy / ln, dx / ln
    ax, ay = p0[0] + dx * t0 + nx * offset, p0[1] + dy * t0 + ny * offset
    bx, by = p0[0] + dx * t1 + nx * offset, p0[1] + dy * t1 + ny * offset
    return np.array([[ax + nx * half_w, ay + ny * half_w], [bx + nx * half_w, by + ny * half_w],
                     [bx - nx * half_w, by - ny * half_w], [ax - nx * half_w, ay - ny * half_w]], np.float32)

def _poly_cut(frame, poly):
    img_h, img_w = frame.shape[:2]
    x1, y1 = max(0, int(np.floor(poly[:, 0].min()))), max(0, int(np.floor(poly[:, 1].min())))
    x2, y2 = min(img_w, int(np.ceil(poly[:, 0].max())) + 1), min(img_h, int(np.ceil(poly[:, 1].max())) + 1)
    if (x2 - x1) < 6 or (y2 - y1) < 6:
        return None
    roi = frame[y1:y2, x1:x2]
    pmask = np.zeros(roi.shape[:2], np.uint8)
    cv2.fillConvexPoly(pmask, np.round(poly - np.float32([x1, y1])).astype(np.int32), 255)
    area = int(cv2.countNonZero(pmask))
    if area < 30:
        return None
    return roi, _hsv(roi), pmask, area

def _poly_vivid(frame, poly):
    cut = _poly_cut(frame, poly)
    if cut is None:
        return None
    roi, hsv, pmask, area = cut
    vivid = cv2.inRange(hsv, (0, ARMBAND_MIN_SAT, ARMBAND_MIN_VAL), (180, 255, 255))
    mask = cv2.bitwise_and(vivid, pmask)
    hist = cv2.calcHist([hsv], [0, 1, 2], mask, HIST_BINS, HIST_RANGES).astype(np.float32) / float(area)
    return hist, roi, hsv, mask, area

def _torso_quad(kpts):
    ls, rs, lh, rh = kpts[5], kpts[6], kpts[11], kpts[12]
    hips = [p for p in (lh, rh) if p[2] >= FEATURE_KPT_CONF]
    if ls[2] < FEATURE_KPT_CONF or rs[2] < FEATURE_KPT_CONF or not hips:
        return None
    xs = [ls[0], rs[0]] + [p[0] for p in hips]
    y0, bot = min(ls[1], rs[1]), max(p[1] for p in hips)
    x_lo, x_hi = min(xs), max(xs)
    tw, th = x_hi - x_lo, bot - y0
    if tw <= 8 or th <= 15:
        return None
    top = SLEEVE_TORSO_TOP if SLEEVE_MODE else 0.15
    return np.float32([[x_lo + 0.2 * tw, y0 + top * th], [x_hi - 0.2 * tw, y0 + top * th],
                       [x_hi - 0.2 * tw, bot - 0.1 * th], [x_lo + 0.2 * tw, bot - 0.1 * th]])

def _circ_window(hist180, r):
    pad = np.concatenate([hist180[-r:], hist180, hist180[:r]]) if r > 0 else hist180
    return np.convolve(pad, np.ones(2 * r + 1, np.float32), 'valid')

def _shirt_hue_block(frame, kpts):
    quad = _torso_quad(kpts)
    cut = None if quad is None else _poly_cut(frame, quad)
    if cut is None:
        return None
    _roi, hsv, pmask, area = cut
    colored = (pmask > 0) & (hsv[..., 1] >= ARMBAND_SHIRT_MIN_SAT) & (hsv[..., 2] >= 40)
    hh = np.bincount(hsv[..., 0][colored].astype(np.int32), minlength=180)[:180].astype(np.float32) / float(area)
    shirt = _circ_window(hh, 2) >= ARMBAND_SHIRT_HUE_FRAC
    block = np.zeros(180, bool)
    for hue in np.flatnonzero(shirt):
        block[(hue + np.arange(-ARMBAND_SHIRT_HUE_TOL, ARMBAND_SHIRT_HUE_TOL + 1)) % 180] = True
    return block

def _face_skin_vivid(frame, kpts):
    nose, le, re_ = kpts[0], kpts[1], kpts[2]
    if min(nose[2], le[2], re_[2]) < ARMBAND_KPT_CONF:
        return None
    eye_d = float(np.hypot(le[0] - re_[0], le[1] - re_[1]))
    if eye_d < 4:
        return None
    cx = (nose[0] * 2 + le[0] + re_[0]) / 4.0
    cy = (nose[1] * 2 + le[1] + re_[1]) / 4.0 + 0.25 * eye_d
    r = 0.6 * eye_d
    got = _poly_vivid(frame, np.float32([[cx - r, cy - r], [cx + r, cy - r], [cx + r, cy + r], [cx - r, cy + r]]))
    return None if got is None else got[0]

def _armband_scan_refs(frame, kpts, sh, el, wr, other_sh, arm_len, half_w):
    refs = []
    quad = _torso_quad(kpts)
    r = None if quad is None else _poly_vivid(frame, quad)
    if r is not None:
        refs.append(r[0])
    if wr[2] >= ARMBAND_KPT_CONF and np.hypot(wr[0] - el[0], wr[1] - el[1]) >= 10:
        r = _poly_vivid(frame, _strip_poly(el, wr, 0.25, 0.9, half_w))
    else:
        r = _poly_vivid(frame, _strip_poly(sh, el, 1.10, 1.40, half_w))
    fore = None if r is None else r[0]
    if SLEEVE_MODE:
        if fore is not None:
            fore = fore.copy()
            fore[2:, :, :] = 0
            fore[:, 3, :] = 0
            refs.append(fore)
        skin = _face_skin_vivid(frame, kpts)
        if skin is not None:
            refs.append(skin)
    elif fore is not None:
        refs.append(fore)
    cx = (other_sh[0] + sh[0]) / 2.0
    mx = (sh[0] + el[0]) / 2.0
    dy = float(el[1] - sh[1])
    nx = -dy / max(arm_len, 1e-6)
    side = 1.0 if (nx * (mx - cx)) >= 0 else -1.0
    r = _poly_vivid(frame, _strip_poly(sh, el, 0.1, 0.9, 0.15 * arm_len, side * (half_w + 0.3 * arm_len)))
    if r is not None:
        refs.append(r[0])
    return refs

ARMBAND_DIAG = {'why': '', 'txt': ''}
_ARM_IDX = {'R': (6, 8, 10, 5), 'L': (5, 7, 9, 6)}
_DIAG_RANK = ('mo', 'ngan', 'ngoai', 'ao', 'nhat', 'trung_ao', 'lan', 'nho', 'ok')

def _armband_scan_one(frame, kpts, side, shirt_block=None):
    si, ei, wi, oi = _ARM_IDX[side]
    sh, el = kpts[si], kpts[ei]
    if sh[2] < ARMBAND_KPT_CONF or el[2] < ARMBAND_KPT_CONF:
        return 0, None, 'mo', f"vai/khuyu mo ({min(sh[2], el[2]):.2f} < {ARMBAND_KPT_CONF:.2f})"
    arm_len = float(np.hypot(el[0] - sh[0], el[1] - sh[1]))
    if arm_len < 20:
        return 0, None, 'ngan', f"bap tay qua nho ({arm_len:.0f}px) - lai gan / gio tay ra"
    half_w = max(5.0, arm_len * ARMBAND_SCAN_HALF_W)
    got = _poly_vivid(frame, _strip_poly(sh, el, ARMBAND_SCAN_T0, ARMBAND_SCAN_T1, half_w))
    if got is None:
        return 0, None, 'ngoai', "bap tay ra ngoai khung"
    band, roi, hsv, vivid, area = got
    block = _shirt_hue_block(frame, kpts)
    if block is None:
        block = shirt_block
    if block is None:
        return 0, None, 'ao', "chua thay ro than ao (de biet mau ao)"
    refs = _armband_scan_refs(frame, kpts, sh, el, kpts[wi], kpts[oi], arm_len, half_w)
    ref = np.maximum.reduce(refs) if refs else np.zeros_like(band)
    distinct = (band >= 0.005) & (ref < ARMBAND_REF_FRAC * band)

    hue = hsv[..., 0].astype(np.int32)
    hi = (hue * HIST_BINS[0]) // 180
    si_ = (hsv[..., 1].astype(np.int32) * HIST_BINS[1]) // 256
    vi = (hsv[..., 2].astype(np.int32) * HIST_BINS[2]) // 256
    viv = vivid > 0
    other = viv & ~block[hue]
    cand = other & distinct[hi, si_, vi]
    min_px = ARMBAND_SCAN_MIN_AREA * area
    pct = lambda m: 100.0 * float(m.sum()) / float(area)
    if int(viv.sum()) < min_px:
        s90 = float(np.percentile(hsv[..., 1], 90))
        return 0, None, 'nhat', f"khong thay mau RUC tren bap tay (ruc {pct(viv):.1f}% < {100 * ARMBAND_SCAN_MIN_AREA:.0f}%, S cao nhat ~{s90:.0f}, can >= {ARMBAND_MIN_SAT})"
    if int(other.sum()) < min_px:
        return 0, None, 'trung_ao', f"mau ruc tren bap tay TRUNG / GAN mau ao ({pct(viv):.1f}% ruc, khac ao {pct(other):.1f}%)"
    if int(cand.sum()) < min_px:
        blocked = viv & block[hue]
        if int(blocked.sum()) >= int(other.sum()):
            return 0, None, 'trung_ao', f"mau ruc tren bap tay TRUNG / GAN mau ao ({pct(blocked):.1f}% trung ao)"
        return 0, None, 'lan', (f"chi thay mau ruc giong da / cang tay / nen ({pct(other):.1f}%) - tay ao chua du ruc hoac bi che "
                                f"(S cao nhat ~{float(np.percentile(hsv[..., 1], 90)):.0f}, can >= {ARMBAND_MIN_SAT})")

    hh = np.bincount(hue[cand], minlength=180)[:180].astype(np.float32)
    peak = int(np.argmax(_circ_window(hh, ARMBAND_HUE_SPREAD)))
    dist = np.abs(hue - peak)
    dist = np.minimum(dist, 180 - dist)
    pix = cand & (dist <= ARMBAND_HUE_SPREAD)
    if int(pix.sum()) < min_px:
        return 0, None, 'nho', f"mau tay ao chiem qua it ({pct(pix):.1f}% < {100 * ARMBAND_SCAN_MIN_AREA:.0f}% bap tay) - lai gan hon"

    m8 = pix.astype(np.uint8) * 255
    hist = cv2.calcHist([hsv], [0, 1, 2], m8, HIST_BINS, HIST_RANGES).astype(np.float32)
    hist = cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)
    mean_bgr = cv2.mean(roi, mask=m8)[:3]
    return int(pix.sum()), (hist, roi, mean_bgr), 'ok', f"tay ao {pct(pix):.1f}% bap tay"

def extract_armband_color(frame, kpts, shirt_block=None):
    sides = ('R', 'L') if (ARMBAND_SCAN_BOTH_ARMS or SLEEVE_MODE) else ('R',)
    res = {sd: _armband_scan_one(frame, kpts, sd, shirt_block) for sd in sides}
    good = [sd for sd in sides if res[sd][1] is not None]
    if good:
        sd = max(good, key=lambda x: res[x][0])
    else:
        sd = max(sides, key=lambda x: _DIAG_RANK.index(res[x][2]))
    ARMBAND_DIAG.update(why=res[sd][2], txt=res[sd][3])
    return res[sd][1] if res[sd][1] is not None else _NONE3

def _armband_track_one(frame, sh, el, block=None):
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

    hsv = _hsv(roi)
    mask = cv2.inRange(hsv, (0, ARMBAND_MIN_SAT, ARMBAND_MIN_VAL), (180, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    if block is not None:
        mask[block[hsv[..., 0]]] = 0
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
    hsv = _hsv(roi)
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
    block = _shirt_hue_block(frame, kpts) if (ARMBAND_TRACK_SHIRT_EXCLUDE and not sideways) else None
    found = [r for r in (_armband_track_one(frame, sh, el, block) for sh, el in pairs) if r is not None]
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

def extract_all_features(frame, kpts, only=None, track=False, arms='R', sideways=False, arm_frame=None, shirt_block=None):
    keys = _FEATURE_EXTRACTORS.keys() if only is None else only
    out = {}
    for k in keys:
        if k == 'armband':
            src = frame if arm_frame is None else arm_frame
            if track:
                out[k] = extract_armband_track(src, kpts, arms, sideways)
            else:
                out[k] = extract_armband_color(src, kpts, shirt_block)
        else:
            out[k] = _FEATURE_EXTRACTORS[k](frame, kpts)
    return out

# ── Tăng độ rực màu ảnh trên PC (chỉ cho tay áo) ────────────
_BOOST_LUT = {}

def _boost_luts(gamma, sat_gain, sat_pivot):
    key = (float(gamma), float(sat_gain), int(sat_pivot))
    lut = _BOOST_LUT.get(key)
    if lut is None:
        x = np.arange(256, dtype=np.float32)
        v_lut = 255.0 * np.power(x / 255.0, max(0.05, float(gamma)))
        s_lut = np.where(x > sat_pivot, sat_pivot + (x - sat_pivot) * float(sat_gain), x)
        lut = (np.clip(s_lut, 0, 255).astype(np.float32), np.clip(v_lut, 0, 255).astype(np.float32))
        _BOOST_LUT[key] = lut
    return lut

def boost_colors(frame, gamma=0.6, sat_gain=1.6, sat_pivot=60, dark=24):
    s_lut, v_lut = _boost_luts(gamma, sat_gain, sat_pivot)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    vf, sf = v.astype(np.float32), s.astype(np.float32)
    k = np.clip((vf - dark) / float(max(dark, 1)), 0.0, 1.0)
    v2 = vf + k * (v_lut[v] - vf)
    s2 = sf + k * (s_lut[s] - sf)
    out = cv2.merge((h, np.clip(s2 + 0.5, 0, 255).astype(np.uint8), np.clip(v2 + 0.5, 0, 255).astype(np.uint8)))
    return cv2.cvtColor(out, cv2.COLOR_HSV2BGR)

def exposure_gamma(frame, target_v=125, gamma_min=0.5):
    v = cv2.cvtColor(frame[::4, ::4], cv2.COLOR_BGR2HSV)[..., 2]
    med = float(np.median(v))
    if med >= target_v or med < 2:
        return 1.0
    g = np.log(target_v / 255.0) / np.log(med / 255.0)
    return float(min(1.0, max(gamma_min, g)))

# ── So khớp ───────────────────────────────────────────────────
def _smooth_axis(h, ax, k, circ=False):
    if not k:
        return h
    if circ:
        return (1 - 2 * k) * h + k * (np.roll(h, 1, ax) + np.roll(h, -1, ax))
    p = np.pad(h, [(1, 1) if a == ax else (0, 0) for a in range(h.ndim)], mode='edge')
    lo = tuple(slice(0, -2) if a == ax else slice(None) for a in range(h.ndim))
    hi = tuple(slice(2, None) if a == ax else slice(None) for a in range(h.ndim))
    return (1 - 2 * k) * h + k * (p[lo] + p[hi])

def _color_desc(hist):
    P = np.asarray(hist, np.float32)
    tot = float(P.sum())
    if tot > 0:
        P = P / tot
    b = COLOR_S_BRIDGE
    chroma = P[:, 2:, :].sum(axis=1) + (1.0 - b) * P[:, 1, :]
    gray = P[:, 0, :].sum(axis=0) + b * P[:, 1, :].sum(axis=0)
    chroma = _smooth_axis(_smooth_axis(chroma, 0, COLOR_HUE_SMOOTH, circ=True), 1, COLOR_V_SMOOTH)
    gray = _smooth_axis(gray, 0, COLOR_V_SMOOTH)
    return np.concatenate([chroma.ravel(), gray]).astype(np.float32)

def color_sim(a, b):
    if a is None or b is None:
        return None
    if not COLOR_SOFT_MATCH:
        return float(cv2.compareHist(a, b, cv2.HISTCMP_CORREL))
    return float(cv2.compareHist(_color_desc(a), _color_desc(b), cv2.HISTCMP_CORREL))

def _corr(master, cand, key):
    return color_sim(master.get(key), cand.get(key))

def _armband_presence(master, cand):
    m = master['armband']
    c = cand['armband']
    mx = float(m.max())
    if mx <= 0:
        return None
    sel = m >= ARMBAND_MASTER_BIN_FRAC * mx
    if ARMBAND_S_TOL_BINS > 0 and sel.ndim == 3:
        ext = sel.copy()
        for k in range(1, ARMBAND_S_TOL_BINS + 1):
            ext[:, :-k, :] |= sel[:, k:, :]
        sel = ext
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

# ── Tư thế đủ tốt để quét màu ─────────────────────────────────
def is_good_pose_for_scan(kpts, feature_type='shirt'):
    if kpts is None or len(kpts) < 13:
        return False
    CONF = 0.40
    if feature_type == 'armband':
        sides = ('R', 'L') if (ARMBAND_SCAN_BOTH_ARMS or SLEEVE_MODE) else ('R',)
        for sd in sides:
            sh, el = kpts[_ARM_IDX[sd][0]], kpts[_ARM_IDX[sd][1]]
            if sh[2] > CONF and el[2] > CONF and np.hypot(el[0] - sh[0], el[1] - sh[1]) >= 20:
                return True
        return False
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
    if torso_h < SCAN_MIN_TORSO_PX:
        return False

    if feature_type == 'pants':
        lk, rk = kpts[13], kpts[14]
        if not any(p[2] > CONF for p in (lk, rk)):
            return False
        hip_y = max(lh[1], rh[1])
        knee_y = max(lk[1] if lk[2] > CONF else 0, rk[1] if rk[2] > CONF else 0)
        if knee_y - hip_y < 30:
            return False
    return True

shirt_hue_block = _shirt_hue_block
armband_presence = _armband_presence
