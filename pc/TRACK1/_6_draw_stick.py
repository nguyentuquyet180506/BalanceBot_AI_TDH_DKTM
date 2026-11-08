import cv2
import numpy as np

COL_TORSO = (0, 255, 255)
COL_RIGHT = (0, 140, 255)
COL_LEFT = (255, 120, 0)
COL_LEGS = (0, 255, 0)
COL_HEAD = (255, 255, 255)
COL_JOINT = (0, 0, 255)
COL_UNTRUSTED = (170, 170, 170)


def _looks_normalized(landmarks):
    vals = []
    for lm in landmarks:
        if hasattr(lm, "visibility"):
            return True
        try:
            vals.extend((float(lm[0]), float(lm[1])))
        except Exception:
            continue
    return bool(vals) and max(vals) <= 1.5


_COCO = dict(nose=0, leye=1, reye=2, lear=3, rear=4, ls=5, rs=6, le=7, re=8, lw=9, rw=10,
             lh=11, rh=12, lk=13, rk=14, la=15, ra=16)
_MP = dict(nose=0, leye=2, reye=5, lear=7, rear=8, ls=11, rs=12, le=13, re=14, lw=15, rw=16,
           lh=23, rh=24, lk=25, rk=26, la=27, ra=28)


def get_joint_points(landmarks, w, h, conf_thresh=0.25, coords="auto"):
    total = len(landmarks)
    idx_map = _COCO if total <= 17 else _MP
    if coords == "norm":
        use_norm = True
    elif coords == "pixel":
        use_norm = False
    else:
        use_norm = _looks_normalized(landmarks)

    pts = {}
    for key, idx in idx_map.items():
        if idx >= total:
            pts[key] = None
            continue
        lm = landmarks[idx]
        if hasattr(lm, "visibility"):
            vis, x_val, y_val = lm.visibility, lm.x, lm.y
        elif isinstance(lm, (np.ndarray, list, tuple)):
            vis = lm[2] if len(lm) > 2 else 1.0
            x_val, y_val = lm[0], lm[1]
        else:
            pts[key] = None
            continue
        if vis < conf_thresh:
            pts[key] = None
        else:
            pts[key] = (int(x_val * w) if use_norm else int(x_val),
                        int(y_val * h) if use_norm else int(y_val))
    return pts


def _mid(a, b):
    return ((a[0] + b[0]) // 2, (a[1] + b[1]) // 2)


def draw_stick_figure(frame, landmarks, draw_on_black=False, conf_thresh=0.25, coords="auto",
                      in_place=False, trusted=True, facing=None):
    h, w = frame.shape[:2]
    if draw_on_black:
        canvas = np.zeros_like(frame)
    else:
        canvas = frame if in_place else frame.copy()
    if len(landmarks) == 0:
        return canvas

    pts = get_joint_points(landmarks, w, h, conf_thresh, coords)
    g = lambda k: pts.get(k)

    c_torso = COL_TORSO if trusted else COL_UNTRUSTED
    c_r = COL_RIGHT if trusted else COL_UNTRUSTED
    c_l = COL_LEFT if trusted else COL_UNTRUSTED
    c_leg = COL_LEGS if trusted else COL_UNTRUSTED

    ls, rs, lh, rh = g("ls"), g("rs"), g("lh"), g("rh")
    sh_mid = _mid(ls, rs) if (ls and rs) else None
    sw = int(np.hypot(ls[0] - rs[0], ls[1] - rs[1])) if (ls and rs) else 0

    hip_mid = _mid(lh, rh) if (lh and rh) else (lh or rh)
    if sh_mid and hip_mid:
        cv2.line(canvas, sh_mid, hip_mid, c_torso, 3)
    if ls and rs:
        cv2.line(canvas, ls, rs, c_torso, 3)
    if lh and rh:
        cv2.line(canvas, lh, rh, c_torso, 3)

    head_pts = [p for p in (g("nose"), g("leye"), g("reye"), g("lear"), g("rear")) if p]
    if head_pts:
        cx = int(sum(p[0] for p in head_pts) / len(head_pts))
        cy = int(sum(p[1] for p in head_pts) / len(head_pts))
        if g("lear") and g("rear"):
            radius = max(12, int(np.hypot(g("lear")[0] - g("rear")[0], g("lear")[1] - g("rear")[1]) * 0.75))
        elif sw:
            radius = max(15, sw // 3)
        else:
            radius = 20
        cv2.circle(canvas, (cx, cy), radius, COL_HEAD, 2)
        if sh_mid:
            cv2.line(canvas, (cx, cy + radius), sh_mid, c_torso, 2)

    left_img_label = ("ls" if ls[0] < rs[0] else "rs") if (ls and rs) else "rs"
    img_left_is_right_hand = facing != "back"
    for label, (a, b, c) in (("rs", ("rs", "re", "rw")), ("ls", ("ls", "le", "lw"))):
        col = c_r if ((label == left_img_label) == img_left_is_right_hand) else c_l
        for p, q in ((a, b), (b, c)):
            if g(p) and g(q):
                cv2.line(canvas, g(p), g(q), col, 4)
    for a, b in (("lh", "lk"), ("lk", "la"), ("rh", "rk"), ("rk", "ra")):
        if g(a) and g(b):
            cv2.line(canvas, g(a), g(b), c_leg, 3)

    for key in ("ls", "rs", "le", "re", "lw", "rw", "lh", "rh", "lk", "rk", "la", "ra"):
        if g(key):
            cv2.circle(canvas, g(key), 7 if key in ("lw", "rw") else 5,
                       COL_JOINT if trusted else COL_UNTRUSTED, -1)
    return canvas