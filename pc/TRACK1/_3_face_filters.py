import math
import cv2
import numpy as np
import _0_config as cfg

ENROLL_JITTERS = 5
RECOG_JITTERS = 1

USE_TIGHT_BOX_FOR_DLIB = False

# ============================================================
# NHẬN DIỆN GÓC ĐẦU (không lọc, chỉ phân loại)
# ============================================================

def head_pose(face_info):
    if face_info is None or len(face_info) < 14:
        return None

    ex_r, ey_r, ex_l, ey_l = face_info[4], face_info[5], face_info[6], face_info[7]
    nx, ny = face_info[8], face_info[9]
    my = (face_info[11] + face_info[13]) / 2.0

    eye_dist = abs(ex_l - ex_r)
    if eye_dist < 1.0:
        return {'label': 'nghieng', 'yaw': float('inf'), 'pitch': 0.0, 'roll': 0.0}

    eye_mid_x, eye_mid_y = (ex_r + ex_l) / 2.0, (ey_r + ey_l) / 2.0
    yaw = abs(nx - eye_mid_x) / eye_dist
    mouth_span = my - eye_mid_y
    pitch = (ny - eye_mid_y) / mouth_span if mouth_span > 1.0 else 0.6
    roll = math.degrees(math.atan2(abs(ey_l - ey_r), eye_dist))

    if yaw > cfg.HEAD_YAW_MAX:
        label = 'nghieng'
    elif pitch > cfg.HEAD_PITCH_DOWN:
        label = 'cui'
    elif pitch < cfg.HEAD_PITCH_UP:
        label = 'ngua'
    elif roll > cfg.HEAD_ROLL_MAX:
        label = 'nghieng_dau'
    else:
        label = 'thang'
    return {'label': label, 'yaw': float(yaw), 'pitch': float(pitch), 'roll': float(roll)}

#============================================================
# LỌC ẢNH CHO AI ĐOÁN
#============================================================
def is_sharp_face(bgr_frame, face_box, blur_thresh=None, min_size=None):
    if blur_thresh is None:
        blur_thresh = cfg.FACE_BLUR_THRESH
    if min_size is None:
        min_size = cfg.FACE_MIN_SIZE

    if face_box is None:
        return False, 0.0

    fx, fy, fw, fh = [int(v) for v in face_box]
    if fw < min_size or fh < min_size:
        return False, 0.0

    h_img, w_img = bgr_frame.shape[:2]
    top, left = max(0, fy), max(0, fx)
    bottom, right = min(h_img, fy + fh), min(w_img, fx + fw)

    crop = bgr_frame[top:bottom, left:right]
    if crop.size == 0 or crop.shape[0] < 10 or crop.shape[1] < 10:
        return False, 0.0

    resized = cv2.resize(crop, (100, 100))
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    
    score = cv2.Laplacian(gray, cv2.CV_64F).var()

    return (score >= blur_thresh), score

# ============================================================
# TÍNH ENCODING (dùng chung cho đăng ký và nhận diện)
# ============================================================
def to_encoding_image(bgr_img):
    return np.ascontiguousarray(cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB))

def compute_encoding(bgr_frame, face_box, expand_ratio=0.35, enroll=False, num_jitters=None):
    import face_recognition

    if face_box is None:
        return None, None

    if num_jitters is None:
        num_jitters = ENROLL_JITTERS if enroll else RECOG_JITTERS

    h_img, w_img = bgr_frame.shape[:2]
    fx, fy, fw, fh = [int(v) for v in face_box]

    f_top, f_left = max(0, fy), max(0, fx)
    f_bottom, f_right = min(h_img, fy + fh), min(w_img, fx + fw)
    if f_bottom - f_top < 2 or f_right - f_left < 2:
        return None, None

    pad_w, pad_h = int(fw * expand_ratio), int(fh * expand_ratio)
    top = max(0, fy - pad_h)
    bottom = min(h_img, fy + fh + pad_h)
    left = max(0, fx - pad_w)
    right = min(w_img, fx + fw + pad_w)

    crop = bgr_frame[top:bottom, left:right]
    if crop.size == 0:
        return None, None

    rgb = to_encoding_image(crop)

    crop_h, crop_w = crop.shape[:2]
    if USE_TIGHT_BOX_FOR_DLIB:
        loc_in_crop = (f_top - top, f_right - left, f_bottom - top, f_left - left)
        face_loc = (f_top, f_right, f_bottom, f_left)
    else:
        loc_in_crop = (0, crop_w, crop_h, 0)
        face_loc = (top, right, bottom, left)

    encs = face_recognition.face_encodings(rgb, [loc_in_crop], num_jitters=num_jitters)
    if encs:
        return encs[0], face_loc
    return None, None