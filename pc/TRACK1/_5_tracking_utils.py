import os
import re
import time
import cv2
import numpy as np

from _3_face_filters import compute_encoding, head_pose

# HÀM TIỆN ÍCH                                                 
def get_box_center(box):
    if box is None:
        return None
    return (box[0] + box[2]) // 2, (box[1] + box[3]) // 2

def match_face_to_person(face_center, persons):
    fcx, fcy = face_center
    best_tid = None
    best_area = float('inf')

    for t_id, box in persons:
        px1, py1, px2, py2 = box
        if px1 <= fcx <= px2 and py1 <= fcy <= py2:
            area = (px2 - px1) * (py2 - py1)
            if area < best_area:
                best_area = area
                best_tid = t_id

    return best_tid


# ĐĂNG KÝ MẶT (QUÉT 10 GIÂY ĐA GÓC)

def capture_multi_angle_encodings_10s(
    cam_stream, clean_name, main_win,
    detector, known_face_encodings, save_fn,
    data_folder="face_data"
):
    _this_dir = os.path.dirname(os.path.abspath(__file__))

    print(f"[ĐĂNG KÝ] Quét 10s cho: {clean_name}")
    person_folder = os.path.join(_this_dir, data_folder, clean_name)
    os.makedirs(person_folder, exist_ok=True)

    start_time = time.time()
    DURATION = 10.0
    last_capture_time = 0
    collected_encodings = []
    collected_crops = []
    captured_count = 0
    pose_counts = {}

    while True:
        elapsed = time.time() - start_time
        if elapsed >= DURATION:
            break

        ret, frame = cam_stream.read()
        if not ret or frame is None:
            time.sleep(0.01)
            continue

        h_img, w_img = frame.shape[:2]
        detector.setInputSize((w_img, h_img))
        _, faces = detector.detect(frame)

        disp = frame.copy()

        best_face_box = None
        best_face_info = None
        best_face_area = 0

        if faces is not None and len(faces) > 0:
            for face in faces:
                fbox = face[:4].astype(int)
                fx, fy, fw, fh = fbox
                face_area = fw * fh
                if fw > 15 and fh > 15 and face_area > best_face_area:
                    best_face_area = face_area
                    best_face_box = (fx, fy, fw, fh)
                    best_face_info = face

        now = time.time()
        if now - last_capture_time >= 0.15 and best_face_box is not None:
            enc, face_loc = compute_encoding(frame, best_face_box, expand_ratio=0.35, enroll=True)

            if enc is not None:
                collected_encodings.append(enc)
                pose = head_pose(best_face_info)
                if pose is not None:
                    pose_counts[pose['label']] = pose_counts.get(pose['label'], 0) + 1
                captured_count += 1
                last_capture_time = now

                top, right, bottom, left = face_loc if face_loc else (0, 0, 0, 0)
                crop_img = frame[max(0, top):min(h_img, bottom), max(0, left):min(w_img, right)]
                collected_crops.append(
                    cv2.resize(crop_img, (160, 160)) if crop_img.size > 0 else None
                )

        if best_face_box is not None:
            bx, by, bw, bh = best_face_box
            cv2.rectangle(disp, (bx, by), (bx + bw, by + bh), (0, 255, 0), 2)

        cv2.putText(disp, f"QUET DA GOC MAT: {clean_name} | CON {max(0, int(np.ceil(DURATION - elapsed)))}S",
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(disp, f"DA THU THAP: {captured_count} GOC MAT",
                    (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow(main_win, disp)
        cv2.waitKey(30)

    MIN_SAMPLES = 3
    if len(collected_encodings) < MIN_SAMPLES:
        print(f"[HỦY] Chỉ thu {len(collected_encodings)} mẫu (<{MIN_SAMPLES}), giữ nguyên dữ liệu cũ.")
        return

    if pose_counts:
        print("[ĐĂNG KÝ] Phân bố góc đầu:", ", ".join(f"{k}={v}" for k, v in sorted(pose_counts.items())))

    # ── LƯU TIẾP NỐI (không ghi đè) ──
    old_encs = known_face_encodings.get(clean_name) or []
    if not isinstance(old_encs, list):
        old_encs = [old_encs]

    max_idx = 0
    for fn in os.listdir(person_folder):
        m = re.fullmatch(r"face_(\d+)\.jpg", fn)
        if m:
            max_idx = max(max_idx, int(m.group(1)))
    next_idx = max(max_idx, len(old_encs)) + 1

    for k, crop_img in enumerate(collected_crops):
        if crop_img is not None:
            cv2.imwrite(os.path.join(person_folder, f"face_{next_idx + k}.jpg"), crop_img)

    known_face_encodings[clean_name] = list(old_encs) + list(collected_encodings)
    save_fn(known_face_encodings)
    print(f"[ĐĂNG KÝ] {clean_name}: +{len(collected_encodings)} mẫu mới "
          f"-> tổng {len(known_face_encodings[clean_name])}")