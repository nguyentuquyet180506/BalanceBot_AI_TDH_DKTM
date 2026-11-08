import os
import json
from pathlib import Path

import cv2
import numpy as np

from _2_ai_models import sanitize_name, save_encodings, _resolve_path

IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
MANIFEST = "face_data/imported_images.json"
MAX_SIDE = 1280
MIN_FACE = 40

def _encode_face(bgr, box):
    import face_recognition
    x, y, w, h = box
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    loc = (max(y, 0), min(x + w, bgr.shape[1]),
           min(y + h, bgr.shape[0]), max(x, 0))
    encs = face_recognition.face_encodings(rgb, known_face_locations=[loc], num_jitters=1)
    return encs[0] if encs else None

def _read_image(path):
    data = np.fromfile(str(path), dtype=np.uint8)
    img = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if img is None:
        return None
    h, w = img.shape[:2]
    s = MAX_SIDE / max(h, w)
    if s < 1.0:
        img = cv2.resize(img, (int(w * s), int(h * s)), interpolation=cv2.INTER_AREA)
    return img

def _load_manifest():
    p = _resolve_path(MANIFEST)
    if os.path.exists(p):
        try:
            with open(p, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            pass
    return {}

def _save_manifest(m):
    p = _resolve_path(MANIFEST)
    with open(p + ".tmp", "w", encoding="utf-8") as f:
        json.dump(m, f, indent=2, ensure_ascii=False)
    os.replace(p + ".tmp", p)

def import_images_from_folders(known_face_encodings, detector, data_folder="face_data"):
    root = Path(_resolve_path(data_folder))
    manifest = _load_manifest()
    added_total, changed = 0, False

    for folder in sorted(p for p in root.iterdir() if p.is_dir()):
        name = sanitize_name(folder.name)
        done = manifest.setdefault(name, {})
        added = 0
        for img_path in sorted(folder.iterdir()):
            if img_path.suffix.lower() not in IMG_EXTS:
                continue
            st = img_path.stat()
            sig = f"{st.st_size}_{int(st.st_mtime)}"
            if done.get(img_path.name) == sig:
                continue

            img = _read_image(img_path)
            if img is None:
                print(f"[ANH] Khong doc duoc: {img_path.name}")
                continue

            h, w = img.shape[:2]
            detector.setInputSize((w, h))
            _, faces = detector.detect(img)
            faces = [] if faces is None else [f for f in faces if f[2] >= MIN_FACE and f[3] >= MIN_FACE]
            if not faces:
                print(f"[ANH] {name}/{img_path.name}: khong thay mat -> bo qua")
                done[img_path.name] = sig; changed = True
                continue
            faces.sort(key=lambda f: f[2] * f[3], reverse=True)
            if len(faces) > 1 and faces[1][2] * faces[1][3] > 0.5 * faces[0][2] * faces[0][3]:
                print(f"[ANH] {name}/{img_path.name}: co nhieu mat tuong duong -> bo qua")
                done[img_path.name] = sig; changed = True
                continue

            box = faces[0][:4].astype(int)
            enc = _encode_face(img, tuple(int(v) for v in box))
            done[img_path.name] = sig; changed = True
            if enc is None:
                print(f"[ANH] {name}/{img_path.name}: khong tao duoc encoding")
                continue
            known_face_encodings.setdefault(name, []).append(enc)
            added += 1

        if added:
            print(f"[ANH] {name}: +{added} anh moi (tong {len(known_face_encodings[name])} encoding)")
            added_total += added

    if added_total:
        save_encodings(known_face_encodings)
    if changed:
        _save_manifest(manifest)
    return added_total