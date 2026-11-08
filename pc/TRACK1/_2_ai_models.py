import os
import re
import json
import time
import warnings
from pathlib import Path

import cv2
import torch
import numpy as np
from ultralytics import YOLO

if hasattr(cv2, 'setLogLevel'):
    cv2.setLogLevel(0)

_THIS_DIR = Path(__file__).parent

def _resolve_path(filename):
    p = _THIS_DIR / filename
    return str(p)

def init_ai_models(
    onnx_file="face_detection_yunet_2023mar.onnx",
    data_folder="face_data",
    encodings_file="face_encodings.json",
    cpu_threads=2
):
    torch.set_num_threads(cpu_threads)
    warnings.filterwarnings("ignore", category=UserWarning)

    print("[2/6] Nạp AI YOLO Pose & YuNet...")
    yolo_pose = YOLO(_resolve_path("yolov8n-pose.pt"))

    detector = cv2.FaceDetectorYN.create(
        model=_resolve_path(onnx_file), config="", input_size=(320, 240),
        score_threshold=0.5, nms_threshold=0.3, top_k=5000
    )
    os.makedirs(_resolve_path(data_folder), exist_ok=True)

    known_face_encodings = load_encodings(encodings_file)
    return yolo_pose, detector, known_face_encodings

def sanitize_name(raw_name):
    clean = raw_name.strip().replace(" ", "_")
    clean = re.sub(r'[^A-Za-z0-9_\-]', '', clean)
    if not clean:
        clean = "unknown"
    return clean

def load_encodings(encodings_file="face_encodings.json"):
    filepath = _resolve_path(encodings_file)
    if not os.path.exists(filepath):
        return {}

    try:
        with open(filepath, "r", encoding="utf-8") as f:
            index = json.load(f)
    except Exception as e:
        print("[LỖI] Đọc index:", e)
        backup = f"{filepath}.corrupt.{int(time.time())}"
        try:
            os.replace(filepath, backup)
            print(f"-> Đã sao lưu file hỏng: {backup}")
        except OSError:
            pass
        return {}

    data = {}
    for name, info in index.items():
        try:
            npy_path = _resolve_path(info["npy_file"])
            if os.path.exists(npy_path):
                arr = np.load(npy_path)
                data[name] = [arr[i] for i in range(arr.shape[0])]
            else:
                data[name] = []
        except Exception as e:
            print(f"[LỖI] Nạp '{name}':", e)

    print(f"-> Đã nạp dữ liệu {len(data)} người dùng.")
    return data

def save_encodings(encodings, encodings_file="face_encodings.json"):
    filepath = _resolve_path(encodings_file)

    index = {}
    for name, enc_list in encodings.items():
        if not enc_list:
            continue
        npy_rel = f"face_data/{name}_encodings.npy"
        npy_path = _resolve_path(npy_rel)
        tmp_npy = npy_path + ".tmp"
        with open(tmp_npy, "wb") as f:
            np.save(f, np.array(enc_list))
        os.replace(tmp_npy, npy_path)
        index[name] = {"npy_file": npy_rel, "count": len(enc_list)}

    tmp_json = filepath + ".tmp"
    with open(tmp_json, "w", encoding="utf-8") as f:
        json.dump(index, f, indent=2, ensure_ascii=False)
    os.replace(tmp_json, filepath)