#!/usr/bin/env python3
"""
HỆ THỐNG ROBOT CÂN BẰNG + AI BÁM NGƯỜI — KHỞI CHẠY PHẦN MÁY TÍNH (1 LỆNH)

CÁCH DÙNG:
  python run.py                  -> Mở chương trình chính (mặc định chế độ 3: xem cam, không AI)
  python run.py --mode 5         -> Mở thẳng chế độ 5. 1 tay | 2 cử chỉ | 3 cam | 4 bám người áo+quần | 5 bám người áo+quần+tay áo khác màu
  python run.py <IP_ESP_CAM>     -> Chỉ định IP ESP cam (vd 192.168.4.1 khi nối Wi-Fi AP của xe)
  python run.py --view           -> Chỉ xem camera (tools/receiver.py, không AI, không lái)
  python run.py --control        -> Chỉ mở cửa sổ cần gạt UDP (balancebot_udp.py)
Đang chạy vẫn đổi chế độ được: gõ 1 / 2 / 3 / 4 / 5 rồi Enter trong terminal (hoặc phím số trên cửa sổ video).

Nạp firmware KHÔNG nằm ở đây: dùng upload_balance.bat / upload_cam.bat ở thư mục gốc (2 lệnh độc lập).
"""

import os
import subprocess
import sys

if sys.platform.startswith("win"):
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    if hasattr(sys.stderr, "reconfigure"):
        sys.stderr.reconfigure(encoding="utf-8")

PC_DIR = os.path.dirname(os.path.abspath(__file__))
TRACK1_DIR = os.path.join(PC_DIR, "TRACK1")
TRACK1_MAIN = os.path.join(TRACK1_DIR, "main.py")
VIEWER_SCRIPT = os.path.join(PC_DIR, "tools", "receiver.py")
JOYSTICK_SCRIPT = os.path.join(PC_DIR, "balancebot_udp.py")


def find_python_with_ai():
    candidates = [
        os.path.expanduser(r"~/AppData/Local/Python/bin/python.exe"),
        sys.executable,
        os.path.expanduser(r"~/AppData/Local/Programs/Python/Python312/python.exe"),
        os.path.expanduser(r"~/AppData/Local/Programs/Python/Python311/python.exe"),
        "python",
    ]
    for py in candidates:
        if py != "python" and not os.path.isfile(py):
            continue
        try:
            res = subprocess.run(
                [py, "-c", "import cv2, torch; print('AI_OK')"],
                capture_output=True,
                text=True,
                timeout=15,
            )
            if "AI_OK" in res.stdout:
                return py
        except Exception:
            continue
    return sys.executable


def main():
    args = sys.argv[1:]
    if "--help" in args or "-h" in args:
        print(__doc__)
        return

    python_path = find_python_with_ai()
    print(f"[+] Trình thông dịch Python được chọn: {python_path}")
    passthrough = [a for a in args if a not in ("--view", "--control")]

    if "--view" in args:
        ip = [a for a in passthrough if not a.startswith("-")]
        subprocess.run([python_path, VIEWER_SCRIPT] + ip[:1], cwd=os.path.dirname(VIEWER_SCRIPT))
    elif "--control" in args:
        ip = [a for a in passthrough if not a.startswith("-")]
        subprocess.run([python_path, JOYSTICK_SCRIPT] + ip[:1], cwd=PC_DIR)
    else:
        print("==========================================================")
        print(" [*] ĐANG KHỞI ĐỘNG HỆ THỐNG ROBOT (camera UDP + AI + điều khiển)...")
        print("     -> Tự dò ESP cam qua beacon UDP cổng 4210 (hoặc truyền IP).")
        print("     -> Đổi chế độ: gõ 1 / 2 / 3 / 4 / 5 + Enter tại đây.")
        print("==========================================================")
        subprocess.run([python_path, TRACK1_MAIN] + passthrough, cwd=TRACK1_DIR)


if __name__ == "__main__":
    main()
