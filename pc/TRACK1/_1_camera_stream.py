import socket
import struct
import sys
import threading
import time
from collections import deque

import cv2
import numpy as np

if sys.platform.startswith("win"):
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    if hasattr(sys.stderr, "reconfigure"):
        sys.stderr.reconfigure(encoding="utf-8")

ESP_PORT = 4210
HDR = struct.Struct("<HHHH")


def get_broadcast_targets():
    targets = {"255.255.255.255"}
    try:
        for ip in socket.gethostbyname_ex(socket.gethostname())[2]:
            parts = ip.split(".")
            if len(parts) == 4 and not ip.startswith("127."):
                targets.add(f"{parts[0]}.{parts[1]}.{parts[2]}.255")
    except Exception:
        pass
    return list(targets)


class ThreadedCamera:

    def __init__(self, target_ip=None, port=ESP_PORT, stale_timeout=3.0, src=None, streaming=True):
        if isinstance(src, str) and (src.replace(".", "").isdigit() or ":" in src):
            target_ip = src
        elif target_ip in ("auto", "0", 0, None):
            target_ip = None

        self.target_ip = target_ip
        self.port = port
        self.stale_timeout = stale_timeout

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self.ret = False
        self.frame = None
        self.width = 0
        self.height = 0
        self._frame_id = 0
        self._last_new_frame_time = None
        self.stopped = False
        self._stream_on = bool(streaming)

        self._frame_buffer = deque(maxlen=6)

        self.stat = {"ok": 0, "incomplete": 0, "badjpg": 0}

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.settimeout(0.5)
        self.sock.bind(("", 0))

        self.broadcast_ips = get_broadcast_targets()
        self._keepalive_thread = None
        self._net_thread = None

        if self.target_ip:
            print(f"[1/6] Đang kết nối ESP32-CAM qua UDP tại {self.target_ip}:{self.port}...")
        else:
            print(f"[1/6] Đang tự động dò tìm ESP32-CAM trên mạng WiFi (Port {self.port})...")

    def _keepalive_loop(self):
        prev_ok = prev_bad = 0
        while not self._stop_event.is_set():
            with self._lock:
                ok, bad = self.stat["ok"], self.stat["incomplete"]
                current_ip = self.target_ip

            d_ok, d_bad = ok - prev_ok, bad - prev_bad
            prev_ok, prev_bad = ok, bad
            loss = int(100 * d_bad / (d_ok + d_bad)) if (d_ok + d_bad) else 0
            if not self._stream_on:
                time.sleep(1.0)
                continue
            msg = f"HELLO {loss}".encode("ascii")

            try:
                if current_ip:
                    self.sock.sendto(msg, (current_ip, self.port))
                else:
                    for b_ip in self.broadcast_ips:
                        try:
                            self.sock.sendto(msg, (b_ip, self.port))
                        except OSError:
                            pass
            except OSError:
                pass

            time.sleep(1.0)

    def _net_loop(self):
        cur_id = None
        chunks = {}
        total = 0
        done = False

        while not self._stop_event.is_set():
            try:
                data, addr = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break

            sender_ip = addr[0]

            if data.startswith(b"ESP32_CAM_BEACON"):
                with self._lock:
                    if self.target_ip != sender_ip:
                        self.target_ip = sender_ip
                        print(f"-> [UDP] Đã phát hiện ESP32-S3 Beacon tại IP: {sender_ip}")
                continue

            if len(data) < HDR.size:
                continue

            with self._lock:
                if self.target_ip is None:
                    self.target_ip = sender_ip
                    print(f"-> [UDP] Đã nhận luồng video từ ESP32 tại IP: {sender_ip}")

            fid, idx, tot, plen = HDR.unpack_from(data)
            payload = data[HDR.size : HDR.size + plen]
            if len(payload) != plen or tot == 0 or idx >= tot:
                continue

            if cur_id is None or fid != cur_id:
                if cur_id is not None and ((fid - cur_id) & 0xFFFF) >= 0x8000:
                    continue
                if cur_id is not None and not done:
                    with self._lock:
                        self.stat["incomplete"] += 1
                cur_id, chunks, total, done = fid, {}, tot, False

            if done:
                continue

            chunks[idx] = payload
            if len(chunks) == total:
                done = True
                jpg_bytes = b"".join(chunks[i] for i in range(total))
                frame = cv2.imdecode(np.frombuffer(jpg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)

                if frame is not None:
                    h, w = frame.shape[:2]
                    now = time.monotonic()
                    with self._lock:
                        self.stat["ok"] += 1
                        self._frame_id += 1
                        self.frame = frame
                        self.width = w
                        self.height = h
                        self.ret = True
                        self._last_new_frame_time = now
                        self._frame_buffer.append((self._frame_id, frame))
                else:
                    with self._lock:
                        self.stat["badjpg"] += 1

    def start(self):
        if self._net_thread is None or not self._net_thread.is_alive():
            self._stop_event.clear()
            self.stopped = False
            self._keepalive_thread = threading.Thread(target=self._keepalive_loop, daemon=True)
            self._net_thread = threading.Thread(target=self._net_loop, daemon=True)
            self._keepalive_thread.start()
            self._net_thread.start()
        return self

    def set_streaming(self, on):
        on = bool(on)
        if on == self._stream_on:
            return
        self._stream_on = on
        if on:
            with self._lock:
                current_ip = self.target_ip
            try:
                if current_ip:
                    self.sock.sendto(b"HELLO 0", (current_ip, self.port))
                else:
                    for b_ip in self.broadcast_ips:
                        self.sock.sendto(b"HELLO 0", (b_ip, self.port))
            except OSError:
                pass

    def read(self):
        with self._lock:
            if not (self.ret and self.frame is not None):
                return False, None
            if self._last_new_frame_time is not None:
                if time.monotonic() - self._last_new_frame_time > self.stale_timeout:
                    return False, None
            return True, self.frame.copy()

    def read_latest(self, last_id):
        with self._lock:
            if not self.ret or self.frame is None:
                return False, None, self._frame_id
            if self._last_new_frame_time is not None:
                if time.monotonic() - self._last_new_frame_time > self.stale_timeout:
                    return False, None, self._frame_id

            for fid, frm in self._frame_buffer:
                if fid > last_id:
                    return True, frm.copy(), fid

            return True, None, self._frame_id

    def stop(self):
        self.stopped = True
        self._stop_event.set()
        try:
            self.sock.close()
        except Exception:
            pass
        if self._keepalive_thread and self._keepalive_thread.is_alive():
            self._keepalive_thread.join(timeout=1.0)
        if self._net_thread and self._net_thread.is_alive():
            self._net_thread.join(timeout=1.0)
        print("-> Đã dừng luồng Camera ESP32 UDP.")


class PcCamera:

    def __init__(self, index=0, width=640, height=480, stale_timeout=2.0):
        self.target_ip = None
        self.index = index
        self.stale_timeout = stale_timeout
        api = cv2.CAP_DSHOW if sys.platform.startswith("win") else cv2.CAP_ANY
        self.cap = cv2.VideoCapture(index, api)
        if width and height:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.opened = bool(self.cap.isOpened())
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None
        self.frame = None
        self._frame_id = 0
        self._last_new_frame_time = None
        if self.opened:
            print(f"-> [WEBCAM] Da mo webcam may tinh (so {index}).")
        else:
            print(f"[!] Khong mo duoc webcam may tinh so {index} (doi PC_CAM_INDEX trong _0_config.py, hoac bam Y dung cam ESP).")

    def _loop(self):
        while not self._stop_event.is_set():
            ok, frame = self.cap.read() if self.opened else (False, None)
            if not ok or frame is None:
                time.sleep(0.05)
                continue
            with self._lock:
                self.frame = frame
                self._frame_id += 1
                self._last_new_frame_time = time.monotonic()

    def start(self):
        if self._thread is None or not self._thread.is_alive():
            self._stop_event.clear()
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()
        return self

    def _fresh(self):
        return (self.frame is not None and self._last_new_frame_time is not None
                and time.monotonic() - self._last_new_frame_time <= self.stale_timeout)

    def read(self):
        with self._lock:
            if not self._fresh():
                return False, None
            return True, self.frame.copy()

    def read_latest(self, last_id):
        with self._lock:
            if not self._fresh():
                return False, None, self._frame_id
            if self._frame_id > last_id:
                return True, self.frame.copy(), self._frame_id
            return True, None, self._frame_id

    def set_streaming(self, on):
        pass

    def stop(self):
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        try:
            self.cap.release()
        except Exception:
            pass
        print("-> Da tat webcam may tinh.")
