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
LATE_WINDOW = 16
REDISCOVER_AFTER = 3.0
DIAG_EVERY = 5.0
ESP_MSG = b"ESP32_CAM_"


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


def _log(msg):
    print(time.strftime("[%H:%M:%S] ") + "[CAM ESP] " + msg, flush=True)


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
        self._stream_on_since = time.monotonic()

        self._frame_buffer = deque(maxlen=6)

        self.stat = {"ok": 0, "incomplete": 0, "badjpg": 0, "pkts": 0, "connreset": 0}
        self._last_rx_any = None
        self._esp_status = ""
        self._esp_status_t = None
        self._send_err = ""
        self._lost_logged = False
        self._diag_t = 0.0
        self._diag_snap = dict(self.stat)

        self.sock = self._new_socket()

        self.broadcast_ips = get_broadcast_targets()
        self._keepalive_thread = None
        self._net_thread = None

        if self.target_ip:
            print(f"[1/6] Đang kết nối ESP32-CAM qua UDP tại {self.target_ip}:{self.port}...")
        else:
            print(f"[1/6] Đang tự động dò tìm ESP32-CAM trên mạng WiFi (Port {self.port})...")

    @staticmethod
    def _new_socket():
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.settimeout(0.5)
        sock.bind(("", 0))
        return sock

    def _reopen_socket(self, why):
        _log(f"Loi socket nhan anh ({why}) -> tao lai socket")
        try:
            new = self._new_socket()
        except OSError as e:
            _log(f"Chua tao lai duoc socket: {e} -> thu lai sau 1 s")
            time.sleep(1.0)
            return
        old, self.sock = self.sock, new
        try:
            old.close()
        except OSError:
            pass
        self._send_hello(0, broadcast=True)

    def _send_hello(self, loss, broadcast=False):
        msg = f"HELLO {loss}".encode("ascii")
        with self._lock:
            current_ip = self.target_ip
        dests = [current_ip] if current_ip else []
        if broadcast or not current_ip:
            dests += self.broadcast_ips
        err = ""
        for d in dests:
            try:
                self.sock.sendto(msg, (d, self.port))
            except OSError as e:
                err = f"gui HELLO toi {d} loi: {e}"
        self._send_err = err

    def _keepalive_loop(self):
        prev_ok = prev_bad = 0
        while not self._stop_event.is_set():
            with self._lock:
                ok, bad = self.stat["ok"], self.stat["incomplete"]
            d_ok, d_bad = ok - prev_ok, bad - prev_bad
            prev_ok, prev_bad = ok, bad
            loss = int(100 * d_bad / (d_ok + d_bad)) if (d_ok + d_bad) else 0
            if not self._stream_on:
                time.sleep(1.0)
                continue
            now = time.monotonic()
            gap = self._no_frame_for(now)
            self._send_hello(loss, broadcast=gap >= REDISCOVER_AFTER)
            if gap >= REDISCOVER_AFTER and now - self._diag_t >= DIAG_EVERY:
                self._diag_t = now
                _log(self._diagnose(now, gap))
            time.sleep(1.0)

    def _no_frame_for(self, now):
        with self._lock:
            last = self._last_new_frame_time
        ref = self._stream_on_since if last is None else max(last, self._stream_on_since)
        return now - ref

    def _diagnose(self, now, gap):
        with self._lock:
            st = dict(self.stat)
            ip = self.target_ip
        d = {k: st[k] - self._diag_snap.get(k, 0) for k in st}
        self._diag_snap = st
        head = f"KHONG CO HINH {gap:.0f}s (ESP {ip or 'chua biet IP'}:{self.port}). "
        esp_recent = self._esp_status and self._esp_status_t is not None and now - self._esp_status_t <= DIAG_EVERY + 2
        if esp_recent:
            return head + "ESP co tra loi nhung bao: " + self._esp_status
        if d["incomplete"] > 0 and d["ok"] == 0:
            return head + (f"Nhan duoc goi anh nhung {d['incomplete']} khung deu THIEU GOI (mat goi Wi-Fi: song yeu / nghen / "
                           "ESP qua nong). Thu dua xe lai gan router.")
        if d["badjpg"] > 0 and d["ok"] == 0:
            return head + f"Nhan du goi nhung {d['badjpg']} anh JPEG HONG (giai ma loi) - cam bien / cap cam co the bi loi."
        if self._last_rx_any is not None and now - self._last_rx_any < gap:
            return head + "ESP con lien lac (co goi tra loi) nhung khong gui anh - xem log Serial cua ESP cam (monitor_cam.bat)."
        why = ("ESP tat / dang khoi dong lai / chua vao Wi-Fi; may tinh khac mang Wi-Fi voi ESP; "
               "hoac tuong lua Windows chan UDP 4210 (chay fix_firewall.bat, cho phep python.exe).")
        if d["connreset"] > 0:
            why = f"Windows bao cong 4210 cua ESP dang DONG ({d['connreset']} lan) = ESP dang khoi dong lai / chua chay xong. " + why
        if self._send_err:
            why = self._send_err + ". " + why
        return head + "KHONG nhan duoc goi nao tu ESP (da gui HELLO" + (" + broadcast" if gap >= REDISCOVER_AFTER else "") + "). " + why

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
            except ConnectionResetError:
                with self._lock:
                    self.stat["connreset"] += 1
                continue
            except OSError as e:
                if self._stop_event.is_set():
                    break
                self._reopen_socket(e)
                cur_id, chunks, total, done = None, {}, 0, False
                continue

            sender_ip = addr[0]
            now = time.monotonic()

            if data.startswith(ESP_MSG):
                self._last_rx_any = now
                self._learn_ip(sender_ip, "goi tra loi")
                if data.startswith(b"ESP32_CAM_STATUS"):
                    txt = data[len(b"ESP32_CAM_STATUS"):].decode("ascii", "replace").strip()
                    if txt != self._esp_status:
                        _log("ESP bao: " + txt)
                    self._esp_status, self._esp_status_t = txt, now
                continue

            if len(data) < HDR.size:
                continue

            fid, idx, tot, plen = HDR.unpack_from(data)
            payload = data[HDR.size : HDR.size + plen]
            if len(payload) != plen or tot == 0 or idx >= tot:
                continue
            self._last_rx_any = now
            self._learn_ip(sender_ip, "luong anh")
            with self._lock:
                self.stat["pkts"] += 1

            if cur_id is None or fid != cur_id:
                if cur_id is not None:
                    back = (cur_id - fid) & 0xFFFF
                    if 0 < back <= LATE_WINDOW:
                        continue
                    if LATE_WINDOW < back < 0x8000:
                        _log(f"So khung nhay tu {cur_id} ve {fid}: ESP vua khoi dong lai -> nhan luong moi")
                    if not done:
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
                    with self._lock:
                        gap = None if self._last_new_frame_time is None else now - self._last_new_frame_time
                        self.stat["ok"] += 1
                        self._frame_id += 1
                        self.frame = frame
                        self.width = w
                        self.height = h
                        self.ret = True
                        self._last_new_frame_time = now
                        self._frame_buffer.append((self._frame_id, frame))
                    if gap is None:
                        _log(f"Da co hinh tu {sender_ip} ({w}x{h})")
                    elif gap > self.stale_timeout:
                        _log(f"Da BAT LAI hinh tu {sender_ip} sau {gap:.1f}s mat hinh ({w}x{h})")
                    self._esp_status = ""
                else:
                    with self._lock:
                        self.stat["badjpg"] += 1

    def _learn_ip(self, ip, what):
        with self._lock:
            old = self.target_ip
            if old == ip:
                return
            self.target_ip = ip
        if old is None:
            _log(f"Da tim thay ESP32 cam tai IP {ip} ({what})")
        else:
            _log(f"ESP32 cam DOI IP: {old} -> {ip} ({what}); lenh xe cung gui sang IP moi")

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
            self._stream_on_since = time.monotonic()
            self._diag_t = 0.0
            self._send_hello(0, broadcast=True)
            if self._send_err:
                _log(self._send_err)

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
