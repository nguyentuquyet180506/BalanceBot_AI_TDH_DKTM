import queue
import socket
import struct
import sys
import threading
import time

import cv2
import numpy as np

MANUAL_IP = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] != "auto" else None
ESP_PORT = 4210
HDR = struct.Struct("<HHHH")
WINDOW = "ESP32-S3 UDP Camera"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.settimeout(1.0)
sock.bind(("", 0))

stop_event = threading.Event()
target_ip = MANUAL_IP
state_lock = threading.Lock()

stat = {"ok": 0, "incomplete": 0, "qdrop": 0, "badjpg": 0, "shown": 0}

frame_q: "queue.Queue[bytes]" = queue.Queue(maxsize=1)


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


broadcast_ips = get_broadcast_targets()


def keepalive():
    prev_ok = prev_bad = 0
    while not stop_event.is_set():
        with state_lock:
            ok, bad = stat["ok"], stat["incomplete"]
            current = target_ip
        d_ok, d_bad = ok - prev_ok, bad - prev_bad
        prev_ok, prev_bad = ok, bad
        loss = int(100 * d_bad / (d_ok + d_bad)) if (d_ok + d_bad) else 0
        msg = b"HELLO %d" % loss
        try:
            if current:
                sock.sendto(msg, (current, ESP_PORT))
            else:
                for b_ip in broadcast_ips:
                    try:
                        sock.sendto(msg, (b_ip, ESP_PORT))
                    except OSError:
                        pass
        except OSError:
            pass
        time.sleep(1)


def push_latest(jpg: bytes):
    try:
        frame_q.put_nowait(jpg)
    except queue.Full:
        try:
            frame_q.get_nowait()
            with state_lock:
                stat["qdrop"] += 1
        except queue.Empty:
            pass
        try:
            frame_q.put_nowait(jpg)
        except queue.Full:
            pass


def net_loop():
    global target_ip
    cur_id = None
    chunks: dict = {}
    total = 0
    done = False

    while not stop_event.is_set():
        try:
            data, addr = sock.recvfrom(2048)
        except socket.timeout:
            continue
        except OSError:
            break

        sender_ip = addr[0]

        if data.startswith(b"ESP32_CAM_BEACON"):
            with state_lock:
                if target_ip != sender_ip:
                    target_ip = sender_ip
                    print(f"[+] Da phat hien ESP32-S3 tai IP: {sender_ip}")
            continue

        if len(data) < HDR.size:
            continue

        with state_lock:
            if target_ip is None:
                target_ip = sender_ip
                print(f"[+] Da nhan stream tu ESP32 tai IP: {sender_ip}")

        fid, idx, tot, plen = HDR.unpack_from(data)
        payload = data[HDR.size : HDR.size + plen]
        if len(payload) != plen or tot == 0 or idx >= tot:
            continue

        if cur_id is None or fid != cur_id:
            if cur_id is not None and ((fid - cur_id) & 0xFFFF) >= 0x8000:
                continue
            if cur_id is not None and not done:
                with state_lock:
                    stat["incomplete"] += 1
            cur_id, chunks, total, done = fid, {}, tot, False

        if done:
            continue

        chunks[idx] = payload
        if len(chunks) == total:
            done = True
            with state_lock:
                stat["ok"] += 1
            push_latest(b"".join(chunks[i] for i in range(total)))


class Enhancer:

    def __init__(self):
        self.enhance = False
        self.clahe_on = False
        self.upscale = False
        self.prev = None
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    def _temporal_denoise(self, img):
        f = img.astype(np.float32)
        if self.prev is None or self.prev.shape != f.shape:
            self.prev = f
            return img
        diff = cv2.GaussianBlur(cv2.absdiff(f, self.prev).mean(axis=2), (0, 0), 2)
        motion = np.clip((diff - 4.0) / 12.0, 0.0, 1.0)[..., None]
        w = 0.35 + 0.65 * motion
        out = self.prev + (f - self.prev) * w
        self.prev = out
        return np.clip(out, 0, 255).astype(np.uint8)

    @staticmethod
    def _unsharp(img, amount=0.6, sigma=1.2):
        blur = cv2.GaussianBlur(img, (0, 0), sigma)
        return cv2.addWeighted(img, 1.0 + amount, blur, -amount, 0)

    def process(self, img):
        if self.enhance:
            img = self._temporal_denoise(img)
            img = cv2.bilateralFilter(img, 5, 20, 5)
        else:
            self.prev = None
        if self.clahe_on:
            lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            img = cv2.cvtColor(cv2.merge((self.clahe.apply(l), a, b)), cv2.COLOR_LAB2BGR)
        if self.upscale:
            img = cv2.resize(img, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
        if self.enhance or self.upscale:
            img = self._unsharp(img)
        return img


def main():
    threading.Thread(target=keepalive, daemon=True).start()
    threading.Thread(target=net_loop, daemon=True).start()

    if MANUAL_IP:
        print(f"[*] Ket noi toi ESP32 tai {MANUAL_IP}:{ESP_PORT}")
    else:
        print(f"[*] Dang tu dong do tim ESP32 tren mang WiFi (port {ESP_PORT})...")
    print("[*] ESC thoat | f fullscreen | e tang chat luong | c CLAHE | u phong to 2x")

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    enh = Enhancer()
    fullscreen = False
    t0 = time.time()

    while True:
        try:
            jpg = frame_q.get(timeout=0.2)
        except queue.Empty:
            jpg = None

        if jpg is not None:
            img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                with state_lock:
                    stat["badjpg"] += 1
            else:
                cv2.imshow(WINDOW, enh.process(img))
                with state_lock:
                    stat["shown"] += 1

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        elif key == ord("f"):
            fullscreen = not fullscreen
            cv2.setWindowProperty(
                WINDOW,
                cv2.WND_PROP_FULLSCREEN,
                cv2.WINDOW_FULLSCREEN if fullscreen else cv2.WINDOW_NORMAL,
            )
        elif key == ord("e"):
            enh.enhance = not enh.enhance
            print(f"[*] Tang chat luong: {'BAT' if enh.enhance else 'TAT'}")
        elif key == ord("c"):
            enh.clahe_on = not enh.clahe_on
            print(f"[*] CLAHE: {'BAT' if enh.clahe_on else 'TAT'}")
        elif key == ord("u"):
            enh.upscale = not enh.upscale
            print(f"[*] Phong to 2x: {'BAT' if enh.upscale else 'TAT'}")

        elapsed = time.time() - t0
        if elapsed > 5:
            with state_lock:
                s = dict(stat)
                stat["shown"] = 0
                ip = target_ip or "chua ro"
            print(
                f"[ESP32 {ip}] hien thi {s['shown'] / elapsed:.1f} FPS | "
                f"tong frame day du {s['ok']} | mat goi {s['incomplete']} | bo (queue) {s['qdrop']} | JPEG loi {s['badjpg']}"
            )
            t0 = time.time()

    stop_event.set()
    cv2.destroyAllWindows()
    sock.close()


if __name__ == "__main__":
    main()