#!/usr/bin/env python3

import math
import socket
import sys
import threading
import time

DEFAULT_HOST = "192.168.1.200"
FALLBACK_IP = "192.168.4.1"
DEFAULT_PORT = 4210
MAX_V = 3600
MAX_W = 90
DEAD = 0.18
SEND_MS = 20
PING_S = 0.5
RX_LOST_S = 1.5


def resolve_ip(target):
    if not target or target == "auto":
        return DEFAULT_HOST
    parts = target.split(".")
    if len(parts) == 4 and all(p.isdigit() for p in parts):
        return target
    try:
        return socket.gethostbyname(target)
    except Exception:
        return FALLBACK_IP


class BalanceBotUDP:

    def __init__(self, ip=None, port=DEFAULT_PORT):
        self.target = ip or DEFAULT_HOST
        self.ip = resolve_ip(self.target)
        self.port = port
        self.seq = 0
        self.running = False
        self.claimed = False
        self.sock = None
        self.rx_thread = None
        self.lock = threading.Lock()
        self.telemetry = {
            "armed": False,
            "autoArm": False,
            "ready": False,
            "moving": False,
            "pitch": 0.0,
            "v": 0.0,
            "yawErr": 0.0,
            "last_rx": 0.0,
            "connected": False,
        }
        self._init_socket()

    def _init_socket(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

    def set_ip(self, ip):
        if ip and ip != self.ip:
            with self.lock:
                self.ip = ip
            if self.claimed:
                self.send("H")

    def next_seq(self):
        with self.lock:
            self.seq += 1
            return self.seq

    def send(self, msg):
        try:
            self.sock.sendto(msg.encode("utf-8"), (self.ip, self.port))
        except OSError:
            pass

    def connect(self, claim=True):
        if not self.running:
            self.running = True
            self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self.rx_thread.start()
        if claim:
            self.claim()

    def claim(self):
        self.claimed = True
        self.send("H")

    def release(self):
        if self.claimed:
            self.stop()
        self.claimed = False

    def _rx_loop(self):
        last_hello = time.time()
        while self.running:
            now = time.time()
            if self.telemetry["connected"] and now - self.telemetry["last_rx"] > RX_LOST_S:
                self.telemetry["connected"] = False
            if self.claimed and not self.telemetry["connected"] and now - last_hello > 3.0:
                new_ip = resolve_ip(self.target) if self.target != self.ip else self.ip
                if new_ip != self.ip:
                    self.ip = new_ip
                last_hello = now
                self.send("H")

            try:
                data, addr = self.sock.recvfrom(256)
                p = data.decode("utf-8", errors="ignore").split(",")
                if p[0] == "T" and len(p) >= 8:
                    with self.lock:
                        self.ip = addr[0]
                        self.telemetry["armed"] = (p[1] == "1")
                        self.telemetry["autoArm"] = (p[2] == "1")
                        self.telemetry["ready"] = (p[3] == "1")
                        self.telemetry["moving"] = (p[4] == "1")
                        self.telemetry["pitch"] = float(p[5])
                        self.telemetry["v"] = float(p[6])
                        self.telemetry["yawErr"] = float(p[7])
                        self.telemetry["last_rx"] = time.time()
                        self.telemetry["connected"] = True
                    continue
            except (BlockingIOError, OSError, ValueError):
                pass
            time.sleep(0.005)

    def drive(self, v, w):
        v = max(min(round(v), MAX_V), -MAX_V)
        w = max(min(round(w), MAX_W), -MAX_W)
        sq = self.next_seq()
        self.send(f"D,{sq},{v},{w}")

    def stop(self):
        sq = self.next_seq()
        self.send(f"S,{sq}")

    def arm(self):
        sq = self.next_seq()
        self.send(f"A,{sq}")

    def disarm(self):
        for _ in range(3):
            self.send("X")

    def ping(self):
        self.send("P")

    def close(self):
        if self.claimed:
            self.stop()
        self.claimed = False
        self.running = False
        if self.rx_thread and self.rx_thread.is_alive():
            self.rx_thread.join(timeout=0.5)
        if self.sock:
            self.sock.close()


# ==================== GUI TKINTER (KHI CHAY TRUC TIEP) ====================
def run_gui(target=None):
    import tkinter as tk

    bot = BalanceBotUDP(target)
    bot.connect()

    root = tk.Tk()
    root.title("BalanceBot UDP Controller")
    status = tk.StringVar(value=f"Dang ket noi: {bot.ip}:{DEFAULT_PORT}...")
    cmd_text = tk.StringVar(value="v=0  w=0")

    top_frame = tk.Frame(root)
    top_frame.pack(fill="x", padx=12, pady=(8, 2))
    tk.Label(top_frame, text=f"Xe: {bot.ip}:{DEFAULT_PORT}", font=("TkDefaultFont", 9)).pack(side="left")
    tk.Label(top_frame, text=f"Web: http://{bot.ip}", font=("TkDefaultFont", 9, "italic"), fg="#0d6efd").pack(side="right")

    tk.Label(root, textvariable=status, font=("TkDefaultFont", 10, "bold")).pack(pady=(2, 2))
    tk.Label(root, textvariable=cmd_text).pack()

    SIZE = 300
    R = SIZE // 2 - 24
    canvas = tk.Canvas(root, width=SIZE, height=SIZE, bg="#f2f5f4", highlightthickness=0)
    canvas.pack(padx=12, pady=6)
    c = SIZE // 2
    canvas.create_oval(c - R, c - R, c + R, c + R, outline="#bcd0c7", width=2)
    canvas.create_oval(c - R * DEAD, c - R * DEAD, c + R * DEAD, c + R * DEAD, outline="#cfe0d8", dash=(4, 3))
    knob_id = canvas.create_oval(c - 34, c - 34, c + 34, c + 34, fill="white", outline="#12775c", width=2)

    bar = tk.Frame(root)
    bar.pack(pady=(0, 10))
    tk.Button(bar, text="ARM", width=9, bg="#12775c", fg="white", command=bot.arm).pack(side="left", padx=4)
    tk.Button(bar, text="DUNG", width=9, command=bot.stop).pack(side="left", padx=4)
    tk.Button(bar, text="DISARM", width=9, bg="#b83b38", fg="white", command=bot.disarm).pack(side="left", padx=4)

    knob_pos = [0.0, 0.0]
    is_pressed = [False]
    is_driving = [False]
    last_tx = [0.0]

    def stick_values():
        dx, dy = knob_pos[0], knob_pos[1]
        n = min(math.hypot(dx, dy) / R, 1.0)
        if n < DEAD:
            return 0, 0
        t = (n - DEAD) / (1 - DEAD)
        ang = math.atan2(-dx, -dy)
        return round(t * math.cos(ang) * MAX_V), round(t * math.sin(ang) * MAX_W)

    def move(e):
        dx, dy = e.x - c, e.y - c
        d = math.hypot(dx, dy)
        if d > R:
            dx, dy = dx / d * R, dy / d * R
        knob_pos[0], knob_pos[1] = dx, dy
        canvas.coords(knob_id, c + dx - 34, c + dy - 34, c + dx + 34, c + dy + 34)

    def press(e):
        is_pressed[0] = True
        move(e)

    def release(_):
        is_pressed[0] = False
        knob_pos[0], knob_pos[1] = 0.0, 0.0
        canvas.coords(knob_id, c - 34, c - 34, c + 34, c + 34)

    canvas.bind("<ButtonPress-1>", press)
    canvas.bind("<B1-Motion>", lambda e: move(e) if is_pressed[0] else None)
    canvas.bind("<ButtonRelease-1>", release)

    def tick():
        now = time.time()
        v, w = stick_values() if is_pressed[0] else (0, 0)
        if v != 0 or w != 0:
            bot.drive(v, w)
            is_driving[0] = True
            last_tx[0] = now
            cmd_text.set(f"v={v}  w={w}")
        else:
            if is_driving[0]:
                bot.stop()
                bot.stop()
                is_driving[0] = False
                cmd_text.set("v=0  w=0")
            elif now - last_tx[0] >= PING_S:
                bot.ping()
                last_tx[0] = now

        t = bot.telemetry
        if now - t["last_rx"] < 2.0:
            mode = "ARM" if t["armed"] else ("Cho tu ARM" if t["autoArm"] else "DISARM")
            status.set(f"{mode} | Pitch: {t['pitch']:.1f}° | V: {t['v']:.0f} | YawErr: {t['yawErr']:.1f}°")
        else:
            status.set(f"Dang tim xe: {bot.ip}:{DEFAULT_PORT}...")

        root.after(SEND_MS, tick)

    root.protocol("WM_DELETE_WINDOW", lambda: (bot.close(), root.destroy()))
    tick()
    root.mainloop()


if __name__ == "__main__":
    run_gui(sys.argv[1] if len(sys.argv) > 1 else None)
