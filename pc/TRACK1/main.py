import os
import time
import queue
import subprocess
import sys
import threading
import cv2
import numpy as np

if sys.platform.startswith("win"):
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    if hasattr(sys.stderr, "reconfigure"):
        sys.stderr.reconfigure(encoding="utf-8")
import _0_config as cfg
from _1_camera_stream import ThreadedCamera, PcCamera
from _2_ai_models import init_ai_models, save_encodings, sanitize_name
from _3_face_filters import head_pose, is_sharp_face
from _4_async_worker import init_async_face_thread, face_job_queue, face_result_queue
from _5_tracking_utils import (
    match_face_to_person, get_box_center, capture_multi_angle_encodings_10s,
)
from _5_color_match import extract_all_features, compute_pursuit_score, is_good_pose_for_scan
from _6_draw_stick import draw_stick_figure
from _8_gesture import GestureClassifier, KeypointSmoother
from _7_kal_fil import TrajectoryPredictor, box_overlap_ratio, gate_radius
from _9_facing import (raw_view, view_with_motion, FacingEstimator, arms_for, CONCLUSIVE,
                       FRONT, BACK, SIDE_R, SIDE_L, UNK, VIEW_TEXT)
from _10_robot_link import RobotLink
from _11_motion import FollowController, SearchPlanner, GestureManeuver

from _5_color_match import HIST_BINS


def _text_k(img):
    return cfg.UI_TEXT_SCALE * img.shape[1] / 400.0


def _put_text(img, text, org, font, scale, color, thickness=1, *args, **kwargs):
    return cv2.putText(img, text, org, font, scale * _text_k(img), color, 1, *args, **kwargs)


def _put_label(img, text, x, y, bg, fg, scale=0.45):
    sc = scale * _text_k(img)
    (tw, th), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, sc, 1)
    y = max(th + base + 4, y)
    cv2.rectangle(img, (x, y - th - base - 4), (x + tw + 6, y), bg, -1)
    cv2.putText(img, text, (x + 3, y - base - 2), cv2.FONT_HERSHEY_SIMPLEX, sc, fg, 1)


# ── 4 chế độ điều khiển ──
MODE_MANUAL, MODE_GESTURE, MODE_FOLLOW, MODE_CAMERA = 1, 2, 3, 4
MODE_NAMES = {MODE_MANUAL: "TAY (web / can gat UDP)", MODE_GESTURE: "CU CHI (AI)",
              MODE_FOLLOW: "BAM NGUOI (AI)", MODE_CAMERA: "CAM KHONG AI (do duong)"}
MODE_ALIASES = {"1": 1, "tay": 1, "manual": 1, "2": 2, "cuchi": 2, "cu chi": 2, "gesture": 2,
                "3": 3, "bam": 3, "bamnguoi": 3, "bam nguoi": 3, "follow": 3, "4": 4, "cam": 4, "camera": 4}
JOYSTICK_SCRIPT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "balancebot_udp.py")
SPEED_TB, TURN_TB = "Toc do tick/s", "Xoay do/s"
TERMINAL_HELP = """
================= DOI CHE DO (go roi Enter) =================
  1 | tay     : dieu khien TAY (web / cua so can gat UDP), tat camera + AI
  2 | cuchi   : dieu khien bang CU CHI tay (AI)
  3 | bam     : BAM NGUOI (AI), mat dau thi quay 30 do moi 6 s
  4 | cam     : chi xem CAMERA (khong AI) de do duong, lai bang can gat / web
  y : (che do 2, 3) chuyen tu WEBCAM MAY TINH sang CAM ESP de track (truoc do xe dung yen)
  r : bo chu, quay lai tim chu bang webcam | x : DISARM khan cap | a : ARM | q : thoat | h : tro giup
  (Cua so video: phim 1-4, X van dung; N, R, S, Q giu nguyen nhu cu)
=============================================================="""

def _anchor(box, kpts):
    if kpts is not None and len(kpts) > 6 and kpts[5][2] >= cfg.ANCHOR_KPT_CONF and kpts[6][2] >= cfg.ANCHOR_KPT_CONF:
        return (int((kpts[5][0] + kpts[6][0]) / 2), int((kpts[5][1] + kpts[6][1]) / 2))
    return get_box_center(box)

def _scale_proxy(box, kpts, frame_h):
    if kpts is not None and len(kpts) > 6 and kpts[5][2] >= cfg.ANCHOR_KPT_CONF and kpts[6][2] >= cfg.ANCHOR_KPT_CONF:
        if box[3] < frame_h - 3:
            return max(float(box[3]) - (float(kpts[5][1]) + float(kpts[6][1])) / 2.0, 1.0), 'd'
        return max(float(np.hypot(kpts[5][0] - kpts[6][0], kpts[5][1] - kpts[6][1])), 1.0), 'w'
    return float(max(box[3] - box[1], 1)), 'h'


class RobotTrackerPipeline:

    def __init__(self, target_ip=None, start_mode=None):
        self.MAIN_WIN_NAME = "All-In-One Robot System"
        cv2.namedWindow(self.MAIN_WIN_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.MAIN_WIN_NAME, 960, 540)

        self.frame_h = 240
        self.frame_w = 320

        self.yolo_pose, self.detector, self.known_face_encodings = init_ai_models()
        init_async_face_thread(self.known_face_encodings)
        start_mode = start_mode if start_mode in MODE_NAMES else cfg.START_MODE
        self.esp_stream = ThreadedCamera(target_ip=target_ip, streaming=False).start()
        self.pc_cam = None
        self.cam_stream = self.esp_stream
        self.source = "pc"
        time.sleep(0.5)

        self.target_track_id = None
        self.target_name = ""
        self.frame_count = 0
        self.lost_start_time = None
        self.master_features = None
        self.last_known_center = None

        self.is_typing_name = False
        self.typed_name = ""
        self._face_dbg = []

        self.STEP_BACK_LOST_TIMEOUT = cfg.STEP_BACK_LOST_TIMEOUT

        self.preview_rois = {}
        self.preview_colors = {}

        self.current_persons = []
        self.person_kpts = {}

        self.stick_mode = False
        self.gesture = GestureClassifier()
        self.kp_smoother = KeypointSmoother()
        self._kp_smooth = None
        self._armband_ok_until = 0.0
        self.robot_command = "DUNG"

        self.kalman_mode = True
        self.predictor = TrajectoryPredictor()

        self._last_valid_pose_time = None
        self.WATCHDOG_TIMEOUT = cfg.WATCHDOG_TIMEOUT

        self._epoch = 0

        self._lock_confirm_count = 0
        self._lock_confirm_tid = None
        self.LOCK_CONFIRM_NEEDED = cfg.LOCK_CONFIRM_NEEDED
        self._lock_confirm_name = None
        self._lock_confirm_last_time = 0.0
        self.LOCK_CONFIRM_WINDOW = cfg.LOCK_CONFIRM_WINDOW

        self._pending_cmd = None
        self._pending_n = 0
        self.CMD_CONFIRM_NEEDED = cfg.CMD_CONFIRM_NEEDED

        self._yolo_ran_this_frame = False
        self._last_frame_id = -1

        self.YOLO_CONF = cfg.YOLO_CONF
        self.YOLO_IOU = cfg.YOLO_IOU

        self._hist_cache = {}

        self._sample_buf = {}
        self._sample_roi = {}

        self._kal_occluded = True
        self._kal_direction = "Dung im"
        self._target_armband_visible = False
        self.KAL_DIR_THRESH = cfg.KAL_DIR_THRESH

        self._arm_mismatch = {}
        self._crossing = False
        self._cross_start = None
        self._need_reconfirm = False
        self._measure_ok = False

        self.facing = FacingEstimator()
        self._view_cache = {}

        self._pick_reason = None
        self._color_rej = 0
        self._match_tid, self._match_n = None, 0
        self._last_good_time = None
        self._last_box_h = 0

        self._last_ok = None
        self._known_others = {}
        self._contested = False
        self._reacq_tid, self._reacq_n, self._reacq_h = None, 0, 0.0
        self._reacq_note = ""
        self._loss_gate_tids = set()

        # ===== Chế độ điều khiển + gửi lệnh xuống xe (thêm mới; không đụng logic nhận diện / bám) =====
        self.robot = RobotLink(ip_provider=lambda: self.esp_stream.target_ip)
        self.follow = FollowController()
        self.search = SearchPlanner()
        self.maneuver = GestureManeuver()
        self._saved_owner = None
        self._handover = False
        self._handover_colors = False
        self._handover_tid, self._handover_n, self._handover_note = None, 0, ""
        self._init_sliders()
        self._joy_proc = None
        self._term_q = queue.Queue()
        self._quit = False
        self.mode = None
        self._apply_mode(start_mode)
        threading.Thread(target=self._terminal_loop, daemon=True).start()

    def reset_target_state(self):
        self.target_track_id = None
        self.target_name = ""
        self.master_features = None
        self.last_known_center = None
        self.lost_start_time = None
        self.preview_rois = {}
        self.preview_colors = {}
        self._last_valid_pose_time = None
        self._kal_occluded = True
        self._kal_direction = "Dung im"
        self._lock_confirm_count = 0
        self._lock_confirm_tid = None
        self._lock_confirm_name = None
        self._pending_cmd, self._pending_n = None, 0

        self.robot_command = "DUNG"
        self._hist_cache = {}
        self._sample_buf = {}
        self._sample_roi = {}
        self._target_armband_visible = False
        self._armband_ok_until = 0.0
        self.gesture.reset()
        self._arm_mismatch = {}
        self._crossing = False
        self._cross_start = None
        self._need_reconfirm = False
        self._measure_ok = False
        self.facing.reset()
        self.kp_smoother.reset()
        self._kp_smooth = None
        self._view_cache = {}
        self._pick_reason = None
        self._color_rej = 0
        self._match_tid, self._match_n = None, 0
        self._last_good_time = None
        self._last_box_h = 0
        self._last_ok = None
        self._known_others = {}
        self._contested = False
        self._reacq_tid, self._reacq_n, self._reacq_h = None, 0, 0.0
        self._reacq_note = ""
        self._loss_gate_tids = set()

        self._epoch += 1
        self.predictor.reset()
        _flush_queue(face_job_queue)
        _flush_queue(face_result_queue)
        self.follow.reset()
        self.search.reset()
        self.maneuver.abort(self.robot)
        self.robot.cancel_action()
        self.robot.stop()

    def run(self):
        while not self._quit:
            self._poll_terminal()
            if self._quit:
                break
            if self.mode == MODE_MANUAL:
                if self._manual_screen():
                    break
                continue
            ret, frame, fid = self.cam_stream.read_latest(self._last_frame_id)
            if ret and frame is None:
                time.sleep(0.001)
                continue
            if not ret or frame is None:
                self._set_command("DUNG")
                self.robot.cancel_action()
                self.maneuver.abort(self.robot)
                self.robot.stop()
                h_err = self.frame_h if hasattr(self, 'frame_h') and self.frame_h else 240
                w_err = self.frame_w if hasattr(self, 'frame_w') and self.frame_w else 320
                error_screen = np.zeros((h_err, w_err, 3), dtype=np.uint8)
                cx, cy = w_err // 2, h_err // 2
                pc_src = self.pc_cam is not None and self.cam_stream is self.pc_cam
                _put_text(error_screen, "KHONG CO HINH WEBCAM MAY TINH!" if pc_src else "MAT KET NOI CAMERA ESP32!",
                            (max(10, cx - 130), max(30, cy - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
                _put_text(error_screen, "Kiem tra webcam | 'Y' dung cam ESP | 'Q' thoat" if pc_src else
                            "Dang tu dong tim lai qua UDP... | 'Q' thoat",
                            (max(10, cx - 145), max(50, cy + 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                cv2.imshow(self.MAIN_WIN_NAME, error_screen)
                key = cv2.waitKey(100) & 0xFF
                if self._mode_key(key):
                    break
                continue

            self._last_frame_id = fid
            self.frame_count += 1
            self.frame_h, self.frame_w = frame.shape[:2]
            display_frame = frame.copy()
            if self.mode == MODE_CAMERA:
                if self._camera_screen(display_frame):
                    break
                continue

            self._yolo_ran_this_frame = False

            if not self.is_typing_name:
                results = self.yolo_pose.track(frame, persist=True, classes=0, verbose=False, imgsz=(cfg.YOLO_IMGSZ_STICK if self.stick_mode else cfg.YOLO_IMGSZ), iou=self.YOLO_IOU, conf=self.YOLO_CONF)
                self._yolo_ran_this_frame = True
                self._measure_ok = False
                self._hist_cache = {}
                self._view_cache = {}
                raw_persons = []
                self.person_kpts = {}

                if results[0].boxes is not None and results[0].boxes.id is not None:
                    boxes = results[0].boxes.xyxy.cpu().numpy()
                    track_ids = results[0].boxes.id.int().cpu().numpy()
                    kpts_data = results[0].keypoints.data.cpu().numpy() if results[0].keypoints is not None else []

                    for i, (box, t_id) in enumerate(zip(boxes, track_ids)):
                        raw_persons.append((t_id, list(map(int, box))))
                        if len(kpts_data) > i:
                            self.person_kpts[t_id] = kpts_data[i]

                self.current_persons = [
                    (t_id, box) for t_id, box in raw_persons
                    if t_id in self.person_kpts
                ]

                if self.source == "esp" and self.target_track_id is None and self._saved_owner is not None:
                    self._handover = True
                # ---------- BÀN GIAO CHỦ TỪ WEBCAM SANG CAM ESP (sau khi bấm Y) ----------
                if self._handover:
                    self._handover_step(frame)
                # ---------- TÌM CHỦ NHÂN MỚI (BẰNG MẶT) ----------
                elif self.target_track_id is None:
                    self.preview_rois = {}
                    self.preview_colors = {}

                    if len(self.current_persons) > 0:
                        h_img, w_img = frame.shape[:2]
                        self.detector.setInputSize((w_img, h_img))
                        _, faces = self.detector.detect(frame)
                        self._face_dbg = []

                        if faces is not None and len(faces) > 0:
                            for face in faces:
                                fbox = face[:4].astype(int)
                                fx, fy, fw, fh = fbox
                                if fw < 10 or fh < 10:
                                    continue

                                is_sharp, blur_score = is_sharp_face(frame, (fx, fy, fw, fh))
                                if not is_sharp:
                                    continue

                                fcx, fcy = fx + fw // 2, fy + fh // 2

                                pose = head_pose(face)
                                is_profile = pose is not None and pose['label'] != 'thang'
                                if cfg.HEAD_POSE_DEBUG and pose is not None:
                                    self._face_dbg.append((fx, fy, pose))

                                matched_tid = match_face_to_person(
                                    (fcx, fcy), self.current_persons
                                )

                                if matched_tid is not None and face_job_queue.empty():
                                    try:
                                        face_job_queue.put_nowait(
                                            (frame, (fx, fy, fw, fh), matched_tid, is_profile, self._epoch)
                                        )
                                    except queue.Full:
                                        pass

                    while not face_result_queue.empty():
                        try:
                            result = face_result_queue.get_nowait()
                        except queue.Empty:
                            break

                        if len(result) == 5:
                            name, _, _, matched_tid, job_epoch = result
                        else:
                            name, _, _, matched_tid = result
                            job_epoch = -1

                        if job_epoch >= 0 and job_epoch != self._epoch:
                            continue

                        if name is not None:
                            now_t = time.time()
                            same = (self._lock_confirm_tid == matched_tid
                                    and self._lock_confirm_name == name
                                    and now_t - self._lock_confirm_last_time <= self.LOCK_CONFIRM_WINDOW)
                            self._lock_confirm_count = self._lock_confirm_count + 1 if same else 1
                            self._lock_confirm_tid = matched_tid
                            self._lock_confirm_name = name
                            self._lock_confirm_last_time = now_t

                            if self._lock_confirm_count >= self.LOCK_CONFIRM_NEEDED:
                                self.target_track_id = matched_tid
                                self.target_name = name
                                self.lost_start_time = None
                                self.master_features = None
                                self._lock_confirm_count = 0
                                self._lock_confirm_tid = None
                                self._lock_confirm_name = None

                                _flush_queue(face_job_queue)
                                _flush_queue(face_result_queue)
                                break
                        else:
                            if self._lock_confirm_tid == matched_tid:
                                self._lock_confirm_count = 0
                                self._lock_confirm_tid = None
                                self._lock_confirm_name = None

                # ---------- ĐÃ KHÓA CHỦ NHÂN BẰNG MẶT ----------
                else:
                    _flush_queue(face_job_queue)
                    self._face_dbg = []

                    if self.master_features is None:
                        self._sample_shirt_after_lock(frame)
                    else:
                        self._track_with_memory(frame)

            # ===== HIỂN THỊ GIAO DIỆN =====
            if (self.stick_mode or self.mode == MODE_GESTURE) and not self.is_typing_name:
                self._handle_gesture(display_frame)

            self._watchdog_check()

            # --- KALMAN ---
            if self.kalman_mode and self.target_track_id is not None:
                pred = self.predictor.predict()

                if self._yolo_ran_this_frame:
                    target_person = next((p for p in self.current_persons if p[0] == self.target_track_id), None)
                    if self._measure_ok and target_person is not None:
                        self._kalman_measure(target_person, display_frame.shape[0])
                    self._kal_occluded = not self._measure_ok

                    v_x, v_y = self.predictor.velocity
                    thr, direction = self.KAL_DIR_THRESH, "Dung im"
                    if abs(v_x) > thr or abs(v_y) > thr:
                        if abs(v_x) > abs(v_y):
                            direction = "Phai" if v_x > 0 else "Trai"
                        else:
                            direction = "Xuong" if v_y > 0 else "Len"
                    self._kal_direction = direction

                h_img, w_img = display_frame.shape[:2]
                if self._crossing:
                    status_text, status_color = "Status: Dang cat nhau - giu Kalman", (0, 165, 255)
                elif self._kal_occluded:
                    status_text, status_color = "Status: Bi che khuat", (0, 0, 255)
                else:
                    status_text, status_color = "Status: Dang tracking", (0, 255, 0)
                _put_text(display_frame, status_text, (w_img - 170, h_img - 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 2)
                _put_text(display_frame, f"Kal: {self._kal_direction}", (w_img - 170, h_img - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                if pred is not None:
                    cv2.circle(display_frame, pred, 8, (255, 0, 255), -1)
                    _put_text(display_frame, "Kalman", (pred[0] + 10, pred[1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)

            # --- HIỂN THỊ TRẠNG THÁI ---
            if self._handover:
                _put_text(display_frame, f"CAM ESP: NHAN LAI CHU {self._saved_owner[0].upper()} - {self._handover_note}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            elif self.target_track_id is None:
                _put_text(display_frame, "TRANG THAI: TIM CHU NHAN...",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
            else:
                if self.master_features is None:
                    _put_text(display_frame,
                                f"[DA NHAN DIEN: {self.target_name.upper()}] DANG QUET AO - HAY LUI RA DE LO AO...",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
                else:
                    if self.lost_start_time is None:
                        msg = f"TRUY TIM CHU NHAN ({self.target_name.upper()})" + self._missing_hint()
                        color = (0, 255, 0)
                    else:
                        msg = "KHONG THAY CHU NHAN - DANG TIM LAI"
                        color = (0, 165, 255)
                    _put_text(display_frame, msg, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

            dbg_txt = self._track_debug_text()
            if dbg_txt:
                _put_text(display_frame, dbg_txt, (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)

            if cfg.HEAD_POSE_DEBUG:
                for fx, fy, pose in self._face_dbg:
                    _put_text(display_frame,
                                f"{pose['label']} y{pose['yaw']:.2f} p{pose['pitch']:.2f} r{pose['roll']:.0f}",
                                (fx, max(15, fy - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

            self._draw_color_chips(display_frame)

            for t_id, [x1, y1, x2, y2] in self.current_persons:
                if self.target_track_id is not None and t_id == self.target_track_id and self.lost_start_time is None:
                    if self._target_armband_visible:
                        owner_color = (0, 255, 0)
                        lbl = f"CHU NHAN: {self.target_name} [CO BANG TAY]"
                    else:
                        owner_color = (0, 165, 255)
                        lbl = f"CHU NHAN: {self.target_name}"

                    cv2.rectangle(display_frame, (x1, y1), (x2, y2), owner_color, 2)
                    _put_label(display_frame, lbl, x1, y1, owner_color, (0, 0, 0), 0.5)
                else:
                    purple_color = (255, 0, 255)
                    cv2.rectangle(display_frame, (x1, y1), (x2, y2), purple_color, 2)
                    lbl = f"NGUOI LA (ID:{t_id})"
                    _put_label(display_frame, lbl, x1, y1, purple_color, (255, 255, 255), 0.45)

            if self.is_typing_name:
                h, w = display_frame.shape[:2]
                cv2.rectangle(display_frame, (50, h // 2 - 60), (w - 50, h // 2 + 60), (0, 0, 0), -1)
                cv2.rectangle(display_frame, (50, h // 2 - 60), (w - 50, h // 2 + 60), (0, 255, 255), 2)
                _put_text(display_frame, "NHAP TEN NGUOI DUNG MOI (ENTER: OK | ESC: HUY):",
                            (70, h // 2 - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                _put_text(display_frame, f"> {self.typed_name}_",
                            (70, h // 2 + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

            self._drive_robot(display_frame)

            cv2.imshow(self.MAIN_WIN_NAME, display_frame)
            key = cv2.waitKey(1) & 0xFF

            if not self.is_typing_name:
                if key in [ord("n"), ord("N")]:
                    self.is_typing_name = True
                    self.typed_name = ""
                elif key in [ord("r"), ord("R")]:
                    self._restart_owner()
                elif key in [ord("y"), ord("Y")]:
                    self._switch_to_esp()
                elif key in [ord("s"), ord("S")]:
                    if self.mode != MODE_GESTURE:
                        self._apply_mode(MODE_GESTURE)
                    else:
                        self.stick_mode = not self.stick_mode
                        self.gesture.reset()
                        self.kp_smoother.reset()
                        self._kp_smooth = None
                        self._set_command("DUNG")
                        self._last_valid_pose_time = None
                elif key in [ord("q"), ord("Q")]:
                    break
                elif self._mode_key(key):
                    break
            else:
                if key == 13 and self.typed_name.strip():
                    clean_name = sanitize_name(self.typed_name)
                    self.is_typing_name = False
                    self._set_command("DUNG")
                    self.robot.cancel_action()
                    self.maneuver.abort(self.robot)
                    self.robot.stop()
                    capture_multi_angle_encodings_10s(
                        self.cam_stream, clean_name, self.MAIN_WIN_NAME,
                        self.detector, self.known_face_encodings, save_encodings
                    )
                elif key == 8:
                    self.typed_name = self.typed_name[:-1]
                elif key == 27:
                    self.is_typing_name = False
                    self.typed_name = ""
                elif 32 <= key <= 126:
                    self.typed_name += chr(key)

        self._set_command("DUNG")
        self._close_joystick()
        self.robot.release()
        self.robot.close()
        self.esp_stream.stop()
        if self.pc_cam is not None:
            self.pc_cam.stop()
        cv2.destroyAllWindows()
        print("Thoát hệ thống thành công!")

    # ------------------------------------------------------------------
    #  CHẾ ĐỘ ĐIỀU KHIỂN + GỬI LỆNH XUỐNG XE (thêm mới)
    # ------------------------------------------------------------------
    def _apply_mode(self, m):
        if m not in MODE_NAMES or m == self.mode:
            return
        self.mode = m
        self.robot.cancel_action()
        self.maneuver.abort(self.robot)
        self.robot.stop()
        self.follow.reset()
        self.search.reset()
        self._set_command("DUNG")
        self.gesture.reset()
        self.kp_smoother.reset()
        self._kp_smooth = None
        self._last_valid_pose_time = None
        self.stick_mode = (m == MODE_GESTURE)
        self._update_sources()
        if m in (MODE_GESTURE, MODE_FOLLOW):
            self._close_joystick()
            self.robot.claim()
        else:
            self.robot.release()
            self._open_joystick()
        print(f"\n>>> CHE DO {m}: {MODE_NAMES[m]}")
        if m == MODE_FOLLOW:
            print("    Xe bam chu (giu gan tam). Mat dau > %.0f s: quay %d do moi %.0f s." %
                  (cfg.SEARCH_START_DELAY, cfg.SEARCH_TURN_DEG, cfg.SEARCH_PERIOD))
        elif m == MODE_GESTURE:
            print("    Tay phai gio: TIEN 3s | tay trai gio: LUI 3s | dang tay: quay %d do, chay 1s, quay lai | bat cheo: DUNG"
                  % cfg.GESTURE_SIDE_TURN_DEG)
        else:
            print(f"    Web: http://{self.esp_stream.target_ip or cfg.ROBOT_DEFAULT_IP}  (hoac http://192.168.4.1 qua Wi-Fi BalanceBot-Web)")
        if m in (MODE_GESTURE, MODE_FOLLOW):
            print("    " + ("Dang dung WEBCAM MAY TINH (xe dung yen) - bam Y de chuyen sang cam ESP" if self.source == "pc"
                            else "Dang track bang CAM ESP (R: quay lai webcam)"))

    def _open_joystick(self):
        if not cfg.MANUAL_OPEN_JOYSTICK or (self._joy_proc is not None and self._joy_proc.poll() is None):
            return
        ip = self.esp_stream.target_ip or cfg.ROBOT_DEFAULT_IP
        try:
            self._joy_proc = subprocess.Popen([sys.executable, JOYSTICK_SCRIPT, ip])
            print("    Da mo cua so can gat UDP (dong no lai neu muon lai bang web).")
        except Exception as e:
            self._joy_proc = None
            print(f"[!] Khong mo duoc cua so can gat: {e}")

    def _close_joystick(self):
        p, self._joy_proc = self._joy_proc, None
        if p is not None and p.poll() is None:
            p.terminate()
            try:
                p.wait(timeout=1.0)
            except Exception:
                pass

    def _terminal_loop(self):
        print(TERMINAL_HELP)
        while True:
            try:
                line = input()
            except (EOFError, KeyboardInterrupt, OSError, RuntimeError):
                return
            self._term_q.put(line.strip().lower())

    def _poll_terminal(self):
        while True:
            try:
                cmd = self._term_q.get_nowait()
            except queue.Empty:
                return
            if cmd in MODE_ALIASES:
                self._apply_mode(MODE_ALIASES[cmd])
            elif cmd in ("q", "quit", "thoat", "exit"):
                self._quit = True
            elif cmd in ("x", "disarm"):
                self.robot.disarm()
                print("[XE] Da gui DISARM")
            elif cmd in ("a", "arm"):
                if self.robot.claimed:
                    self.robot.arm()
                    print("[XE] Da gui ARM (xe phai dang dung thang)")
                else:
                    print("[XE] Che do 1 / 4: bam ARM tren cua so can gat hoac web")
            elif cmd in ("r", "reset"):
                self._restart_owner()
            elif cmd in ("y", "esp"):
                self._switch_to_esp()
            elif cmd in ("h", "help", "?", ""):
                t = self.robot.telemetry
                print(f"Che do hien tai: {self.mode} - {MODE_NAMES[self.mode]} | xe: "
                      f"{'ARM' if t['armed'] else 'DISARM'} pitch {t['pitch']:+.1f}" if t['connected'] else
                      f"Che do hien tai: {self.mode} - {MODE_NAMES[self.mode]}")
                print(TERMINAL_HELP)
            else:
                print(f"Lenh khong hop le: '{cmd}' (go h de xem tro giup)")

    def _mode_key(self, key):
        if key in (ord("q"), ord("Q")):
            return True
        if ord("1") <= key <= ord("4"):
            self._apply_mode(key - ord("0"))
        elif key in (ord("s"), ord("S")) and self.mode != MODE_GESTURE:
            self._apply_mode(MODE_GESTURE)
        elif key in (ord("y"), ord("Y")):
            self._switch_to_esp()
        elif key in (ord("x"), ord("X")):
            self.robot.disarm()
            print("[XE] Da gui DISARM")
        return False

    def _manual_screen(self):
        panel = np.zeros((230, 600, 3), dtype=np.uint8)
        ip = self.esp_stream.target_ip or cfg.ROBOT_DEFAULT_IP
        joy = self._joy_proc is not None and self._joy_proc.poll() is None
        lines = [("CHE DO 1: DIEU KHIEN TAY (tat camera + AI)", (0, 255, 255), 0.6),
                 (f"Web: http://{ip}   |   Wi-Fi xe: http://192.168.4.1", (255, 255, 255), 0.5),
                 ("Cua so can gat UDP: " + ("DANG MO" if joy else "da dong -> dung web"), (255, 255, 255), 0.5),
                 ("Doi che do: phim 2 / 3 / 4 o day hoac go trong terminal", (200, 200, 200), 0.5),
                 ("X: DISARM khan cap  |  Q: thoat", (200, 200, 200), 0.5)]
        for i, (txt, col, sc) in enumerate(lines):
            _put_text(panel, txt, (15, 40 + i * 40), cv2.FONT_HERSHEY_SIMPLEX, sc, col, 1 if sc < 0.6 else 2)
        cv2.imshow(self.MAIN_WIN_NAME, panel)
        return self._mode_key(cv2.waitKey(50) & 0xFF)

    def _camera_screen(self, frame):
        h = frame.shape[0]
        _put_text(frame, "[4] CAM KHONG AI - lai bang can gat / web", (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        _put_text(frame, "1-4: doi che do | X: DISARM | Q: thoat", (10, h - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        cv2.imshow(self.MAIN_WIN_NAME, frame)
        return self._mode_key(cv2.waitKey(1) & 0xFF)

    # ── Nguồn ảnh: webcam máy tính (tìm / quét chủ) <-> cam ESP (track) ──
    def _update_sources(self):
        ai = self.mode in (MODE_GESTURE, MODE_FOLLOW)
        want_pc = ai and self.source == "pc"
        want_esp = self.mode == MODE_CAMERA or (ai and self.source == "esp")
        self.esp_stream.set_streaming(want_esp)
        if want_pc and self.pc_cam is None:
            self.pc_cam = PcCamera(cfg.PC_CAM_INDEX, cfg.PC_CAM_WIDTH, cfg.PC_CAM_HEIGHT).start()
        elif not want_pc and self.pc_cam is not None:
            self.pc_cam.stop()
            self.pc_cam = None
        new = self.pc_cam if want_pc else self.esp_stream
        if new is not self.cam_stream:
            self.cam_stream = new
            self._last_frame_id = -1
            self._reset_yolo_tracker()

    def _reset_yolo_tracker(self):
        pred = getattr(self.yolo_pose, "predictor", None)
        for t in (getattr(pred, "trackers", None) or []):
            try:
                t.reset()
            except Exception:
                pass

    def _switch_to_esp(self):
        if self.mode not in (MODE_GESTURE, MODE_FOLLOW):
            print("[Y] Chi dung o che do 2 / 3")
            return
        if self.source == "esp":
            print("[Y] Dang track bang cam ESP roi (R: bo chu, quay lai webcam)")
            return
        has_owner = self.target_track_id is not None
        feats = self.master_features
        self._saved_owner = (self.target_name, feats) if has_owner else None
        self._handover_colors = has_owner and feats is not None and not cfg.HANDOVER_RESCAN_COLORS
        self.reset_target_state()
        self._handover = has_owner
        self._handover_tid, self._handover_n, self._handover_note = None, 0, "dung 1 MINH truoc cam ESP"
        self.source = "esp"
        self._update_sources()
        if has_owner:
            print(f"[Y] Chuyen sang CAM ESP: dung 1 MINH truoc xe de nhan lai chu '{self._saved_owner[0]}'"
                  + (" (giu mau tu webcam)" if self._handover_colors else " (se quet lai mau ao tren cam ESP)"))
        else:
            print("[Y] Chuyen sang CAM ESP: chua co chu -> tim chu bang mat tren cam ESP nhu cu")

    def _restart_owner(self):
        self.reset_target_state()
        self._saved_owner = None
        self._handover = False
        if self.source != "pc":
            self.source = "pc"
            self._update_sources()
        print("[AI] Bo chu - tim / quet chu lai bang WEBCAM MAY TINH (xe dung yen; Y de chuyen sang cam ESP)")

    def _handover_step(self, frame):
        name, feats = self._saved_owner
        if len(self.current_persons) != 1:
            self._handover_tid, self._handover_n = None, 0
            self._handover_note = "can DUNG 1 NGUOI truoc cam ESP" if self.current_persons else "chua thay ai"
            return
        tid, box = self.current_persons[0]
        kpts = self.person_kpts.get(tid)
        if self._handover_colors and feats is not None:
            view, _ = raw_view(kpts, 0.0)
            got = extract_all_features(frame, kpts, only=tuple(feats.keys()), track=True,
                                       arms=arms_for(view), sideways=view in (SIDE_R, SIDE_L))
            score, _arm = compute_pursuit_score(feats, {k: v[0] for k, v in got.items() if v[0] is not None},
                                                relax=cfg.OFFVIEW_RELAX)
            if score < 0.0:
                self._handover_tid, self._handover_n = None, 0
                self._handover_note = "mau ao / bang tay khong khop"
                return
        self._handover_n = self._handover_n + 1 if self._handover_tid == tid else 1
        self._handover_tid = tid
        self._handover_note = f"xac nhan {self._handover_n}/{cfg.HANDOVER_CONFIRM_TICKS}"
        if self._handover_n < cfg.HANDOVER_CONFIRM_TICKS:
            return
        self._handover = False
        self.target_track_id, self.target_name = tid, name
        self.lost_start_time = None
        self._need_reconfirm = self._contested = False
        self.last_known_center = _anchor(box, kpts)
        self._last_box_h = max(box[3] - box[1], 1)
        self._last_good_time = time.time()
        if self._handover_colors:
            self.master_features = feats
        else:
            self.master_features = None
            self.preview_rois, self.preview_colors, self._sample_buf, self._sample_roi = {}, {}, {}, {}
        print(f"[ESP] Da nhan chu '{name}' tren cam ESP (ID {tid})")

    # ── Thanh trượt tốc độ (chế độ 2, 3; mỗi chế độ nhớ số riêng) ──
    def _init_sliders(self):
        self._speed = {MODE_GESTURE: int(cfg.GESTURE_V), MODE_FOLLOW: int(cfg.FOLLOW_V_MAX)}
        self._turn = {MODE_GESTURE: int(cfg.GESTURE_W), MODE_FOLLOW: int(cfg.FOLLOW_W_MAX)}
        self._slider_mode = None
        cv2.createTrackbar(SPEED_TB, self.MAIN_WIN_NAME, self._speed[MODE_FOLLOW], cfg.SPEED_SLIDER_MAX, lambda _v: None)
        cv2.createTrackbar(TURN_TB, self.MAIN_WIN_NAME, self._turn[MODE_FOLLOW], cfg.TURN_SLIDER_MAX, lambda _v: None)

    def _sync_sliders(self):
        if self.mode not in self._speed:
            return
        if self._slider_mode != self.mode:
            self._slider_mode = self.mode
            cv2.setTrackbarPos(SPEED_TB, self.MAIN_WIN_NAME, self._speed[self.mode])
            cv2.setTrackbarPos(TURN_TB, self.MAIN_WIN_NAME, self._turn[self.mode])
            return
        sp = cv2.getTrackbarPos(SPEED_TB, self.MAIN_WIN_NAME)
        tn = cv2.getTrackbarPos(TURN_TB, self.MAIN_WIN_NAME)
        if sp >= 0:
            self._speed[self.mode] = min(int(sp), cfg.SPEED_SLIDER_MAX)
        if tn >= 0:
            self._turn[self.mode] = min(int(tn), cfg.TURN_SLIDER_MAX)

    # ── Chọn lệnh cho xe theo chế độ ──
    def _owner_view(self):
        if (self.target_track_id is None or self.master_features is None or self.lost_start_time is not None
                or self._crossing or not self._measure_ok or self.last_known_center is None):
            return None
        return (self.last_known_center[0], self._last_box_h, self.frame_w, self.frame_h)

    def _hold_reason(self):
        if self.source != "esp":
            return "WEBCAM MAY TINH: xe dung yen - bam Y de chuyen sang cam ESP"
        if self._handover:
            return "dang nhan lai chu tren cam ESP - xe dung yen"
        return None

    def _drive_robot(self, display_frame):
        self._sync_sliders()
        note = ""
        if self.mode == MODE_GESTURE:
            note = self._gesture_step(time.time())
        elif self.mode == MODE_FOLLOW:
            note = self._follow_step(time.time())
        self._draw_robot_hud(display_frame, note)

    def _gesture_step(self, now):
        hold = self._hold_reason()
        if hold or not self.stick_mode or self.robot_command == "DUNG TAI VI TRI":
            self.maneuver.abort(self.robot)
            self.robot.stop()
            return hold or "cu chi TAM DUNG (bam S de bat lai)"
        return "cu chi: " + self.maneuver.update(now, self.robot_command, self._owner_view(), self.robot,
                                                 self._speed[MODE_GESTURE], self._turn[MODE_GESTURE],
                                                 facing=self.facing.stable)

    def _follow_step(self, now):
        hold = self._hold_reason()
        if hold or self.target_track_id is None or self.master_features is None:
            self.search.reset()
            self.robot.cancel_action()
            self.robot.stop()
            if hold:
                return hold
            return "cho nhan mat chu" if self.target_track_id is None else "dang quet ao - xe dung yen"
        v_max, w_max = self._speed[MODE_FOLLOW], self._turn[MODE_FOLLOW]
        if self.lost_start_time is not None:
            self.robot.stop()
            return "TIM CHU: " + self.search.update(now, self.lost_start_time, self.follow.last_side, self.robot,
                                                    w=min(cfg.SEARCH_TURN_W, w_max))
        self.search.reset()
        if self._crossing:
            self.robot.cancel_action()
            self.robot.stop()
            return "dang cat nhau - dung cho"
        if self._measure_ok and self.last_known_center is not None:
            self.robot.cancel_action()
            v, w = self.follow.update(self.last_known_center[0], self._last_box_h, self.frame_w, self.frame_h,
                                      v_max=v_max, w_max=w_max)
            self.robot.set_target(v, w)
            return "bam: " + self.follow.dbg
        self.robot.stop()
        return "cho do vi tri chu"

    def _draw_robot_hud(self, frame, note):
        h_img = frame.shape[0]
        t = self.robot.telemetry
        if not self.robot.claimed:
            link = "XE: nha quyen"
        elif t["connected"]:
            link = "XE: " + ("ARM" if t["armed"] else ("CHO TU ARM" if t["autoArm"] else "DISARM")) + f" p{t['pitch']:+.1f}"
        else:
            link = "XE: MAT LIEN LAC"
        v, w = self.robot.command
        sl = f" | thanh {self._speed[self.mode]}/{self._turn[self.mode]}" if self.mode in self._speed else ""
        _put_text(frame, f"[{self.mode}] {MODE_NAMES[self.mode]} | {link} | v{v:+.0f} w{w:+.0f}{sl}",
                    (10, h_img - 88), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 200, 0), 1)
        if note:
            _put_text(frame, note, (10, h_img - 70), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 200, 0), 1)

    def _chip_color(self, name):
        if name == "armband" and self.master_features is not None and "armband" in self.master_features:
            h = self.master_features["armband"]
            i, j, k = np.unravel_index(int(np.argmax(h)), h.shape)
            hsv = np.uint8([[[min(179, (i + 0.5) * 180 / HIST_BINS[0]), min(255, (j + 0.5) * 256 / HIST_BINS[1]),
                              min(255, (k + 0.5) * 256 / HIST_BINS[2])]]])
            return tuple(int(c) for c in cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0])
        return self.preview_colors.get(name)

    def _draw_color_chips(self, frame):
        if self.target_track_id is None and not self.preview_colors:
            return
        h_img, w_img = frame.shape[:2]
        k = w_img / 400.0
        sz = max(8, int(round(w_img * cfg.UI_CHIP_FRAC)))
        gap = max(3, sz // 4)
        margin = max(4, int(round(6 * k)))
        x0 = w_img - margin - (3 * sz + 2 * gap)
        pad = max(2, gap // 2)
        cv2.rectangle(frame, (x0 - pad, margin - pad), (w_img - margin + pad, margin + sz + pad), (25, 25, 25), -1)
        for i, name in enumerate(("shirt", "pants", "armband")):
            x = x0 + i * (sz + gap)
            col = self._chip_color(name)
            if col is not None:
                cv2.rectangle(frame, (x, margin), (x + sz, margin + sz), col, -1)
                cv2.rectangle(frame, (x, margin), (x + sz, margin + sz), (255, 255, 255), 1)
            else:
                cv2.rectangle(frame, (x, margin), (x + sz, margin + sz), (110, 110, 110), 1)

    def _handle_gesture(self, display_frame):
        h_img, w_img = display_frame.shape[:2]
        raw_kpts = self.person_kpts.get(self.target_track_id) if self.target_track_id is not None else None
        trusted = self._target_is_trusted()

        if raw_kpts is None:
            self.kp_smoother.reset()
            self._kp_smooth = None
            kpts = None
        else:
            if self._yolo_ran_this_frame or self._kp_smooth is None:
                self._kp_smooth = self.kp_smoother.update(raw_kpts)
            kpts = self._kp_smooth

        if kpts is not None:
            draw_stick_figure(display_frame, kpts, conf_thresh=0.20, coords="pixel",
                              in_place=True, trusted=trusted, facing=self.facing.stable)

        if kpts is None or not trusted:
            self.gesture.reset()
            self._set_command("DUNG")
            if self.target_track_id is None:
                why = "chua co chu nhan"
            elif kpts is None or self.lost_start_time is not None:
                why = "khong thay chu nhan"
            elif self._need_reconfirm:
                why = "cho xac nhan lai chu (thay bang tay hoac chay sat cam, khong co nguoi khac)"
            else:
                why = "chua thay bang tay khop (xoay tay phai ra phia camera)"
            _put_text(display_frame, f"CU CHI: CHUA NHAN LENH - {why}", (10, h_img - 45),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (170, 170, 170), 1)
        elif not self.stick_mode:
            self.gesture.reset()
            self._set_command("DUNG")
            _put_text(display_frame, "CU CHI: TAM DUNG (bat cheo tay) - bam S de nhan lenh lai", (10, h_img - 45),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (170, 170, 170), 1)
        else:
            if self._yolo_ran_this_frame:
                self._last_valid_pose_time = time.monotonic()
                stable = self.gesture.update(kpts, facing=self.facing.stable, profile=self._is_sideways(self.target_track_id))
                if stable == "DUNG TAI VI TRI":
                    self.stick_mode = False
                self._set_command(stable)
            dbg = self.gesture.last_dbg
            if "why" in dbg:
                txt = f"raw={self.gesture.last_raw} ({dbg['why']})"
            else:
                fmt = lambda t: f"{t[0]}{'' if t[1] is None else f' {t[1]:.0f}'}"
                txt = f"raw={self.gesture.last_raw} | R:{fmt(dbg['R'])} L:{fmt(dbg['L'])}"
            _put_text(display_frame, txt, (10, h_img - 45), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        moving = self.robot_command not in ("DUNG", "DUNG TAI VI TRI")
        _put_text(display_frame, f"LENH ROBOT: {self.robot_command}", (10, h_img - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if moving else (0, 200, 255), 2)

    def _target_is_trusted(self):
        return (self.target_track_id is not None
                and self.target_track_id != -1
                and self.lost_start_time is None
                and not self._need_reconfirm
                and (not cfg.GESTURE_NEED_ARMBAND or time.time() < self._armband_ok_until))

    def _set_command(self, cmd):
        if cmd in ("DUNG", "DUNG TAI VI TRI"):
            self.robot_command = cmd
            self._pending_cmd, self._pending_n = None, 0
            return
        if cmd == self._pending_cmd:
            self._pending_n += 1
        else:
            self._pending_cmd, self._pending_n = cmd, 1
        self.robot_command = cmd if self._pending_n >= self.CMD_CONFIRM_NEEDED else "DUNG"

    # ------------------------------------------------------------------
    #  QUÉT MÀU & BÁM BẰNG MÀU
    # ------------------------------------------------------------------
    def _target_overlapped(self, box):
        for oid, obox in self.current_persons:
            if oid == self.target_track_id:
                continue
            if min(box[2], obox[2]) > max(box[0], obox[0]) and min(box[3], obox[3]) > max(box[1], obox[1]):
                return True
        return False

    def _collect_feature(self, name, frame, kpts, box):
        if self.master_features is not None and name in self.master_features:
            return

        buf = self._sample_buf.setdefault(name, [])
        if not is_good_pose_for_scan(kpts, name) or self._target_overlapped(box):
            buf.clear()
            return

        hist, roi, mean_bgr = extract_all_features(frame, kpts, only=(name,))[name]
        if hist is None:
            buf.clear()
            return

        if buf and cv2.compareHist(buf[0], hist, cv2.HISTCMP_CORREL) < cfg.FEATURE_SAMPLE_MIN_CORR:
            buf.clear()
        buf.append(hist)
        self._sample_roi[name] = (roi, mean_bgr)
        if len(buf) < cfg.FEATURE_SAMPLES_NEEDED:
            return

        avg = cv2.normalize(np.mean(buf, axis=0).astype(np.float32), None, 0, 1, cv2.NORM_MINMAX)
        if self.master_features is None:
            self.master_features = {}
        self.master_features[name] = avg
        roi_last, bgr = self._sample_roi[name]
        self.preview_rois[name] = cv2.resize(roi_last, (80, 80))
        self.preview_colors[name] = (int(bgr[0]), int(bgr[1]), int(bgr[2]))
        buf.clear()
        print(f"[MAU] Da chot mau {name.upper()} (khong cap nhat nua)")

    def _sample_shirt_after_lock(self, frame):
        target_person = next((p for p in self.current_persons if p[0] == self.target_track_id), None)
        if target_person is None:
            if self.lost_start_time is None:
                self.lost_start_time = time.time()
            elif time.time() - self.lost_start_time >= self.STEP_BACK_LOST_TIMEOUT:
                self.reset_target_state()
            return

        self.lost_start_time = None
        self._measure_ok = True
        _, box = target_person
        self.last_known_center = _anchor(box, self.person_kpts.get(self.target_track_id))
        if self._yolo_ran_this_frame and self.target_track_id in self.person_kpts:
            self._collect_feature('shirt', frame, self.person_kpts[self.target_track_id], box)

    def _capture_missing_features(self, frame, box):
        missing = [n for n in ('pants', 'armband') if n not in self.master_features]
        if not missing:
            return
        tid = self.target_track_id
        if len(self.current_persons) > 1:
            score, _ = compute_pursuit_score(self.master_features, self._get_cand_hists(frame, tid))
            if score < cfg.FEATURE_CAPTURE_MIN_SCORE:
                return
        for n in missing:
            self._collect_feature(n, frame, self.person_kpts[tid], box)

    def _eval_candidates(self, frame):
        pred = self.predictor.position()
        coast = self.predictor.coast_time()
        ref = pred if pred is not None else self.last_known_center
        cands, seen = [], set()

        for t_id, box in self.current_persons:
            if t_id not in self.person_kpts:
                continue
            seen.add(t_id)
            hists = self._get_cand_hists(frame, t_id)
            view = self._get_view(t_id)
            is_cur = (t_id == self.target_track_id)
            relax = cfg.OFFVIEW_RELAX if (is_cur and view in (BACK, SIDE_R, SIDE_L)) else 1.0
            pursuit = lambda h, **kw: compute_pursuit_score(self.master_features, h, relax=relax, **kw)

            score, arm = pursuit(hists)
            if (arm == 'match' and view == SIDE_L) or (arm == 'mismatch' and view not in CONCLUSIVE):
                score, arm = pursuit({k: v for k, v in hists.items() if k != 'armband'})

            if arm == 'mismatch':
                n = self._arm_mismatch.get(t_id, 0) + 1
                self._arm_mismatch[t_id] = n
                if n < cfg.ARMBAND_MISMATCH_FRAMES:
                    score, arm = pursuit(hists, armband_strict=False)
            else:
                self._arm_mismatch[t_id] = 0

            if score < 0.0:
                if is_cur and arm != 'mismatch' and not self._need_reconfirm:
                    self._color_rej += 1
                    if self._color_rej >= cfg.COLOR_REJECT_FRAMES:
                        continue
                    score = cfg.VALID_ID_MIN_SCORE
                else:
                    continue
            elif is_cur:
                self._color_rej = 0

            cx, cy = _anchor(box, self.person_kpts.get(t_id))
            bh = max(box[3] - box[1], 1)
            if ref is None:
                dist, in_gate = 0.0, True
            else:
                d = float(np.hypot(cx - ref[0], cy - ref[1]))
                dist, in_gate = d / bh, d <= gate_radius(bh, coast)

            cands.append({'tid': t_id, 'box': box, 'score': score, 'arm': arm, 'in_gate': in_gate, 'view': view,
                          'measured': 'shirt' in hists,
                          'cost': cfg.W_COLOR * (1.0 - score) + cfg.W_DIST * dist})

        for k in [k for k in self._arm_mismatch if k not in seen]:
            del self._arm_mismatch[k]
        return cands

    def _pick_candidate(self, viable, tid):
        self._pick_reason = None
        if not viable:
            return None
        cur = next((c for c in viable if c['tid'] == tid), None)

        # ---- (1) Băng tay khớp = bằng chứng mạnh nhất ----
        matched = [c for c in viable if c['arm'] == 'match']
        if not matched:
            self._match_tid, self._match_n = None, 0
        else:
            if len(matched) == 1 or matched[1]['cost'] - matched[0]['cost'] >= cfg.AMBIGUOUS_COST_MARGIN:
                mt = matched[0]
                if mt['tid'] != tid:
                    self._match_n = self._match_n + 1 if self._match_tid == mt['tid'] else 1
                    self._match_tid = mt['tid']
                    if self._match_n < cfg.MATCH_SWITCH_FRAMES:
                        if cur is not None and not self._need_reconfirm and cur['score'] >= cfg.VALID_ID_MIN_SCORE:
                            self._pick_reason = 'keep'
                            return cur
                        return None
                else:
                    self._match_tid, self._match_n = None, 0
                self._pick_reason = 'match'
                return mt
            if cur is not None and cur in matched and not self._need_reconfirm:
                self._pick_reason = 'keep'
                return cur
            return None

        # ---- (2) Đang chờ xác nhận lại (sau cắt nhau / mất dấu) mà không có băng tay khớp ----
        if self._need_reconfirm:
            return None

        # ---- (3) ID chủ đã biến mất khỏi danh sách: có thể chỉ là YOLO gán nhãn ID mới cho cùng 1 người ----
        if cur is None:
            c = self._handoff_pick(viable, tid)
            if c is not None:
                self._pick_reason = 'handoff'
                return c
            return None

        # ---- (4) Bám bình thường: giữ nguyên ID hiện tại nếu điểm màu còn đủ để coi là đúng người ----
        if cur['score'] >= cfg.VALID_ID_MIN_SCORE:
            self._pick_reason = 'keep'
            return cur
        return None

    def _reacquire_by_approach(self, frame, cands, now):
        def fail(note, reset=True):
            self._reacq_note = note
            if reset:
                self._reacq_tid, self._reacq_n = None, 0
            return None

        ok = self._last_ok
        if ok is None:
            return fail("khong co moc 'chac la chu'")
        if self._contested:
            return fail("co nguoi chen vao -> chi con bang tay khop")
        if now - ok['t'] > cfg.REACQ_MAX_LOST:
            return fail("mat qua lau -> chi con bang tay khop")
        fh = frame.shape[0]
        if not (ok['approach'] >= cfg.REACQ_APPROACH_RATE or ok['h'] >= cfg.REACQ_CLOSE_FRAC * fh):
            return fail("luc mat chu khong chay sat cam -> cho bang tay khop")

        ref = self.predictor.position() or ok['anchor']
        if ref is None:
            return fail("khong co vi tri tham chieu")
        coast = self.predictor.coast_time()
        people = []
        for t_id, box in self.current_persons:
            ax, ay = _anchor(box, self.person_kpts.get(t_id))
            bh = max(box[3] - box[1], 1)
            d = float(np.hypot(ax - ref[0], ay - ref[1]))
            people.append({'tid': t_id, 'box': box, 'bh': bh, 'in_gate': d <= gate_radius(max(bh, ok['h']), coast) * cfg.REACQ_GATE_MULT})
        inside = [p for p in people if p['in_gate']]
        self._loss_gate_tids.update(p['tid'] for p in inside)
        if len(self._loss_gate_tids) > 1:
            self._contested = True
            return fail("co nguoi chen vao cong luc mat chu -> chi con bang tay khop")
        if len(inside) > 1:
            self._contested = True
            return fail("nhieu nguoi trong cong -> chi con bang tay khop")
        if not inside:
            return fail("chua thay ai trong cong")
        cand = inside[0]
        for p in people:
            if p is not cand and box_overlap_ratio(cand['box'], p['box']) > cfg.CROSS_OVERLAP:
                self._contested = True
                return fail("co nguoi chong len -> chi con bang tay khop")
        seen = self._known_others.get(cand['tid'])
        if seen is not None and now - seen <= cfg.REACQ_KNOWN_TTL:
            return fail("day la nguoi da dung canh chu (nguoi la)")
        pbox = list(ok['box'])
        if ok['anchor'] is not None:
            dx, dy = ref[0] - ok['anchor'][0], ref[1] - ok['anchor'][1]
            pbox = [pbox[0] + dx, pbox[1] + dy, pbox[2] + dx, pbox[3] + dy]
        if box_overlap_ratio(cand['box'], pbox) < cfg.REACQ_BOX_OVERLAP:
            return fail("nguoi xuat hien o cho khac cho chu bien mat")
        c = next((c for c in cands if c['tid'] == cand['tid']), None)
        if c is None:
            return fail("bi mau / bang tay loai")
        if cand['bh'] < ok['h'] * cfg.REACQ_H_MIN:
            return fail("nguoi o xa hon luc chu bien mat")
        if self._reacq_tid == cand['tid'] and cand['bh'] < self._reacq_h * 0.95:
            return fail("nguoi dang lui xa")

        self._reacq_n = self._reacq_n + 1 if self._reacq_tid == cand['tid'] else 1
        self._reacq_tid, self._reacq_h = cand['tid'], cand['bh']
        if self._reacq_n < cfg.REACQ_CONFIRM_TICKS:
            self._reacq_note = f"chu chay sat cam, dang xac nhan {self._reacq_n}/{cfg.REACQ_CONFIRM_TICKS}"
            return None
        self._reacq_tid, self._reacq_n, self._reacq_note = None, 0, ""
        c['by'] = 'approach'
        self._pick_reason = 'approach'
        return c


    def _handoff_pick(self, viable, tid):
        pos = self.predictor.position()
        if pos is None or self.predictor.coast_time() > cfg.HANDOFF_MAX_GAP:
            return None
        if any(t == tid for t, _ in self.current_persons):
            return None
        cands = [c for c in viable if c['arm'] != 'mismatch' and c.get('measured', True) and c['score'] >= cfg.REACQUIRE_MIN_SCORE]
        if len(cands) != 1:
            return None
        c = cands[0]
        bh = max(c['box'][3] - c['box'][1], 1)
        ax, ay = _anchor(c['box'], self.person_kpts.get(c['tid']))
        for oid, ob in self.current_persons:
            if oid == c['tid']:
                continue
            ox, oy = _anchor(ob, self.person_kpts.get(oid))
            if min(np.hypot(ox - ax, oy - ay), np.hypot(ox - pos[0], oy - pos[1])) < cfg.HANDOFF_CLEAR * bh:
                return None
        return c

    def _others_near_last(self):
        if self.last_known_center is None:
            return True
        for oid, ob in self.current_persons:
            if oid == self.target_track_id:
                continue
            ox, oy = _anchor(ob, self.person_kpts.get(oid))
            if np.hypot(ox - self.last_known_center[0], oy - self.last_known_center[1]) < cfg.HANDOFF_CLEAR * max(self._last_box_h, 1):
                return True
        return False

    def _vx_hint(self):
        if self.predictor.position() is None or self.predictor.coast_time() > 1.0:
            return 0.0
        return self.predictor.velocity[0]

    def _kalman_measure(self, target_person, frame_h):
        box = target_person[1]
        kpts = self.person_kpts.get(self.target_track_id)
        size, kind = _scale_proxy(box, kpts, frame_h)
        self.predictor.update(_anchor(box, kpts), height=size, kind=kind)
        ms = self.predictor.motion_state(max(box[3] - box[1], 1))
        known = ms['known']
        self.facing.update(kpts,
                           vx=self.predictor.velocity[0] if known else 0.0,
                           approach=ms['approach'] if known else 0.0,
                           motion_dir=ms['dir'] if known else 0)

    def _is_sideways(self, t_id):
        return self._get_view(t_id) in (SIDE_R, SIDE_L)

    def _track_debug_text(self):
        if self.target_track_id is None or self.master_features is None:
            return ""
        parts = [f"HUONG:{VIEW_TEXT[self.facing.stable]}"]
        box = dict(self.current_persons).get(self.target_track_id)
        if box is not None and self.kalman_mode:
            ms = self.predictor.motion_state(box[3] - box[1])
            if ms['known']:
                if ms['sideways']:
                    parts.append("DI NGANG " + ("->PHAI" if ms['dir'] > 0 else "->TRAI"))
                parts.append(f"TIEN-SAT-CAM:{ms['approach']:+.2f}")
        if self._need_reconfirm:
            parts.append("BAM LAI: " + (self._reacq_note or "cho bang tay khop / chu chay sat cam"))
        return " | ".join(parts)

    def _get_view(self, t_id):
        v = self._view_cache.get(t_id)
        if v is None:
            kpts = self.person_kpts.get(t_id)
            if t_id == self.target_track_id:
                box = dict(self.current_persons).get(t_id)
                bh = max(box[3] - box[1], 1) if box is not None else 1
                ms = self.predictor.motion_state(bh)
                v, _ = raw_view(kpts, self._vx_hint())
                v = view_with_motion(v, ms['dir'] if ms['known'] else 0)
                if v == UNK and self.facing.stable != FRONT:
                    v = self.facing.stable
            else:
                v, _ = raw_view(kpts, 0.0)
            self._view_cache[t_id] = v
        return v

    def _track_with_memory(self, frame):
        if not self._yolo_ran_this_frame:
            return
        tid = self.target_track_id
        boxes = dict(self.current_persons)
        tbox = boxes.get(tid)
        now = time.time()

        crossing = tbox is not None and any(
            box_overlap_ratio(tbox, b) > cfg.CROSS_OVERLAP for oid, b in boxes.items() if oid != tid)
        if crossing:
            if self._cross_start is None:
                self._cross_start = now
            if now - self._cross_start <= cfg.CROSS_MAX_FREEZE:
                self._crossing = True
                self._contested = True
                self._need_reconfirm = True
                self._target_armband_visible = False
                self._armband_ok_until = 0.0
                return
        else:
            self._cross_start = None
        self._crossing = False

        cands = self._eval_candidates(frame)
        kal_expired = self.predictor.coast_time() >= cfg.GATE_MAX_COAST
        viable = sorted((c for c in cands if c['in_gate'] or c['arm'] == 'match' or kal_expired),
                        key=lambda c: c['cost'])
        best = self._pick_candidate(viable, tid)
        if best is None:
            best = self._reacquire_by_approach(frame, cands, now)

        if best is None:
            self._target_armband_visible = False
            self._armband_ok_until = 0.0
            soft = (not self._need_reconfirm and self._last_good_time is not None
                    and now - self._last_good_time < cfg.SOFT_LOSS_GRACE and not self._others_near_last())
            if not soft:
                self._need_reconfirm = True
            if self.lost_start_time is None:
                self.lost_start_time = now
            return

        if best['tid'] != tid:
            if not best['in_gate']:
                self.predictor.reset()
            self.target_track_id = best['tid']
            self.kp_smoother.reset()
            self._kp_smooth = None
        if self._pick_reason in ('match', 'approach'):
            self._need_reconfirm = False
            self._contested = False
            self._reacq_tid, self._reacq_n, self._reacq_note = None, 0, ""

        self.lost_start_time = None
        self._loss_gate_tids = set()
        self._last_good_time = now
        self._last_box_h = max(best['box'][3] - best['box'][1], 1)
        self.last_known_center = _anchor(best['box'], self.person_kpts.get(best['tid']))
        self._target_armband_visible = (best['arm'] == 'match')
        if best['arm'] == 'match':
            self._armband_ok_until = now + cfg.GESTURE_ARMBAND_GRACE
        self._measure_ok = True
        if not self._need_reconfirm:
            self._last_ok = {'t': now, 'h': float(best['box'][3] - best['box'][1]), 'approach': self.predictor.approach_rate(),
                             'anchor': self.last_known_center, 'tid': best['tid'], 'box': list(best['box'])}
            for oid, _b in self.current_persons:
                if oid != best['tid']:
                    self._known_others[oid] = now
            for k in [k for k, t in self._known_others.items() if now - t > 2 * cfg.REACQ_KNOWN_TTL]:
                del self._known_others[k]
            self._capture_missing_features(frame, best['box'])

    def _memory_text(self):
        names = {'shirt': 'AO', 'pants': 'QUAN', 'armband': 'BANG TAY'}
        return "+".join(names[k] for k in names if self.master_features and k in self.master_features) or "-"

    def _missing_hint(self):
        miss = [n for k, n in (('pants', 'QUAN'), ('armband', 'BANG TAY (DEO BANG MAU RUC)'))
                if self.master_features and k not in self.master_features]
        return (" | DUNG THANG DE QUET " + "/".join(miss)) if miss else ""

    def _watchdog_check(self):
        if not self.stick_mode or self.robot_command in ("DUNG", "DUNG TAI VI TRI"):
            return
        last = self._last_valid_pose_time
        if last is None or time.monotonic() - last > self.WATCHDOG_TIMEOUT:
            self._set_command("DUNG")

    def _get_cand_hists(self, frame, t_id):
        kpts = self.person_kpts.get(t_id)
        if kpts is None:
            return {}
        keys = tuple(self.master_features.keys()) if self.master_features else ('shirt',)
        view = self._get_view(t_id)
        arms = arms_for(view)
        sideways = view in (SIDE_R, SIDE_L)
        cache_key = (t_id, keys, arms, sideways)
        cached = self._hist_cache.get(cache_key)
        if cached is None:
            feats = extract_all_features(frame, kpts, only=keys, track=True, arms=arms, sideways=sideways)
            cached = {k: v[0] for k, v in feats.items() if v[0] is not None}
            self._hist_cache[cache_key] = cached
        return cached

def _flush_queue(q):
    while not q.empty():
        try:
            q.get_nowait()
        except queue.Empty:
            break

if __name__ == "__main__":
    target_ip, start_mode, args = None, None, sys.argv[1:]
    i = 0
    while i < len(args):
        a = args[i]
        if a in ("--mode", "-m") and i + 1 < len(args):
            start_mode = MODE_ALIASES.get(args[i + 1].lower())
            i += 1
        elif a.startswith("--mode="):
            start_mode = MODE_ALIASES.get(a.split("=", 1)[1].lower())
        elif not a.startswith("-"):
            target_ip = a
        i += 1
    app = RobotTrackerPipeline(target_ip=target_ip, start_mode=start_mode)
    app.run()