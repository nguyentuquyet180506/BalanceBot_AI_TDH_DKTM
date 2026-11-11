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
from _5_color_match import (extract_all_features, compute_pursuit_score, is_good_pose_for_scan, boost_colors, exposure_gamma,
                            color_sim, shirt_hue_block, ARMBAND_DIAG, armband_presence)
from _6_draw_stick import draw_stick_figure
from _8_gesture import GestureClassifier, KeypointSmoother
from _7_kal_fil import TrajectoryPredictor, box_overlap_ratio, gate_radius
from _9_facing import (raw_view, view_with_motion, FacingEstimator, arms_for, CONCLUSIVE,
                       FRONT, BACK, SIDE_R, SIDE_L, UNK, VIEW_TEXT)
from _10_robot_link import RobotLink
from _11_motion import FollowController, SearchPlanner, GestureManeuver



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


# ── 5 chế độ điều khiển ──
MODE_MANUAL, MODE_GESTURE, MODE_CAMERA, MODE_FOLLOW, MODE_FOLLOW_ARM = 1, 2, 3, 4, 5
FOLLOW_MODES = (MODE_FOLLOW, MODE_FOLLOW_ARM)
AI_MODES = (MODE_GESTURE,) + FOLLOW_MODES
MODE_NAMES = {MODE_MANUAL: "TAY (web / can gat UDP)", MODE_GESTURE: "CU CHI (AI)",
              MODE_CAMERA: "CAM KHONG AI (do duong)", MODE_FOLLOW: "BAM NGUOI: AO + QUAN (AI)",
              MODE_FOLLOW_ARM: "BAM NGUOI: AO + QUAN + TAY AO (AI)"}
MODE_ALIASES = {"1": 1, "tay": 1, "manual": 1, "2": 2, "cuchi": 2, "cu chi": 2, "gesture": 2,
                "3": 3, "cam": 3, "camera": 3,
                "4": 4, "bam": 4, "bamnguoi": 4, "bam nguoi": 4, "follow": 4, "aoquan": 4,
                "5": 5, "bam3": 5, "bamday": 5, "bangtay": 5, "full": 5}
JOYSTICK_SCRIPT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "balancebot_udp.py")
SPEED_TB, TURN_TB = "Toc do tick/s", "Xoay do/s"
TERMINAL_HELP = """
================= DOI CHE DO (go roi Enter) =================
  1 | tay     : dieu khien TAY (web / cua so can gat UDP), tat camera + AI
  2 | cuchi   : dieu khien bang CU CHI tay (AI)
  3 | cam     : chi xem CAMERA (khong AI) de do duong, lai bang can gat / web
  4 | bam     : BAM NGUOI bang AO + QUAN (khong dung tay ao)
  5 | bamday  : BAM NGUOI bang AO + QUAN + TAY AO
                (4, 5: mat dau thi XOAY TAI CHO 10 do, dung yen 5 s, lap lai)
  y : (che do 2, 4, 5) chuyen tu WEBCAM MAY TINH sang CAM ESP de track (truoc do xe dung yen)
  r : bo chu, quay lai tim chu bang webcam | x : DISARM khan cap | a : ARM | q : thoat | h : tro giup
  (Cua so video: phim 1-5, X, N = dang ky mat, R, S = bat/tat cu chi, Q)
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

        self._tent_tid = None
        self._tent_noarm = 0
        self._tent_cand, self._tent_cand_n = None, 0
        self._tent_banned = {}
        self._tent_hist = {}
        self._creacq_tid, self._creacq_n = None, 0
        self._scan_note = ""
        self._scan_cache = (None, True, "")
        self._scan_prev = None
        self._shirt_block = None
        self._handover_arm_ok = False
        self._y_force_until = 0.0

        self._fl_snap = None
        self._backup = None
        self._exp_cache = (None, None)
        self._exp_gamma = None

        # ===== Chế độ điều khiển + gửi lệnh xuống xe =====
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
        self._tent_tid, self._tent_noarm = None, 0
        self._tent_cand, self._tent_cand_n = None, 0
        self._tent_banned = {}
        self._tent_hist = {}
        self._creacq_tid, self._creacq_n = None, 0
        self._scan_note, self._scan_cache, self._scan_prev = "", (None, True, ""), None
        self._shirt_block = None
        self._fl_snap = None
        self._backup = None

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
                        face_img = self._face_frame(frame)
                        h_img, w_img = frame.shape[:2]
                        self.detector.setInputSize((w_img, h_img))
                        _, faces = self.detector.detect(face_img)
                        self._face_dbg = []

                        if faces is not None and len(faces) > 0:
                            for face in faces:
                                fbox = face[:4].astype(int)
                                fx, fy, fw, fh = fbox
                                if fw < 10 or fh < 10:
                                    continue

                                is_sharp, blur_score = is_sharp_face(face_img, (fx, fy, fw, fh))
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
                                            (face_img, (fx, fy, fw, fh), matched_tid, is_profile, self._epoch)
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
            if cfg.EXPOSURE_SHOW and self.mode in AI_MODES:
                display_frame = self._exposed(frame).copy()
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
                                f"[DA NHAN DIEN: {self.target_name.upper()}] DANG QUET AO - " + (self._scan_note.upper() or "HAY LUI RA DE LO AO..."),
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
                    if self._tent_tid is not None and t_id == self._tent_tid:
                        owner_color = (0, 215, 255)
                        lbl = f"TAM THEO: {self.target_name} (khong bang {self._tent_noarm}/{cfg.TENTATIVE_REJECT_FRAMES})"
                    elif self._target_armband_visible:
                        owner_color = (0, 255, 0)
                        lbl = f"CHU NHAN: {self.target_name} [DUNG TAY AO]"
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
    #  CHẾ ĐỘ ĐIỀU KHIỂN + GỬI LỆNH XUỐNG XE
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
        self._tent_tid, self._tent_noarm = None, 0
        self._tent_cand, self._tent_cand_n = None, 0
        self._backup = None
        self._fl_snap = None
        self.stick_mode = (m == MODE_GESTURE)
        self._update_sources()
        if m in AI_MODES:
            self._close_joystick()
            self.robot.claim()
        else:
            self.robot.release()
            self._open_joystick()
        print(f"\n>>> CHE DO {m}: {MODE_NAMES[m]}")
        if m in FOLLOW_MODES:
            print("    Bam bang: " + ("AO + QUAN (khong dung tay ao)" if m == MODE_FOLLOW else
                                      "AO + QUAN + TAY AO (mat chu: dung ao + quan thi TAM THEO, khong thay tay ao thi bo)"))
            print("    Xe bam chu (giu gan tam, nguoi cao ~%d%% khung). Mat dau > %.0f s: XOAY TAI CHO %d do, dung yen %.0f s, lap lai." %
                  (round(cfg.FOLLOW_TARGET_H * 100), cfg.SEARCH_START_DELAY, cfg.SEARCH_TURN_DEG, cfg.SEARCH_PAUSE))
        elif m == MODE_GESTURE:
            print("    Tay phai gio: TIEN 3s | tay trai gio: LUI 3s | dang tay: quay %d do, chay 1s, quay lai | bat cheo: DUNG"
                  % cfg.GESTURE_SIDE_TURN_DEG)
        else:
            print(f"    Web: http://{self.esp_stream.target_ip or cfg.ROBOT_DEFAULT_IP}  (hoac http://192.168.4.1 qua Wi-Fi BalanceBot-Web)")
        if m in AI_MODES:
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
        if ord("1") <= key <= ord("5"):
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
                 ("Doi che do: phim 2 / 3 / 4 / 5 o day hoac go trong terminal", (200, 200, 200), 0.5),
                 ("X: DISARM khan cap  |  Q: thoat", (200, 200, 200), 0.5)]
        for i, (txt, col, sc) in enumerate(lines):
            _put_text(panel, txt, (15, 40 + i * 40), cv2.FONT_HERSHEY_SIMPLEX, sc, col, 1 if sc < 0.6 else 2)
        cv2.imshow(self.MAIN_WIN_NAME, panel)
        return self._mode_key(cv2.waitKey(50) & 0xFF)

    def _camera_screen(self, frame):
        h = frame.shape[0]
        _put_text(frame, "[3] CAM KHONG AI - lai bang can gat / web", (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        _put_text(frame, "1-5: doi che do | X: DISARM | Q: thoat", (10, h - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        cv2.imshow(self.MAIN_WIN_NAME, frame)
        return self._mode_key(cv2.waitKey(1) & 0xFF)

    # ── Nguồn ảnh: webcam máy tính (tìm / quét chủ) <-> cam ESP (track) ──
    def _update_sources(self):
        ai = self.mode in AI_MODES
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
            self._exp_gamma = None
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
        if self.mode not in AI_MODES:
            print("[Y] Chi dung o che do 2 / 4 / 5")
            return
        if self.source == "esp":
            print("[Y] Dang track bang cam ESP roi (R: bo chu, quay lai webcam)")
            return
        has_owner = self.target_track_id is not None
        feats = self.master_features
        if has_owner and cfg.HANDOVER_REQUIRE_SCAN and not cfg.HANDOVER_RESCAN_COLORS:
            need = ['shirt', 'pants'] + (['armband'] if self._use_armband() else [])
            names = {'shirt': 'AO', 'pants': 'QUAN', 'armband': 'TAY AO'}
            miss = [names[k] for k in need if not feats or k not in feats]
            if miss and time.time() > self._y_force_until:
                self._y_force_until = time.time() + cfg.HANDOVER_FORCE_WINDOW
                print(f"[Y] CHUA CHUYEN: chua quet xong {'/'.join(miss)} tren webcam (can de XAC NHAN chu tren cam ESP). "
                      f"Dung thang truoc webcam cho quet xong roi bam Y. Van muon chuyen: bam Y lan nua trong {cfg.HANDOVER_FORCE_WINDOW:.0f} s")
                return
            if miss and (not feats or 'shirt' not in feats):
                print("[Y] Chua co mau ao -> tren cam ESP se tim chu bang MAT")
                has_owner = False
        self._saved_owner = (self.target_name, feats, dict(self.preview_colors)) if has_owner else None
        keep_block = self._shirt_block if has_owner else None
        self._handover_colors = has_owner and feats is not None and not cfg.HANDOVER_RESCAN_COLORS
        self.reset_target_state()
        self._shirt_block = keep_block
        self._handover = has_owner
        self._handover_tid, self._handover_n, self._handover_note = None, 0, "dung 1 MINH truoc cam ESP"
        self._handover_arm_ok = False
        self.source = "esp"
        self._update_sources()
        if has_owner:
            print(f"[Y] Chuyen sang CAM ESP: dung 1 MINH truoc xe, xe QUET XAC NHAN chu '{self._saved_owner[0]}'"
                  + (" bang mau ao / quan" + (" / tay ao" if self._use_armband() else "") + " da nho (khong quet lai; anh ESP duoc phoi sang)"
                     if self._handover_colors else " (se quet lai mau ao tren cam ESP)"))
        else:
            print("[Y] Chuyen sang CAM ESP: chua co chu -> tim chu bang mat tren cam ESP")

    def _restart_owner(self):
        self.reset_target_state()
        self._saved_owner = None
        self._handover = False
        if self.source != "pc":
            self.source = "pc"
            self._update_sources()
        print("[AI] Bo chu - tim / quet chu lai bang WEBCAM MAY TINH (xe dung yen; Y de chuyen sang cam ESP)")

    def _handover_verify(self, frame, kpts, feats):
        view, _ = raw_view(kpts, 0.0)
        keys = tuple(k for k in feats if k != 'armband' or self._use_armband())
        got = extract_all_features(self._color_frame(frame), kpts, only=keys, track=True,
                                   arms=arms_for(view), sideways=view in (SIDE_R, SIDE_L))
        hists = {k: v[0] for k, v in got.items() if v[0] is not None}
        if 'shirt' not in hists:
            return False, "chua thay ro AO (dung thang, lo vai + hong)", None
        if cfg.HANDOVER_NEED_PANTS and 'pants' in feats and 'pants' not in hists:
            return False, "chua thay ro QUAN (lui ra cho cam thay dau goi)", None
        use = {k: feats[k] for k in keys}
        self._handover_arm_pct = (armband_presence(use, hists) if ('armband' in use and 'armband' in hists) else None)
        score, arm = compute_pursuit_score(use, hists)
        if arm == 'mismatch':
            if view in CONCLUSIVE:
                return False, "TAY AO SAI MAU -> khong phai chu", arm
            score, arm = compute_pursuit_score({k: v for k, v in use.items() if k != 'armband'}, hists)[0], 'unseen'
        if score < cfg.HANDOVER_MIN_SCORE:
            why = f"AO / QUAN CHUA KHOP (diem {max(score, 0):.2f} < {cfg.HANDOVER_MIN_SCORE:.2f})"
            if cfg.SCAN_DEBUG:
                parts = [f"{lbl} {color_sim(use[k], hists[k]):.2f}" for k, lbl in (('shirt', 'ao'), ('pants', 'quan'))
                         if k in use and k in hists]
                lim = f"loai neu ao < {cfg.SHIRT_MIN_CORR:.2f} / quan < {cfg.PANTS_MIN_CORR:.2f}"
                why = f"CHUA KHOP: {' '.join(parts)} -> diem {max(score, 0):.2f} < {cfg.HANDOVER_MIN_SCORE:.2f} ({lim})"
                box = next((b for t, b in self.current_persons), None)
                net = self._sharpness(frame, box) if box is not None else None
                if net is not None:
                    why += f" | net {net:.0f}"
            return False, why, arm
        return True, "", arm

    def _handover_step(self, frame):
        name, feats, chip_colors = self._saved_owner
        if len(self.current_persons) != 1:
            self._handover_tid, self._handover_n, self._handover_arm_ok = None, 0, False
            self._handover_note = "can DUNG 1 NGUOI truoc cam ESP" if self.current_persons else "chua thay ai"
            return
        tid, box = self.current_persons[0]
        kpts = self.person_kpts.get(tid)
        if self._handover_tid != tid:
            self._handover_n, self._handover_arm_ok = 0, False
        self._handover_tid = tid
        if self._handover_colors and feats is not None:
            ok, why, arm = self._handover_verify(frame, kpts, feats)
            if not ok:
                self._handover_n, self._handover_arm_ok = 0, False
                self._handover_note = why
                return
            if arm == 'match':
                self._handover_arm_ok = True
        self._handover_n += 1
        need_arm = (self._handover_colors and feats is not None and self._use_armband() and 'armband' in feats
                    and cfg.HANDOVER_NEED_ARMBAND)
        self._handover_note = f"xac nhan ao + quan {min(self._handover_n, cfg.HANDOVER_CONFIRM_TICKS)}/{cfg.HANDOVER_CONFIRM_TICKS}"
        if self._handover_n < cfg.HANDOVER_CONFIRM_TICKS:
            return
        if need_arm and not self._handover_arm_ok:
            self._handover_note = "ao + quan DUNG - gio tay ra cho cam thay TAY AO"
            pct = getattr(self, '_handover_arm_pct', None)
            if cfg.SCAN_DEBUG and pct is not None:
                self._handover_note += f" (mau bang {100 * pct:.1f}%, can >= {100 * cfg.ARMBAND_MIN_AREA:.1f}%)"
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
            self.preview_colors = dict(chip_colors)
        else:
            self.master_features = None
            self.preview_rois, self.preview_colors, self._sample_buf, self._sample_roi = {}, {}, {}, {}
        print(f"[ESP] Da nhan chu '{name}' tren cam ESP (ID {tid})")

    # ── Thanh trượt tốc độ (chế độ 2, 4, 5; mỗi chế độ nhớ số riêng) ──
    def _init_sliders(self):
        self._speed = {MODE_GESTURE: int(cfg.GESTURE_V), MODE_FOLLOW: int(cfg.FOLLOW_V_MAX),
                       MODE_FOLLOW_ARM: int(cfg.FOLLOW_V_MAX)}
        self._turn = {MODE_GESTURE: int(cfg.GESTURE_W), MODE_FOLLOW: int(cfg.FOLLOW_W_MAX),
                      MODE_FOLLOW_ARM: int(cfg.FOLLOW_W_MAX)}
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
        elif self.mode in FOLLOW_MODES:
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
        v_max, w_max = self._speed[self.mode], self._turn[self.mode]
        if self.lost_start_time is not None:
            backing = self._lost_backup_step(now, v_max)
            if backing is not None:
                return backing
            self.robot.stop()
            since = self.lost_start_time + (self._backup['dur'] if self._backup else 0.0)
            people = [t for t, _b in self.current_persons if t not in self._tent_banned]
            return "TIM CHU: " + self.search.update(now, since, self.follow.last_side, self.robot,
                                                    w=min(cfg.SEARCH_TURN_W, w_max), people=people)
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
            self._fl_snap = {'t': now, 'v': v, 'hr': self.follow.last_hr,
                             'approach': self.predictor.approach_rate()}
            tent = "(TAM THEO ao+quan, cho tay ao) " if self._tent_tid is not None else ""
            return "bam: " + tent + self.follow.dbg
        self.robot.stop()
        return "cho do vi tri chu"

    # ── Chủ tiến sát rồi mất khỏi khung: lùi tiếp (chế độ 4, 5) ──
    def _lost_backup_step(self, now, v_max):
        lost = self.lost_start_time
        if self._backup is None or self._backup['lost'] != lost:
            snap = self._fl_snap
            approaching = snap is not None and lost - snap['t'] <= cfg.LOST_BACKUP_MEMORY and (
                snap['v'] < -1.0 or (snap['approach'] >= cfg.LOST_BACKUP_APPROACH and snap['hr'] >= cfg.LOST_BACKUP_MIN_H))
            vb = min(float(cfg.LOST_BACKUP_V or cfg.FOLLOW_V_BACK_MAX), float(v_max))
            go = bool(cfg.LOST_BACKUP and approaching and vb > 0)
            self._backup = {'lost': lost, 'dur': cfg.LOST_BACKUP_TIME if go else 0.0, 'v': vb}
            if go:
                print(f"[BAM] Chu dang tien sat roi mat khoi khung -> xe LUI TIEP toi da {cfg.LOST_BACKUP_TIME:.1f}s ({vb:.0f} tick/s)")
        b = self._backup
        left = b['lost'] + b['dur'] - now
        if left <= 0:
            return None
        self.search.reset()
        self.robot.cancel_action()
        self.robot.set_target(-b['v'], 0.0)
        return f"CHU TIEN SAT -> XE LUI TIEP {left:.1f}s (cho chu hien lai trong khung)"

    def _backed_up(self):
        b = self._backup
        return b is not None and b['dur'] > 0 and b['lost'] == self.lost_start_time

    # ── Chế độ 4 / 5: xác định có dùng tay áo hay không ──
    def _use_armband(self):
        return self.mode == MODE_FOLLOW_ARM

    def _master(self, feats=None):
        mf = self.master_features if feats is None else feats
        if mf is None or self._use_armband() or 'armband' not in mf:
            return mf
        return {k: v for k, v in mf.items() if k != 'armband'}

    # ── Phơi sáng xử lý trên PC sau khi nhận ảnh (không đổi cấu hình cam OV) ──
    def _exposed(self, frame):
        if self._exp_cache[0] == self.frame_count and self._exp_cache[1] is not None:
            return self._exp_cache[1]
        g = exposure_gamma(frame, cfg.EXPOSURE_TARGET_V, cfg.EXPOSURE_GAMMA_MIN)
        a = cfg.EXPOSURE_SMOOTH
        self._exp_gamma = g if self._exp_gamma is None else (1 - a) * self._exp_gamma + a * g
        gg = round(self._exp_gamma, 2)
        if gg >= 0.995 and cfg.EXPOSURE_SAT_GAIN == 1.0:
            img = frame
        else:
            img = boost_colors(frame, gg, cfg.EXPOSURE_SAT_GAIN, 60, cfg.EXPOSURE_DARK)
        self._exp_cache = (self.frame_count, img)
        return img

    def _color_frame(self, frame):
        if cfg.EXPOSURE_ESP_COLORS and self.cam_stream is self.esp_stream:
            return self._exposed(frame)
        return frame

    def _face_frame(self, frame):
        return self._exposed(frame) if cfg.EXPOSURE_FACE else frame

    # ── Chế độ 5: bắt lại TẠM THEO bằng áo + quần ──
    def _tentative_pick(self, viable, now):
        if (not cfg.TENTATIVE_REACQ or self.mode != MODE_FOLLOW_ARM or not self._need_reconfirm
                or not self.master_features or 'armband' not in self.master_features):
            self._tent_cand, self._tent_cand_n = None, 0
            return None
        for k in [k for k, t in self._tent_banned.items() if now - t > cfg.TENTATIVE_BAN_TIME]:
            del self._tent_banned[k]
        ok = []
        for c in viable:
            seen = self._known_others.get(c['tid'])
            if (c['tid'] in self._tent_banned or c['arm'] == 'mismatch' or not c.get('measured', True)
                    or c['score'] < cfg.TENTATIVE_MIN_SCORE or (seen is not None and now - seen <= cfg.REACQ_KNOWN_TTL)):
                continue
            ok.append(c)
        if not ok or (len(ok) > 1 and ok[1]['cost'] - ok[0]['cost'] < cfg.AMBIGUOUS_COST_MARGIN):
            self._tent_cand, self._tent_cand_n = None, 0
            return None
        c = ok[0]
        self._tent_cand_n = self._tent_cand_n + 1 if self._tent_cand == c['tid'] else 1
        self._tent_cand = c['tid']
        if self._tent_cand_n < cfg.TENTATIVE_CONFIRM_TICKS:
            self._reacq_note = f"dung ao + quan, sap tam theo {self._tent_cand_n}/{cfg.TENTATIVE_CONFIRM_TICKS}"
            return None
        self._tent_cand, self._tent_cand_n = None, 0
        self._tent_tid, self._tent_noarm = c['tid'], self._tent_hist.pop(c['tid'], 0)
        self._pick_reason = 'tentative'
        print(f"[BAM] Che do 5: TAM THEO ID {c['tid']} vi dung mau ao + quan (diem {c['score']:.2f}) - dang tim tay ao "
              f"(thay ro tay ma khong dung mau tay ao {cfg.TENTATIVE_REJECT_FRAMES} lan thi bo)")
        return c

    # ── Chế độ 2, 4: nhận lại chủ bằng áo + quần ──
    def _color_reacquire(self, viable, now):
        if (not cfg.COLOR_REACQ or self.mode not in (MODE_GESTURE, MODE_FOLLOW) or not self._need_reconfirm or not self.master_features):
            self._creacq_tid, self._creacq_n = None, 0
            return None
        ok = []
        for c in viable:
            seen = self._known_others.get(c['tid'])
            if not c.get('measured', True) or c['score'] < cfg.COLOR_REACQ_MIN_SCORE:
                continue
            if seen is not None and now - seen <= cfg.REACQ_KNOWN_TTL:
                continue
            ok.append(c)
        if not ok or (len(ok) > 1 and ok[1]['cost'] - ok[0]['cost'] < cfg.AMBIGUOUS_COST_MARGIN):
            self._creacq_tid, self._creacq_n = None, 0
            self._reacq_note = "2 nguoi cung mau ao + quan -> khong doan" if len(ok) > 1 else "cho nguoi dung ao + quan"
            return None
        c = ok[0]
        self._creacq_n = self._creacq_n + 1 if self._creacq_tid == c['tid'] else 1
        self._creacq_tid = c['tid']
        if self._creacq_n < cfg.COLOR_REACQ_TICKS:
            self._reacq_note = f"dung ao + quan, dang xac nhan {self._creacq_n}/{cfg.COLOR_REACQ_TICKS}"
            return None
        self._creacq_tid, self._creacq_n = None, 0
        self._pick_reason = 'color'
        print(f"[BAM] Che do {self.mode}: nhan lai chu = ID {c['tid']} (dung ao + quan, diem {c['score']:.2f})")
        return c

    # ── Chỉ có một người trong khung: bám theo người đó ──
    def _solo_pick(self, cands):
        tid1, box1 = self.current_persons[0]
        c = next((c for c in cands if c['tid'] == tid1), None)
        if tid1 != self.target_track_id:
            pos = self.predictor.position()
            if pos is not None:
                ax, ay = _anchor(box1, self.person_kpts.get(tid1))
                bh = max(box1[3] - box1[1], 1)
                if np.hypot(ax - pos[0], ay - pos[1]) > gate_radius(bh, self.predictor.coast_time()):
                    return None
        if c is None:
            c = {'tid': tid1, 'box': box1, 'score': cfg.VALID_ID_MIN_SCORE, 'arm': 'unseen', 'in_gate': True,
                 'view': self._get_view(tid1), 'measured': False, 'cost': 1.0, 'arm_bad': False}
        elif c['arm'] == 'mismatch':
            c = dict(c, arm='unseen')
        self._color_rej = 0
        self._pick_reason = 'solo'
        return c

    # ── Điều kiện thuận lợi để QUÉT màu ──
    def _sharpness(self, frame, box):
        x1, y1, x2, y2 = (int(v) for v in box)
        roi = frame[max(0, y1):max(0, y2), max(0, x1):max(0, x2)]
        if roi.shape[0] < 20 or roi.shape[1] < 10:
            return None
        g = cv2.cvtColor(cv2.resize(roi, (max(8, int(roi.shape[1] * 160 / roi.shape[0])), 160)), cv2.COLOR_BGR2GRAY)
        return float(cv2.Laplacian(g, cv2.CV_64F).var())

    def _scan_check(self, frame, kpts, box, name='shirt'):
        if self._scan_cache[0] != self.frame_count:
            ok, why = True, ""
            now = time.time()
            anchor = _anchor(box, kpts)
            bh = max(box[3] - box[1], 1)
            prev, self._scan_prev = self._scan_prev, (now, anchor)
            if cfg.SCAN_ALONE and len(self.current_persons) != 1:
                ok, why = False, "chi 1 nguoi trong khung"
            elif self._target_overlapped(box):
                ok, why = False, "nguoi khac dang che"
            elif cfg.SCAN_NEED_FRONT and raw_view(kpts, 0.0)[0] != FRONT:
                ok, why = False, "quay mat ve camera"
            elif prev is not None and 0 < now - prev[0] < 1.0 and (
                    np.hypot(anchor[0] - prev[1][0], anchor[1] - prev[1][1]) / (now - prev[0]) / bh > cfg.SCAN_MAX_SPEED):
                ok, why = False, "dung yen"
            self._scan_cache = (self.frame_count, ok, why)
        ok, why = self._scan_cache[1], self._scan_cache[2]
        if not ok or name == 'armband':
            return ok, why
        fh, fw = frame.shape[:2]
        if box[0] < cfg.SCAN_EDGE_MARGIN * fw or box[2] > fw * (1 - cfg.SCAN_EDGE_MARGIN):
            return False, "dung vao giua khung"
        ls, rs, lh, rh = kpts[5], kpts[6], kpts[11], kpts[12]
        x1, x2 = int(min(ls[0], rs[0], lh[0], rh[0])), int(max(ls[0], rs[0], lh[0], rh[0]))
        y1, y2 = int(min(ls[1], rs[1])), int(max(lh[1], rh[1]))
        roi = self._color_frame(frame)[max(0, y1):max(0, y2), max(0, x1):max(0, x2)]
        if roi.size >= 30:
            v = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)[..., 2]
            med, clip = float(np.median(v)), float((v >= 250).mean())
            if med < cfg.SCAN_MIN_V:
                return False, "thieu sang"
            if med > cfg.SCAN_MAX_V or clip > cfg.SCAN_MAX_CLIPPED:
                return False, "chay sang (tranh den / nang chieu thang)"
        return True, ""

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
            if name == "armband" and not self._use_armband():
                cv2.rectangle(frame, (x, margin), (x + sz, margin + sz), (110, 110, 110), 1)
                cv2.line(frame, (x, margin), (x + sz, margin + sz), (110, 110, 110), 1)
                cv2.line(frame, (x + sz, margin), (x, margin + sz), (110, 110, 110), 1)
                continue
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
                why = ("cho xac nhan lai chu (thay tay ao hoac chay sat cam, khong co nguoi khac)" if self._use_armband()
                       else "cho xac nhan lai chu (dung ao + quan vai luot lien tiep)")
            else:
                why = "chua thay tay ao khop (xoay tay ra phia camera)"
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
        ok, why = self._scan_check(frame, kpts, box, name)
        if ok and not is_good_pose_for_scan(kpts, name):
            ok, why = False, {'shirt': "dung thang, lo ro vai + hong", 'pants': "lui ra cho thay dau goi",
                              'armband': "lo ro bap tay (vai -> khuyu)"}.get(name, "dung thang")
        if not ok:
            buf.clear()
            self._scan_note = why
            return

        cframe = self._color_frame(frame)
        hist, roi, mean_bgr = extract_all_features(cframe, kpts, only=(name,), shirt_block=self._shirt_block)[name]
        if hist is None:
            buf.clear()
            if name == 'armband':
                self._scan_note = ("TAY AO " + ARMBAND_DIAG['txt']) if cfg.SCAN_DEBUG else "chua thay tay ao noi bat, KHAC mau ao"
                net = self._sharpness(cframe, box) if cfg.SCAN_DEBUG else None
                if net is not None:
                    self._scan_note += f" | net {net:.0f}"
            else:
                self._scan_note = f"chua lay duoc mau {name}"
            return

        sim = color_sim(buf[0], hist) if buf else 1.0
        if sim < cfg.FEATURE_SAMPLE_MIN_CORR:
            buf.clear()
        buf.append(hist)
        self._sample_roi[name] = (roi, mean_bgr)
        self._scan_note = f"giu yen, dang quet {name} {len(buf)}/{cfg.FEATURE_SAMPLES_NEEDED}"
        if len(buf) < cfg.FEATURE_SAMPLES_NEEDED:
            return
        self._scan_note = ""

        avg = cv2.normalize(np.mean(buf, axis=0).astype(np.float32), None, 0, 1, cv2.NORM_MINMAX)
        if self.master_features is None:
            self.master_features = {}
        self.master_features[name] = avg
        if name == 'shirt':
            self._shirt_block = shirt_hue_block(cframe, kpts)
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
        wanted = ('pants', 'armband') if self._use_armband() else ('pants',)
        missing = [n for n in wanted if n not in self.master_features]
        if not missing:
            return
        tid = self.target_track_id
        if len(self.current_persons) > 1:
            score, _ = compute_pursuit_score(self._master(), self._get_cand_hists(frame, tid))
            if score < cfg.FEATURE_CAPTURE_MIN_SCORE:
                return
        for n in missing:
            self._collect_feature(n, frame, self.person_kpts[tid], box)

    def _eval_candidates(self, frame):
        pred = self.predictor.position()
        coast = self.predictor.coast_time()
        ref = pred if pred is not None else self.last_known_center
        cands, seen = [], set()
        master = self._master()

        for t_id, box in self.current_persons:
            if t_id not in self.person_kpts:
                continue
            seen.add(t_id)
            hists = self._get_cand_hists(frame, t_id)
            view = self._get_view(t_id)
            is_cur = (t_id == self.target_track_id)
            relax = cfg.OFFVIEW_RELAX if (is_cur and view in (BACK, SIDE_R, SIDE_L)) else 1.0
            pursuit = lambda h, **kw: compute_pursuit_score(master, h, relax=relax, **kw)

            score, arm = pursuit(hists)
            if ((arm == 'match' and view == SIDE_L and not cfg.SLEEVE_MODE)
                    or (arm == 'mismatch' and view not in CONCLUSIVE)):
                score, arm = pursuit({k: v for k, v in hists.items() if k != 'armband'})
            arm_bad = (arm == 'mismatch')

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
                          'arm_bad': arm_bad,
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

        # ---- (1) Tay áo khớp = bằng chứng mạnh nhất ----
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

        # ---- (2) Đang chờ xác nhận lại (sau cắt nhau / mất dấu) mà không có tay áo khớp ----
        if self._need_reconfirm:
            if (self._tent_tid is not None and cur is not None and cur['tid'] == self._tent_tid
                    and cur['arm'] != 'mismatch' and cur['score'] >= cfg.VALID_ID_MIN_SCORE):
                self._pick_reason = 'tentative'
                return cur
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
            return fail("co nguoi chen vao -> chi con tay ao khop")
        backed = self._backed_up()
        if now - ok['t'] > cfg.REACQ_MAX_LOST + (cfg.LOST_BACKUP_TIME if backed else 0.0):
            return fail("mat qua lau -> chi con tay ao khop")
        fh = frame.shape[0]
        if not backed and not (ok['approach'] >= cfg.REACQ_APPROACH_RATE or ok['h'] >= cfg.REACQ_CLOSE_FRAC * fh):
            return fail("luc mat chu khong chay sat cam -> cho tay ao khop")

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
            return fail("co nguoi chen vao cong luc mat chu -> chi con tay ao khop")
        if len(inside) > 1:
            self._contested = True
            return fail("nhieu nguoi trong cong -> chi con tay ao khop")
        if not inside:
            return fail("chua thay ai trong cong")
        cand = inside[0]
        for p in people:
            if p is not cand and box_overlap_ratio(cand['box'], p['box']) > cfg.CROSS_OVERLAP:
                self._contested = True
                return fail("co nguoi chong len -> chi con tay ao khop")
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
            return fail("bi mau / tay ao loai")
        h_min = cfg.REACQ_H_MIN_BACKUP if backed else cfg.REACQ_H_MIN
        if cand['bh'] < ok['h'] * h_min:
            return fail("nguoi o xa hon luc chu bien mat")
        robot_backing = backed and not self.robot.is_still(cfg.SEARCH_STILL_V)
        if self._reacq_tid == cand['tid'] and cand['bh'] < self._reacq_h * 0.95 and not robot_backing:
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
        if self._tent_tid is not None:
            parts.append(f"TAM THEO ao+quan - dang tim tay ao (thay ro tay khong dung mau {self._tent_noarm}/{cfg.TENTATIVE_REJECT_FRAMES})")
        elif self._need_reconfirm:
            parts.append("BAM LAI: " + (self._reacq_note or ("cho tay ao khop / chu chay sat cam" if self._use_armband()
                                                             else "cho nguoi dung ao + quan")))
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
        best = None
        if (cfg.SOLO_TRUST and len(self.current_persons) == 1 and self.lost_start_time is None
                and not self._need_reconfirm):
            best = self._solo_pick(cands)
        if best is None:
            kal_expired = self.predictor.coast_time() >= cfg.GATE_MAX_COAST
            viable = sorted((c for c in cands if c['in_gate'] or c['arm'] == 'match' or kal_expired),
                            key=lambda c: c['cost'])
            best = self._pick_candidate(viable, tid)
            if best is None:
                best = self._reacquire_by_approach(frame, cands, now)
            if best is None:
                best = self._tentative_pick(viable, now)
            if best is None:
                best = self._color_reacquire(viable, now)

        if best is not None and self._pick_reason == 'tentative':
            if best.get('arm_bad'):
                self._tent_noarm += 1
            if self._tent_noarm >= cfg.TENTATIVE_REJECT_FRAMES:
                print(f"[BAM] Che do 5: {self._tent_noarm} lan thay RO tay ma KHONG dung mau tay ao -> chac chan khong phai chu, "
                      f"BO ID {best['tid']}, tim lai")
                self._tent_banned[best['tid']] = now
                self._tent_tid, self._tent_noarm = None, 0
                self._fl_snap = None
                best = None
        if best is None:
            if self._tent_tid is not None:
                self._tent_hist[self._tent_tid] = self._tent_noarm
            self._tent_tid, self._tent_noarm = None, 0
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
        if self._pick_reason in ('match', 'approach', 'color'):
            if self._tent_tid is not None and self._pick_reason == 'match':
                print(f"[BAM] Che do 5: thay TAY AO khop -> xac nhan ID {best['tid']} la chu")
            self._need_reconfirm = False
            self._contested = False
            self._reacq_tid, self._reacq_n, self._reacq_note = None, 0, ""
            self._tent_tid, self._tent_noarm = None, 0
            self._tent_hist = {}

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
        names = {'shirt': 'AO', 'pants': 'QUAN', 'armband': 'TAY AO'}
        return "+".join(names[k] for k in names if self.master_features and k in self.master_features) or "-"

    def _missing_hint(self):
        miss = [n for k, n in (('pants', 'QUAN'), ('armband', 'TAY AO (MAU RUC, KHAC THAN AO)'))
                if self.master_features and k not in self.master_features
                and (k != 'armband' or self._use_armband())]
        if not miss:
            return ""
        return " | DUNG THANG DE QUET " + "/".join(miss) + ((" - " + self._scan_note.upper()) if self._scan_note else "")

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
        master = self._master()
        keys = tuple(master.keys()) if master else ('shirt',)
        view = self._get_view(t_id)
        arms = arms_for(view)
        sideways = view in (SIDE_R, SIDE_L)
        cache_key = (t_id, keys, arms, sideways)
        cached = self._hist_cache.get(cache_key)
        if cached is None:
            feats = extract_all_features(self._color_frame(frame), kpts, only=keys, track=True, arms=arms, sideways=sideways)
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