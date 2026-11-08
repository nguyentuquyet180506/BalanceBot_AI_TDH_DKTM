import queue
import threading
import traceback
import face_recognition
import numpy as np

from _0_config import FACE_TOLERANCE, FACE_TOLERANCE_PROFILE_BONUS
from _3_face_filters import compute_encoding

face_job_queue = queue.Queue(maxsize=2)
face_result_queue = queue.Queue(maxsize=2)

def async_face_recognizer_worker(known_face_encodings, tolerance=FACE_TOLERANCE):
    while True:
        try:
            job = face_job_queue.get(timeout=0.2)
        except queue.Empty:
            continue

        if job is None:
            break

        try:
            bgr_frame, face_box, track_id, is_difficult_pose, job_epoch = job

            effective_tolerance = tolerance + (
                FACE_TOLERANCE_PROFILE_BONUS if is_difficult_pose else 0.0
            )
            best_person = None
            best_distance = effective_tolerance + 1e-6

            enc_snapshot = dict(known_face_encodings)

            if enc_snapshot:
                enc, _ = compute_encoding(bgr_frame, face_box)
                if enc is not None:
                    for person_id, db_encs in enc_snapshot.items():
                        if not db_encs:
                            continue

                        enc_list = db_encs if isinstance(db_encs, list) else [db_encs]
                        dists = face_recognition.face_distance(enc_list, enc)
                        if len(dists) == 0:
                            continue

                        min_dist = np.min(dists)
                        if min_dist < best_distance:
                            best_distance = min_dist
                            best_person = person_id

            final_name = best_person if (best_person and best_distance <= effective_tolerance) else None

            if face_result_queue.full():
                try:
                    face_result_queue.get_nowait()
                except queue.Empty:
                    pass

            face_result_queue.put((final_name, best_distance, face_box, track_id, job_epoch))

        except Exception:
            traceback.print_exc()
            continue


def init_async_face_thread(known_face_encodings, tolerance=FACE_TOLERANCE):
    t = threading.Thread(
        target=async_face_recognizer_worker,
        args=(known_face_encodings, tolerance),
        daemon=True
    )
    t.start()
    return t