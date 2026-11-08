import cv2
from _2_ai_models import load_encodings, _resolve_path
from _2_import_images import import_images_from_folders

def main():
    detector = cv2.FaceDetectorYN.create(
        model=_resolve_path("face_detection_yunet_2023mar.onnx"), config="",
        input_size=(640, 480), score_threshold=0.5, nms_threshold=0.3, top_k=5000
    )
    known = load_encodings()
    n = import_images_from_folders(known, detector)

    print("\n===== KET QUA =====")
    print(f"Anh moi da hoc: {n}")
    for name, encs in sorted(known.items()):
        print(f"  {name}: {len(encs)} encoding")
    if not known:
        print("Chua co ai. Hay bo anh vao face_data/<TEN_NGUOI>/ roi chay lai.")

if __name__ == "__main__":
    main()
    input("\nXong. Nhan Enter de dong...")