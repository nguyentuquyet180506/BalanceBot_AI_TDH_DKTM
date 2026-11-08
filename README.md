# BalanceBot AI — xe cân bằng 2 ESP32-S3 + AI bám người (UDP)

| Phần | Làm gì | Ở đâu |
|---|---|---|
| **ESP32-S3 không cam** (S3-N16R8) | CHỈ giữ cân bằng, nhận lệnh và trả phản hồi qua **UART**. **Không Wi-Fi** | `firmware/balance_esp` |
| **ESP32-S3 có cam** (ESP32-S3-CAM) | Stream ảnh **UDP** về PC, web điều khiển, nhận lệnh UDP, chuyển lệnh xuống ESP cân bằng qua UART | `firmware/cam_esp` |
| **PC** | AI (YOLO-pose, mặt, màu áo, băng tay), 4 chế độ, gửi lệnh **UDP** | `pc/` |
| Dùng chung | Khung UART + bảng giới hạn thông số | `firmware/common/robot_link.h` |

---

## 1. Nối dây UART giữa 2 ESP

| ESP cân bằng | | ESP cam |
|---|---|---|
| GPIO **21** (TX) | ──► | **G14** (RX) |
| GPIO **42** (RX) | ◄── | **G21** (TX) |

## 2. Nạp firmware — 2 lệnh độc lập

```bat
upload_balance.bat COMx     :: ESP cân bằng
```
```bat
upload_cam.bat COMx         :: ESP cam (cắm cổng TTL)
```

## 3. Chạy phần máy tính

```bat
run.bat                 :: mặc định chế độ 4 (xem cam, xe không tự chạy)
run.bat --mode 3        :: mở thẳng chế độ bám người
control.bat             :: chỉ cửa sổ cần gạt UDP
view.bat                :: chỉ xem camera
```

### 4 chế độ — gõ trong TERMINAL rồi Enter (hoặc bấm phím số trên cửa sổ video)

| Gõ | Chế độ | Ảnh | Ai lái |
|---|---|---|---|
| `1` | Tay | tắt | Web hoặc cửa sổ cần gạt UDP (tự mở) |
| `2` | Cử chỉ | webcam → **Y** → cam ESP | AI |
| `3` | Bám người | webcam → **Y** → cam ESP | AI; mất dấu thì quay 30° mỗi 6 s |
| `4` | Xem cam dò đường | cam ESP, không AI | Cửa sổ cần gạt / web |

Phím : **N** đăng ký mặt · **R** bỏ chủ (quay lại webcam) · **S** bật/tắt cử chỉ · **Q** thoát. **1–4** đổi chế độ, **Y** chuyển sang cam ESP, **X** DISARM khẩn cấp.

### Webcam máy tính → Y → cam ESP (chế độ 2, 3)
1. Vào chế độ: tìm mặt chủ và quét màu áo bằng **webcam máy tính**. AI vẫn chạy trên webcam, **xe đứng cân bằng tại chỗ**.
2. Bấm **Y**: chuyển sang cam ESP.
3. **R**: bỏ chủ, quay lại webcam.

Cấu hình trong `pc/TRACK1/_0_config.py`:
- `PC_CAM_INDEX`: số webcam.
- `HANDOVER_RESCAN_COLORS = False`: giữ màu đã quét trên webcam thay vì quét lại.

### Cử chỉ (chế độ 2) — mỗi cử chỉ là một thao tác trọn vẹn
- **Tay phải giơ lên:** tiến 3 s. **Tay trái giơ lên:** lùi 3 s. Không tự chỉnh vị trí.
- **Dang tay phải / trái:** quay 86°, chạy 1 s, quay ngược lại 86°, rồi trong tối đa 5 s tự xoay về phía chủ và tiến lại nếu quá xa (không lùi).
- **Bắt chéo tay:** dừng ngay. Bấm **S** để nhận lệnh lại.
- Muốn lặp lại cùng một lệnh thì buông tay rồi làm lại cử chỉ. Người que luôn được vẽ.

### Bám người (chế độ 3)
- Xe giữ chủ **gần** tâm khung hình chứ không ép đúng tâm: lệch ≤ 15 % thì không xoay, nên không giật.
- Khoảng cách lấy theo chiều cao người trong ảnh: xa thì tiến, quá sát thì lùi.
- Mất dấu quá 1 s: quay 30° về phía chủ biến mất, cứ 6 s quay một lần cho tới khi thấy lại.
- Nếu xe xoay ngược hướng người thì đặt `FOLLOW_TURN_SIGN = -1`.

### Thanh trượt và giao diện
- Thanh **"Toc do tick/s"** (0–3600) và **"Xoay do/s"** (0–90) trên cửa sổ video chỉnh tốc độ AI. Chế độ 2 và 3 nhớ số riêng.
- Web và cửa sổ cần gạt cũng lên tới **3600 tick/s và 90°/s**.
  - Nếu trước đây đã bấm "Lưu khởi động" trên web, xe sẽ vẫn đọc số cũ trong bộ nhớ. Khi đó: DISARM, nhập maxSpeed 3600 / maxYawRate 90, bấm Áp dụng rồi Lưu khởi động.
- **3 ô màu** cỡ móng tay ở góc trên phải: **áo | quần | băng tay**. Ô trống là chưa quét.
- Cỡ chữ chỉnh bằng `UI_TEXT_SCALE`, cỡ ô màu bằng `UI_CHIP_FRAC`.

