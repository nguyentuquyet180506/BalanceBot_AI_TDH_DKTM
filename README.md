# PROJECT_TDH_DKTM — Xe tự cân bằng 2 ESP32-S3 + AI bám người (UDP)

| Phần | Làm gì | Ở đâu |
|---|---|---|
| **ESP32-S3 không cam** (S3-N16R8) | CHỈ giữ cân bằng, nhận lệnh và trả phản hồi qua **UART**. **Không Wi-Fi** (ít dòng nhất) | `firmware/balance_esp` |
| **ESP32-S3 có cam** (ESP32-S3-CAM) | Stream ảnh **UDP** về PC, web điều khiển, nhận lệnh UDP, chuyển lệnh xuống ESP cân bằng qua UART | `firmware/cam_esp` |
| **PC** | AI (YOLO-pose, mặt, màu áo, tay áo khác màu), 5 chế độ, gửi lệnh **UDP** | `pc/` |
| Dùng chung | Khung UART + bảng giới hạn thông số | `firmware/common/robot_link.h` |

```
 Webcam PC (tìm chủ) ─┐
 PC (AI) <──UDP 4210 (ảnh + lệnh + phản hồi)──> ESP CAM <──UART 460800──> ESP CÂN BẰNG ── MPU6500, TB6612, 2 encoder
 Điện thoại <──HTTP (web)─────────────────────> (Wi-Fi)                    (không Wi-Fi)
```

---

## 1. Nối dây UART giữa 2 ESP

| ESP cân bằng | | ESP cam |
|---|---|---|
| GPIO **21** (TX) | ──► | **G14** (RX) |
| GPIO **42** (RX) | ◄── | **G21** (TX) |
| GND | ─── | GND |

## 2. Nạp firmware — 2 lệnh độc lập

```bat
upload_balance.bat COM5     :: ESP cân bằng
```
```bat
upload_cam.bat COM6         :: ESP cam (cắm cổng TTL)
```

## 3. Chạy phần máy tính

```bat
run.bat                 :: mặc định chế độ 3 (xem cam, xe không tự chạy)
run.bat --mode 4        :: mở thẳng chế độ bám người áo + quần
run.bat --mode 5        :: mở thẳng chế độ bám người áo + quần + tay áo khác màu
control.bat             :: chỉ cửa sổ cần gạt UDP
view.bat                :: chỉ xem camera
```

### 5 chế độ — gõ trong TERMINAL rồi Enter (hoặc bấm phím số trên cửa sổ video)

| Gõ | Chế độ | Ảnh | Ai lái |
|---|---|---|---|
| `1` | Tay | tắt | Web hoặc cửa sổ cần gạt UDP (tự mở) |
| `2` | Cử chỉ | webcam → **Y** → cam ESP | AI |
| `3` | Xem cam dò đường | cam ESP, không AI | Cửa sổ cần gạt / web |
| `4` | Bám người: **áo + quần** | webcam → **Y** → cam ESP | AI; không quét / không so tay áo |
| `5` | Bám người: **áo + quần + tay áo khác màu** | webcam → **Y** → cam ESP | AI; tay áo để xác nhận chủ |

Phím tắt: **N** đăng ký mặt · **R** bỏ chủ (quay lại webcam) · **S** bật/tắt cử chỉ · **Q** thoát · **1–5** đổi chế độ · **Y** chuyển sang cam ESP · **X** DISARM khẩn cấp.

### Webcam máy tính → Y → cam ESP (chế độ 2, 4, 5)
1. Vào chế độ: tìm mặt chủ và quét màu áo bằng **webcam máy tính**. AI vẫn chạy trên webcam, **xe đứng cân bằng tại chỗ**.
2. Bấm **Y**: chuyển sang cam ESP. Xe **không quét lại màu** mà **quét xác nhận chủ**: so người trước cam với màu áo / quần (/ tay áo ở chế độ 5)
3. **R**: bỏ chủ, quay lại webcam.

### Thanh trượt và giao diện
- Thanh **"Toc do tick/s"** (0–3600) và **"Xoay do/s"** (0–90) trên cửa sổ video chỉnh tốc độ AI. Chế độ 2, 4, 5 nhớ số riêng.
- Web và cửa sổ cần gạt cũng lên tới **3600 tick/s và 90°/s**.
  - Thông số đã "Lưu khởi động" (NVS) được ưu tiên hơn số trong code. Muốn đổi trần: DISARM, nhập maxSpeed / maxYawRate, bấm Áp dụng rồi Lưu khởi động.
- **3 ô màu** cỡ móng tay ở góc trên phải: **áo | quần | tay áo**. Ô trống là chưa quét; ô gạch chéo (chế độ 4) là không dùng tay áo.
- Cỡ chữ chỉnh bằng `UI_TEXT_SCALE`, cỡ ô màu bằng `UI_CHIP_FRAC`.

---

## 4. Nguyên lý hoạt động

**ESP cân bằng** (`firmware/balance_esp`)
- Vòng điều khiển **200 Hz** (tác vụ ưu tiên cao trên core 1) là nơi duy nhất đọc IMU, ghi motor, đổi gain.
- Góc nghiêng: lọc bù **98 % gyro + 2 % gia tốc kế** từ MPU6500. Tốc độ / quãng đường: 2 encoder.
- Luật điều khiển phản hồi trạng thái `u = -K·[x - xRef, v - vRef, pitch - offset, gyroX]`; hướng `yaw` giữ bằng PD trên gyro Z.
- Lệnh chạy (từ ESP cam qua UART) chỉ được bỏ vào **hộp thư**; vòng 200 Hz tự kiểm tra epoch / thời hạn / giới hạn rồi mới dùng.
  Quá **600 ms** không có lệnh mới (mất lệnh, mất dây UART) thì giảm tốc dừng tại chỗ và vẫn cân bằng.
- Nghiêng quá **42°** = đổ: tắt motor. Dựng thẳng lại, giữ yên **3 s** thì tự cân bằng lại, lấy chỗ và hướng mới làm mốc.
  DISARM bằng tay hoặc lỗi cảm biến thì không tự arm. Không Wi-Fi để ít dòng và không nhiễu vòng cân bằng.

**ESP cam** (`firmware/cam_esp`)
- Chụp JPEG (OV5640), pipeline 2 tác vụ + hàng đợi 1 phần tử: luôn gửi khung **mới nhất**, mạng chậm thì bỏ khung cũ.
- Gửi ảnh qua **UDP 4210**, mỗi khung cắt thành gói 1400 byte + header 8 byte; PC giữ kết nối bằng gói `HELLO` ~1 s/lần.
- Chống nóng: theo nhiệt độ chip tự hạ XCLK / FPS / tăng nén JPEG; rảnh thì hạ xung CPU và cho Wi-Fi ngủ.
- Web điều khiển, IP tĩnh 192.168.1.200, AP `BalanceBot-Web`, mDNS; lệnh UDP dùng chung cổng 4210 với camera.
  `loop()` ngủ bằng `select()` trên socket UDP nên lệnh tới là xử lý ngay.
- Liên kết UART 460800 baud với ESP cân bằng: khung `A5 5A | TYPE | LEN | PAYLOAD | CRC8`, telemetry 50 Hz.
- Không gửi được ảnh thì gửi lý do về PC và in ra Serial; camera treo thì tự khởi động lại camera.

**PC** (`pc/`)
- YOLOv8n-pose (người + 17 keypoint + ID theo dõi), YuNet + face_recognition (nhận mặt chủ lúc đầu).
- Màu áo / quần / tay áo lưu dạng histogram H×S×V, so "mềm"; Kalman vận tốc không đổi dự đoán vị trí chủ làm cổng chọn người.
- Lệnh AI gửi xuống xe đều **50 Hz** dạng cần ảo trượt mượt (v, w); thao tác hẹn giờ (quay đúng số độ) do PC giữ nhịp.

