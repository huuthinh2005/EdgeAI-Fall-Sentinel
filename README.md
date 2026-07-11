# EdgeAI-Fall-Sentinel
# 🚨 Fall-Detection-IoT-STM32

**Hệ thống phát hiện té ngã thời gian thực sử dụng Vi điều khiển nhúng (Edge-AI) và Kiến trúc viễn thông kép (Dual-Communication)**

Đây là kho lưu trữ mã nguồn mở (Repository) thuộc Khóa luận tốt nghiệp của nhóm sinh viên khoa Công nghệ Thông tin. Dự án tập trung giải quyết bài toán phát hiện té ngã cho người cao tuổi và người lao động trong môi trường nguy hiểm thông qua việc nhúng mô hình Học máy (Machine Learning) trực tiếp lên vi điều khiển giới hạn tài nguyên.

---

## 🌟 Tính năng nổi bật (Key Features)

- **Edge-AI Verifier (Thuật toán Lai):** Kết hợp giữa Máy trạng thái hữu hạn (FSM) làm bộ tiền lọc và mô hình Support Vector Machine (SVM) trích xuất 9 đặc trưng động học. Đạt chỉ số **F1-score 94%** trên tập dữ liệu kiểm thử thực tế.
- **Tối ưu hóa DSP (Digital Signal Processing):** Tự động cấu hình cảm biến ở dải đo tối đa ($\pm 16g$, $\pm 2000^\circ/s$) để chống bão hòa tín hiệu (clipping). Tích hợp bộ lọc trung bình (Decimation & Low-pass filter) cộng dồn 20 mẫu để hạ tần số từ 100Hz xuống 5Hz, đồng bộ tuyệt đối với tập dữ liệu SisFall.
- **Viễn thông kép (Dual-Comm):** Cảnh báo song song qua mạng di động (SMS - không phụ thuộc Internet) và giao thức IoT (MQTT - QoS 1) với cơ chế chống đụng độ bộ đệm (UART Collision Avoidance).
- **Tiết kiệm năng lượng (Low-power):** Thiết bị chạy trên xung nhịp dao động nội HSI 8MHz, duy trì dòng tiêu thụ trung bình toàn hệ thống ở mức ~17mA. Đạt thời lượng pin ~63 giờ hoạt động liên tục với viên pin LiPo 1200mAh.
- **Cấu hình từ xa (OTA Configuration):** Cập nhật số điện thoại khẩn cấp trực tiếp qua sóng MQTT và lưu trữ vĩnh viễn vào Flash ROM (Page 62).

---

## 🛠 Kiến trúc Phần cứng (Hardware Architecture)

Hệ thống được thiết kế theo kiến trúc xếp chồng 3 tầng tối ưu cho thiết bị đeo thắt lưng (Waist-mounted wearable):
- **MCU:** STM32F103C8T6 (ARM Cortex-M3).
- **Cảm biến:** MPU6050 (6-DoF Accelerometer & Gyroscope) giao tiếp qua I2C.
- **Viễn thông:** SIM7680C (4G LTE Cat.1 & GNSS/GPS) giao tiếp qua USART2 với cơ chế cấp nguồn cách ly dòng đỉnh 2A.
- **Giao tiếp UI/UX:** Buzzer (Còi hú tiền cảnh báo) & Nút nhấn cứng SOS (Kích hoạt/Hủy báo động).

---

## 📂 Cấu trúc Kho lưu trữ (Repository Structure)

Để đảm bảo tính minh bạch và khả năng tái lập (Reproducibility) theo yêu cầu đánh giá khoa học, kho lưu trữ này chứa toàn bộ các thành phần hệ thống:

* `/firmware_stm32/`: Mã nguồn C/C++ nạp cho vi điều khiển STM32 (Triển khai luồng ngắt Timer 100Hz, thuật toán SVM tuyến tính cực nhẹ thời gian chạy < 1ms, quản lý tập lệnh AT).
* `/ai_training_artifacts/`: (Nơi chứa Dataset và Python Scripts)
  * File Jupyter Notebook xử lý tập dữ liệu SisFall.
  * Script trích xuất đặc trưng và huấn luyện mô hình SVM (Tìm kiếm siêu tham số và xác định ngưỡng phân quyết $\tau = 1.102146$).
* `/android_app/`: Mã nguồn ứng dụng giám sát trên hệ điều hành Android (Kotlin). Giao tiếp với thiết bị qua EMQX Broker và lưu log sự kiện vào Firebase.

---

## 🚀 Giới hạn của Nguyên mẫu (Limitations & Future Work)

1. **Bảo mật OTA:** Chức năng nhận số điện thoại qua MQTT hiện đang hoạt động ở mức nguyên mẫu. Trong tương lai cần tích hợp chứng chỉ TLS/SSL trên EMQX Broker và xác thực định dạng JSON để thiết lập Ranh giới tin cậy (Trust Boundary).
2. **Kích thước mạch:** Phiên bản hiện tại sử dụng linh kiện cắm (Through-hole) phục vụ mục đích kiểm thử. Có thể thu gọn diện tích bằng cách thiết kế mạch in PCB linh kiện dán (SMD).

---

## 👥 Tác giả (Authors)

* **Sinh viên thực hiện:**
  * Bùi Tấn Đạt 
  * Hà Hữu Thịnh 
* **Giảng viên hướng dẫn:** * ThS. Hồ Lê Minh Toàn, TS. Phạm Công Thiện

*Đồ án hoàn thành vào Tháng 07/2026.*
