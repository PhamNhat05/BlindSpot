# Hệ Thống Cảnh Báo Điểm Mù

## Linh Kiện

| Linh kiện                 | Số lượng |
|---------------------------|----------|
| STM32F103C8T6 (Blue Pill) |     1    |
| Cảm biến siêu âm AJ-SR04M |     2    |
| LED                       |     3    |
| Buzzer                    |     1    |
| Nút nhấn                  |     1    |
| LCD 16x2 I2C              |     1    |

## Nguyên Lý Hoạt Động

MCU phát xung **TRIG 10µs** → cảm biến bắn sóng siêu âm → đo thời gian chân **ECHO** phản hồi → tính khoảng cách.

- **< 50 cm:** LED sáng + còi kêu (DANGER)
- **50–100 cm:** LED nhấp nháy (WARNING)
- **> 100 cm:** Tắt hết (SAFE)

Nút nhấn dùng để **tắt/bật còi** (mute). Kết quả hiển thị lên LCD qua I2C. 
Toàn bộ chạy đa nhiệm với **FreeRTOS** (3 task: đọc cảm biến, xử lý cảnh báo, hiển thị LCD).

## Phần Mềm

- **Keil MDK (µVision)** — IDE biên dịch và nạp firmware cho STM32