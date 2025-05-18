#include <HardwareSerial.h>
#include "protocol_handler.h"

// Định nghĩa macro bật/tắt in debug
#define DEBUG_SIGNAL_OUTPUT 

// Định nghĩa chân RX cho SIGNAL (thay đổi nếu cần)
#define SIGNAL_RX_PIN 17 // Ví dụ: sử dụng chân GPIO16
#define SIGNAL_TX_PIN 18 // Chân GPIO15 để xuất SIGNAL

// Khởi tạo UART1 cho cả RX và TX
HardwareSerial SIGNAL_uart(1);
// Khởi tạo đối tượng SIGNAL
bfs::signalRx signal_in(&SIGNAL_uart, SIGNAL_RX_PIN, SIGNAL_TX_PIN, true);
bfs::signalTx signal_out(&SIGNAL_uart, SIGNAL_RX_PIN, SIGNAL_TX_PIN, true);
bfs::signalData data_in, data_out;

void setup() {
  Serial.begin(115200);
  Serial.println("Bắt đầu đọc dữ liệu  điều khiển...");

  // Khởi tạo cổng nối tiếp
  SIGNAL_uart.begin(200000, SERIAL_8E2, SIGNAL_RX_PIN, SIGNAL_TX_PIN);
  signal_in.Begin();
  signal_out.Begin();
}

void loop() {
  // Đọc dữ liệu điều khiển in
  if (signal_in.Read()) {
  data_in = signal_in.data();
  // data_out = data_in; // Gán trực tiếp

  
  // Sao chép trạng thái (nếu cần)
  // data_out.lost_frame = data_in.lost_frame;
  // data_out.failsafe = data_in.failsafe;
  // data_out.ch[2] = 1400;

  signal_out.data(data_out);
  signal_out.Write();

  #ifdef DEBUG_SIGNAL_OUTPUT
      Serial.println("--- SIGNAL đã đọc và xuất ---");
      for (int i = 0; i < 12; i++) { // Hiển thị 12 kênh
        Serial.printf("CH%2d In: %4d\tCH%2d Out: %4d\t", i + 1, data_in.ch[i], i + 1, data_out.ch[i]);
      }
      Serial.printf("\nCH17 In: %d\tCH18 In: %d\tLost Frame In: %d\tFailsafe In: %d", data_in.ch17, data_in.ch18, data_in.lost_frame, data_in.failsafe);
      Serial.printf("\nCH17 Out: %d\tCH18 Out: %d\tLost Frame Out: %d\tFailsafe Out: %d\n", data_out.ch17, data_out.ch18, data_out.lost_frame, data_out.failsafe);
  #endif
  }
  //delay(2);
}