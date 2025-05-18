#include <HardwareSerial.h>

HardwareSerial LidarSerial(2); // UART1

void setup() {
  Serial.begin(115200);       // Monitor
  LidarSerial.begin(115200, SERIAL_8N1, 19, -1); // TF-Luna, RX=16, TX=17

  Serial.println("Đang khởi động TF-Luna...");
}

void loop() {

 int distance  = readLidar();
 Serial.printf("\ndistance : %d cm ",distance);

}

int readLidar() {
  static uint8_t buf[9];
  static uint32_t last_valid = 0;
  static int last_distance = -1;

  // Đọc từng byte và tìm header
  while (LidarSerial.available() > 0) {
    // Dịch buffer
    memmove(buf, buf + 1, 8);     // Dịch 8 byte đầu sang trái
    buf[8] = LidarSerial.read();  // Thêm byte mới vào cuối

    // Kiểm tra header 0x59 0x59 ở vị trí [0] và [1]
    if (buf[0] == 0x59 && buf[1] == 0x59) {
      // Tính checksum
      uint8_t checksum = 0;
      for (int i = 0; i < 8; i++) checksum += buf[i];

      // Xác thực checksum
      if (checksum == buf[8]) {
        uint16_t distance = buf[2] + (buf[3] << 8);  // Byte 2-3: khoảng cách
        last_valid = millis();
        last_distance = distance;
        return distance;
      }
    }
  }
  // Timeout 200ms
  if (millis() - last_valid > 500) return -1;
  return last_distance;
}