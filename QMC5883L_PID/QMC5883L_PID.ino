#include <QMC5883LCompass.h>
#include <math.h>
#include "Wire.h"

QMC5883LCompass compass;
int data_out = 0;

float target_angle = 0;   // Góc mục tiêu
float current_angle = 0;  // Góc hiện tại


void setup() {
  Serial.begin(115200);
  Serial.println("Khởi động hệ thống...");
  Wire.begin(10, 11);  // Bỏ comment nếu bạn dùng chân SDA/SCL tùy chỉnh

  compass.init();
  // compass.setCalibrationOffsets(686.00, 1303.00, 4370.00);
  // compass.setCalibrationScales(1.88, 1.76, 0.53);

  compass.setCalibrationOffsets(-312.00, 131.00, 61.00);
  compass.setCalibrationScales(0.82, 1.11, 1.13);
  // Khởi tạo giá trị PWM mặc định
  // Đặt góc mục tiêu ban đầu
  target_angle = readCompassHeading();
}

void loop() {
  current_angle = readCompassHeading();
  YAW_CONTROLL(target_angle);

  Serial.print("Góc: ");
  Serial.print(current_angle);
  Serial.print(" | PWM YAW: ");
  Serial.println(data_out);

  delay(20);  // Chu kỳ điều khiển
}

void YAW_CONTROLL(float target) {
  static float kpYAW = 5;
  static float kiYAW = 0.05;
  static float kdYAW = 0.0;
  // Hệ số PID
  static long Lasttime = micros();
  static float eprevYAW = 0, eintegralYAW = 0;
  long now = micros();
  float deltaT = (now - Lasttime) / 1e6;
  Lasttime = now;

  float eYaw = target - current_angle;
  float dedt = (eYaw - eprevYAW) / deltaT;
  eintegralYAW += eYaw * deltaT;

  float u = (kpYAW * eYaw + kdYAW * dedt + kiYAW * eintegralYAW);
  constrain(u, -400, 400);
  // Gán PWM cho kênh YAW (giới hạn từ 1000–2000)
  data_out = constrain(993 + (int)u, 173, 1810);
  // Serial.printf("Kênh YAW: %d, sai số e :%d\n", data_out, e);
  eprevYAW = eYaw;
}

float readCompassHeading() {
  compass.read();
  int x = compass.getX(), y = compass.getY();
  float heading = atan2(y, -x) * 180.0 / M_PI;
  return (heading < 0) ? heading + 360 : heading;
}
float calculateAngleDifference(float angle1, float angle2) {
  float diff = angle1 - angle2;
  while (diff > 180) diff -= 360;
  while (diff < -180) diff += 360;
  return diff;
}

