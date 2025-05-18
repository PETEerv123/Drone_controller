#include "PID.h"

void PID ::altitude_hold(int targetAltitude, int current_Atitude,float* data_u) {
  static long prevT = micros();
  static float eprev = 0, eintegral = 0;
  long currT = micros();
  float deltaT = (currT - prevT) / 1e6;
  prevT = currT;

  float e = targetAltitude - current_Atitude;
  float dedt = (e - eprev) / deltaT;
  eintegral += e * deltaT;

  float u = (kpThrottle * e + kdThrottle * dedt + kiThrottle * eintegral);
  u = constrain(u, -PID_ALTITUDE_RANGE, PID_ALTITUDE_RANGE);

  *data_u = constrain(993 + u, 174, 1810);  // kênh throttle
  eprev = e;
#ifdef readPIDattitude
  Serial.printf("altitude:%d\n", current_Atitude);
  Serial.printf("u:%.2f\n", (int)u);
  // Serial.printf("\ne:%d\n", e);
  Serial.printf("data_outCH[2]:%.2f\n", *data_u);
#endif
}


void PID::heading_hold(float targethold, float current_angle,float* data_u) {
  // Hệ số PID
  static long Lasttime = micros();
  static float eprevYAW = 0, eintegralYAW = 0;
  long now = micros();
  float deltaT = (now - Lasttime) / 1e6;
  Lasttime = now;

  float eYaw = targethold - current_angle;
  float dedt = (eYaw - eprevYAW) / deltaT;
  eintegralYAW += eYaw * deltaT;

  float u = (KpYaw * eYaw + KdYaw * dedt + KiYaw * eintegralYAW);
  constrain(u, -PID_HEADING_RANGE, PID_HEADING_RANGE);
  // Gán PWM cho kênh YAW (giới hạn từ 1000–2000)
  *data_u = constrain(993 + u, 174, 1810);  // kenh so 4 , YAW
  // Serial.printf("Kênh YAW: %d, sai số e :%d\n", data_out, e);

  eprevYAW = eYaw;

#ifdef debugYaw
  Serial.printf("Góc:%.2f\n", current_angle);
  Serial.printf("PWM YAW:%d", *data_u);
#endif
}
void PID::target(int targetx, int targety,int current_x,int current_y,int* data_x,int* data_y ) {
  // Điều khiển Pitch dựa trên y
  static long LasttimeY = micros();
  static float last_pitch_error = 0, pitch_integral = 0;
  long nowY = micros();
  float deltaTY = (nowY - LasttimeY) / 1e6;
  float pitch_error = targety - current_y;
  pitch_integral += pitch_error * deltaTY;  // dt = 10ms
  float pitch_derivative = (pitch_error - last_pitch_error) / deltaTY;
  float pitch_output = KpPitch * pitch_error + KiPitch * pitch_integral + KdPitch * pitch_derivative;
  constrain(pitch_output, -400, 400);


  static long LasttimeX = micros();
  static float last_roll_error = 0, roll_integral = 0;
  long nowX = micros();
  float deltaTX = (nowX - LasttimeX) / 1e6;
  float roll_error = targetx - current_x ;
  roll_integral += roll_error * deltaTX;
  float roll_derivative = (roll_error - last_roll_error) / deltaTX;
  float roll_output = KpRoll * roll_error + KiRoll * roll_integral + KdRoll * roll_derivative;
  constrain(roll_output, -400, 400);

  // Cập nhật giá trị điều khiển
  *data_x = constrain(993 + (int)pitch_output, 174, 1810);  // Pitch
  *data_y = constrain(993 + (int)roll_output, 174, 1810);   // Roll

  // Kiểm tra điều kiện thả bóng

  last_pitch_error = pitch_error;
  last_roll_error = roll_error;
#ifdef debugPitchRoll_PID
  Serial.print("X:%d,Y:%d\n", *data_x, *data_y );
#endif
}