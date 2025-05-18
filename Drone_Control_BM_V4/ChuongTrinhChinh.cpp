#include "ChuongTrinhChinh.h"
#include "Drone_Control_BM_V4.h"
#include <HardwareSerial.h>
#include "protocol_handler.h"
#include "Arduino.h"
#include <QMC5883LCompass.h>
#include <Wire.h>
#include "config.h"
#include "ESPSoftwareSerial.h"
#include <ArduinoJson.h>

QMC5883LCompass compass;

int dx = 0, dy = 0;
unsigned long ball_start_time = 0;
bool counting = false;
bool ball_releasing = false;
unsigned long wait_for_ball_drop = 0;
//  #define DEBUG_LIDAR          // Bật debug Lidar
// #define DEBUG_SIGNAL_OUTPUT  // Bật debug tín hiệu
//#define readPIDattitude  // PIDthrottle
// #define debugYaw
////////////////////////
bool lastModeManual = false;
bool pitchstate = false;
bool rollstate = false;
int distance = -1;
// Khai báo biến toàn cục
float targetYaw = 0.0;      // Góc mục tiêu của la bàn
float current_angle = 0.0;  // Góc hiện tại
///
volatile int altitude = 0;

unsigned long lidarLastUpdate = 0;
unsigned long QM5883LastUpdate = 0;


int ball_dropped_count = 0;
// Prototype functions
// void readUART3Data();
void PID_target(int x, int y);
void handleFlightState();
void updateControls();
//hàm đọc cảm biến độ cao //
int readLidar();
// ham doc la ban
float readCompassHeading();
/////////////////////////////////////////
void altitude_hold(int targetPos);
bool hold_control(int duration_ms);
bool pitch_control(int target_value, int duration_ms);
bool roll_control(int value, int duration_ms);  // kênh ROLL  kênh 1
void heading_hold(float target);
float calculateAngleDifference(float target, float current);
void drop_ball();
bool checkPositionStability(int targetOffsetX, int targetOffsetY, float toleranceRadius, unsigned long Time_PositionStability_Check);
bool running = false;

// Biến trạng thái hệ thống
unsigned long stateStartTime = 0;

struct StateMessage {
  bool stage0 = false;
  bool stage1 = false;
  bool stage2 = false;
  bool stage3 = false;
  bool stage4 = false;
  bool stage5 = false;
  bool stage6 = false;
  bool stage7 = false;
} msgFlags;
void message_once(State state, const char* msg);
EspSoftwareSerial::UART softSerial;
void KhoiTao() {
  Serial.begin(115200);

  Wire.begin(QCM5883_SDA, QCM5883_SCL);  // SDA = 10 SCL = 11
  compass.init();
  // Cấu hình calibration nếu cần
  // compass.setCalibrationOffsets(686.00, 1303.00, 4370.00);
  // compass.setCalibrationScales(1.88, 1.76, 0.53);
  // compass.setCalibrationOffsets(-521.00, -123.00, 450.00);1
  // compass.setCalibrationScales(0.76, 0.91, 1.73);1


  //của con BN880
  //  compass.setCalibrationOffsets(759.00, 995.00, 2429.00);
  //  compass.setCalibrationScales(1.46, 1.49, 0.61); //

  ///new compass calibration
  compass.setCalibrationOffsets(64.00, 89.00, 67.00);
  compass.setCalibrationScales(1.04, 0.99, 0.98);
  LidarSerial.begin(115200, SERIAL_8N1, LidarSerial_RX, -1);  //19

  // Khởi tạo UART1 cho RC (cả RX và TX)
  SIGNAL_uart.begin(200000, SERIAL_8E2, SIGNAL_RX_PIN, SIGNAL_TX_PIN);
  signal_in.Begin();
  signal_out.Begin();
  targetYaw = readCompassHeading();
  pinMode(BALL_DROP_PIN, OUTPUT);
  digitalWrite(BALL_DROP_PIN, LOW);

  softSerial.begin(BAUD_RATE, EspSoftwareSerial::SWSERIAL_8N1, RXD, TXD, false);

  if (!softSerial) {
    Serial.println("Failed to start software serial on ESP32!");
    while (1)
      ;
  }
  Serial.println("Hệ thống đã sẵn sàng");
}

void ChayChuongTrinhChinh() {
  // Đọc dữ liệu Lidar liên tục
  if ((millis() - lidarLastUpdate) > 2) {
    altitude = readLidar();
    lidarLastUpdate = millis();
    // #ifdef debug
    //   Serial.printf("\ndistance: %d",altitude);
    // #endif
  }

  if ((millis() - QM5883LastUpdate) > 5) {
    current_angle = readCompassHeading();
    QM5883LastUpdate = millis();
#ifdef debugYaw
    Serial.printf("\n current_angle : %f", current_angle);
#endif
  }
  // Xử lý tín hiệu điều khiển
  if (signal_in.Read()) {

    // #ifdef DEBUG_SIGNAL_OUTPUT
    //   Serial.printf("Pitch: %d",data_out.ch[0]);
    // #endif
    data_in = signal_in.data();
    updateControls();
    handleFlightState();
    // PIDthrottle(TARGET_ALTITUDE);
    signal_out.data(data_out);
    signal_out.Write();
  }
}

void updateControls() {
  // Ánh xạ giá trị điều khiển


  // CHUYỂN SANG MANUAL
  if ((data_in.ch[7] > MIN_MANUAL_ATCTIVE && data_in.ch[7] < MAX_MANUAL_ATCTIVE) && !lastModeManual) {
    currentState = MANUAL;
    stateStartTime = millis();
    lastModeManual = true;
#ifdef DEBUG_SIGNAL_OUTPUT
    Serial.println("Manual");
#endif
  }  // CHUYỂN SANG AUTO
  else if ((data_in.ch[7] > MIN_AUTO_ATCTIVE && data_in.ch[7] < MAX_AUTO_ATCTIVE) && lastModeManual) {
#ifdef DEBUG_SIGNAL_OUTPUT
    Serial.println("Auto");
#endif
    lastModeManual = false;
    currentState = AUTO;
    stateStartTime = millis();
  }
  //   } else if ((data_in.ch[7] > MIN_CONFIG_ACTIVE && data_in.ch[7] < MAX_CONFIG_ACTIVE) && !lastModeManual) {
  // #ifdef DEBUG_SIGNAL_OUTPUT
  //     Serial.println("Config");
  // #endif
  //     lastModeManual = true;
  //     currentState = CONFIG;
  //     stateStartTime = millis();
  //   }
  // #ifdef DEBUG_SIGNAL_OUTPUT
  //   Serial.println("--- TÍN HIỆU ĐIỀU KHIỂN ---");
  //   Serial.printf("Roll: %d | Pitch: %d | Throttle: %d | Yaw: %d | Mode: %d\n",
  //                 data_out.ch[0], data_out.ch[1], data_out.ch[2], data_out.ch[3], data_in.ch[7]);
  //   Serial.println("--- SIGNAL đã đọc và xuất ---");
  //   for (int i = 0; i < 12; i++) {  // Hiển thị 12 kênh
  //     Serial.printf("CH%2d In: %4d\tCH%2d Out: %4d\t", i + 1, data_in.ch[i], i + 1, data_out.ch[i]);
  //   }
  //   Serial.printf("\nCH17 In: %d\tCH18 In: %d\tLost Frame In: %d\tFailsafe In: %d", data_in.ch17, data_in.ch18, data_in.lost_frame, data_in.failsafe);
  //   Serial.printf("\nCH17 Out: %d\tCH18 Out: %d\tLost Frame Out: %d\tFailsafe Out: %d\n", data_out.ch17, data_out.ch18, data_out.lost_frame, data_out.failsafe);
  // #endif
}

void handleFlightState() {
  data_out.ch[0] = 997;
  data_out.ch[1] = MIDDLE_SIGNAL_VALUE;
  data_out.ch[2] = MIDDLE_SIGNAL_VALUE;
  data_out.ch[3] = MIDDLE_SIGNAL_VALUE;

  switch (currentState) {
    case MANUAL:
      data_out = data_in;
      break;
    case AUTO:
      Serial.println("Kích hoạt chế độ tự động sau 5s");
      if (millis() - stateStartTime >= WAIT_TO_AUTO) {
        currentState = Stage_1;
        message_once(Stage_1, "Bắt đầu bay tự đông chuyển sang STAGE 2");
      }
      break;
    // case CONFIG:
    //   softSerial.print("1");
    //   // data_out = data_in;
    //   altitude_hold(100);
    //   KpPitch = map(data_out.ch[8], 1000, 2000, 0.1, 10);
    //   KpRoll = map(data_out.ch[8], 1000, 2000, 0.1, 10);

    //   KdPitch = map(data_out.ch[9], 1000, 2000, 0.01, 1);
    //   KdRoll = map(data_out.ch[9], 1000, 2000, 0.01, 1);
    //   PID_target(targetOffsetX, targetOffsetY);
    //   break;
    case Stage_1:
      Serial.println("drone qua trình tăng đạt độ cao");
      altitude_hold(TARGET_ALTITUDE);
      // if (altitude > 50) heading_hold(targetYaw);
#ifdef debug
      Serial.printf("\ndistance:%d cm", altitude);
#endif
      if (altitude >= TARGET_ALTITUDE - ALTITIDE_THRESHOLD && altitude <= TARGET_ALTITUDE + ALTITIDE_THRESHOLD) {
        currentState = Stage_2;
        message_once(Stage_2, "Đạt độ cao ổn định, chuyển qua STAGE 2");
      }
      break;
    case Stage_2:
      // Serial.println("Bắt đầu bay tiến 5s");
      Serial.println("Tạo đà bay tiến 5s");
      // heading_hold(targetYaw);
      if (pitch_control(VALUE_STAGE_2, DURATION_STAGE_2)) {
        currentState = Stage_3;
        message_once(Stage_3, "Đã thả 1 bóng, chuyển qua STAGE 3 ");
      }
      break;
    case Stage_3:  /// Giữ middle trục pitch cho tới vị trí mục tiêu 1 vàng => thả bóng 1
      // Serial.println("bắt đầu bay tiến 2s");
      Serial.println(" Giữ middle trục pitch cho tới vị trí mục tiêu 1 vàng => thả bóng 1");
      // heading_hold(targetYaw);
      // altitude_hold(TARGET_ALTITUDE);
      if (pitch_control(VALUE_STAGE_3, DURATION_STAGE_3)) {
        drop_ball();
        currentState = Stage_4;
        message_once(Stage_4, "Đã thả 2 bóng");
      }
      break;
    case Stage_4:  // // Giữ middle trục pitch cho tới vị trí mục tiêu 2 vàng => Thả bóng 2
      // Serial.println("Drone di chuyển sang 2m trái trong 1.5s");
      Serial.println("Giữ middle trục pitch cho tới vị trí mục tiêu 2 vàng");
      // heading_hold(targetYaw);
      if (pitch_control(VALUE_STAGE_4, DURATION_STAGE_4)) {
        drop_ball();
        currentState = Stage_5;
        message_once(Stage_5, "Đã thả 3 bóng");
      }
      break;
    case Stage_5:  //// Phanh lại giảm tốc độ một chút cho drone trôi về mục tiêu xanh dương (pitch(850,1000))
      // Serial.println("Drone di chuyển 4m sang phải trong 3s");
      Serial.println("Phanh lại giảm tốc độ một chút cho drone trôi về mục tiêu xanh dương");
      // heading_hold(targetYaw);
      if (pitch_control(VALUE_STAGE_5, DURATION_STAGE_5)) {
        // drop_ball();
        currentState = Stage_6;
        message_once(Stage_6, "Đã trôi về mục tiêu xanh dương");
      }
      break;

    case Stage_6:  // / Giữ middle trục pitch 2s rồi thả bóng
      // Serial.println("Bắt đầu di chuyển sang trái 2m trong 1.5s");
      Serial.println("Giữ middle trục pitch 2s rồi thả bóng");
      // heading_hold(targetYaw);
      if (pitch_control(VALUE_STAGE_6, DURATION_STAGE_6)) {
        currentState = Stage_7;
        // message_once(Stage_7, "Chuyển sang Stage 7");
        message_once(Stage_7, "Đã thả 3 quả");
      }
      break;
    case Stage_7:
      // Serial.println("Tiến lên phía trước 3s");
      Serial.println(" Giữ middle trục pitch 2s rồi thả bóng");
      heading_hold(targetYaw);
      // altitude_hold(TARGET_ALTITUDE);
      if (pitch_control(VALUE_STAGE_7, DURATION_STAGE_7)) {
        drop_ball();
        currentState = Stage_8;
        // message_once(LANDING, "Đã thả 5 bóng");
        message_once(Stage_8, "Đã thả 4 bóng");
      }
      break;
    case Stage_8:
      Serial.println(" Giữ middle trục pitch 2s rồi thả bóng");
      // heading_hold(targetYaw);
      // altitude_hold(TARGET_ALTITUDE);
      if (pitch_control(VALUE_STAGE_8, DURATION_STAGE_8)) {
        drop_ball();
        currentState = LANDING;
        // message_once(LANDING, "Đã thả 5 bóng");
        message_once(LANDING, "Đã thả 5 bóng");
      }
      break;
    case LANDING:  // Drone di chuyển sang trái 2m -  roll_control(1400, 1500);
      altitude_hold(LADING_VALUE);
      Serial.println("Đang hạ cánh");
      break;
  }
}

void altitude_hold(int targetAltitude) {
  static long prevT = micros();
  static float eprev = 0, eintegral = 0;
  long currT = micros();
  float deltaT = (currT - prevT) / 1e6;
  prevT = currT;

  float e = targetAltitude - altitude;
  float dedt = (e - eprev) / deltaT;
  eintegral += e * deltaT;

  float u = (kpThrottle * e + kdThrottle * dedt + kiThrottle * eintegral);
  u = constrain(u, -200, PID_ALTITUDE_RANGE);

  data_out.ch[2] = constrain(993 + u, 174, 1810);  // kênh throttle
#ifdef readPIDattitude
  Serial.printf("altitude:%d\n", altitude);
  Serial.printf("u:%d\n", u);
  // Serial.printf("\ne:%d\n", e);
  Serial.printf("\ndata_outCH[2]:%d\n", data_out.ch[2]);
#endif
  eprev = e;
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
  // if (millis() - last_valid > 500) return -1;
  // return last_distance;
}

// bool hold_control(int duration_ms) {
//   static unsigned long startTime_hold = 0;
//   static bool active = false;

//   data_out.ch[0] = MIDDLE_SIGNAL_VALUE;
//   data_out.ch[1] = MIDDLE_SIGNAL_VALUE;
//   data_out.ch[2] = MIDDLE_SIGNAL_VALUE;
//   data_out.ch[3] = MIDDLE_SIGNAL_VALUE;

//   if (!active) {
//       startTime_hold = millis();
//       active = true;
//     }

//     if (active && (millis() - startTime_hold >= duration_ms)) {
//       active = false;
//       return true;
//     } else return false;
// }

bool pitch_control(int value, int duration_ms) {
  static unsigned long startTimeP = 0;
  static bool active = false;
  data_out.ch[1] = value;
  if (!active) {
    startTimeP = millis();
    active = true;
  }

  if (active && (millis() - startTimeP >= duration_ms)) {
    data_out.ch[1] = 993;
    active = false;
    startTimeP = millis();
    return true;
  } else {
    return false;
  }
}


bool roll_control(int value, int duration_ms) {
  static unsigned long startTimeR = 0;
  static bool active = false;
  data_out.ch[0] = value;
  if (!active) {
    startTimeR = millis();
    active = true;
  }

  if (active && (millis() - startTimeR >= duration_ms)) {
    data_out.ch[0] = 993;
    active = false;
    return true;
  } else return false;
}

float readCompassHeading() {
  compass.read();
  int x = compass.getX();
  int y = compass.getY();
  float heading = atan2(y, -x) * 180.0 / M_PI;
  return (heading < 0) ? heading + 360 : heading;
}

float calculateAngleDifference(float target, float current) {
  float diff = target - current;
  diff = fmod(diff + 180, 360) - 180;
  return diff < -180 ? diff + 360 : diff;
}
void heading_hold(float target) {
  // Hệ số PID
  static long Lasttime = micros();
  static float eprevYAW = 0, eintegralYAW = 0;
  long now = micros();
  float deltaT = (now - Lasttime) / 1e6;
  Lasttime = now;

  float eYaw = target - current_angle;
  float dedt = (eYaw - eprevYAW) / deltaT;
  eintegralYAW += eYaw * deltaT;

  float u = (KpYaw * eYaw + KdYaw * dedt + KiYaw * eintegralYAW);
  constrain(u, -PID_HEADING_RANGE, PID_HEADING_RANGE);
  // Gán PWM cho kênh YAW (giới hạn từ 1000–2000)
  data_out.ch[3] = constrain(993 + (int)u, 174, 1810);  // kenh so 4 , YAW
  // Serial.printf("Kênh YAW: %d, sai số e :%d\n", data_out, e);

  eprevYAW = eYaw;

#ifdef debugYaw
  Serial.print("Góc: ");
  Serial.print(current_angle);
  Serial.print(" | PWM YAW: ");
  Serial.println(data_out.ch[3]);
#endif
}


void message_once(State state, const char* msg) {
  switch (state) {
    case Stage_1:
      if (!msgFlags.stage1) {
#ifdef debug
        Serial.println(msg);
#endif
        msgFlags.stage1 = true;
      }
      break;
    case Stage_2:
      if (!msgFlags.stage2) {
#ifdef debug
        Serial.println(msg);
#endif
        msgFlags.stage2 = true;
      }
      break;
    case Stage_3:
      if (!msgFlags.stage3) {
#ifdef debug
        Serial.println(msg);
#endif
        msgFlags.stage3 = true;
      }
      break;
    case Stage_4:
      if (!msgFlags.stage4) {
#ifdef debug
        Serial.println(msg);
#endif
        msgFlags.stage4 = true;
      }
      break;
    case Stage_5:
      if (!msgFlags.stage5) {
#ifdef debug
        Serial.println(msg);
#endif
        msgFlags.stage5 = true;
      }
      break;
    case Stage_6:
      if (!msgFlags.stage6) {
#ifdef debug
        Serial.println(msg);
#endif
        msgFlags.stage6 = true;
      }
      break;
    case Stage_7:
      if (!msgFlags.stage7) {
#ifdef debug
        Serial.println(msg);
#endif
        msgFlags.stage7 = true;
      }
      break;
  }
}
void readUART3Data() {
  while (softSerial.available()) {
    char c = softSerial.read();
    input += c;

    if (c == '}') {
      StaticJsonDocument<200> doc;
      DeserializationError error = deserializeJson(doc, input);

      if (!error) {
        int cmd = doc["cmd"];
        targetOffsetX = doc["dx"];
        targetOffsetY = doc["dy"];

        Serial.printf("cmd: %d, dx: %d, dy: %d\n", cmd, dx, dy);

        // Gửi phản hồi trở lại (echo)
        softSerial.printf("{\"status\":\"ok\",\"cmd\":%d}\n", cmd);

        if (cmd == 1) {
          Serial.println("Thực hiện hành động CMD 1...");
        }
      } else {
        Serial.print("JSON parse error: ");
        Serial.println(error.c_str());
      }

      input = "";  // Reset buffer
    }

    if (input.length() > 200) {
      input = "";
      Serial.println("Input quá dài.");
    }
  }
}

void PID_target(int x, int y) {
  // Điều khiển Pitch dựa trên y
  static long LasttimeY = micros();
  static float last_pitch_error = 0, pitch_integral = 0;
  long nowY = micros();
  float deltaTY = (nowY - LasttimeY) / 1e6;
  float pitch_error = y;
  pitch_integral += pitch_error * deltaTY;  // dt = 10ms
  float pitch_derivative = (pitch_error - last_pitch_error) / deltaTY;
  float pitch_output = KpPitch * pitch_error + KiPitch * pitch_integral + KdPitch * pitch_derivative;
  constrain(pitch_output, -400, 400);


  static long LasttimeX = micros();
  static float last_roll_error = 0, roll_integral = 0;
  long nowX = micros();
  float deltaTX = (nowX - LasttimeX) / 1e6;
  float roll_error = x;
  roll_integral += roll_error * deltaTX;
  float roll_derivative = (roll_error - last_roll_error) / deltaTX;
  float roll_output = KpRoll * roll_error + KiRoll * roll_integral + KdRoll * roll_derivative;
  constrain(roll_output, -400, 400);

  // Cập nhật giá trị điều khiển
  data_out.ch[1] = constrain(993 + (int)pitch_output, 174, 1810);  // Pitch
  data_out.ch[0] = constrain(993 + (int)roll_output, 174, 1810);   // Roll

  // Kiểm tra điều kiện thả bóng

  last_pitch_error = pitch_error;
  last_roll_error = roll_error;
}
void drop_ball() {
  data_out.ch[4] = BALL_DROP_TRIGGER;
  signal_out.data(data_out);
  signal_out.Write();
  delay(1000);
  data_out.ch[4] = BALL_DROP_IDLE;
}

// bool drop_ball(){
//  static bool ball_releasing = false;
//  static unsigned long ball_start_time = 0;

//   if (!ball_releasing) {
//     data_out.ch[4] = BALL_DROP_TRIGGER;
//     ball_start_time = millis();
//     ball_releasing = true;
//   } else if (ball_releasing && (millis() - ball_start_time >= BALL_DROP_WAIT)) {
//     data_out.ch[4] = BALL_DROP_IDLE;
//     ball_dropped_count++;
//     Serial.printf("Đã thả bóng! Tổng: %d\n", ball_dropped_count);
//     ball_releasing = false;
//     return true;
//   } else return false;
// }