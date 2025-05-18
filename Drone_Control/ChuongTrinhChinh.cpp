#include "ChuongTrinhChinh.h"
#include "Drone_Control.h"
#include <HardwareSerial.h>
#include "protocol_handler.h"
#include "Arduino.h"
#include <QMC5883LCompass.h>
#include "math.h"
#include <Wire.h>
#include "config.h"
#include "ESPSoftwareSerial.h"
#include <ArduinoJson.h>
#include "PID.h"
QMC5883LCompass compass;
PID pid;


bool ball_releasing = false;
unsigned long wait_for_ball_drop = 0;
////////////////////////
bool lastModeManual = false;
bool pitchstate = false;
bool rollstate = false;
int distance = -1;
// Khai báo biến toàn cục
float targetYaw = 0.0;    // Góc mục tiêu của la bàn
float current_angle = 0;  // Góc hiện tại
///
volatile int altitude = 0;

unsigned long lidarLastUpdate = 0;
unsigned long QM5883LastUpdate = 0;

int targetOffsetX = 0;
int targetOffsetY = 0;

int ball_dropped_count = 0;
// Prototype functions
// void readUART3Data();
void handleFlightState();
void updateControls();
//hàm đọc cảm biến độ cao //
int readLidar();
// ham doc la ban
float readCompassHeading();
/////////////////////////////////////////
bool hold_control(int duration_ms);
bool pitch_control(int target_value, int duration_ms);
bool roll_control(int value, int duration_ms);  // kênh ROLL  kênh 1
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
struct sendingMessage {
  bool stage_0 = false;
  bool stage_1 = false;
  bool stage_2 = false;
  bool stage_3 = false;
  bool stage_4 = false;
  bool stage_5 = false;
  bool stage_6 = false;
  bool stage_7 = false;
} FlagsUart;
void sendRAS(State state, const char* msg);
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

  ///V1
  // compass.setCalibrationOffsets(-312.00, 131.00, 61.00);
  // compass.setCalibrationScales(0.82, 1.11, 1.13);

  //V2
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
  Serial.println("ESP32 SoftwareSerial Ready.");
  softSerial.printf("hello\n");
  delay(800);
  softSerial.printf("hello\n");
  delay(800);
  Serial.println("Hệ thống đã sẵn sàng");
}

void ChayChuongTrinhChinh() {
  // // Đọc dữ liệu Lidar liên tục
  // if ((millis() - lidarLastUpdate) > 2) {
  //   altitude = readLidar();
  //   lidarLastUpdate = millis();
  //   // #ifdef debug
  //   //   Serial.printf("\ndistance: %d",altitude);
  //   // #endif
  // }

  //   if ((millis() - QM5883LastUpdate) > 5) {
  //     current_angle = readCompassHeading();
  //     QM5883LastUpdate = millis();
  // #ifdef debugYaw
  //     Serial.printf("\ncurrent_angle: %.2f", current_angle);
  // #endif
  //   }
  // Xử lý tín hiệu điều khiển
  // if (signal_in.Read()) {

    // #ifdef DEBUG_SIGNAL_OUTPUT
    //   Serial.printf("Pitch: %d",data_out.ch[0]);
    // #endif
    // data_in = signal_in.data();
    // updateControls();
    // handleFlightState();
    // // PIDthrottle(TARGET_ALTITUDE);
    // signal_out.data(data_out);
    // signal_out.Write();
  // }
}

void updateControls() {
  // Ánh xạ giá trị điều khiển
  // CHUYỂN SANG MANUAL
  if ((data_in.ch[7] > MIN_MANUAL_ATCTIVE && data_in.ch[7] < MAX_MANUAL_ATCTIVE) && !lastModeManual) {
    currentState = MANUAL;
    stateStartTime = millis();
    lastModeManual = true;
#ifdef debug
    Serial.println("Manual");
#endif
  }  // CHUYỂN SANG AUTO
  else if ((data_in.ch[7] > MIN_AUTO_ATCTIVE && data_in.ch[7] < MAX_AUTO_ATCTIVE) && lastModeManual) {
#ifdef debug
    Serial.println("Auto");
#endif
    lastModeManual = false;
    currentState = AUTO;
    stateStartTime = millis();
  }
//   } else if ((data_in.ch[7] > MIN_CONFIG_ACTIVE && data_in.ch[7] < MAX_CONFIG_ACTIVE) && !lastModeManual) {
// #ifdef debug
//     Serial.println("Config");
// #endif
//     lastModeManual = true;
//     currentState = CONFIG;
//     stateStartTime = millis();
//   }
#ifdef DEBUG_SIGNAL_OUTPUT
  Serial.println("--- TÍN HIỆU ĐIỀU KHIỂN ---");
  Serial.printf("Roll: %d | Pitch: %d | Throttle: %d | Yaw: %d | Mode: %d\n",
                data_out.ch[0], data_out.ch[1], data_out.ch[2], data_out.ch[3], data_in.ch[7]);
  Serial.println("--- SIGNAL đã đọc và xuất ---");
  for (int i = 0; i < 12; i++) {  // Hiển thị 12 kênh
    Serial.printf("CH%2d In: %4d\tCH%2d Out: %4d\t", i + 1, data_in.ch[i], i + 1, data_out.ch[i]);
  }
  Serial.printf("\nCH17 In: %d\tCH18 In: %d\tLost Frame In: %d\tFailsafe In: %d", data_in.ch17, data_in.ch18, data_in.lost_frame, data_in.failsafe);
  Serial.printf("\nCH17 Out: %d\tCH18 Out: %d\tLost Frame Out: %d\tFailsafe Out: %d\n", data_out.ch17, data_out.ch18, data_out.lost_frame, data_out.failsafe);
#endif
}

// void handleFlightState() {
//   data_out.ch[0] = MIDDLE_SIGNAL_VALUE;
//   data_out.ch[1] = MIDDLE_SIGNAL_VALUE;
//   data_out.ch[2] = MIDDLE_SIGNAL_VALUE;
//   data_out.ch[3] = MIDDLE_SIGNAL_VALUE;

//   switch (currentState) {
//     case MANUAL:
//       data_out = data_in;
//       break;
//     case CONFIG:
//       altitude_hold(200);
//       KpPitch = map(data_out.ch[8], 1000, 2000, 0.1, 10);
//       KpRoll = map(data_out.ch[8], 1000, 2000, 0.1, 10);

//       KdPitch = map(data_out.ch[9], 1000, 2000, 0.01, 1);
//       KdRoll = map(data_out.ch[9], 1000, 2000, 0.01, 1);
//       PID_target(targetOffsetX, targetOffsetY);
//       break;
//     case AUTO:
//       Serial.println("Kích hoạt chế độ tự động sau 5s");
//       if (millis() - stateStartTime >= WAIT_TO_AUTO) {
//         currentState = Stage_1;
//         message_once(Stage_1, "Bắt đầu bay tự đông chuyển sang STAGE 2");
//       }
//       break;
//     case Stage_1:
//       sendRAS(Stage_5, "0");
//       Serial.println("drone qua trình tăng đạt độ cao");
//       // altitude_hold(TARGET_ALTITUDE);
//       if (altitude > 50) heading_hold(targetYaw);
//       // #ifdef debug
//       //       Serial.printf("\ndistance:%d cm", altitude);
//       // #endif
//       if (altitude >= TARGET_ALTITUDE - ALTITIDE_THRESHOLD && altitude <= TARGET_ALTITUDE + ALTITIDE_THRESHOLD) {
//         currentState = Stage_2;
//         message_once(Stage_2, "Đạt độ cao ổn định, chuyển qua STAGE 2");
//       }
//       break;
//     case Stage_2:
//       sendRAS(Stage_2, "1");
//       Serial.println("Bắt đầu bay tiến 5s");
//       // heading_hold(targetYaw);
//       if (pitch_control(VALUE_STAGE_2, DURATION_STAGE_2)) {
//         PID_target(targetOffsetX, targetOffsetY);
//         if (checkPositionStability(targetOffsetX, targetOffsetY, toleranceRadius, Time_PositionStability_Check)) {
//           drop_ball();
//           currentState = Stage_3;
//           message_once(Stage_3, "Đã thả 1 bóng, chuyển qua STAGE 3 ");
//         }
//       }
//       break;
//     case Stage_3:  //Drone di chuyển về phía trước 3m - pitch_control(1600,2000);
//       sendRAS(Stage_3, "2");
//       Serial.println("bắt đầu bay tiến 2s");
//       // heading_hold(targetYaw);
//       // altitude_hold(TARGET_ALTITUDE);
//       if (pitch_control(VALUE_STAGE_3, DURATION_STAGE_3)) {
//         PID_target(targetOffsetX, targetOffsetY);
//         if (checkPositionStability(targetOffsetX, targetOffsetY, toleranceRadius, Time_PositionStability_Check)) {
//           drop_ball();
//           currentState = Stage_4;
//           message_once(Stage_4, "Đã thả 2 bóng");
//         }
//       }
//       break;
//     case Stage_4:  // Drone di chuyển sang trái 2m - roll_control(1400, 1500);
//       sendRAS(Stage_4, "3");
//       Serial.println("Drone di chuyển sang 2m trái trong 1.5s");
//       // heading_hold(targetYaw);
//       if (roll_control(VALUE_STAGE_4, DURATION_STAGE_4)) {
//         PID_target(targetOffsetX, targetOffsetY);
//         if (checkPositionStability(targetOffsetX, targetOffsetY, toleranceRadius, Time_PositionStability_Check)) {
//           drop_ball();
//           currentState = Stage_5;
//           message_once(Stage_5, "Đã thả 3 bóng");
//         }
//       }
//       break;
//     case Stage_5:  // Drone di chuyển sang phải 4m - roll_control(1600, 3000);
//       Serial.println("Drone di chuyển 4m sang phải trong 3s");
//       // heading_hold(targetYaw);
//       if (roll_control(VALUE_STAGE_5, DURATION_STAGE_5)) {
//         PID_target(targetOffsetX, targetOffsetY);
//         if (checkPositionStability(targetOffsetX, targetOffsetY, toleranceRadius, Time_PositionStability_Check)) {
//           drop_ball();
//           currentState = Stage_6;
//           message_once(Stage_6, "Đã thả 4 bóng");
//         }
//       }
//       sendRAS(Stage_5, "4");
//       break;

//     case Stage_6:  // Drone di chuyển sang trái 2m -  roll_control(1400, 1500);
//       sendRAS(Stage_6, "5");
//       Serial.println("Bắt đầu di chuyển sang trái 2m trong 1.5s");
//       // heading_hold(targetYaw);
//       if (roll_control(VALUE_STAGE_6, DURATION_STAGE_6)) {
//         PID_target(targetOffsetX, targetOffsetY);
//         if (checkPositionStability(targetOffsetX, targetOffsetY, toleranceRadius, Time_PositionStability_Check)) {
//           currentState = Stage_7;
//           message_once(Stage_7, "Chuyển sang Stage 7");
//         }
//       }
//       break;
//     case Stage_7:
//       sendRAS(Stage_7, "6");
//       Serial.println("Tiến lên phía trước 3s");
//       // heading_hold(targetYaw);
//       // altitude_hold(TARGET_ALTITUDE);
//       if (pitch_control(VALUE_STAGE_7, DURATION_STAGE_7)) {
//         PID_target(targetOffsetX, targetOffsetY);
//         if (checkPositionStability(targetOffsetX, targetOffsetY, toleranceRadius, Time_PositionStability_Check)) {
//           drop_ball();
//           currentState = LANDING;
//           message_once(LANDING, "Đã thả 5 bóng");
//         }
//       }
//       break;
//     case LANDING:  // Drone di chuyển sang trái 2m -  roll_control(1400, 1500);
//       altitude_hold(LADING_VALUE);
//       Serial.println("Đang hạ cánh");
//       break;
//   }
// }



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

// float calculateAngleDifference(float target, float current) {
//   float diff = target - current;
//   diff = fmod(diff + 180, 360) - 180;
//   return diff < -180 ? diff + 360 : diff;
// }


void sendRAS(State state, const char* msg) {
  switch (state) {
    case Stage_1:
      if (!FlagsUart.stage_1) {
#ifdef debug
        softSerial.print(msg);
#endif
        FlagsUart.stage_1 = true;
      }
      break;
    case Stage_2:
      if (!FlagsUart.stage_2) {
#ifdef debug
        softSerial.print(msg);
#endif
        FlagsUart.stage_2 = true;
      }
      break;
    case Stage_3:
      if (!FlagsUart.stage_3) {
#ifdef debug
        softSerial.print(msg);
#endif
        FlagsUart.stage_3 = true;
      }
      break;
    case Stage_4:
      if (!FlagsUart.stage_4) {
#ifdef debug
        softSerial.print(msg);
#endif
        FlagsUart.stage_4 = true;
      }
      break;
    case Stage_5:
      if (!FlagsUart.stage_5) {
#ifdef debug
        softSerial.print(msg);
#endif
        FlagsUart.stage_5 = true;
      }
      break;
    case Stage_6:
      if (!FlagsUart.stage_6) {
#ifdef debug
        softSerial.print(msg);
#endif
        FlagsUart.stage_6 = true;
      }
      break;
    case Stage_7:
      if (!FlagsUart.stage_7) {
#ifdef debug
        softSerial.print(msg);
#endif
        FlagsUart.stage_7 = true;
      }
      break;
  }
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

        Serial.printf("cmd: %d, dx: %d, dy: %d\n", cmd, targetOffsetX, targetOffsetY);

        // Gửi phản hồi trở lại (echo)
        softSerial.printf("{\"status\":\"ok\",\"cmd\":%d}\n", cmd);
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

void drop_ball() {
  data_out.ch[4] = BALL_DROP_TRIGGER;
  signal_out.data(data_out);
  signal_out.Write();
  delay(200);
  data_out.ch[4] = BALL_DROP_IDLE;
}
bool checkPositionStability(int targetOffsetX, int targetOffsetY, float toleranceRadius, unsigned long Time_PositionStability_Check) {
  // Biến static để giữ trạng thái qua các lần gọi hàm
  static bool isWithinTolerance = false;  // Cờ báo hiệu mục tiêu có đang nằm trong ngưỡng không
  static unsigned long startTime = 0;     // Thời điểm bắt đầu nằm trong ngưỡng

  unsigned long currentTime = millis();

  // Tính khoảng cách từ vị trí mục tiêu đến tâm (0,0)
  float offsetDistance = sqrt(pow(targetOffsetX, 2) + pow(targetOffsetY, 2));

  // Kiểm tra xem mục tiêu có đang nằm trong bán kính chấp nhận trong chu kỳ hiện tại không
  bool Position_WithinTolerance_Checked = (offsetDistance <= toleranceRadius);

  // --- Quản lý trạng thái nằm trong ngưỡng ---
  if (Position_WithinTolerance_Checked) {
    // Nếu hiện tại đang nằm trong ngưỡng:
    if (!isWithinTolerance) {
      // Nếu ở lần trước KHÔNG trong ngưỡng (bây giờ mới vào ngưỡng)
      startTime = currentTime;   // Ghi lại thời điểm bắt đầu vào ngưỡng
      isWithinTolerance = true;  // Đặt cờ là đang trong ngưỡng
      //Serial.println("Vao khu vuc trong ngưỡng chấp nhận.")
    }
    // Nếu đã ở trong ngưỡng từ trước, không làm gì, startTime và isWithinTolerance giữ nguyên
  } else {
    // Nếu hiện tại KHÔNG nằm trong ngưỡng:
    if (isWithinTolerance) {
      // Nếu ở lần trước CÓ trong ngưỡng (bây giờ mới ra ngoài)
      isWithinTolerance = false;  // Đặt cờ là không còn trong ngưỡng
      startTime = 0;              // Reset thời điểm bắt đầu (quan trọng)
      // Có thể thêm Serial.println("Ra khoi khu vuc chap nhan.") de debug
    }
    // Nếu đã ở ngoài ngưỡng từ trước, không làm gì, trạng thái giữ nguyên
  }

  // --- Kiểm tra xem đã ổn định đủ thời gian dwell chưa ---
  if (isWithinTolerance) {
    // Chỉ kiểm tra nếu đang nằm trong ngưỡng
    if (currentTime - startTime >= Time_PositionStability_Check) {
      // Đã nằm trong ngưỡng và thời gian trôi qua >= thời gian dwell mong muốn
      // Có thể thêm Serial.println("Da on dinh o vi tri muc tieu.") de debug

      // Tùy chọn: Nếu bạn chỉ muốn hàm trả về true MỘT LẦN cho mỗi lần vào ngưỡng
      // và không báo true nữa cho đến khi ra khỏi ngưỡng rồi quay lại, bạn có thể reset trạng thái ở đây:
      // isWithinTolerance = false; // Reset để không kích hoạt lại ngay lập tức
      // startTime = 0;

      return true;  // --> Đã ổn định! Trả về true
    }
  }

  // Nếu không thỏa mãn điều kiện đủ thời gian dwell hoặc không nằm trong ngưỡng, trả về false
  return false;
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

