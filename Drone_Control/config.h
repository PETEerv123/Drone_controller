#ifndef _config_H
#define _config_H

#define MIN_SIGNAL_VALUE 173
#define MIDDLE_SIGNAL_VALUE 993
#define MAX_SIGNAL_VALUE 1810

#define MIN_MANUAL_ATCTIVE  300
#define MAX_MANUAL_ATCTIVE  1100

#define MIN_AUTO_ATCTIVE 1400
#define MAX_AUTO_ATCTIVE 1700

#define MIN_CONFIG_ACTIVE 800 // 400
#define MAX_CONFIG_ACTIVE 1100 // 600

#define TARGET_ALTITUDE 300  // 4m = 400cm // độ cao mong muốn
#define LADING_VALUE 150 // 150 hạ cánh mong muốn 

#define DROP_DURATION 1400 // xung 1400ms để thả

#define BALL_DROP_TRIGGER 1400
#define BALL_DROP_IDLE 500
#define BALL_DROP_WAIT 400 //Thời gian chờ thả bóng

#define WAIT_TO_AUTO 100

#define VALUE_STAGE_2 1150  //Pitch Tiến lên, tạo đà di chuyển cho drone
#define DURATION_STAGE_2 2000

#define VALUE_STAGE_3 993//Tiến lên mục tiêu vàng 1 rồi thả bóng
#define DURATION_STAGE_3  3000

#define VALUE_STAGE_4 993 //sang trái mục tiêu đỏ 1
#define DURATION_STAGE_4  1500

#define VALUE_STAGE_5 700 //lùi lại để thả tiếp
#define DURATION_STAGE_5  2000

#define VALUE_STAGE_6 1000 //lùi lại thả tiếp
#define DURATION_STAGE_6  2000

#define VALUE_STAGE_7 1000 //tiến lên mục tiêu xanh dương
#define DURATION_STAGE_7  2000
//


// float VALUE_STAGE_2 = 0.0;
// float DURATION_STAGE_2 = 0.0;
// float VALUE_STAGE_3 = 0.0;
// float DURATION_STAGE_3 = 0.0;
// float VALUE_STAGE_4 = 0.0;
// float DURATION_STAGE_4 = 0.0;
// float VALUE_STAGE_5 = 0.0;
// float DURATION_STAGE_5 = 0.0;
// float VALUE_STAGE_6 = 0.0;
// float DURATION_STAGE_6 = 0.0;
// float VALUE_STAGE_7 = 0.0;
// float DURATION_STAGE_7 = 0.0;
enum State { Stage_0,        // Idle, Đặt ở khu vực khởi động,
             Stage_1,        //Drone cất cánh lên độ cao 400cm - altitude_hold(400)
             Stage_2,        // Drone di chuyển về phía trước 7m - pitch_control(1600,5000);
             Stage_3,        // Drone di chuyển về phía trước 3m - pitch_control(1600,2000);
             Stage_4,        //Drone di chuyển sang trái 2m - roll_control(1400, 1500);
             Stage_5,        // Drone di chuyển sang phải 4m - roll_control(1600, 3000);
             Stage_6,        //Drone di chuyển sang trái 2m -  roll_control(1400, 1500);
             Stage_7,        //Drone di chuyển tiến về phía trước 4m (pitch_control(1600, 3000);
             LANDING,
             AUTO,
             MANUAL,
             CONFIG,
             STOP };
State currentState = STOP;




float toleranceRadius = 30;
unsigned long Time_PositionStability_Check = 3000;
extern unsigned long ball_start_time ;
extern unsigned long wait_for_ball_drop ; 
extern bool counting ;
extern bool ball_releasing ;
#define TARGET_THRESHOLD 30  // Ngưỡng sai số cho vị trí đích

#define ALTITIDE_THRESHOLD 20

//////////// khai bao chan GPIO//////
#define BALL_DROP_PIN 47
#define SIGNAL_RX_PIN 17  // Chân GPIO17 để nhận tín hiệu RC
#define SIGNAL_TX_PIN 18  // Chân GPIO18 để xuất SIGNAL
#define QCM5883_SCL 11    // la ban
#define QCM5883_SDA 10
#define LidarSerial_RX 19  // chan RC

#define RXD 15
#define TXD 16
#define BAUD_RATE 38400



String input = "";
// Khởi tạo UART1 cho cả RX và TX
HardwareSerial SIGNAL_uart(1);
// Khởi tạo đối tượng SIGNAL
bfs::signalRx signal_in(&SIGNAL_uart, SIGNAL_RX_PIN, SIGNAL_TX_PIN, true);
bfs::signalTx signal_out(&SIGNAL_uart, SIGNAL_RX_PIN, SIGNAL_TX_PIN, true);
bfs::signalData data_in, data_out;

HardwareSerial LidarSerial(2);  // UART2

#endif