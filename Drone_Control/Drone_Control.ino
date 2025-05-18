#include "ChuongTrinhChinh.h"
TaskHandle_t Task1;
TaskHandle_t Task2;
void setup() {
  xTaskCreatePinnedToCore(
    ChayChuongTrinhChinh, /* Task function. */
    "Task1",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    0,         /* priority of the task */
    &Task1,    /* Task handle to keep track of created task */
    0);        /* pin task to core 0 */
  delay(500);

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
    Task2code, /* Task function. */
    "Task2",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task2,    /* Task handle to keep track of created task */
    1);        /* pin task to core 1 */
  delay(500);
  KhoiTao();
}
//Task1code: blinks an LED every 1000 ms
void ChayChuongTrinhChinh( void * pvParameters ){
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    ChayChuongTrinhChinh();
    vTaskDelay(1);
  } 
}

//Task2code: blinks an LED every 700 ms
void Task2code( void * pvParameters ){
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    readUART3Data();
    vTaskDelay(1);
  }
}
void loop() {
}

