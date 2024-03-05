#include <AccelStepper.h>
#define LINEAR_DIRECTION_PIN 5      // 距離センサー用のモーターの方向
#define LINEAR_STEPPER_PIN 4        // 距離センサー用のモーターのステップ
#define LINEAR_ENABLE_PIN 3         // 距離センサー用のEnableピン
#define linearStepsPerRevolution 100 // 距離センサー用のステップ解像度
#define motorInterfaceType 1
AccelStepper linearStepper = AccelStepper(motorInterfaceType, LINEAR_STEPPER_PIN, LINEAR_DIRECTION_PIN);

#define IR_SENSOR_PIN A1            // 距離センサー

#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

#define LOADCELL_DT 6               // ロードセルのDT
#define LOADCELL_SCK 7              // ロードセルのSCK
//HX711 constructor:
HX711_ADC LoadCell(LOADCELL_DT, LOADCELL_SCK);

#define SWITCH 8                    // ロータリーエンコーダーについているスイッチ
#define ROTARY_ENCODER_DT  9        // ロータリーエンコーダーDT
#define ROTARY_ENCODER_CLK 10       // ロータリーエンコーダーCLK
int oldSwitchVal = 0;
int mode = 1;
const int mode_count = 10;

// Rotation Table
#define ROTATION_TABLE_ENABLE 20       // ターンテーブル用モーターEnable
#define ROTATION_TABLE_STEP 22         // ターンテーブル用モーターSTEP
#define ROTATION_TABLE_DIR 21          // ターンテーブル用モーターDIR
AccelStepper rotationTableStepper = AccelStepper(motorInterfaceType, ROTATION_TABLE_STEP, ROTATION_TABLE_DIR);

unsigned long time_data;
unsigned long prev_time;
const unsigned long one_rotation_interval = 2000;

unsigned long linearStepCount = 0;

// IR Sensor
// 0 - 1024で遷移、近いほど小さい
int irSensorValue = 1024;

// LoadCell
float loadCellValue = 0.0;

// 再生のフラグ
boolean isPlayed = true;

void setup() {
  Serial.begin(57600);
  Serial.println();
  Serial.println("Starting...");
  pinMode(LINEAR_DIRECTION_PIN, OUTPUT);
  pinMode(LINEAR_STEPPER_PIN, OUTPUT);
  pinMode(LINEAR_ENABLE_PIN, OUTPUT);
  // Analog PinをDigitalに
  pinMode(ROTATION_TABLE_ENABLE, OUTPUT);
  pinMode(ROTATION_TABLE_STEP, OUTPUT);
  pinMode(ROTATION_TABLE_DIR, OUTPUT);
  // Rotary Encoder
  pinMode(SWITCH, INPUT);
  prev_time = 0;
  //calibrateLoadcell();
  linearStepper.setMaxSpeed(300.0);
  linearStepper.setAcceleration(30.0);

  rotationTableStepper.setMaxSpeed(300.0);
  rotationTableStepper.setAcceleration(10.0);
  rotationTableStepper.setSpeed(-300);
}

void loop() {
  unsigned long current_time = millis();
  // Rotation Table
  //  TODO: time_data % 20 == 0を外すと何故か動かない
  if (current_time % 20 == 0 && isPlayed) {
    Serial.println("rotate");
    rotateTable();
    if (current_time - prev_time >= one_rotation_interval) {
      playLinearActuator();
      prev_time = current_time;
    } else {
      //pauseLinearActuator();
    }
  } else if (!isPlayed) {
    pauseLinearActuator();
  }

  // ロータリーエンコーダーのスイッチ
  int switchVal = digitalRead(SWITCH);

  if (switchVal == HIGH && oldSwitchVal == 0) {
    mode++;
    if (mode > mode_count) {
      mode = 1;
    }
    delay(50); // button debounce
  }
  oldSwitchVal = switchVal;

  // ロードセルの値を取得
  //loadCellValue = getLoadCellValue();
  // IRセンサーの値を取得
  irSensorValue = analogRead(IR_SENSOR_PIN);
  // Raspberry Piに送信
  sendSerialToPi(loadCellValue, irSensorValue, isPlayed);

  // Switch for playing
  //  if(digitalRead(SWITCH) == LOW) {
  //    isPlayed = !isPlayed;
  //  }
}

// Rotation Tableを回転させる関数
void rotateTable() {
  rotationTableStepper.runSpeed();
}

// 距離センサー用のステッピングモーターを回転させる関数
void playLinearActuator() {
  digitalWrite(LINEAR_ENABLE_PIN, LOW);
  linearStepper.move(-200);
  linearStepper.run();

}

// 距離センサー用のステッピングモーターをポーズする関数
void pauseLinearActuator() {
  digitalWrite(LINEAR_ENABLE_PIN, HIGH);
}

// ロードセルをキャリブレーションする
void calibrateLoadcell() {
  LoadCell.begin();
  float calibrationValue; // calibration value (see example file "Calibration.ino")
  calibrationValue = 696.0; // uncomment this if you want to set the calibration value in the sketch
#if defined(ESP8266)|| defined(ESP32)
  //EEPROM.begin(512); // uncomment this if you use ESP8266/ESP32 and want to fetch the calibration value from eeprom
#endif
  //EEPROM.get(calVal_eepromAdress, calibrationValue); // uncomment this if you want to fetch the calibration value from eeprom

  unsigned long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
    Serial.println("Startup is complete");
  }
}

// ロードセルの値を取得
float getLoadCellValue() {
  static boolean newDataReady = 0;
  const int serialPrintInterval = 0; //increase value to slow down serial print activity
  float i = 0.0;
  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > time_data + serialPrintInterval) {
      i = LoadCell.getData();
      //      Serial.print("Load_cell output val: ");
      //      Serial.println(i);
      newDataReady = 0;
      time_data = millis();
    }
  }
  return i;
}
// 送るデータ
// loadCellValue, IRSensorValue, isPlayed,
void sendSerialToPi(float loadCellValue, int irSensorValue, bool isPlayed) {
  Serial.println((String)"LoadCellValue:" + loadCellValue + ", irSensorValue:" + irSensorValue + ", isPlayed:" + isPlayed + ", mode:" + mode);
  //  Serial.println(irSensorValue);
  //  Serial.println(isPlayed);
}
