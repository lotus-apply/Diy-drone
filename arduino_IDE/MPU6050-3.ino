// モーター制御の切り替えができるArduinoスケッチ
#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>
#include <EEPROM.h>

MPU6050 mpu;

#define BRUSHLESS 0
#define BRUSHED   1

// EEPROMのアドレス（float 4バイトずつ）
#define ADDR_P 0
#define ADDR_I 4
#define ADDR_D 8
#define ADDR_BASE1 12
#define ADDR_BASE2 16
#define ADDR_BASE3 20
#define ADDR_BASE4 24



// ピン定義
#define motor1Pin  3
#define motor2Pin  5
#define motor3Pin  6
#define motor4Pin  11

unsigned long lastPrintTime = 0;  // 前回の出力時間を記録

int flaglode = 0;

float error = 0, previousError = 0, integral = 0;
int m1 = 0, m2 = 0, m3 = 0, m4 = 0;

// PIDパラメータ
float kp = 0; //比例ゲイン
float ki = 0; //積分ゲイン
float kd = 0; //微分ゲイン

int baseThrottle = 1500;  // モーター最低動作値

int Throttle = -500;  // モータースロットル値

int motorOffset1 =0 ;  // モーターoffset
int motorOffset2 =0 ;  // モーターoffset
int motorOffset3 =0 ;  // モーターoffset
int motorOffset4 =0 ;  // モーターoffset

// モータータイプ:  0 = ブラシモーター(PWM), 1= ブラシレス(ESC)
int motorType = BRUSHED ; //0


float setPitch = 0; // リモコンでドローン操作の時これを変更
float setRoll = 0; // リモコンでドローン操作の時これを変更
float setYaw = 0;  // リモコンでドローン操作の時これを変更
float P = 0, I = 0, D = 0;
float pitch, roll, yaw;
float errPitchPrev = 0, errRollPrev = 0, errYawPrev = 0;
float integralPitch = 0, integralRoll = 0, integralYaw = 0;

unsigned long prevTime = 0;




// Servoオブジェクト（ブラシレス用）
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;



// 受信バッファ
String inputString = "";
bool stringComplete = false;

void savePIDtoEEPROM() {
  EEPROM.put(ADDR_P, kp);
  EEPROM.put(ADDR_I, ki);
  EEPROM.put(ADDR_D, kd);
  //EEPROM.commit();  // AVRでは不要、ESP32/ESP8266なら必要
}

void loadPIDfromEEPROM() {

  EEPROM.get(ADDR_P, P);  // EEPROM アドレス 0 に保存した P
  EEPROM.get(ADDR_I, I);  // アドレス 4 に保存した I
  EEPROM.get(ADDR_D, D);  // アドレス 8 に保存した D
 // 読み込んだ値を制御に使う変数に渡す
  kp = P;
  ki = I;
  kd = D;
}


// 保存する関数にゃ
void saveBaseThrottleToEEPROM() {
  EEPROM.put(ADDR_BASE1, motorOffset1);
  EEPROM.put(ADDR_BASE2, motorOffset2);
  EEPROM.put(ADDR_BASE3, motorOffset3);
  EEPROM.put(ADDR_BASE4, motorOffset4);
}


// 読み込む関数にゃ
void loadBaseThrottleFromEEPROM() {
  EEPROM.get(ADDR_BASE1, motorOffset1);
  EEPROM.get(ADDR_BASE2, motorOffset2);
  EEPROM.get(ADDR_BASE3, motorOffset3);
  EEPROM.get(ADDR_BASE4, motorOffset4);
}

const int MPU = 0x68;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  // MPU6050を起動
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1
  Wire.write(0);     // 起動
  Wire.endTransmission();

  // DLPF設定: CONFIGレジスタ（0x1A）を 0x03 に設定（42Hz）
  Wire.beginTransmission(MPU);
  Wire.write(0x1A);      // CONFIG
  Wire.write(0x03);      // DLPF = 3
  Wire.endTransmission();

  Serial.println("MPU6050 初期化＆DLPF設定完了にゃ！");
//}
//void setup() {
//  Wire.begin();
//  Serial.begin(9600);
  

//  mpu.initialize();
//  if (!mpu.testConnection()) {
//    Serial.println("MPU6050 接続失敗");
//    while (1);
//  }
    
  
  inputString.reserve(200); // メモリ確保
  

  

  // ブラシレス初期化（ESC）
  esc1.attach(motor1Pin);
  esc2.attach(motor2Pin);
  esc3.attach(motor3Pin);
  esc4.attach(motor4Pin);
  
  pinMode(motor1Pin, OUTPUT); pinMode(motor2Pin, OUTPUT); pinMode(motor3Pin, OUTPUT); pinMode(motor4Pin, OUTPUT);

  flaglode = 1 ;

  // ─── ここで EEPROM から読み込み ───
  loadPIDfromEEPROM();

  loadBaseThrottleFromEEPROM(); // EEPROMから読み込み


  // 起動時に現在の補正値を送信（フォーマットは自由にゃ）
  //Serial.print("CORRECTION"); Serial.print(",");
  //Serial.print(motorOffset1); Serial.print(",");
  //Serial.print(motorOffset2); Serial.print(",");
  //Serial.print(motorOffset3); Serial.print(",");
  //Serial.println(motorOffset4);

  //Serial.print("CORRECTION2"); Serial.print(",");
  //Serial.print(P); Serial.print(",");
  //Serial.print(I); Serial.print(",");
  //Serial.println(D);
  

  

  prevTime = millis();
 

  
}

float ax, ay, az;
float ax_filtered = 0, ay_filtered = 0, az_filtered = 0;
float alpha = 0.09; // 0.1～0.3くらいが目安

void loop() {
  if (stringComplete) {
    processSerialCommand(inputString);
    inputString = "";
    stringComplete = false;
  }

  //////////////////////////////////////////////////////////////////
  //スムージング////////////////////////////////////////////////////
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // 加速度Xのアドレス
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);

  int16_t raw_ax = Wire.read() << 8 | Wire.read();
  int16_t raw_ay = Wire.read() << 8 | Wire.read();
  int16_t raw_az = Wire.read() << 8 | Wire.read();

  ax = raw_ax / 16384.0;  // ±2g換算
  ay = raw_ay / 16384.0;
  az = raw_az / 16384.0;

  // スムージング適用
  ax_filtered = smooth(ax, ax_filtered, alpha);
  ay_filtered = smooth(ay, ay_filtered, alpha);
  az_filtered = smooth(az, az_filtered, alpha);

  //Serial.print("AX: "); Serial.print(ax_filtered);
  //Serial.print(" AY: "); Serial.print(ay_filtered);
  //Serial.print(" AZ: "); Serial.println(az_filtered);
  //////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////
  
  ///////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////
  // IMU取得
  //int16_t ax, ay, az, gx, gy, gz;
  //mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float axg = ax_filtered;     //float axg = ax / 16384.0;
  float ayg = ay_filtered;     //float ayg = ay / 16384.0;
  float azg = az_filtered;     //float azg = az / 16384.0;

  pitch = atan2(axg, sqrt(ayg * ayg + azg * azg)) * 180.0 / PI;
  roll  = atan2(ayg, sqrt(axg * axg + azg * azg)) * 180.0 / PI;
  yaw   = 0;
  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0;
  prevTime = now;

  //float gzg = gz / 131.0;
  //yaw += gzg * dt;

  //if (yaw > 180) yaw -= 360;
  //f (yaw < -180) yaw += 360;

  // PID制御計算
  float errPitch = setPitch - pitch;  //設定値と実際の差
  float errRoll  = setRoll - roll;    //設定値と実際の差
  //float errYaw   = setYaw - yaw;

  integralPitch += errPitch * dt;   //積分値
  integralRoll  += errRoll * dt;    //積分値
  //integralYaw   += errYaw * dt;

  float dPitch = (errPitch - errPitchPrev) / dt;  //微分値
  float dRoll  = (errRoll  - errRollPrev)  / dt;  //微分値
  //float dYaw   = (errYaw   - errYawPrev)   / dt;

  errPitchPrev = errPitch;  //設定値と実際の差を記憶する
  errRollPrev  = errRoll;   //設定値と実際の差を記憶する
  //errYawPrev   = errYaw;

  float pitchOut = kp * errPitch + ki * integralPitch + kd * dPitch;
  float rollOut  = kp * errRoll  + ki * integralRoll  + kd * dRoll;
  //float yawOut   = kp * errYaw   + ki * integralYaw   + kd * dYaw;

  // ベーススロットル（調整用）＋補正（上下限制限あり）
  //int baseThrottle = 1500;  // モーター最低動作値
  
  
  //int m1 = constrain(baseThrottle + motorOffset1 + pitchOut + rollOut, 1000, 2000);  // FL   
  //int m2 = constrain(baseThrottle + motorOffset2 - pitchOut + rollOut, 1000, 2000);  // FR
  //int m3 = constrain(baseThrottle + motorOffset3 + pitchOut - rollOut, 1000, 2000);  // BL
  //int m4 = constrain(baseThrottle + motorOffset4 - pitchOut - rollOut, 1000, 2000);  // BR

  int m11 = constrain(baseThrottle + motorOffset1 + Throttle + pitchOut + rollOut, 1000, 2000);  // FL
  int m22 = constrain(baseThrottle + motorOffset2 + Throttle - pitchOut + rollOut, 1000, 2000);  // FR
  int m33 = constrain(baseThrottle + motorOffset3 + Throttle + pitchOut - rollOut, 1000, 2000);  // BL
  int m44 = constrain(baseThrottle + motorOffset4 + Throttle - pitchOut - rollOut, 1000, 2000);  // BR


  //モータへの出力　
  updateMotors(m11, m22, m33, m44);

  
  //delay(20);

  if (millis() - lastPrintTime > 200) {
    Serial.print("motorChart"); Serial.print(",");
    Serial.print(pitch); Serial.print(",");
    Serial.print(roll);  Serial.print(",");
    Serial.print(yaw);   Serial.print(",");
    Serial.print(m11);    Serial.print(",");
    Serial.print(m22);    Serial.print(",");
    Serial.print(m33);    Serial.print(",");
    Serial.println(m44);  // 最後だけ println
  lastPrintTime = millis();
}



  

  

  ///////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////
}

// 文字列受信時のイベント
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
      break;
    } else {
      inputString += inChar;
    }
  }
}

// シリアルコマンドの処理
void processSerialCommand(String command) {
  if (command.startsWith("SET_PID")) {
    command.remove(0, 8);
    int first = command.indexOf(',');
    int second = command.indexOf(',', first + 1);

    kp = command.substring(0, first).toFloat();
    ki = command.substring(first + 1, second).toFloat();
    kd = command.substring(second + 1).toFloat();

    Serial.println("ACK_PID");
    savePIDtoEEPROM();
    return;
  }

  if (command.startsWith("SET_MOTOR_TYPE")) {
    int type = command.substring(9).toInt();
    motorType = type;
    Serial.println("ACK_MOTOR_TYPE");
    return;
  }


  if (command.startsWith("MOTOR")) {
    command.remove(0, 6);
    int p1 = command.indexOf(',');
    int p2 = command.indexOf(',', p1 + 1);
    int p3 = command.indexOf(',', p2 + 1);

    motorOffset1 = command.substring(0, p1).toInt();
    motorOffset2 = command.substring(p1 + 1, p2).toInt();
    motorOffset3 = command.substring(p2 + 1, p3).toInt();
    motorOffset4 = command.substring(p3 + 1).toInt();
     Serial.println("ACK_MOTOR_OFFSET");
     saveBaseThrottleToEEPROM();
    //updateMotors(m1, m2, m3, m4);
  }

  if (command.startsWith("ELEVATOR")) {
    command.remove(0, 9);
    int p1 = command.indexOf(',');
    
    Throttle = command.substring(0, p1).toInt();
     Serial.println("ACK_ELEVATOThrottle");
    //saveBaseThrottleToEEPROM();
    //updateMotors(m1, m2, m3, m4);
  }

  if (command.startsWith("STOP")) {
    Throttle = -500;
    Serial.println("ACK_STOP");
    return;
  }

  if (command.startsWith("initial")) {
    lodeinitial();
    return;
  }

}

// モーター出力処理
void updateMotors(int m1, int m2, int m3, int m4) {
  //if (motorType == BRUSHLESS) {
  //  esc1.writeMicroseconds(m1);
  //  esc2.writeMicroseconds(m2);
  //  esc3.writeMicroseconds(m3);
  //  esc4.writeMicroseconds(m4);
  //} else {
    writeMotorPWM(motor1Pin, m1);
    writeMotorPWM(motor2Pin, m2);
    writeMotorPWM(motor3Pin, m3);
    writeMotorPWM(motor4Pin, m4);

    
  //}
}

// ブラシモーター用PWM出力
void writeMotorPWM(int pin, int value) {
  value = constrain(value, 1000, 2000);
  int duty = map(value, 1000, 2000, 0, 255); // PWM用にマッピング
  //analogWrite(pin, duty);

  //////////////////////////////////////////////////////////////////////////
  //非線形マッピング（指数スケーリング）で調整を緩やかにする
  float scaled = pow((float)duty / 255.0, 2.0); // 2.0 = カーブの強さ
  int duty2 = scaled * 255;
  analogWrite(pin, duty2);
  //////////////////////////////////////////////////////////////////////////

}

void lodeinitial(){
  
  // 起動時に現在の補正値を送信（フォーマットは自由にゃ）
  Serial.print("CORRECTION"); Serial.print(",");
  Serial.print(motorOffset1); Serial.print(",");
  Serial.print(motorOffset2); Serial.print(",");
  Serial.print(motorOffset3); Serial.print(",");
  Serial.println(motorOffset4);

  Serial.print("CORRECTION2"); Serial.print(",");
  Serial.print(P); Serial.print(",");
  Serial.print(I); Serial.print(",");
  Serial.println(D);
  
}

float smooth(float current, float previous, float alpha) {
  return alpha * current + (1 - alpha) * previous;
}

