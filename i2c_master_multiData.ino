/** masterの書き込みスケッチ **/
/* !!!read me!!!

    neutralVal +- 5 で閾値を設定し，ニュートラル時には全アクチュエータを解放する．
    sleepTime でスリープ時間を設定し，スリープ時には全アクチュエータを解放する．(不要？)
    変換関数
*/

#include <Wire.h>
#define ANALOG_NUM 3
#define TOTAL_ANALOG_NUM ANALOG_NUM * 2
#define Threshold 5

//能動入力が0~2, 受動入力が3~5
int masterVal[ANALOG_NUM];
int slaveVal[ANALOG_NUM];
int analogVal[TOTAL_ANALOG_NUM];

int pumpSupplyPin[ANALOG_NUM] = {3, 6, 10};
int pumpVacuumPin[ANALOG_NUM] = {5, 9, 11};
int valveSupplyPin[ANALOG_NUM] = {14, 16, 18};
int valveVacuumPin[ANALOG_NUM] = {15, 17, 19};

float a = 0.95;
int filteredVal[TOTAL_ANALOG_NUM][2]; //0~255
int maxVal[TOTAL_ANALOG_NUM] = {0}; //0~255
int minVal[TOTAL_ANALOG_NUM] = {255}; //0~255
int neutralVal = 50;

#define RESOLUSION 100
int rate[TOTAL_ANALOG_NUM]; //0~100

int PWM[ANALOG_NUM] = {0};
bool bDeform[ANALOG_NUM] = {false};
bool bPolarity[ANALOG_NUM] = {false};
bool bNeutral[ANALOG_NUM] = {false};

#define LED 8
#define SW 7
boolean bLed = false;
boolean bRealtime = false;
int swVal = 0;
int swCount = 0;

int slaveStatus = 0;
int sendSwitch = 0;
int receiveSwitch = 0;

int oldBLed = 0;
int booleanDelta = 0;

#define RE 4
boolean bReset = false;
int reVal = 0;
int oldReVal = 0;

//PID
int delta[TOTAL_ANALOG_NUM][2] = {{0}, {0}};
int absDelta[TOTAL_ANALOG_NUM] = {0};
float dt = 0; //処理周期
float preTime;
float integral;
float KP = 6.0; //Pゲイン
float KI = 10.0; //Iゲイン
float KD = 1.0; //Dゲイン
float p = 0.0;
float i = 0.0;
float d = 0.0;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  pinMode(LED, OUTPUT);
  pinMode(SW, INPUT);
  pinMode(RE, INPUT);
  digitalWrite(LED, LOW);
  digitalWrite(SW, LOW);
  digitalWrite(RE, LOW);

  for (int i = 0; i < ANALOG_NUM; i++) {
    pinMode(pumpSupplyPin[i], OUTPUT);
    pinMode(pumpVacuumPin[i], OUTPUT);
    pinMode(valveSupplyPin[i], OUTPUT);
    pinMode(valveVacuumPin[i], OUTPUT);
  }

  for (int i = 0; i < ANALOG_NUM; i++) {
    digitalWrite(pumpSupplyPin[i], LOW);
    digitalWrite(pumpVacuumPin[i], LOW);
    digitalWrite(valveSupplyPin[i], LOW);
    digitalWrite(valveVacuumPin[i], LOW);
  }

}


void loop() {

  for (int i = 0; i < ANALOG_NUM; i++) {
    analogVal[i] = analogRead(i) / 4;
  }

  /*--from slave--*/

  Wire.requestFrom(8, 5, true);
  if ( Wire.available() > 4 ) {
    analogVal[3] = Wire.read();
    analogVal[4] = Wire.read();
    analogVal[5] = Wire.read();
    slaveStatus = Wire.read(); //相手のステータス
    receiveSwitch = Wire.read(); //スイッチの受信
  }


  /*--function--*/

  for (int i = 0; i < TOTAL_ANALOG_NUM; i++) { //値のフィルタリング&maxVal,minの更新，rate変換
    adjustData(i);
  }

  //printRate();
  printMinMax();

  /*--to openFrameWorks--*/
  //設営終わったら消す

  if (Serial.available() > 0) {
    for (int i = 0; i < TOTAL_ANALOG_NUM; i++) {
      Serial.write(rate[i]);
    }
    Serial.read();
  }

  /*--to slave--*/
  

  Wire.beginTransmission(8);
  for (int i = 0; i < TOTAL_ANALOG_NUM; i++) {
    Wire.write(rate[i]);
  }
  Wire.write(bLed);//自分のステータス
  if (booleanDelta == -1 && slaveStatus == 1) { //masterがon, かつslaveもonの時,slaveをoffにする
    sendSwitch = 3;
  } else {
    sendSwitch = 0;
  }
  Wire.write(sendSwitch);//相手のスイッチ
  Wire.endTransmission();

  /*--*/

  switchPlay();//スイッチ

  workRealtime();//

  switchReset();//値のリセット

  //切り替えなかったら消す
  if (receiveSwitch == 3) {
    bLed = !bLed;
    bRealtime = !bRealtime;
  }
  booleanDelta = oldBLed - (int)bLed;
  oldBLed = (int)bLed;

  for (int i = 0; i < TOTAL_ANALOG_NUM; i++) { //値の更新
    filteredVal[i][0] = filteredVal[i][1];
  }

  delay(100 / 3);
}

void adjustData(int _number) {

  filteredVal[_number][1] = a * filteredVal[_number][0] + (1 - a) * analogVal[_number]; //フィルタリング
  rate[_number] = map(filteredVal[_number][1], minVal[_number], maxVal[_number], RESOLUSION, 0); //マッピング

  mappingUp(_number);

  if (filteredVal[_number][1] > maxVal[_number]) { //最大値
    maxVal[_number] = filteredVal[_number][1];
  }
  if (filteredVal[_number][1] < minVal[_number]) { //最小値
    minVal[_number] = filteredVal[_number][1];
  }
}

int mappingDown(int number) { //下方修正
  float mapGain = .2;//0~1.0
  rate[number] = mapGain * rate[number] + ((1 - mapGain) * 0.01) * pow(rate[number], 2);
  return rate[number];
}

int mappingUp(int number) { //上方修正
  float mapGain = .3;//0~1.0
  rate[number] = (1 + mapGain) * rate[number] - (0.01 * mapGain) * pow(rate[number], 2);
  return rate[number];
}

void switchPlay() {
  swVal = digitalRead(SW);

  if (swVal == HIGH) {
    swCount += 1;
  } else {
    swCount = 0;
  }

  if (swCount == 10) {
    bLed = !bLed;
    bRealtime = !bRealtime;
    Serial.println("start!!!!");
  }

  if (bLed) {
    digitalWrite(LED, HIGH);

  } else {
    digitalWrite(LED, LOW);
  }

  delay(1);
}

void switchReset() {
  reVal = digitalRead(RE);

  if (reVal == HIGH) {
    for (int i = 0; i < TOTAL_ANALOG_NUM; i++) {
      minVal[i] = {255};
      maxVal[i] = {0};
    }
  }
}

void workRealtime() {
  if (bRealtime == true) {
    //input
    for (int i = 0; i < ANALOG_NUM; i++) {
      fbJudge(i + 3, i); //teacherが左
      fbOutput(i + 3, i);
    }
  } else {
    for (int i = 0; i < ANALOG_NUM; i++) {
      sendDigitalExhaust(i);
    }
  }
}

void fbJudge(int teacher, int child) { //目標値，センサー値
  //  dt = 100 / 3;

  delta[teacher][0] = delta[teacher][1]; //過去の偏差を格納

  delta[teacher][1] = rate[teacher] - rate[child]; //**偏差の更新**
  absDelta[teacher] = abs(delta[teacher][1]); //偏差の絶対値
  //  integral += (delta[teacher][1] + delta[teacher][0]) / 2.0 * dt;

  //int dd = delta[teacher][1] - delta[teacher][0];//偏差の変化量

  p = KP * delta[teacher][1]; //定数*偏差(0~100) 255 =  gain * 100
  //  i = KI * integral;
  //  d = KD * dd / dt;

  setPWM_PID(p, 0, 0, child);

  if (absDelta[teacher] >= Threshold) {
    bDeform[teacher] = true;
  } else {
    bDeform[teacher] = false;
  }

  if (delta[teacher][1] > 0) {
    bPolarity[teacher] = true;
  } else if (delta[teacher][1] < 0) {
    bPolarity[teacher] = false;
  }

  if (neutralVal - Threshold < rate[teacher] && rate[teacher] < neutralVal + Threshold) { //45~ 55のとき
    bNeutral[teacher] = true;
  } else {
    bNeutral[teacher] = false;
  }

}

int setPWM_PID(int p, int i, int d, int number) {
  //pwmに変換
  PWM[number] = abs(p + i + d);
  if (PWM[number] < 50) {
    PWM[number] = 0;
  } else if (PWM[number] >= 255) {
    PWM[number] = 255;
  }
  return PWM[number];
}

void fbOutput(int teacher, int child) {
  if (bNeutral[teacher] == true) { //ニュートラルかどうか
    sendDigitalExhaust(child);
  } else {
    if (bDeform[teacher] == true) { // 偏差があるかどうか
      if (bPolarity[teacher] == true) { //正負の判定
        sendDigitalSupply(child, PWM[child]);
      } else {
        sendDigitalVacuum(child, PWM[child]);
      }
    } else {
      sendDigitalClose(child);
    }
  }
}

//--------------------------------------

void sendDigitalSupply(int number, int PWM) {
  digitalWrite(valveSupplyPin[number], HIGH);
  digitalWrite(valveVacuumPin[number], LOW);
  analogWrite(pumpSupplyPin[number], PWM);
  analogWrite(pumpVacuumPin[number], 0);
}

void sendDigitalVacuum(int number, int PWM) {
  digitalWrite(valveSupplyPin[number], LOW);
  digitalWrite(valveVacuumPin[number], HIGH);
  analogWrite(pumpSupplyPin[number], 0);
  analogWrite(pumpVacuumPin[number], PWM);
}

void sendDigitalClose(int number) {
  digitalWrite(valveSupplyPin[number], HIGH);
  digitalWrite(valveVacuumPin[number], LOW);
  analogWrite(pumpSupplyPin[number], 0);
  analogWrite(pumpVacuumPin[number], 0);
}

void sendDigitalExhaust(int number) {
  digitalWrite(valveSupplyPin[number], LOW);
  digitalWrite(valveVacuumPin[number], LOW);
  analogWrite(pumpSupplyPin[number], 0);
  analogWrite(pumpVacuumPin[number], 0);
}

void printRate() {
  Serial.print("rate0: ");
  Serial.print(rate[0]);
  Serial.print(", rate1: ");
  Serial.print(rate[1]);
  Serial.print(", rate2: ");
  Serial.print(rate[2]);
  Serial.print("rate3: ");
  Serial.print(rate[3]);
  Serial.print(", rate4: ");
  Serial.print(rate[4]);
  Serial.print(", rate5: ");
  Serial.println(rate[5]);
}

void printMinMax() {
  Serial.print("[");
  Serial.print(rate[0]);
  Serial.print(":");
  Serial.print(filteredVal[0][1]);
  Serial.print("]");
  Serial.print(minVal[0]);
  Serial.print(" - ");
  Serial.print(maxVal[0]);

  Serial.print(", [");
  Serial.print(rate[1]);
  Serial.print(":");
  Serial.print(filteredVal[1][1]);
  Serial.print("]");
  Serial.print(minVal[1]);
  Serial.print(" - ");
  Serial.print(maxVal[1]);

  Serial.print(", [");
  Serial.print(rate[2]);
  Serial.print(":");
  Serial.print(filteredVal[2][1]);
  Serial.print("]");
  Serial.print(minVal[2]);
  Serial.print(" - ");
  Serial.print(maxVal[2]);

  Serial.print(", [");
  Serial.print(rate[3]);
  Serial.print(":");
  Serial.print(filteredVal[3][1]);
  Serial.print("]");
  Serial.print(minVal[3]);
  Serial.print(" - ");
  Serial.print(maxVal[3]);

  Serial.print(", [");
  Serial.print(rate[4]);
  Serial.print(":");
  Serial.print(filteredVal[4][1]);
  Serial.print("]");
  Serial.print(minVal[4]);
  Serial.print(" - ");
  Serial.print(maxVal[4]);

  Serial.print(", [");
  Serial.print(rate[5]);
  Serial.print(":");
  Serial.print(filteredVal[5][1]);
  Serial.print("]");
  Serial.print(minVal[5]);
  Serial.print(" - ");
  Serial.println(maxVal[5]);

}


void pinAttachTest() {
  //  for (int i = 0; i < 3; i++) {
  //    if (slaveVal[i] == 255) {
  //      analogWrite(pumpPin[i], 255); //0:
  //      analogWrite(pumpPin[i], 255);
  //      digitalWrite(valvePin[i], HIGH);
  //      digitalWrite(valvePin[i], HIGH);
  //    } else {
  //      analogWrite(pumpPin[i], 0);
  //      analogWrite(pumpPin[i], 0);
  //      digitalWrite(valvePin[i], LOW);
  //      digitalWrite(valvePin[i], LOW);
  //    }
  //  }
}
