//ライブラリの導入
#include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP085.h>
#include <Servo.h>
#include <SD.h>
#include <XBee.h>

#ifdef _ESP32_HAL_I2C_H_
// デバイスのI2Cアドレス(Default モード)
#define SENSOR_ADRS  0x48
#define SDA_PIN 21
#define SCL_PIN 22
#endif

//モーターピンを指定
const int motorPin = 14;
//フライトピンを指定
const int flightPin = 26;
// CSを5に設定
const uint8_t cs_SD = 5; 
//SDfileを設定
const char* fname = "/ESP_Densou_version3_3.txt";

//モーター固有の静止値
const int motor_static = 85;
//モーターを動作させる時に必要な値
const int motor_kinetic = 1000;

//フライトピンが外れた時間:time_m
static unsigned long releasing_time;
static unsigned long test_time;
static unsigned long now_time;
static unsigned long delta_time;
int n = 0;
//フライトピンは装着時、HIGHとする
boolean flightState = HIGH; 
//フライトピンが外されからプログラムの実行回数
int i = 0;
//機体が上昇してからのカウント
int j = 0;
//気圧値の変化が0になってからのカウント
int k = 0;
//パラシュート解放の限界時間
const int limit_time = 15000;
//モーター動作終了時間
const int finish_time = 20000;
//前ループにおける気圧値
float last_alti = 0;


File fo;
MPU9250 mySensor;
Adafruit_BMP085 bmp;
Servo myservo;

//モーター動作関数
void flight_motor(){
 //Serial.println("Motor action!!!");
 //Serial.println(" angle:  ");
 //Serial.println("     ");
 //Serial.println(motor_kinetic);
 //Serial.println("     ");
 myservo.write(motor_kinetic);
 //割り込み終了後loop関数に戻る
}

//モーター停止関数
void motor_unactive(){
  //Serial.println("Stoping motor ... ...");
  myservo.write(motor_static);
}

//MPU9250の設定に関する関数
void MPU_setup(){
  //MPU9250との通信準備
  //SDA,SCLを指定
#ifdef _ESP32_HAL_I2C_H_
Wire.begin(SDA_PIN, SCL_PIN); 
#else
Wire.begin();
#endif

mySensor.setWire(&Wire);
mySensor.beginAccel();
mySensor.beginMag();
mySensor.beginGyro();
 
// you can set your own offset for mag values
// mySensor.magXOffset = -50;
// mySensor.magYOffset = -55;
// mySensor.magZOffset = -10;
}

//BMP180に関する関数
void BMP_setup(){
  //BMP180センサーとの通信準備
//後に変更するコード→最終的にはエラーが出ても動作するように
  if (!bmp.begin()) {
  //Serial.println("BMP180 connection NG");
  }
  //Serial.println("BMP180 connection OK");
}

//データを収集、SDに格納に関する関数
//加速度と気圧の変数-ポインタを定義
float data(float *ax_point, float *alti_point){
//変数を定義
float temperature,pressure,alti,temp;
float ax,ay,az,gx,gy,gz,mx,my,mz,a_sqrt,m_hori;

//データを変数に代入

ax = mySensor.accelX();
ay = mySensor.accelY();
az = mySensor.accelZ();
gx = mySensor.gyroX();
gy = mySensor.gyroY();
gz = mySensor.gyroZ();
mx = mySensor.magX();
my = mySensor.magY();
mz = mySensor.magZ();
a_sqrt = mySensor.accelSqrt();
m_hori = mySensor.magHorizDirection();
temperature = bmp.readTemperature();
pressure = bmp.readSealevelPressure();
alti = bmp.readAltitude();
//Serial.println(String("temp: ")+temperature+String("| pressure: ")+pressure+String("| alti: ")+alti);
Serial.println(String("| pressure: ")+pressure);
Serial.println(String("| alti: ")+alti);

//シリアル通信よりデータの表示
mySensor.accelUpdate();
//Serial.println("print accel values");
//Serial.println("accelX: " + String(ax));
//Serial.println("accelY: " + String(ay));
//Serial.println("accelZ: " + String(az));
//Serial.println("accelSqrt: " + String(a_sqrt));

mySensor.gyroUpdate();
//Serial.println("print gyro values");
//Serial.println("gyroX: " + String(gx));
//Serial.println("gyroY: " + String(gy));
//Serial.println("gyroZ: " + String(gz));

mySensor.magUpdate();
//Serial.println("print mag values");
//Serial.println("magX: " + String(mx));
//Serial.println("maxY: " + String(my));
//Serial.println("magZ: " + String(mz));
//Serial.println("horizontal direction: " + String(m_hori));
 
//Serial.println("at " + String(millis()) + "ms");

//SDCardに出力
// 書き込みモードでファイルを開く
  fo = SD.open(fname, FILE_APPEND);
  // 書き込む
  if(fo){
    fo.println(String("ax: ")+ax+String("| ay: ")+ay+String("| az: ")+az+String("| a_sqrt:")+a_sqrt);
    fo.println(String("gx: ")+gx+String("| gy: ")+gy+String("| gz: ")+gz);
    fo.println(String("mx: ")+mx+String("| my: ")+my+String("| mz: ")+mz+("| m_hori:")+m_hori);
    fo.println(String("temp: ")+temperature+String("| pressure: ")+pressure+String("| alti: ")+alti);
    fo.println(String("time: ")+millis()+String("ms"));
    fo.close();
  }
  else{
    //Serial.println("Writing to SD is Error");
  }

  //ポインタに値を代入
  *ax_point = ax;
  *alti_point = alti;
}

//気圧の変化が無くなるとパラシュート解放に至る関数
//ポインタ使えばもっと綺麗にかけるなあ
void highest_detector(float alti){
  float delta =  alti - last_alti;
  Serial.println(String("delta:  ")+delta);
  if(abs(delta) < 0.20){
    k ++;
    if(k>=5&&k<16){
      flight_motor();
      Serial.println("work!!!!!!!!!!!!!!!!!!!!");
    }
    else if(k>15){
      motor_unactive();
      Serial.println("after action");
    }
    else{
      Serial.println("rising ... ");
      Serial.println(String("delta")+delta);
    }
  }
  last_alti = alti;
}

//機体の上昇を検知する関数
void rise_detector(float ax,float alti){
  j ++;
  Serial.println(String("loop: ")+j);
  //加速度データが正の値を10回取り続けると・・・
  if(j > 50){
    //flight()関数を呼び出し
    highest_detector(alti);
    Serial.println("detecting rising");
   }
   //10回未満だと
   else{
    motor_unactive();
   }
}
   
//フライトピンが外れてから時間を計測してくれる関数
int flight_timer(){
  
 //フライトピンからの電圧の読み込み
 flightState = digitalRead(flightPin);

 //変数を定義
 float ax, alti;
 
 //フライトピンが解除しなくても,センサーデータは取得できるように ->冗長性
 //加速度, 気圧の変数をアドレス化
 data(&ax, &alti);
 Serial.println(String("ax :  ")+ax);

 //MPU9250の設置方向に合わせてください
 //MPU9250が逆向きなので、負の値をとる
    if(ax < 40){
      //ax>0だとダメなのであとで調整
      rise_detector(ax,alti);
      Serial.println("This rocket detected rising... ....");
      }
    //j=50以上ならばとにかく気圧差分関数で移動
    else if(j > 50){
      highest_detector(alti);
    }
    else{
      motor_unactive();
      //j = 0;
    }
    
 if (flightState == LOW ){
  //Serial.println("...Flight Pin is released...");
  //フライトピンが外されてから何週目のプログラムかを表示
  //Serial.println(String("Loop from leasing pin: ")+i);

  //フライトピンが外れた時間:releasing_timeを取得
  if(i == 0){
  releasing_time = millis();
  //Serial.println("Now release flight pin!!");
  }

  //フライトピンが外れた時刻の時間を計測
  //delay()は休止期間、すべての動作を止めてしまうので却下
  unsigned int timer = millis() - releasing_time;

  //time_dif:フライトピンが外されてからの時間を表示
  //Serial.println(String("Time :  ")+timer);
  
  //フライトピンが外れた時間(i=0のとき)以外、変数time_mは更新されない
  i = i+1;

   //限界の時間を過ぎると・・・
   if(timer > limit_time && finish_time > timer){
    //Serial.println("start event!"); 
    //Serial.println(String("Time :  ") + timer);
    //flight()関数を呼び出し
    flight_motor();
   }
   else if(timer > finish_time){
    //Serial.println("after action");
    motor_unactive();
   }
   else{
     motor_unactive();
   //Serial.println("... This motor already operated");
     }
   }
}

//初期化関数
void setup() {
while(!Serial);

//シリアル通信の開始
Serial.begin(9600);

//センサー接続
MPU_setup();
BMP_setup();

//フライトピンのピン指定
  pinMode(flightPin,INPUT);
  
//モーター接続
  myservo.attach(motorPin);
  pinMode(motorPin,OUTPUT);

//SD接続
if(SD.begin(cs_SD, SPI, 24000000, "/sd")){
    //Serial.println("Connection to SD module is OK!");
 // 書き込みモードでファイルを開く 
  }
else{
  //Serial.println("Connection SD module ... NG ...");
  }
}

//電池の耐久試験
void test_timer(){
 if(n==0){
    now_time = millis(); 
  }
  n++;
  test_time = millis();
  // 書き込みモードでファイルを開く
  fo = SD.open(fname, FILE_APPEND);
  delta_time = (test_time - now_time)/60000;
  // 書き込む
  if(n%7000==0){
    if(fo){
      fo.println(String("time: ")+delta_time+String("min"));
      fo.close();
    }
  }
}

void loop() {
  flight_timer();
  test_timer();
}
