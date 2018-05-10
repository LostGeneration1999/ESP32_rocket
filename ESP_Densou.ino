
#include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP085.h>
#include <skADT7410.h>
#include <Servo.h>
#include <SD.h>
#ifdef _ESP32_HAL_I2C_H_
#define SENSOR_ADRS     0x48       // デバイスのI2Cアドレス
#define SDA_PIN 21
#define SCL_PIN 22
#endif


const int motorPin = 14;
const int flightPin = 26;
const uint8_t cs_SD = 5; // GPIO5=CS
const char* fname = "/test1.txt";

//モーター固有の静止値
const int motor_static = 85;
//モーターを動作させる時に必要な値
const int motor_kinetic = 1000;
//フライトピンが外れた時間:time_m
static unsigned long time_m;
//フライトピンは装着時、HIGHとする
boolean flightState = HIGH; 
//フライトピンが外されからプログラムの実行回数
int i = 0;
int ans;

// 温度センサーライブラリの生成を行う
skADT7410  Temp(SENSOR_ADRS);
File fo;
MPU9250 mySensor;
Adafruit_BMP085 bmp;
Servo myservo;


void flight(){
 const int pos = motor_kinetic;
 Serial.println(" angle:  ");
 Serial.println("     ");
 Serial.println("     ");
 Serial.println(pos);
 Serial.println("     ");
 Serial.println("     ");
 myservo.write(pos);
 //割り込み終了後loop関数に戻る
}

void setup() {
while(!Serial);
 
Serial.begin(9600);
Serial.println("started");

//MPU9250との通信準備
#ifdef _ESP32_HAL_I2C_H_
// for esp32
Wire.begin(SDA_PIN, SCL_PIN); //sda, scl
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

//BMP180センサーとの通信準備
//後に変更するコード→最終的にはエラーが出ても動作するように
  if (!bmp.begin()) {
  Serial.println("BMP180 connection NG");
  while (1) {};
  }
  Serial.println("BMP180 connection OK");

// 温度センサーの初期化を行う(16bitの解像度 動作モードはシャットダウン)
     ans = Temp.Begin() ;
     if (ans == 0) Serial.println("Initialization normal") ;
     else {
          Serial.print("Initialization abnormal ans=") ;
          Serial.println(ans) ;
     }
     // 動作モードを"連続測定モード"にする
     Temp.ActionMode(ADT_MODE_CONTINUE) ;
 
 //フライトピンのピン指定
  pinMode(flightPin,INPUT);
  
  //モーター接続準備
  myservo.attach(motorPin);
  pinMode(motorPin,OUTPUT);


// SDライブラリを初期化
  SD.begin(cs_SD, SPI, 24000000, "/sd");   
}

void loop() {
float temperature,pressure,alti,temp;
float ax,ay,az,gx,gy,gz,mx,my,mz,a_sqrt,m_hori;

//機体内部の温度変化を測定
ans = Temp.Read(&temp) ;
if (ans == 0) {
          Serial.print(temp,4) ;
          Serial.write(0xDF) ; // ℃
          Serial.println("C") ;
     } else Serial.println("NG") ;

//データの変数を定義
ax = mySensor.accelX();
ay = mySensor.accelY();
az = mySensor.accelZ();
gx = mySensor.accelX();
gy = mySensor.accelY();
gz = mySensor.accelZ();
mx = mySensor.accelX();
my = mySensor.accelY();
mz = mySensor.accelZ();
a_sqrt = mySensor.accelSqrt();
m_hori = mySensor.magHorizDirection();
temperature = bmp.readTemperature();
pressure = bmp.readSealevelPressure();
alti = bmp.readAltitude();
Serial.println(String("temp: ")+temperature+String("| pressure: ")+pressure+String("| alti: ")+alti);



mySensor.accelUpdate();
Serial.println("print accel values");
Serial.println("accelX: " + String(ax));
Serial.println("accelY: " + String(ay));
Serial.println("accelZ: " + String(az));
Serial.println("accelSqrt: " + String(a_sqrt));

mySensor.gyroUpdate();
Serial.println("print gyro values");
Serial.println("gyroX: " + String(gx));
Serial.println("gyroY: " + String(gy));
Serial.println("gyroZ: " + String(gz));

mySensor.magUpdate();
Serial.println("print mag values");
Serial.println("magX: " + String(mx));
Serial.println("maxY: " + String(my));
Serial.println("magZ: " + String(mz));
Serial.println("horizontal direction: " + String(m_hori));
 
Serial.println("at " + String(millis()) + "ms");
delay(100);

//SDCardに出力
// 書き込みモードでファイルを開く
  fo = SD.open(fname, FILE_WRITE);
    // 書き込む
    fo.println(String("ax: ")+ax+String("| ay: ")+ay+String("| az: ")+az+String("| a_sqrt:")+a_sqrt);
    fo.println(String("gx: ")+gx+String("| gy: ")+gy+String("| gz: ")+gz);
    fo.println(String("mx: ")+mx+String("| my: ")+my+String("| mz: ")+mz+("| m_hori:")+m_hori);
    fo.println(String("temp: ")+temperature+String("| pressure: ")+pressure+String("| alti: ")+alti);
    fo.println(String("temp_hybrid: ")+temp);
    Serial.println("Writing now");
    // ファイルを閉じる
  fo.close();
  delay(500);
  
 //フライトピンからの電圧の読み込み
 flightState = digitalRead(flightPin);
 
 //フライトピン解除もしくは気圧センサーが0になったときにカウントが始まる
 if (flightState == LOW ){
  Serial.println("LOW");
  
  //フライトピンが外されてから何週目のプログラムかを表示
  Serial.println(i);

  //フライトピンが外れた時間:time_mを取得
  if(i == 0){
  time_m = millis();
  }


    
  //フライトピンが外れた時刻の時間を計測
  //delay()は休止期間、すべての動作を止めてしまうので却下
  unsigned long time_dif = millis() - time_m;

  //time_m:フライトピンが外された時刻を表示
  //Serial.println(String("Time :  ")+time_m);

  //time_dif:フライトピンが外されてからの時間を表示
  Serial.println(String("Time :  ")+time_dif);
  
  //フライトピンが外れた時間(i=0のとき)以外、変数time_mは更新されない
  i = i+1;
  //8秒後~10秒後にかけてモーターは動作
  if(time_dif <= 10000 && time_dif >= 8000){
    Serial.println("start event!"); 
    Serial.println(String("Time :  ") + time_dif);
    Serial.println("     ");
    Serial.println("     ");
    //flight()関数を呼び出し
    flight();
  }
  else{
    //10秒後~ではモータは静止
    myservo.write(motor_static); 
    Serial.println("After behavior");
  }
}
 //動作しないときモータには85(サーボモータ固有の回転停止する値)を与える
 //サーボモータによって値は変わる
else{
  myservo.write(motor_static); 
  Serial.println(flightState);
  Serial.println("stop mode...");
}
delay(900);
}