#define PHONE_NUMBER "+84328813393"
#include <WiFi.h>
#include <Wire.h>
#include <GY6050.h> 

GY6050 gyro(0x68);  // Create Gyro object with "0x68" as I2C address (most likely the address of your MPU6050)

#include <TinyGPS++.h>
#include <SoftwareSerial.h>
/*
   This sample sketch demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   9600-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const int RXPin = 26, TXPin = 27;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);
#define BLYNK_TEMPLATE_ID "TMPxxxxxx"
#define BLYNK_TEMPLATE_NAME "Device"
#define BLYNK_AUTH_TOKEN "29PonHqoRU5aR_UWMtGTWECuEbZ5iLGH"

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial


#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

// Thông tin mạng Wi-Fi
char ssid[] = "Thu Thao";
char pass[] = "12052002";
// Tọa độ mặc định của marker
//10.897855, 106.881086
float latitude = 10.897855;    //
float longitude = 106.881086;  //
int AX, AY, AZ, GX, GY, GZ;
long times = 0;
#include <HardwareSerial.h>

#define simSerial Serial2
#define MCU_SIM_BAUDRATE 115200
#define MCU_SIM_TX_PIN 17
#define MCU_SIM_RX_PIN 16

int sobuoc = 0;
bool tt = 0, tt1 = true,tt2 = true;
#define SOS 25
#define gas 34
#define coi 19
#include "DHT.h"

#define DHTPIN 18
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

#define TRIGGER_PIN 12  // Chân GPIO nối với Trig
#define ECHO_PIN 14     // Chân GPIO nối với Echo
#define quang 15
#define den 23
WidgetMap myMap(V10);
// Hàm đọc tọa độ từ GPS
void displayInfo() {
  Serial.print(F("Location: "));
  if (gps.location.isValid()) {
    latitude = gps.location.lat(), 6;
    longitude = gps.location.lng(), 6;
    Serial.print(latitude);
    Serial.print(F(","));
    Serial.print(longitude);
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid()) {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid()) {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}



void setup() {
  Serial.begin(115200);
  pinMode(SOS, INPUT_PULLUP);
  pinMode(quang, INPUT_PULLUP);
  pinMode(gas, INPUT);
  pinMode(coi, OUTPUT);
  pinMode(den, OUTPUT);
  pinMode(TRIGGER_PIN, OUTPUT);  // Đặt TRIGGER_PIN làm đầu ra
  pinMode(ECHO_PIN, INPUT);      // Đặt ECHO_PIN làm đầu vào
  ss.begin(GPSBaud);
  dht.begin();
  Serial.println(F("DeviceExample.ino"));
  Serial.println(F("A simple demonstration of TinyGPS++ with an attached GPS module"));
  Serial.print(F("Testing TinyGPS++ library v. "));
  Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass, "blynk-server.com", 8080);


  Serial.println("Server đang chạy!");
  Serial.println(WiFi.localIP());
  gyro.initialisation();
  simSerial.begin(MCU_SIM_BAUDRATE, SERIAL_8N1, MCU_SIM_RX_PIN, MCU_SIM_TX_PIN);
  delay(1000);
  // Check AT Command
  sim_at_cmd("AT");

  // Product infor
  sim_at_cmd("ATI");

  // Check SIM Slot
  sim_at_cmd("AT+CPIN?");

  // Check Signal Quality
  sim_at_cmd("AT+CSQ");

  sim_at_cmd("AT+CIMI");
}

void loop() {
  Blynk.run();
  digitalWrite(den,digitalRead(quang));  
  
  int nongdogas = map(analogRead(gas),4095,2320,100,0);
  if(nongdogas <= 0) nongdogas = 0;
  Serial.print("GAS: ");
  Serial.println(analogRead(gas));
   
  if(nongdogas >= 10){
    tt = 1;
    if(tt2){
    nhantin("CANH BAO NONG DO CON/GAS VUOT QUA MUC CHO PHEP");
    }
    tt2 = false;
  }
  else tt2 = true;
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
 
  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.println(F("°C "));
  Blynk.virtualWrite(V0, String(t,1));
  Blynk.virtualWrite(V1, String(h,0));
  Blynk.virtualWrite(V3, nongdogas);
  int index = 1;  // Thêm khai báo trước khi dùng
  myMap.location(index, latitude, longitude, "Vi Tri Non");
  MPU();
  tenga();
  if (digitalRead(SOS) == 0) {
    tt = !tt;
    digitalWrite(coi, tt);
    if (tt) {
      nhantin("NUT NHAN SOS DA DUOC NHAN");
    }
    delay(300);
  }
  if (tt) {
    digitalWrite(coi, HIGH);
    delay(100);
    digitalWrite(coi, LOW);
    delay(100);
  }
  digitalWrite(TRIGGER_PIN, LOW);   // Đảm bảo chân TRIGGER_PIN ở mức LOW
  delayMicroseconds(2);             // Chờ 2 micro giây
  digitalWrite(TRIGGER_PIN, HIGH);  // Đưa tín hiệu TRIGGER_PIN lên mức HIGH
  delayMicroseconds(10);            // Giữ mức HIGH trong 10 micro giây
  digitalWrite(TRIGGER_PIN, LOW);   // Đưa TRIGGER_PIN về mức LOW

  // Đo thời gian Echo ở mức HIGH
  long duration = pulseIn(ECHO_PIN, HIGH);

  // Tính khoảng cách (đơn vị cm)
  float khoangcach = (duration / 2.0) * 0.0343;
  Blynk.virtualWrite(V2, khoangcach);
  Serial.print("Khoảng cách đo được: ");
  Serial.print(khoangcach);
  Serial.println(" cm");
  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      displayInfo();
}

void MPU() {
  AX = gyro.refresh('A', 'X');
  AY = gyro.refresh('A', 'Y');
  AZ = gyro.refresh('A', 'Z');
  GX = gyro.refresh('G', 'X');
  GY = gyro.refresh('G', 'Y');
  GZ = gyro.refresh('G', 'Z');
  Serial.print("AcX = ");
  Serial.print(AX);  // Ask for the X axis of the Accelerometer and print it
  Serial.print(" | AcY = ");
  Serial.print(AY);  // Ask for the Y axis of the Accelerometer and print it
  Serial.print(" | AcZ = ");
  Serial.print(AZ);  // Ask for the Z axis of the Accelerometer and print it
  Serial.print(" | Tmp = ");
  Serial.print(gyro.refresh('T', 'C'));  // Ask for the Temperature in Celsius and print it
  Serial.print(" C | ");
  Serial.print(gyro.refresh('T', 'F'));  // Ask for the Temperature in Farenheit and print it
  Serial.print(" F");
  Serial.print(" | GyX = ");
  Serial.print(GX);  // Ask for the X axis of the Gyroscope and print it
  Serial.print(" | GyY = ");
  Serial.print(GY);  // Ask for the Y axis of the Gyroscope and print it
  Serial.print(" | GyZ = ");
  Serial.println(GZ);  // Ask for the Z axis of the Gyroscope and print it, then carriage return
  delay(25);
}

void tenga() {
  if (AX > 88 || AX < -50 || AY > 70 || AY < -70) {
    if (millis() - times >= 2000) {
      tt = true;
      Serial.println("CANH BAO TE NGA");
      if (tt1) {
        nhantin("CANH BAO! NGUOI THAN DANG TE NGA");
      }
      tt1 = false;
    }
  } else {
    tt1 = true;
    times = millis();
  }
}

void nhantin(String sms) {
  Serial.println("DANG NHAN TIN");
  sim_at_cmd("AT+CMGF=1");
  String temp = "AT+CMGS=\"";
  temp += (String)PHONE_NUMBER;
  temp += "\"";
  sim_at_cmd(temp);
  String message = sms + " - TOA DO: lat " + String(latitude, 6) + " long: " + String(longitude, 6);
  sim_at_cmd(message);


  // End charactor for SMS
  sim_at_send(0x1A);
}
void sim_at_wait() {
  delay(100);
  while (simSerial.available()) {
    Serial.write(simSerial.read());
  }
}
bool sim_at_send(char c) {
  simSerial.write(c);
  return true;
}
bool sim_at_cmd(String cmd) {
  simSerial.println(cmd);
  sim_at_wait();
  return true;
}
