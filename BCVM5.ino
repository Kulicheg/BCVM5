#include <Wire.h>
#include <SPI.h>
#include <GyverTimer.h>
#include <EEPROM.h>
#include <FM24C256.h>
#include <Adafruit_BMP280.h>
#include <GyverTM1637.h>
#include "mpu9250.h"
// RX, TX ,VCC , GND
//I2C device found at address 0x50 !  EEPROM
//I2C device found at address 0x68 !  AXL
//I2C device found at address 0x76 !  BMP

#define CLK 12
#define DIO 11
#define BUTTON 13
#define GND 14
#define BUZZER 15

#define MOSFET1 10
#define MOSFET2 9
#define MOSFET3 0

#define BMP_SCK (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS (10)

unsigned long millisshift;
unsigned int counter = 0;

struct systemSettings {
  int logPos;
  bool MOSFET_1_IS_FIRED;
  bool MOSFET_2_IS_FIRED;
  bool MOSFET_3_IS_FIRED;
  bool fallen;
  int apogee;
  int MaxSpeed;
  long int seaLevel;
  long int pressure;
  long int altitude;
  long int temperature;
};

struct systemSettings set;

GTimer telemetryRun(MS, 1000);
FM24C256 drive(0x50);
GyverTM1637 display(CLK, DIO);
Adafruit_BMP280 bmp;
bfs::Mpu9250 IMU;
//A,B,C,D,E,F,G,H,J,L,N,O,P,S,U,Y,a,b,c,d,e,f,h,i,j,l,n,o,q,r,t,u,y,dash,under,equal,empty,degree

void nDelay(unsigned long delayTime) {
  unsigned long finishTime;
  finishTime = millis() + delayTime;
  while (millis() < finishTime) {
  }
}


void beeper(int milsec) {
  //digitalWrite(BUZZER, HIGH);
  delay(milsec);
  digitalWrite(BUZZER, LOW);
}

void getLog() {
  unsigned long timeStamp;
  unsigned char byte;
  unsigned int count;
  unsigned char logger[64];
  Serial.println(F("--------------------------------------------------"));
  Serial.println(F("---------------- [CAPITAN'S  LOG] ----------------"));
  Serial.println(F("--------------------------------------------------"));

  count = 0;
  while (count < EEPROM.length()) {
    EEPROM.get(count, timeStamp);
    count = count + 4;
    Serial.print(String(timeStamp) + "\t");
    do {
      byte = EEPROM.read(count);
      count++;
      if (byte == 0) {
        Serial.print("\r\n");
        break;
      } else {
        Serial.write(byte);
      }

    } while (42);
    //Serial.print("\r\n");
  }
  Serial.println(F("--------------------------------------------------"));
  Serial.println(F("--------------------------------------------------"));
}

void toLog(String message) {
  unsigned char logger[64];
  unsigned long timeStamp;
  unsigned char byte;
  unsigned int count;

  timeStamp = millis() - millisshift;
  message.toCharArray(logger, 32);
  logger[32] = 0;
  if (set.logPos < EEPROM.length() - strlen(logger) - 4) {
    EEPROM.put(set.logPos, timeStamp);
    set.logPos = set.logPos + 4;
    count = 0;
    do {
      byte = logger[count];

      EEPROM.write(set.logPos, byte);
      count++;
      set.logPos++;
    } while (byte != 0);
  } else {
    Serial.print(String(timeStamp) + "\t");
    Serial.println("EEPROM is full!");
  }
  Serial.print(String(timeStamp) + "\t");
  Serial.println(message);
}

void getBmp280(void) {

  set.pressure = bmp.readPressure();
  set.altitude = bmp.readAltitude(set.seaLevel / 100.00);
  set.temperature = bmp.readTemperature();
}



void getTelemetry() {
  toLog("getTelemetry()");
  getBmp280();
  Serial.print(F("altitude = "));
  Serial.println(set.altitude);
  Serial.print(F("pressure = "));
  Serial.println(set.pressure);
  Serial.print(F("temperature = "));
  Serial.println(set.temperature);
}

// FRAM detection
bool framTest() {
  unsigned char testbyte, readbyte;
  display.clear();
  for (unsigned int q = 32511; q < 32768; q++) {
    testbyte = random(255);
    drive.write(q, testbyte);
    readbyte = drive.read(q);
    if (readbyte != testbyte) {
      toLog("FRAM error!");
      display.displayByte(_dash, _2, _4, _C);
      return false;
    }
  }
  toLog("FRAM OK.");
  display.displayByte(_empty, _2, _4, _C);
  return true;
}

void sayHello(void) {
  display.displayByte(_0, _0, _0, _0);
  delay(200);
  for (int i = 1; i <= 9; i++) {
    display.displayInt(i * 1111);
    delay(200);
  }
}

void setup() {

  set.MOSFET_1_IS_FIRED = false;
  set.MOSFET_2_IS_FIRED = false;
  set.MOSFET_3_IS_FIRED = false;

  pinMode(MOSFET1, OUTPUT);  //MOSFET#1
  pinMode(MOSFET2, OUTPUT);  //MOSFET#2
  pinMode(MOSFET3, OUTPUT);  //MOSFET#3
  pinMode(BUTTON, INPUT);    //BUTTON PIN
  pinMode(BUZZER, OUTPUT);   //BUZZER
  pinMode(GND, OUTPUT);      //GND
  digitalWrite(GND, LOW);


  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);
  Serial.println("Start.");
  millisshift = millis();
  set.logPos = 0;

  beeper(250);

  display.clear();
  display.brightness(7);

  //DISPLAY test
  sayHello();

  //FRAM test
  if (!framTest()) {
    while (1) {
    }
  }
  delay(1000);
  //BMP280 test
  display.clear();
  if (!bmp.begin()) {
    display.displayByte(_dash, _2, _8, _0);
    toLog(F("BMP280 ERROR!"));
    while (1) {
    }
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X2,     /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X4,       /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  display.displayByte(_empty, _2, _8, _0);
  toLog(F("BMP280 OK."));
  delay(1000);
  // IMU test
  display.clear();
  IMU.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);
  if (!IMU.Begin()) {
    toLog(F("IMU ERROR!"));
    display.displayByte(_dash, _G, _y, _r);
    while (1) {
    }
  }
  IMU.ConfigAccelRange(bfs::Mpu9250::ACCEL_RANGE_16G);         //16HZ
  IMU.ConfigSrd(19);                                           //50HZ
  IMU.ConfigGyroRange(bfs::Mpu9250::GYRO_RANGE_1000DPS);       // 1000 degrees per sec
  IMU.ConfigDlpfBandwidth(bfs::Mpu9250::DLPF_BANDWIDTH_41HZ);  // low pass filter

  toLog(F("IMU OK!"));
  display.displayByte(_empty, _G, _y, _r);
  delay(1000);


  set.seaLevel = bmp.readPressure();
  display.displayInt(set.seaLevel * 0.00750062);
  toLog("GND pressure " + String(set.seaLevel * 0.00750062) + " mmHg");
  delay(1000);
}

void loop() {
  display.displayByte(_empty, _F, _L, _Y);

  if (telemetryRun.isReady()) {
    getTelemetry();
    counter++;
  }

if (counter == 100) {
  counter++;
  telemetryRun.stop();
  toLog("................");
  getLog();

  }
}
