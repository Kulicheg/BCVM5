#include <Wire.h>
#include <SPI.h>
#include <GyverTimer.h>
#include "GyverFilters.h"
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
#define MIN_ALTITUDE 2
#define RAMTOP 32500
#define SPEED_DELAY 200
#define TELEMETRY_DELAY 1000

unsigned long millisshift;
unsigned int counter = 0;

struct systemSettings {
  int logPos;
  bool MOSFET_1_IS_FIRED;
  bool MOSFET_2_IS_FIRED;
  bool MOSFET_3_IS_FIRED;
  bool fallen;
  long int apogee;
  long int maxSpeed;
  long int seaLevel;
  long int temperature;
  unsigned int packSize;
  unsigned int framPos;
  long int altNew;
  long int altOld;
};

struct telemetryStruct {
  long int timeStamp;
  long int pressure;
  long int altitude;
  long int speed;
  unsigned char accelX;
  unsigned char accelY;
  unsigned char accelZ;
  unsigned char gyrosX;
  unsigned char gyrosY;
  unsigned char gyrosZ;
};


struct systemSettings set;
struct telemetryStruct telemetry;

GTimer telemetryRun(MS, TELEMETRY_DELAY);
GTimer SpeedMeterRun(MS, SPEED_DELAY);
RingAverage<long int, 5> altFilter;

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

//Read logbook record
void readLog() {
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

//Write logbook record
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

//BMP-280 data collect
void getBmp280(void) {

  telemetry.pressure = bmp.readPressure();
  telemetry.altitude = bmp.readAltitude(set.seaLevel / 100.00);
}

//MPU-9250 data collect
bool getGyros(void) {
  bool result = IMU.Read();
  if (result) {

    telemetry.accelX = IMU.accel_x_mps2() * 10;
    telemetry.accelY = IMU.accel_y_mps2() * 10;
    telemetry.accelZ = IMU.accel_z_mps2() * 10;

    //1 рад × 180/π = 57,296°
    telemetry.gyrosX = IMU.gyro_x_radps();
    telemetry.gyrosY = IMU.gyro_y_radps();
    telemetry.gyrosZ = IMU.gyro_z_radps();
  } else {
    telemetry.accelX = 0;
    telemetry.accelY = 0;
    telemetry.accelZ = 0;
    telemetry.gyrosX = 0;
    telemetry.gyrosY = 0;
    telemetry.gyrosZ = 0;
  }
  return result;
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

float speedOmeter() {
  long int speed;
  set.altNew = bmp.readAltitude(set.seaLevel / 100.00);
  speed = (set.altNew - set.altOld) * 1000 / SPEED_DELAY;
  set.altOld = set.altNew;
  return speed;
}

long int altitudeFiltered(void) {

  return altFilter.filtered(bmp.readAltitude(set.seaLevel));
}

// Collect all telemetry
void getTelemetry() {
  //
  getBmp280();
  getGyros();

  Serial.print(F("altitude = "));
  Serial.println(telemetry.altitude);
  Serial.print(F("pressure = "));
  Serial.println(telemetry.pressure);

  Serial.print(F("accelX = "));
  Serial.println(telemetry.accelX);
  Serial.print(F("accelY = "));
  Serial.println(telemetry.accelY);
  Serial.print(F("accelZ = "));
  Serial.println(telemetry.accelZ);

  Serial.print(F("gyrosX = "));
  Serial.println(telemetry.gyrosX);
  Serial.print(F("gyrosY = "));
  Serial.println(telemetry.gyrosY);
  Serial.print(F("gyrosZ = "));
  Serial.println(telemetry.gyrosZ);
}


void fallingSense() {
  long int filtered = altitudeFiltered();

  if (!Fallen && telemetry.altitude > 10 and telemetry.speed < 20) {

    if ((oldAltitude - newAltitude) > 2) {
      Apogee = oldAltitude;
      toLog("Falling detected " + String(Apogee));
      EEPROM.put(945, Apogee);
      Fallen = true;

      if (DEBUG_OUT) {
        Serial.println(String(millis() - millisshift) + ":Falling detected at:" + String(Apogee));
      }
    }

    FirstTime = millis();
    if (FirstTime - SecondTime > 500) {
      SecondTime = millis();
      oldAltitude = newAltitude;
      newAltitude = filtered;
    }
  }
}

bool writeTelemetry(void) {
  unsigned char *telemetry_bytes;
  unsigned int bufPos = 0;

  if (set.framPos > RAMTOP) {
    Serial.println("FRAM is full.");
    return false;
  }

  telemetry_bytes = (unsigned char *)&telemetry;

  do {
    drive.write(set.framPos + bufPos, telemetry_bytes + bufPos);
    bufPos++;
  } while (bufPos < set.packSize);

  set.framPos = set.framPos + set.packSize;
  return true;
}

void readTelemetry(void) {
  byte packet[set.packSize];
  unsigned int recPos = 0;
  float gx, gy, gz;  // G force
  float ax, ay, az;  // Accel speed

  Serial.println(F("  Time\t Alt\t Spd\t Prs\t gx\t gy\t gz\t aX\t aY\t aZ"));
  Serial.println(" ");

  do {
    for (int intern = 0; intern < set.packSize; intern++) {
      packet[intern] = drive.read(recPos + intern);
    }
    memcpy(&telemetry, packet, set.packSize);

    gx = telemetry.accelX / 10;
    gy = telemetry.accelY / 10;
    gz = telemetry.accelZ / 10;

    //1 рад × 180/π = 57,296°
    ax = telemetry.gyrosX * 57.296;
    ay = telemetry.gyrosY * 57.296;
    az = telemetry.gyrosZ * 57.296;

    Serial.print(telemetry.timeStamp);
    Serial.print("\t");
    Serial.print(telemetry.altitude);
    Serial.print("\t");
    Serial.print(telemetry.speed);
    Serial.print("\t");
    Serial.print(telemetry.pressure);
    Serial.print("\t");
    Serial.print(gx, 2);
    Serial.print("\t");
    Serial.print(gy, 2);
    Serial.print("\t");
    Serial.print(gz, 2);
    Serial.print("\t");
    Serial.print(round(ax));
    Serial.print("\t");
    Serial.print(round(ay));
    Serial.print("\t");
    Serial.println(round(az));

    recPos = recPos + set.packSize;
  } while (recPos <= RAMTOP);

  Serial.println(F("-----------------------------------------------------"));
  Serial.print("Readed ");
  Serial.print(recPos);
  Serial.println(F(" bytes of data"));
}

void setup() {

  set.MOSFET_1_IS_FIRED = false;
  set.MOSFET_2_IS_FIRED = false;
  set.MOSFET_3_IS_FIRED = false;
  set.packSize = sizeof(telemetry);
  set.altNew = 0;
  set.altOld = 0;


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
  set.temperature = bmp.readTemperature();
  toLog("Site temp = " + String(set.temperature));
  Serial.print(F("Site temperature = "));
  Serial.println(set.temperature);
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
  IMU.ConfigGyroRange(bfs::Mpu9250::GYRO_RANGE_2000DPS);       // 2000 degrees per sec (5.5 RPS)
  IMU.ConfigDlpfBandwidth(bfs::Mpu9250::DLPF_BANDWIDTH_41HZ);  // low pass filter

  toLog(F("IMU OK!"));
  display.displayByte(_empty, _G, _y, _r);
  delay(1000);


  set.seaLevel = bmp.readPressure();
  display.displayInt(set.seaLevel * 0.00750062);
  toLog("GND pressure " + String(set.seaLevel * 0.00750062) + " mmHg");
  delay(1000);

  display.displayByte(_empty, _F, _L, _Y);

  do {
    //no fly - no telemetry
  } while (bmp.readAltitude(set.seaLevel) < MIN_ALTITUDE);
  EEPROM.update(990, 0x00);  // Clear Apogee;
  EEPROM.update(994, 0x00);  // Clear MaxSpeed;
}

void loop() {
  bool written;

  if (telemetryRun.isReady()) {
    getTelemetry();
    written = writeTelemetry();
    counter++;
    //if (!writeTelemetry()) {
    //telemetryRun.stop();
    //}
  }

  if (SpeedMeterRun.isReady()) {
    telemetry.speed = speedOmeter();

    if (telemetry.speed > set.maxSpeed) {
      set.maxSpeed = telemetry.speed;
      EEPROM.update(994, set.MaxSpeed);
    }
    Serial.print(F("speed = "));
    Serial.println(telemetry.speed);
    Serial.print(F("maxSpeed = "));
    Serial.println(set.maxSpeed);
  }

  if (counter == 5 || written == 0) {
    counter++;
    telemetryRun.stop();
    toLog("Telemetry Stopped");
    readLog();
    readTelemetry();
  }
}
