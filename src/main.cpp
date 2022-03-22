#include <Arduino.h>
#include "module.cpp"

#include <SPI.h>
#include "time.h"
#include <Preferences.h>

#include <TinyGPSPlus.h> //gps uart
#include <SoftwareSerial.h>

#include <Adafruit_SSD1306.h> //oled
#include <Adafruit_GFX.h>
#include "Adafruit_Sensor.h"

#include <Adafruit_ADXL345_U.h> //adxl345
#include "Adafruit_MLX90614.h"  //temperature

#include "MAX30105.h" //max30102
#include "heartRate.h"
#include "spo2_algorithm.h"

#include "MPU9250.h" //9-axis sensor

#include "MAX17043.h" //fuel gauge

#include <Firebase_ESP_Client.h> //firebase
#include <addons/TokenHelper.h>
#define API_KEY "AIzaSyDGj1Fouh114V3w65thJrYYNigxP1KjbMQ"
#define FIREBASE_PROJECT_ID "isdp-testdb"
#define USER_EMAIL "isdp_esp_default_acc@isdp6.dont.change"
#define USER_PASSWORD "!adUINsa*(76jka^k12"

#include <Wire.h>
#define SDA2 21
#define SCL2 22

#include <WiFi.h>
#include "BluetoothSerial.h"
#define BtDevice "How'rU_HRMS"

struct Button
{
  const uint8_t PIN;
  bool state;
};

// global var
String WIFI_SSID = "";
String WIFI_PASSWORD = "";
String UID = "";
String documentPath = "activity";

Button StartStopBtn = {35, false};
Button NextBtn = {34, false};

const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 28800; //+8 hour
const int daylightOffset_sec = 0;
unsigned long timeElapsed;

uint32_t irBuffer[90];
uint32_t redBuffer[90];
int32_t bufferLength;
int32_t spo2;
int8_t validSPO2;
int32_t heartRate;
int32_t heartRateBuffer[20];
int8_t validHeartRate;
int32_t hrAvg = 0;

bool exitLoop = false;

static const int RXPin = 19, TXPin = 18;
static const uint32_t GPSBaud = 9600;

RTC_DATA_ATTR int bootCount = 0;

float pitch_buffer[6] = {0, 0, 0, 0, 0, 0};
float roll_buffer[6] = {0, 0, 0, 0, 0, 0};
float accX_buffer[6] = {0, 0, 0, 0, 0, 0};
float accY_buffer[6] = {0, 0, 0, 0, 0, 0};
float accZ_buffer[6] = {0, 0, 0, 0, 0, 0};
float AdxaccX_buffer[6] = {0, 0, 0, 0, 0, 0};
float AdxaccY_buffer[6] = {0, 0, 0, 0, 0, 0};
float AdxaccZ_buffer[6] = {0, 0, 0, 0, 0, 0};

// sensor & module
BluetoothSerial SerialBT;
Preferences storeSetting;

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Adafruit_SSD1306 display(128, 64, &Wire, -1);
Adafruit_MLX90614 mlx;
MAX30105 particleSensor;
MAX17043 FuelGauge;
MPU9250 mpu;

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
FirebaseJson doc;

SoftwareSerial ss(RXPin, TXPin);
TinyGPSPlus gps;

// function prototype
void IRAM_ATTR Next();
void IRAM_ATTR StartStop();

int menu();
int settingMenu();
void batteryMenu();
void batteryMenuText(char *text1);
void settingMenuText(char *text1, char *text2);
void realTimeText(int c1, int c2, char *text1);
void mainMenuText(char *num, char *text, char *title);

void configureMenuButton();
void configureOled();
void configureFuelGauge();
void ConfigureWifi();
void ConfigOperation();
void ConfigureUid();
bool connectWifi();
void getSSID_PASSWORD();

void setupMax30102();
void setupMpu();
void setupMlx();
void setupAdx();
void setupGPS();
void initialDisp(char *ini);

void initiateFirestore();
void wifiSituationalControlLoop();
void uploadSituationalControlLoop();
bool uploadActivity();
void selectOperation(int program_selection);
void startActivity(uint32_t *startTimeMillis);
void getTimeElapsed(uint32_t *startTimeMillis);

void recordGPS();
void recordTimeMillis(bool isstart);
void recordSpoHeartrate(int jsonArrayCounter);
void recordTemperature(int jsonArrayCounter);
void recordMove(int buffer_position, float *pitchavg, float *rollavg);
void record_position(int jsonArrayCounter, float pitchavg, float rollavg, float maxMovX, float maxMovY, float maxMovZ, float maxAccX, float maxAccY, float maxAccZ, float standAdxX, float standAdxY, float standAdxZ, float standAccX, float standAccY, float standAccZ, int *state);
void warmUpGPS();
void warmUpMax(int32_t length);
void warmUpMpu(float *pitchavg, float *rollavg, float *standPitch, float *standRoll, float *standAccX, float *standAccY, float *standAccZ, float *standAdxX, float *standAdxY, float *standAdxZ);

unsigned long get_Epoch_Time();
void realTimeDisplay();
void OxyBpm();

void sensor_sleep();
void sensor_wakeup();
//****

void setup()
{

  Serial.begin(115200);
  Wire.begin(SDA2, SCL2, 100000);
  Serial.println("starting up");

  configureOled();

  setupGPS();
  initialDisp("Booting Up .");
  delay(50);

  configureMenuButton();

  // lightsleep wakeup button 33 at high awake
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 1);
  initialDisp("Booting Up ..");
  delay(50);

  configureFuelGauge();
  initialDisp("Booting Up ...");

  setupMax30102();
  initialDisp("Booting Up ....");

  setupMpu();
  setupAdx();

  setupMlx();
  initialDisp("Booting Up ......");

  getSSID_PASSWORD();
  initiateFirestore();
  initialDisp("Booting Up .......");

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  initialDisp("Let's Go!!");
}

void loop()
{
  int program_selection = 0;

  program_selection = menu();

  selectOperation(program_selection);

  delay(500);
}

//####################################################################################################################//
void selectOperation(int program_selection)
{
  if (program_selection == 1)
  {
    uint32_t startTimeMillis;
    while (1)
    {
      ++bootCount;
      startActivity(&startTimeMillis);

      recordTimeMillis(false);
      recordGPS();

      doc.set(String(titleLoc) + "stringValue", "Health Check Demo" + String(bootCount));
      doc.set(String(uidLoc) + "stringValue", UID);
      doc.set(String(notesLoc) + "stringValue", "routine monitoring");
      doc.set(String(bpUpLoc), 0);
      doc.set(String(bpLowLoc), 0);
      // Serial.print("time elapsed : ");
      // Serial.println(millis() - startTimeMillis);

      if (Firebase.ready())
      {
        Serial.println("token ready");
      }

      bool status = uploadActivity();
      if (!status)
      {
        // uploadSituationalControlLoop();
        Serial.println("fail upload");
        doc.clear();
      }

      if (exitLoop)
      {
        exitLoop = false;
        break;
      }
    }
  }
  else if (program_selection == 2)
  {
    realTimeDisplay();
  }
  else if (program_selection == 3)
  {
    ConfigOperation();
    StartStopBtn.state = false;
  }
  else if (program_selection == 4)
  {
    delay(1000);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("Sleeping"));
    display.display();
    sensor_sleep();
    esp_light_sleep_start();
    sensor_wakeup();
  }
  else if (program_selection == 5)
  {
    batteryMenu();
  }

  delay(100);
  StartStopBtn.state = false;
}

void startActivity(uint32_t *startTimeMillis)
{
  float standPitch = 0;
  float standRoll = 0;

  float standAccX = 0;
  float standAccY = 0;
  float standAccZ = 0;

  float pitchavg = 0;
  float rollavg = 0;

  float standAdxX, standAdxY, standAdxZ = 0;

  float maxMovX, maxMovY, maxMovZ = 0;
  float maxAccX, maxAccY, maxAccZ = 0;

  int state = 1; // 1 stand, 2 walk, 3 run, 4 sit, 5 lay

  Serial.println("running task");
  StartStopBtn.state = false;
  bufferLength = 90;
  bool newActivity = true;
  int jsonArrayCounter = 0;
  int buffer_position = 0;

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println(F("Stand, Hand"));
  display.setCursor(0, 25);
  display.println(F("Aside 5sec"));
  display.display();
  delay(2500);

  warmUpMpu(&pitchavg, &rollavg, &standPitch, &standRoll, &standAccX, &standAccY, &standAccZ, &standAdxX, &standAdxY, &standAdxZ);

  static uint32_t prev_ms = millis();

  warmUpMax(bufferLength);

  *startTimeMillis = millis();

  recordTimeMillis(true);

  while (jsonArrayCounter < 1800)
  { // operational Loop

    for (byte i = 25; i < 90; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    for (byte i = 70; i < 90; i++)
    {
      if (newActivity)
      {
        heartRateBuffer[i - 70] = heartRate;
        hrAvg += heartRate - 10;
        newActivity = false;
      }
      else if (heartRate > 0)
      {
        heartRateBuffer[i - 70] = heartRateBuffer[i - 69];
        hrAvg += heartRateBuffer[i - 69];
      }

      while (particleSensor.available() == false)
        particleSensor.check();

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample();
    }

    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

    if (heartRate > 0)
    {
      heartRateBuffer[19] = heartRate - 10;
      hrAvg += heartRate;
    }

    if (mpu.update())
    {
      recordMove(buffer_position, &pitchavg, &rollavg);
      if (++buffer_position > 5)
      {
        buffer_position = 0;
      }
    }

    if (millis() > prev_ms + 940)
    {
      prev_ms = millis();

      maxMovX = AdxaccX_buffer[0];
      maxMovY = AdxaccY_buffer[0];
      maxMovZ = AdxaccZ_buffer[0];
      maxAccX = accX_buffer[0];
      maxAccY = accY_buffer[0];
      maxAccZ = accZ_buffer[0];

      for (int i = 1; i < 6; i++)
      {
        if (maxMovX < AdxaccX_buffer[i])
        {
          maxMovX = AdxaccX_buffer[i];
        }
        if (maxMovY < AdxaccY_buffer[i])
        {
          maxMovY = AdxaccY_buffer[i];
        }
        if (maxMovZ < AdxaccZ_buffer[i])
        {
          maxMovZ = AdxaccZ_buffer[i];
        }
        if (maxAccX < accX_buffer[i])
        {
          maxAccX = accX_buffer[i];
        }
        if (maxAccY < accY_buffer[i])
        {
          maxAccY = accY_buffer[i];
        }
        if (maxAccZ < accZ_buffer[i])
        {
          maxAccZ = accZ_buffer[i];
        }
      }

      record_position(jsonArrayCounter, pitchavg, rollavg, maxMovX, maxMovY, maxMovZ, maxAccX, maxAccY, maxAccZ, standAdxX, standAdxY, standAdxZ, standAccX, standAccY, standAccZ, &state);
      recordSpoHeartrate(jsonArrayCounter);
      recordTemperature(jsonArrayCounter);
      getTimeElapsed(startTimeMillis);
      jsonArrayCounter++;
    }

    hrAvg = 0;
    if (StartStopBtn.state)
    {
      exitLoop = true;
      StartStopBtn.state = false;
      break;
    }
  }
}

void OxyBpm()
{
  static uint32_t prev_ms = millis();
  bufferLength = 100;
  display.clearDisplay();
  realTimeText(0, 0, "Calculating");

  // calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  for (byte i = 0; i < bufferLength; i++)
  {
    while (particleSensor.available() == false) // do we have new data?
      particleSensor.check();                   // Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); // We're finished with this sample so move to next sample

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  // Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while (1)
  {
    // dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {

      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    // take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) // do we have new data?
        particleSensor.check();                   // Check the sensor for new data

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); // We're finished with this sample so move to next sample

      // send samples and calculation result to terminal program through UART
    }

    // After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
    if (millis() > prev_ms + 1000)
    {
      prev_ms = millis();
      display.clearDisplay();

      realTimeText(0, 0, "BPM: ");
      display.setCursor(60, 0);
      display.print(heartRate);

      realTimeText(0, 25, "SPO2: ");
      display.setCursor(60, 25);
      display.print(spo2);
      realTimeText(0, 45, "'C: ");
      display.setCursor(60, 45);
      display.print(mlx.readObjectTempC());
      display.display();
    }

    if (StartStopBtn.state)
    {
      exitLoop = true;
      break;
    }
  }
}

void realTimeDisplay()
{
  display.clearDisplay();
  realTimeText(0, 0, "Real Time Data");
  while (!exitLoop)
  {
    display.clearDisplay();
    OxyBpm();
  }
  exitLoop = false;
  StartStopBtn.state = false;
}

// component setup
void initiateFirestore()
{
  // displayoled "setting up <newline> firestore"
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println(F("Setting up"));
  display.setCursor(0, 25);
  display.println(F("firestore"));
  display.display();
  delay(300);

  bool status = connectWifi();
  if (!status)
  {
    wifiSituationalControlLoop();
  }

  config.api_key = API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);

  // displayoled "Token <newline> retrieved"
  display.clearDisplay();
  // display.setTextSize(2);
  display.setCursor(0, 0);
  display.println(F("Token"));
  display.setCursor(0, 25);
  display.println(F("Retrieved"));
  display.display();
}

void configureOled()
{
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // infinite loop, before proceed
  }

  display.clearDisplay();
  display.setTextSize(2, 2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(15, 15);
  display.println("How R U :D :3 :P");
  display.display();
  delay(200);
}

void configureFuelGauge()
{
  FuelGauge.reset();
  FuelGauge.quickStart();
}

void setupGPS()
{
  ss.begin(GPSBaud);

  Serial.println(F("DeviceExample.ino"));
  Serial.println(F("A simple demonstration of TinyGPSPlus with an attached GPS module"));
  Serial.print(F("Testing TinyGPSPlus library v. "));
  Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();

  for (int i = 0; i < 20; i++)
  {
    delay(50);
    while (ss.available() > 0)
      if (gps.encode(ss.read()))
        warmUpGPS();
      else if (millis() > 5000 && gps.charsProcessed() < 10)
      {
        Serial.println(F("No GPS detected: check wiring."));
        while (true)
          ;
      }
  }
}

void setupMax30102()
{

  while (!particleSensor.begin(Wire, I2C_SPEED_STANDARD, 0x57))
  {
    Serial.println("MAX30102 was not found");
    delay(1000);
  }

  // Max30102 setting
  byte ledBrightness = 5;
  byte sampleAverage = 2;
  byte ledMode = 2;
  byte sampleRate = 200;
  int pulseWidth = 215;
  int adcRange = 2048;

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
}

void setupMpu()
{
  MPU9250Setting setting;
  setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

  while (!mpu.setup(0x68))
  { // change to your own address
    Serial.println("MPU connection failed. ");
    delay(1000);
  }

  mpu.verbose(true);

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println(F("Set sensor"));
  display.setCursor(0, 25);
  display.println(F("still"));
  display.display();
  delay(100);

  mpu.calibrateAccelGyro();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("Wave in 8"));
  display.setCursor(0, 25);
  display.println(F("figure"));
  display.display();
  delay(100);
  mpu.calibrateMag();

  display.clearDisplay();
  display.setCursor(0, 25);
  display.println(F("Done"));
  display.display();
  delay(100);

  mpu.setMagneticDeclination(-0.14);
  mpu.setFilterIterations(10);
  mpu.verbose(false);
}

void setupAdx()
{

  if (!accel.begin(0x53))
  {
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while (1)
      ;
  }
  accel.setRange(ADXL345_RANGE_2_G);
}

void setupMlx()
{
  while (!mlx.begin(0x5A))
  {
    Serial.println("Could not find a valid MLX9016 sensor, check wiring!");
    delay(1000);
  }
}

void configureMenuButton()
{
  // Set gpio
  pinMode(StartStopBtn.PIN, INPUT_PULLDOWN);
  pinMode(NextBtn.PIN, INPUT_PULLDOWN);

  // attach pin to intterupt function
  attachInterrupt(StartStopBtn.PIN, StartStop, FALLING);
  attachInterrupt(NextBtn.PIN, Next, FALLING);
}

// sensor warmup
void warmUpGPS()
{
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    double lat = (gps.location.lat());
    Serial.print(lat);
    Serial.print(F(","));
    double lng = (gps.location.lng());
    Serial.print(lng);
  }
  else
  {
    Serial.print(F("INVALID"));
  }
  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

void warmUpMpu(float *avgpitch, float *avgroll, float *standPitch, float *standRoll, float *standAccX, float *standAccY, float *standAccZ, float *standAdxX, float *standAdxY, float *standAdxZ)
{
  static uint32_t prev_ms = millis();
  float accY, accZ, accX = 0;

  int x = 0;
  while (x < 6)
  {
    if (mpu.update())
    {
      if (millis() > prev_ms + 100)
      {
        pitch_buffer[x] = mpu.getPitch();
        *avgpitch += pitch_buffer[x];

        roll_buffer[x] = mpu.getRoll();
        *avgroll += roll_buffer[x];

        accX_buffer[x] = abs(mpu.getAccX());
        accX += accX_buffer[x];

        accY_buffer[x] = abs(mpu.getAccY());
        accY += accY_buffer[x];

        accZ_buffer[x] = abs(mpu.getAccZ());
        accZ += accZ_buffer[x];

        prev_ms = millis();
        x++;
      }
    }
  }

  *standPitch = *avgpitch = *avgpitch / 6;
  *standRoll = *avgroll = *avgroll / 6;
  *standAccX = accX / 6;
  *standAccY = accY / 6;
  *standAccZ = accZ / 6;

  sensors_event_t event;
  accel.getEvent(&event);

  *standAdxX = abs(event.acceleration.x);
  *standAdxY = abs(event.acceleration.y);
  *standAdxZ = abs(event.acceleration.z);

  Serial.println("first value");
  Serial.println((String)*standPitch + " " + *standRoll + " " + *standAccX + " " + *standAccY + " " + *standAccZ);
}

void warmUpMax(int32_t length)
{
  for (byte i = 0; i < length; i++)
  {
    while (particleSensor.available() == false)
      particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
}

// data logging
void recordGPS()
{
  double lat;
  double lng;
  bool done = false;
  for (int i = 0; i < 20; i++)
  {
    delay(50);
    while (ss.available() > 0)
      if (gps.encode(ss.read()))
      {
        if (gps.location.isValid())
        {
          lat = gps.location.lat();
          Serial.print(lat);
          Serial.print(F(","));
          lng = gps.location.lng();
          Serial.println(lng);
          done = true;
        }
        else
        {
          Serial.print(F("INVALID"));
        }
      }
      else if (millis() > 5000 && gps.charsProcessed() < 10)
      {
        Serial.println(F("No GPS detected: check wiring."));
        while (true)
          ;
      }

    if (done)
    {
      break;
    }
  }

  doc.set(String(position_latitudeLoc), lat);
  doc.set(String(position_logitudeLoc), lng);
}

void recordSpoHeartrate(int jsonArrayCounter)
{
  int32_t averageHeartRate = hrAvg / 20;

  doc.set(String(heartrateLoc) + "[" + String(jsonArrayCounter) + "]/doubleValue", averageHeartRate);
  doc.set(String(oximeterLoc) + "[" + String(jsonArrayCounter) + "]/doubleValue", spo2);

  Serial.print(F(", Avg HR="));
  Serial.print(averageHeartRate, DEC);

  Serial.print(F(", SPO2="));
  Serial.println(spo2, DEC);
}

void recordTemperature(int jsonArrayCounter)
{
  float temperature = mlx.readObjectTempC();

  doc.set(String(temperatureLoc) + "[" + String(jsonArrayCounter) + "]/doubleValue", temperature);

  Serial.print("Body = ");
  Serial.print(temperature);
  Serial.println("*C");
}

void recordMove(int buffer_position, float *pitchavg, float *rollavg)
{
  float pitch, roll;

  pitch = mpu.getPitch();
  *pitchavg = *pitchavg - (pitch_buffer[buffer_position] - pitch) / 6;
  pitch_buffer[buffer_position] = pitch;
  Serial.println(pitch);

  roll = mpu.getRoll();
  *rollavg = *rollavg - (roll_buffer[buffer_position] - roll) / 6;
  roll_buffer[buffer_position] = roll;
  Serial.println(roll);
  // accX = abs(mpu.getAccX());
  // *accXavg = *accXavg - (accX_buffer[buffer_position] - accX) / 6;
  // accX_buffer[buffer_position] = accX;

  // accY = abs(mpu.getAccY());
  // *accYavg = *accYavg - (accY_buffer[buffer_position] - accY) / 6;
  // accY_buffer[buffer_position] = accY;

  // accZ = abs(mpu.getAccZ());
  // *accZavg = *accZavg - (accZ_buffer[buffer_position] - accZ) / 6;
  // accZ_buffer[buffer_position] = accZ;

  accX_buffer[buffer_position] = abs(mpu.getAccX());
  accY_buffer[buffer_position] = abs(mpu.getAccY());
  accZ_buffer[buffer_position] = abs(mpu.getAccZ());

  sensors_event_t event;
  accel.getEvent(&event);

  AdxaccX_buffer[buffer_position] = abs(event.acceleration.x);
  AdxaccY_buffer[buffer_position] = abs(event.acceleration.y);
  AdxaccZ_buffer[buffer_position] = abs(event.acceleration.z);
}

void record_position(int jsonArrayCounter, float pitchavg, float rollavg, float maxMovX, float maxMovY, float maxMovZ, float maxAccX, float maxAccY, float maxAccZ, float standAdxX, float standAdxY, float standAdxZ, float standAccX, float standAccY, float standAccZ, int *state)
{
  // 1 move, 2 stand, 3 lay
  // moving
  if (maxMovY > standAdxY - 5 && (maxMovX > standAdxX + 2))
  { // proper value require
    if (pitchavg > -40 && pitchavg < 40)
    {
      if (maxMovX > standAdxX + 6)
      {
        *state = 3;
        // Serial.println("Running");
      }
      else
      {
        *state = 2;
        // Serial.println("walking");
      }
    }
  }
  else
  {
    if ((pitchavg > -130 && pitchavg < -50) || (rollavg > -35 && rollavg < 35))
    { 
      *state = 5;
      // Serial.println("lay");
    }
    else if ((pitchavg > -40 && pitchavg < 40))
    {
      if (maxMovZ > 6) // adx
      {
        *state = 4;
        // Serial.println("sit");
      }
      else
      {
        *state = 1;
        // Serial.println("stand");
      }
    }
  }


  switch (*state)
  {
  case 1:
    Serial.println("stand");
    break;
  case 2:
    Serial.println("walk");
    break;
  case 3:
    Serial.println("run");
    break;
  case 4:
    Serial.println("sit");
    break;
  case 5:
    Serial.println("lay");
    break;
  default:
    break;
  }

  Serial.println((String)pitchavg + " " + rollavg + " " + maxMovX + " " + maxMovY + " " + maxMovZ + " " + maxAccX + " " + maxAccY + " " + maxAccZ + " " + standAdxX + " " + standAdxY + " " + standAdxZ + " " + standAccX + " " + standAccY + " " + standAccZ);
}

// utility function
void IRAM_ATTR StartStop()
{
  StartStopBtn.state = true;
}

void IRAM_ATTR Next()
{
  NextBtn.state = true;
}

void sensor_sleep()
{
  display.dim(true);
  particleSensor.shutDown();
  mpu.sleep(true);
}

void sensor_wakeup()
{
  display.dim(false);
  particleSensor.wakeUp();
  mpu.sleep(false);
}

bool connectWifi()
{
  if (WiFi.isConnected())
  {
    return true;
  }
  int i = 0;
  WiFi.begin(WIFI_SSID.c_str(), WIFI_PASSWORD.c_str());
  Serial.print("connectiing to wifi");

  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
    i++;
    if (i > 40)
    {
      // displayoled "fail, check <newline> connection
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println(F("Fail, Check"));
      display.setCursor(0, 25);
      display.println(F("connection"));
      display.display();
      delay(500);
      return false;
    }
  }
  return true;
}

void getSSID_PASSWORD()
{
  // displayoled "retrieving <newline> setting"
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("Retrieving"));
  display.setCursor(0, 25);
  display.println(F("setting"));
  display.display();

  storeSetting.begin("rhms-app", false);
  WIFI_SSID = storeSetting.getString("WIFI_SSID", "NULL");
  WIFI_PASSWORD = storeSetting.getString("WIFI_PASSWORD", "NULL");
  UID = storeSetting.getString("UID", "NULL");
  storeSetting.end();

  if (WIFI_SSID.equals("NULL"))
  {
    ConfigureWifi();
  }
}

void recordTimeMillis(bool isstart)
{
  unsigned long Epoch_Time = get_Epoch_Time();
  String time_s = String(Epoch_Time) + "000";

  if (isstart)
  {
    doc.set(String(dateLoc) + "doubleValue", time_s);
    doc.set(String(starttimeLoc) + "doubleValue", time_s);
  }
  else
  {
    doc.set(String(endtimeLoc) + "doubleValue", time_s);
  }
}

unsigned long get_Epoch_Time()
{
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    // Serial.println("Failed to obtain time");
    return (0);
  }
  time(&now);
  return now;
}

void ConfigureWifi()
{

  settingMenuText("WiFi", "Setting");
  delay(300);
  // bluetooth start
  SerialBT.begin(BtDevice);

  settingMenuText("Receiving:", "SSID");
  static uint32_t prev_ms = millis();
  bool blinki = false;
  while (1)
  {
    if (millis() > prev_ms + 500)
    {
      prev_ms = millis();
      if (blinki)
      {
        settingMenuText("Receiving:", "SSID");
        blinki = !blinki;
      }
      else
      {
        settingMenuText("", "");
        blinki = !blinki;
      }
    }

    if (SerialBT.available())
      break;
  }

  storeSetting.begin("rhms-app", false);

  WIFI_SSID = SerialBT.readString();
  WIFI_SSID.remove(WIFI_SSID.length() - 2, 2);
  storeSetting.putString("WIFI_SSID", WIFI_SSID);
  delay(500);

  prev_ms = millis();
  blinki = false;
  while (1)
  {
    if (millis() > prev_ms + 500)
    {
      prev_ms = millis();
      if (blinki)
      {
        settingMenuText("Receiving:", "Password");
        blinki = !blinki;
      }
      else
      {
        settingMenuText("", "");
        blinki = !blinki;
      }
    }

    if (SerialBT.available())
      break;
  }

  WIFI_PASSWORD = SerialBT.readString();
  WIFI_PASSWORD.remove(WIFI_PASSWORD.length() - 2, 2);
  storeSetting.putString("WIFI_PASSWORD", WIFI_PASSWORD);

  storeSetting.end();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Setup");
  display.setCursor(15, 20);
  display.println("complete");
  display.display();

  SerialBT.end();
  delay(500);
}

void ConfigureUid()
{
  settingMenuText("User ID", "Setting");
  delay(300);

  SerialBT.begin(BtDevice);

  static uint32_t prev_ms = millis();
  bool blinki = false;
  while (!SerialBT.available())
  {
    if (millis() > prev_ms + 500)
    {
      prev_ms = millis();
      if (blinki)
      {
        settingMenuText("Receiving:", "User ID");
        blinki = !blinki;
      }
      else
      {
        settingMenuText("", "");
        blinki = !blinki;
      }
    }
  }

  storeSetting.begin("rhms-app", false);

  UID = SerialBT.readString();
  UID.remove(UID.length() - 2, 2);
  storeSetting.putString("UID", UID);

  storeSetting.end();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Setup");
  display.setCursor(0, 20);
  display.println("complete");
  display.display();

  SerialBT.end();
  delay(500);
}

bool uploadActivity()
{
  bool status = connectWifi();

  if (!status)
  {
    wifiSituationalControlLoop();
    Serial.println("connect to wifi fail");
  }

  bool tokenStatus = Firebase.ready();
  while (!tokenStatus)
  {
    initiateFirestore();
    tokenStatus = Firebase.ready();
    delay(1000);
  }
  // displayoled "token <newline> ready"
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("Token"));
  display.setCursor(0, 25);
  display.println(F("Ready"));
  display.display();

  Serial.println("token ready");

  // displayoled " uploading"
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("Uploading"));
  display.display();
  if (Firebase.Firestore.createDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath.c_str(), doc.raw()))
  {
    Serial.println("upload complete");
    doc.clear();
  }
  else
  {
    // displayoled "error <newline> upload"
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("Error"));
    display.setCursor(0, 25);
    display.println(F("Upload"));
    display.display();

    Serial.println(fbdo.errorReason());

    return false;
  }
  return true;
}

void getTimeElapsed(uint32_t *startTimeMillis)
{
  timeElapsed = (millis() - *startTimeMillis);
  int minute = timeElapsed / (60 * 1000);
  int second = (timeElapsed / 1000) % 60;
  display.setTextSize(2);
  display.clearDisplay();
  display.setCursor(0, 15);
  display.print("Duration ");
  display.setCursor(10, 40);
  display.print(minute);
  display.print(" : ");
  display.print(second);
  display.display();
}

// menu list
void batteryMenu()
{
  StartStopBtn.state = false;
  // selection number, use to return selection value
  int current_sel = 1;

  batteryMenuText("Percentage");
  display.setTextSize(2);
  display.print(FuelGauge.getSoC());
  display.print(" %");
  display.display();

  // while not click ok button (StartStop)
  while (!StartStopBtn.state)
  {

    // shift selection when Next button detect change state
    if (NextBtn.state)
    {
      current_sel++;

      // change current selection
      if (current_sel == 4)
      {
        batteryMenuText("Percentage");
        display.print(FuelGauge.getSoC());
        display.print(" %");
        display.display();
        current_sel = 1;
      }
      else if (current_sel == 2)
      {
        batteryMenuText("Remaining");
      }
      else if (current_sel == 3)
      {
        batteryMenuText("Voltage");
        display.print(FuelGauge.getVCell());
        display.print(" V");
        display.display();
      }

      // reset next button state
      NextBtn.state = false;
    }

    delay(10);
  }
  // reset startstop button state after operation is selected
  StartStopBtn.state = false;
  delay(100);
}

int menu()
{
  Serial.println("Menu GO");
  // selection number, use to return selection value
  int current_sel = 1;

  // print for the first time, change display to oled
  mainMenuText("1.", "RTM Rec.", "Menu :");
  // while not click ok button (StartStop)
  while (!StartStopBtn.state)
  {

    // shift selection when Next button detect change state

    if (NextBtn.state)
    {
      Serial.println(NextBtn.state);
      delay(100);
      NextBtn.state = false;
      current_sel++;

      // change current selection
      if (current_sel == 6)
      {
        mainMenuText("1.", "RTM Rec.", "Menu :");
        current_sel = 1;
      }
      else if (current_sel == 2)
      {
        mainMenuText("2.", "View Only", "Menu :");
      }
      else if (current_sel == 3)
      {
        mainMenuText("3.", "Setting", "Menu :");
      }
      else if (current_sel == 4)
      {
        mainMenuText("4.", "Sleep", "Menu :");
      }
      else if (current_sel == 5)
      {
        mainMenuText("5.", "Battery", "Menu :");
      }

      // reset next button state
      NextBtn.state = false;
    }
    delay(100);
    NextBtn.state = false;
    delay(200);
  }
  // reset startstop button state after operation is selected
  StartStopBtn.state = false;
  delay(100);

  // return selection
  return current_sel;
}

int settingMenu()
{
  // selection number, use to return selection value
  int current_sel = 1;

  // print for the first time, change display to oled
  mainMenuText("1.", "WIFI", "Set :");

  // while not click ok button (StartStop)
  while (!StartStopBtn.state)
  {

    // shift selection when Next button detect change state
    if (NextBtn.state)
    {
      current_sel++;

      // change current selection
      if (current_sel == 4)
      {
        mainMenuText("1.", "WiFI", "Set :");
        current_sel = 1;
      }
      else if (current_sel == 2)
      {
        mainMenuText("2.", "Account", "Set :");
      }
      else if (current_sel == 3)
      {
        mainMenuText("3.", "Exit", "Set :");
      }

      // reset next button state
      NextBtn.state = false;
    }

    delay(100);
  }
  // reset startstop button state after operation is selected
  StartStopBtn.state = false;
  delay(100);

  // return selection
  return current_sel;
}

void ConfigOperation()
{
  int setting_select = settingMenu();

  if (setting_select == 1)
  { // WiFi
    ConfigureWifi();

    // operation 2
  }
  else if (setting_select == 2)
  { // Account
    ConfigureUid();
  }
  else if (setting_select == 3)
  {
    exit;
  }
}

// control loop
void uploadSituationalControlLoop()
{
  // selection number, use to return selection value
  int current_sel = 1;

  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print("Upload Op.  Fail!");
  display.display();
  delay(2000);

  // Retry Upload
  // set wifi

  // while not click ok button (StartStop)
  while (!StartStopBtn.state)
  {

    // shift selection when Next button detect change state
    if (NextBtn.state)
    {
      current_sel++;

      if (current_sel == 3)
      {
        mainMenuText("1", "Retry Up.", "select :");
        current_sel = 1;
      }
      else if (current_sel == 2)
      {
        mainMenuText("2", "Remaining", "select :");
      }
      // reset next button state
      NextBtn.state = false;
    }

    delay(10);
  }
  bool status = true;
  switch (current_sel)
  {
  case 1:
    status = uploadActivity();
    break;
  case 2:
    ConfigureWifi();
    status = uploadActivity();
    break;
  default:
    break;
  }

  if (!status)
  {
    uploadSituationalControlLoop();
  }

  // reset startstop button state after operation is selected
  StartStopBtn.state = false;
  delay(100);
}

void wifiSituationalControlLoop()
{
  // selection number, use to return selection value
  int current_sel = 1;

  // displayoled
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print("Conn Wifi     Fail!");
  display.display();
  delay(2000);

  mainMenuText("1", "Reconnect", "Wifi:");
  // while not click ok button (StartStop)
  while (!StartStopBtn.state)
  {

    // shift selection when Next button detect change state
    if (NextBtn.state)
    {
      current_sel++;

      if (current_sel == 3)
      {
        mainMenuText("1", "Reconnect", "Wifi:");
        current_sel = 1;
      }
      else if (current_sel == 2)
      {
        mainMenuText("2", "Setting", "Wifi :");
      }
      // reset next button state
      NextBtn.state = false;
    }

    delay(10);
  }
  Serial.println("outside wifi control loop");
  bool status = true;
  switch (current_sel)
  {
  case 1:
    status = connectWifi();
    break;
  case 2:
    ConfigureWifi();
    status = connectWifi();
    break;
  default:
    break;
  }

  if (!status)
  {
    wifiSituationalControlLoop();
  }

  // reset startstop button state after operation is selected
  StartStopBtn.state = false;
  delay(100);
}

// text display
void mainMenuText(char *num, char *text, char *title)
{
  display.clearDisplay();

  display.setCursor(0, 0);
  display.println(F(title));
  display.setCursor(50, 20);
  display.println(F(num));

  display.setCursor(0, 45);
  display.println(F(text));
  display.display();
}

void settingMenuText(char *text1, char *text2)
{
  display.clearDisplay();

  display.setCursor(5, 0);
  display.setTextSize(1);
  display.println(F("[Setting]"));
  display.setTextSize(2);
  display.setCursor(0, 20);
  display.println(F(text1));

  display.setCursor(0, 45);
  display.println(F(text2));
  display.display();
}

void realTimeText(int c1, int c2, char *text1)
{
  display.setTextSize(2);
  display.setCursor(c1, c2);
  display.print(F(text1));
  display.display();
}

void batteryMenuText(char *text1)
{
  display.clearDisplay();
  // Battery Menu
  display.setCursor(5, 0);
  display.setTextSize(1);
  display.println(F("[Battery]"));
  display.setTextSize(2);

  display.setCursor(0, 20);
  display.println(F(text1));

  display.display();
}

void initialDisp(char *ini)
{
  display.clearDisplay();
  display.setCursor(0, 15);
  display.println(F(ini));
  display.display();
}
