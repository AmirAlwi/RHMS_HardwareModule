#include "module.cpp"
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <SPI.h>
#include "time.h"
#include <Preferences.h>
#include "BluetoothSerial.h"
#include <Adafruit_GFX.h>
#include "Adafruit_Sensor.h"
#include <SoftwareSerial.h>

#include <TinyGPSPlus.h> //gps

#include <Adafruit_SSD1306.h> //oled

#include "Adafruit_MLX90614.h" //temperature

#include "MAX30105.h" //max30102
#include "heartRate.h"
#include "spo2_algorithm.h"

#include "MPU9250.h" //9-axis sensor

#include "MAX17043.h" //fuel gauge

#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

struct Button
{
  const uint8_t PIN;
  bool state;
};

#define API_KEY "AIzaSyDGj1Fouh114V3w65thJrYYNigxP1KjbMQ"
#define FIREBASE_PROJECT_ID "isdp-testdb"
#define USER_EMAIL "isdp_esp_default_acc@isdp6.dont.change"
#define USER_PASSWORD "!adUINsa*(76jka^k12"

#define BtDevice "How'rU_HRMS"
#define SDA2 21
#define SCL2 22
#define SDA1 18
#define SCL1 19

String WIFI_SSID = "";
String WIFI_PASSWORD = "";
String UID = "";
String documentPath = "demo";

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

// initiate sensor & module
BluetoothSerial SerialBT;
Preferences storeSetting;
MAX17043 FuelGauge;
Adafruit_SSD1306 display(128, 64, &Wire, -1);
MAX30105 particleSensor;
MPU9250 mpu;
Adafruit_MLX90614 mlx;
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
FirebaseJson doc;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

// function prototype
void IRAM_ATTR Next();
void IRAM_ATTR StartStop();
void batteryMenu();
void batteryMenuText(char *text1);
void configureMenuButton();
void configureOled();
void configureFuelGauge();
void ConfigureWifi();
void ConfigOperation();
void ConfigureUid();
bool connectWifi();
void getSSID_PASSWORD();
void initiateFirestore();
int menu();
void mainMenuText(char *num, char *text, char *title);
void recordMpu();
void recordSpoHeartrate(int jsonArrayCounter);
void recordTemperature(int jsonArrayCounter);
int settingMenu();
void selectOperation(int program_selection);
void settingMenuText(char *text1, char *text2);
void setupMax30102();
void setupMpu();
void setupMlx();
void startActivity(uint32_t *startTimeMillis);
bool uploadActivity();
void uploadSituationalControlLoop();
void warmUpMpu();
void warmUpMax(int32_t length);
void wifiSituationalControlLoop();
void OxyBpm();
void realTimeDisplay();
void realTimeText(int c1, int c2, char *text1);
void initialDisp(char *ini);
void warmUpGPS();
void setupGPS();
unsigned long get_Epoch_Time();
void getTimeElapsed(uint32_t *startTimeMillis);
void recordTimeMillis(bool isstart);
void recordGPS();
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

  delay(1000);
}

//####################################################################################################################//
// menu selection display
void initialDisp(char *ini)
{
  display.clearDisplay();
  display.setCursor(0, 15);
  display.println(F(ini));
  display.display();
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
  }
  // reset startstop button state after operation is selected
  StartStopBtn.state = false;
  Serial.println();
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
      if (current_sel == 3)
      {
        mainMenuText("1.", "WiFI", "Set :");
        current_sel = 1;
      }
      else if (current_sel == 2)
      {
        mainMenuText("2.", "Account", "Set :");
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
}

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

void batteryMenu()
{
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

void configureMenuButton()
{
  // Set gpio
  pinMode(StartStopBtn.PIN, INPUT_PULLDOWN);
  pinMode(NextBtn.PIN, INPUT_PULLDOWN);

  // attach pin to intterupt function
  attachInterrupt(StartStopBtn.PIN, StartStop, FALLING);
  attachInterrupt(NextBtn.PIN, Next, FALLING);
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
      else

          if (millis() > 5000 && gps.charsProcessed() < 10)
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
  delay(1);
  mpu.calibrateAccelGyro();

  // mpu.calibrateMag();

  mpu.setMagneticDeclination(-0.14);
  mpu.setFilterIterations(10);
  mpu.verbose(false);
}

void setupMlx()
{
  while (!mlx.begin(0x5A))
  {
    Serial.println("Could not find a valid MLX9016 sensor, check wiring!");
    delay(1000);
  }
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

void recordMpu()
{
  Serial.println("Yaw, Pitch, Roll: added to doc");
  Serial.print(mpu.getYaw(), 1);
  Serial.print(", ");
  Serial.print(mpu.getPitch(), 1);
  Serial.print(", ");
  Serial.println(mpu.getRoll(), 1);
}

void recordTemperature(int jsonArrayCounter)
{
  float temperature = mlx.readObjectTempC();

  doc.set(String(temperatureLoc) + "[" + String(jsonArrayCounter) + "]/doubleValue", temperature);

  Serial.print("Body = ");
  Serial.print(temperature);
  Serial.println("*C");
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

void recordTimeMillis(bool isstart){
  unsigned long Epoch_Time = get_Epoch_Time();
  String time_s = String(Epoch_Time) + "000";

  if(isstart){
    doc.set(String(dateLoc) + "doubleValue", time_s);
    doc.set(String(starttimeLoc) + "doubleValue", time_s);
  } else{
     doc.set(String(endtimeLoc) + "doubleValue", time_s);
  }

}

void warmUpMpu()
{

  static uint32_t prev_ms = millis();

  int x = 0;
  while (x < 10)
  {
    if (mpu.update())
    {
      if (millis() > prev_ms + 100)
      {
        log_d("recorded value %d and %d", mpu.getEulerX(), mpu.getYaw());
        prev_ms = millis();
        x++;
      }
    }
  }
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

void selectOperation(int program_selection)
{
  if (program_selection == 1)
  {
    uint32_t startTimeMillis;
    while (1)
    {
      startActivity(&startTimeMillis);

      recordGPS();
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
  }
  else if (program_selection == 4)
  {
    delay(1000);
    esp_light_sleep_start();
  }
  else if (program_selection == 5)
  {
    batteryMenu();
  }
}

void startActivity(uint32_t *startTimeMillis)
{
  bufferLength = 90;
  bool newActivity = true;
  int jsonArrayCounter = 0;

  warmUpMpu();

  static uint32_t prev_ms = millis();

  warmUpMax(bufferLength);

  *startTimeMillis = millis();
  
  recordTimeMillis(true);
  doc.set(String(titleLoc) + "stringValue", "Activity");
  doc.set(String(uidLoc) + "stringValue", UID);
  doc.set(String(notesLoc) + "stringValue", "routine monitoring");

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
      if (millis() > prev_ms + 940)
      {
        prev_ms = millis();
        recordMpu();
        recordSpoHeartrate(jsonArrayCounter);
        recordTemperature(jsonArrayCounter);
        getTimeElapsed(startTimeMillis);

        jsonArrayCounter++;
      }
    }
    hrAvg = 0;
    if (StartStopBtn.state)
    {
      recordTimeMillis(false);
      exitLoop = true;
      StartStopBtn.state = false;
      break;
    }
  }
  recordTimeMillis(false);
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
      break;
    }
  }
}

void realTimeDisplay()
{
  display.clearDisplay();
  realTimeText(0, 0, "Real Time Data");
  while (1)
  {
    display.clearDisplay();
    OxyBpm();
  }
}

void realTimeText(int c1, int c2, char *text1)
{
  display.setTextSize(2);
  display.setCursor(c1, c2);
  display.print(F(text1));
  display.display();
}

void recordGPS(){
   if (gps.location.isValid())
  {
    float lat = (gps.location.lat(),6);
    Serial.print(lat, 6);
    Serial.print(F(","));
    float lng = (gps.location.lng(),6);
    Serial.println(lng, 6);
    doc.set(String(position_latitudeLoc) ,lat);
    doc.set(String(position_logitudeLoc), lng);
  }
  else
  {
    Serial.print(F("INVALID"));
  }
}

void warmUpGPS()
{
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    float lat = (gps.location.lat(),6);
    Serial.print(lat, 6);
    Serial.print(F(","));
    float lng = (gps.location.lng(),6);
    Serial.print(lng, 6);
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
// interrupt call funcion
void IRAM_ATTR StartStop()
{
  StartStopBtn.state = true;
}

void IRAM_ATTR Next()
{
  NextBtn.state = true;
}