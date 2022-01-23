#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <SPI.h>
#include <Preferences.h>
#include "BluetoothSerial.h"
#include <Adafruit_GFX.h>
#include "Adafruit_Sensor.h"
#include "module.cpp"

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

#define API_KEY "AIzaSyDGj1Fouh114V3w65thJrYYNigxP1KjbMQ"
#define FIREBASE_PROJECT_ID "isdp-testdb"
#define USER_EMAIL "isdp_esp_default_acc@isdp6.dont.change"
#define USER_PASSWORD "!adUINsa*(76jka^k12"

// #define UID "Vi4wQZdhw8PzYKZXK1ffQH1GJc43"

#define BtDevice "How'rU_HRMS"
#define MAX17043_ADDRESS 0x36
#define SDA2 21
#define SCL2 22
#define SDA1 18
#define SCL1 19

String WIFI_SSID = "";
String WIFI_PASSWORD = "";
String UID = "";

Button StartStopBtn = {18, false};
Button NextBtn = {19, false};

TwoWire I2C2 = TwoWire(1);
TwoWire I2C1 = TwoWire(0);

uint32_t irBuffer[100];
uint32_t redBuffer[100];

int32_t bufferLength;
int32_t spo2;
int8_t validSPO2;
int32_t heartRate;
int32_t heartRateBuffer[20];
int8_t validHeartRate;

int32_t hrAvg = 0;

// function prototype
void IRAM_ATTR Next();
void IRAM_ATTR StartStop();
void configureMenuButton();
void configureOled();
void configureFuelGauge();
int menu();
int settingMenu();
void ConfigureWifi();
void ConfigureUid();
void ConfigOperation();
void mainMenuText(char *num, char *text);
void settingMenuText(char *text1, char *text2);
void batteryMenu();
void BatteryMenuText(char *text1);
void setupI2c();
void setupMax30102();
void setupMpu();
void recordMpu();
void recordSpoHeartrate();
void recordTemperature();
void warmUpMax(int32_t length);
void warmUpMpu();
void setupMlx();
void runOperation(int *program_selection);
void startActivity();
void initiateFirestore();
void connectWifi();
void getSSID_PASSWORD();
void uploadActivity(char* documentPath);

void setup()
{

  Serial.begin(115200);
  Wire.begin();

  configureOled();

  configureFuelGauge();

  configureMenuButton();

  // lightsleep wakeup button 33 at high awake
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 1);

  setupI2c();

  setupMax30102();

  setupMpu();

  setupMlx();

  getSSID_PASSWORD();
  initiateFirestore();

  //display startup image

}

void loop()
{
  int count = 0;

  int program_selection = 0;

  program_selection = menu();

  runOperation(program_selection);

  delay(1000);
}

//####################################################################################################################//
// menu selection display
int menu()
{
  // selection number, use to return selection value
  int current_sel = 1;

  // print for the first time, change display to oled
  mainMenuText("1.", "Activity");

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
        mainMenuText("1.", "Activity");
        current_sel = 1;
      }
      else if (current_sel == 2)
      {
        mainMenuText("2.", "Real Time");
      }
      else if (current_sel == 3)
      {
        mainMenuText("3.", "Setting");
      }
      else if (current_sel == 4)
      {
        mainMenuText("4.", "Sleep");
      }
      else if (current_sel == 5)
      {
        mainMenuText("5.", "Battery");
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
  mainMenuText("1.", "WIFI");

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
        mainMenuText("1.", "WiFI");
        current_sel = 1;
      }
      else if (current_sel == 2)
      {
        mainMenuText("2.", "Account");
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
  while (!SerialBT.available() > 0)
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
  }

  storeSetting.begin("my-app", false);

  WIFI_SSID = SerialBT.readString();
  WIFI_SSID.remove(WIFI_SSID.length() - 2, 2);
  storeSetting.putString("WIFI_SSID", WIFI_SSID);
  delay(500);

  prev_ms = millis();
  blinki = false;
  while (!SerialBT.available() > 0)
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
  }
  WIFI_PASSWORD = SerialBT.readString();
  WIFI_PASSWORD.remove(WIFI_PASSWORD.length() - 2, 2);
  storeSetting.putString("WIFI_PASSWORD", WIFI_PASSWORD);

  storeSetting.end();
  delay(500);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Setup");
  display.setCursor(15, 20);
  display.println("complete");
  display.display();

  delay(500);
  // display.clearDisplay();
  SerialBT.end();
}

void ConfigureUid()
{
  settingMenuText("User ID", "Setting");
  delay(300);

  SerialBT.begin(BtDevice); // Bluetooth device name
  static uint32_t prev_ms = millis();
  bool blinki = false;
  while (!SerialBT.available() > 0)
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

  storeSetting.begin("my-app", false);

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

  delay(500);
  SerialBT.end();
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

void mainMenuText(char *num, char *text)
{
  display.clearDisplay();

  // Menu
  display.setCursor(0, 0);
  display.println(F("Menu :"));
  display.setCursor(50, 20);
  display.println(F(num));

  display.setCursor(0, 45);
  display.println(F(text));
  display.display();
}

void settingMenuText(char *text1, char *text2)
{
  display.clearDisplay();

  // Menu
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

  BatteryMenuText("Percentage");
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

      //      display.setTextSize(2);
      //      display.setCursor(0, 55);

      // change current selection
      if (current_sel == 4)
      {
        BatteryMenuText("Percentage");
        display.print(FuelGauge.getSoC());
        display.print(" %");
        display.display();
        current_sel = 1;
      }
      else if (current_sel == 2)
      {
        BatteryMenuText("Remaining");
      }
      else if (current_sel == 3)
      {
        BatteryMenuText("Voltage");
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

void BatteryMenuText(char *text1)
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

  display.setTextSize(2, 2);
  display.setTextColor(SSD1306_WHITE);
  display.display();
  display.clearDisplay();
}

void configureFuelGauge()
{
  FuelGauge.reset();
  FuelGauge.quickStart();
}

void setupI2c()
{
  I2C1.begin(SDA1, SCL1, 400000);
  I2C2.begin(SDA2, SCL2, 100000);
}

void setupMax30102()
{

  while (!particleSensor.begin(I2C1, I2C_SPEED_FAST, 0x57))
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

  while (!mpu.setup(0x68, setting, I2C2))
  { // change to your own address
    Serial.println("MPU connection failed. ");
    delay(1000);
  }

  mpu.verbose(true);
  delay(1);
  mpu.calibrateAccelGyro();

  mpu.calibrateMag();

  mpu.setMagneticDeclination(-0.14);
  mpu.setFilterIterations(10);
  mpu.verbose(false);
}

void setupMlx()
{
  while (!mlx.begin(0x5A, &I2C2))
  {
    Serial.println("Could not find a valid MLX9016 sensor, check wiring!");
    delay(1000);
  }
}

void recordSpoHeartrate()
{

  // if (count > 3)
  // {
  //   count = 0;
  //   hrAvg = 0;
  // }
  // if (heartRate > 0)
  // {
  //   hrAvg = hrAvg + heartRate - 10;
  //   count++;
  // }

  Serial.print(F(", HR="));
  Serial.print(heartRate - 10, DEC);

  Serial.print(F(", Avg HR="));
  Serial.print(hrAvg / 20, DEC);

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

void recordTemperature()
{
  Serial.print("Body = ");
  Serial.print(mlx.readObjectTempC());

  Serial.println("*C");
  Serial.println();
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
  connectWifi();

  config.api_key = API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
}

void connectWifi()
{
  int i = 0;
  WiFi.begin(WIFI_SSID.c_str(), WIFI_PASSWORD.c_str());
  Serial.print("connectiing to wifi");

  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
    i++;
    if(i>60){
      //display fail connect, check connection
      break;
    }
  }
}

void getSSID_PASSWORD()
{
  storeSetting.begin("rhms-app", false);
  WIFI_SSID = storeSetting.getString("WIFI_SSID", "NULL");
  WIFI_PASSWORD = storeSetting.getString("WIFI_PASSWORD", "NULL");
  storeSetting.end();

  if (WIFI_SSID.equals("NULL")){
    ConfigureWifi();
  }
}

void uploadActivity(char* documentPath){
    if (Firebase.Firestore.createDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath.c_str(), doc.raw()))
    Serial.printf("ok\n%s\n\n", fbdo.payload().c_str());
  else
    Serial.println(fbdo.errorReason());
}

void runOperation(int program_selection)
{
  if (program_selection == 1)
  {
    startActivity();
  }
  else if (program_selection == 2)
  {
    Serial.printf("second choice\n");
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

void startActivity()
{
  bufferLength = 90;
  bool newActivity = true;
  int count = 0;
  int jsonArrayCounter =0;

  warmUpMpu();

  static uint32_t prev_ms = millis();

  warmUpMax(bufferLength);

  while (count < 1800)
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
      if (millis() > prev_ms + 1000)
      {
        recordMpu();
        recordSpoHeartrate();
        recordTemperature();

        prev_ms = millis();
      }
    }
    hrAvg = 0;

    if (StartStopBtn.state)
    {
      StartStopBtn.state = false;
      break;
    }
  }
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