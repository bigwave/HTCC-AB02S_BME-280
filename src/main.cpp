/**
 * This is an example of joining, sending and receiving data via LoRaWAN using a more minimal interface.
 * 
 * The example is configured for OTAA, set your keys into the variables below.
 * 
 * The example will upload a counter value periodically, and will print any downlink messages.
 * 
 * David Brodrick.
 */
#include "LoRaWanMinimal_APP.h"
#include "lora/system/timeServer.h"
#include "Arduino.h"
#include "GPS_Air530.h"
#include <Wire.h>
#include "HT_SSD1306Wire.h"
#include <TimeLib.h>
#include "CubeCell_NeoPixel.h"
#include "CayenneLPP.h"
#include "Seeed_BME280.h"
#include <CircularBuffer.h>
#include "chipIdNameLookup.cpp"
#include "version.h"
#include "font.h"

#define ANSI_ESCAPE_SEQUENCE(c) "\33[" c
#define ESC_RESET ANSI_ESCAPE_SEQUENCE("0m")
#define ESC_BOLD_ON ANSI_ESCAPE_SEQUENCE("1m")
#define ESC_ITALICS_ON ANSI_ESCAPE_SEQUENCE("3m")
#define ESC_UNDERLINE_ON ANSI_ESCAPE_SEQUENCE("4m")
#define ESC_INVERSE_ON ANSI_ESCAPE_SEQUENCE("7m")
#define ESC_STRIKETHROUGH_ON ANSI_ESCAPE_SEQUENCE("9m")
#define ESC_BOLD_OFF ANSI_ESCAPE_SEQUENCE("22m")
#define ESC_ITALICS_OFF ANSI_ESCAPE_SEQUENCE("23m")
#define ESC_UNDERLINE_OFF ANSI_ESCAPE_SEQUENCE("24m")
#define ESC_INVERSE_OFF ANSI_ESCAPE_SEQUENCE("27m")
#define ESC_STRIKETHROUGH_OFF ANSI_ESCAPE_SEQUENCE("29m")
#define ESC_FG_BLACK ANSI_ESCAPE_SEQUENCE("30m")
#define ESC_FG_RED ANSI_ESCAPE_SEQUENCE("31m")
#define ESC_FG_GREEN ANSI_ESCAPE_SEQUENCE("32m")
#define ESC_FG_YELLOW ANSI_ESCAPE_SEQUENCE("33m")
#define ESC_FG_BLUE ANSI_ESCAPE_SEQUENCE("34m")
#define ESC_FG_MAGENTA ANSI_ESCAPE_SEQUENCE("35m")
#define ESC_FG_CYAN ANSI_ESCAPE_SEQUENCE("36m")
#define ESC_FG_WHITE ANSI_ESCAPE_SEQUENCE("37m")
#define ESC_FG_DEFAULT ANSI_ESCAPE_SEQUENCE("39m")
#define ESC_BG_BLACK ANSI_ESCAPE_SEQUENCE("40m")
#define ESC_BG_RED ANSI_ESCAPE_SEQUENCE("41m")
#define ESC_BG_GREEN ANSI_ESCAPE_SEQUENCE("42m")
#define ESC_BG_YELLOW ANSI_ESCAPE_SEQUENCE("43m")
#define ESC_BG_BLUE ANSI_ESCAPE_SEQUENCE("44m")
#define ESC_BG_MAGENTA ANSI_ESCAPE_SEQUENCE("45m")
#define ESC_BG_CYAN ANSI_ESCAPE_SEQUENCE("46m")
#define ESC_BG_WHITE ANSI_ESCAPE_SEQUENCE("47m")
#define ESC_BG_DEFAULT ANSI_ESCAPE_SEQUENCE("49m")
#define ESC_CURSOR_POS(L, C) ANSI_ESCAPE_SEQUENCE(#L ";" #C "H")
#define ESC_CURSOR_UP(L) ANSI_ESCAPE_SEQUENCE(#L "A")
#define ESC_CURSOR_DOWN(L) ANSI_ESCAPE_SEQUENCE(#L "B")
#define ESC_CURSOR_FORWARD(C) ANSI_ESCAPE_SEQUENCE(#C "C")
#define ESC_CURSOR_BACKWARD(C) ANSI_ESCAPE_SEQUENCE(#C "D")
#define ESC_CURSOR_POS_SAVE ANSI_ESCAPE_SEQUENCE("s")
#define ESC_CURSOR_POS_RESTORE ANSI_ESCAPE_SEQUENCE("u")
#define ESC_ERASE_DISPLAY ANSI_ESCAPE_SEQUENCE("2J")
#define ESC_ERASE_LINE ANSI_ESCAPE_SEQUENCE("K")

#define ESC_CURSOR_OFF ANSI_ESCAPE_SEQUENCE("?25l")
#define ESC_CURSOR_ON ANSI_ESCAPE_SEQUENCE("?25h")

#define delta_width 10
#define delta_height 10
static unsigned char delta_bits[] = {
    0x30, 0xfc, 0x78, 0xfc, 0x68, 0xfc, 0xc4, 0xfc, 0xc4, 0xfc, 0x82, 0xfd,
    0x82, 0xfd, 0x01, 0xff, 0x01, 0xff, 0xff, 0xff};

#define SATELLITE_IMAGE_WIDTH 12
#define SATELLITE_IMAGE_HEIGHT 12
const uint8_t SATELLITE_IMAGE[] PROGMEM = {
    0x00, 0xf2, 0x00, 0xf7, 0x90, 0xf3, 0x38, 0xf1, 0x7c, 0xf0, 0xf8, 0xf0,
    0xf0, 0xf1, 0xe4, 0xf0, 0x4e, 0xfa, 0x07, 0xf9, 0x02, 0xf4, 0x00, 0xf3};

CubeCell_NeoPixel pixels(1, RGB, NEO_GRB + NEO_KHZ800);

//SSD1306Wire display(0x3c, 500000, I2C_NUM_0, GEOMETRY_128_64, GPIO10); // addr , freq , i2c group , resolution , rst
SSD1306Wire display(0x3c, 500000, SDA, SCL, GEOMETRY_128_64, GPIO10); // addr , freq , SDA, SCL, resolution , rst
Air530Class Air530;

//when gps waked, if in GPS_UPDATE_TIMEOUT, gps not fixed then into low power mode
#define GPS_UPDATE_TIMEOUT 120000

//once fixed, GPS_CONTINUE_TIME later into low power mode
#define GPS_CONTINUE_TIME 10000
/*
 * set LoraWan_RGB to Active,the RGB active in loraWan
 * RGB red means sending;
 * RGB purple means joined done;
 * RGB blue means RxWindow1;
 * RGB yellow means RxWindow2;
 * RGB green means received done;
 */
#define INT_GPIO USER_KEY
//Set these OTAA parameters to match your app/node in TTN
/* OTAA para*/
uint8_t appEui[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t appKey[] = {0xD2, 0x05, 0x9E, 0xDD, 0x99, 0x16, 0x92, 0xB9, 0x4D, 0x82, 0x62, 0xC1, 0xC9, 0x28, 0xA0, 0xC9};

uint16_t userChannelsMask[6] = {0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
int serialPos = 0;
static uint32_t lastGpsAge = UINT32_MAX;
static time_t lastGpsFixTime = 0;
static TinyGPSLocation lastGpsFix;
static time_t lastLoRaWanAck = 0;
static time_t lastLoRaWanHeartbeat = 0;
static uint8_t lastRssi;
static DeviceClass_t lorawanClass = LORAWAN_CLASS;
typedef struct
{
  time_t time;
  float latitude;
  float longitude;
  float altitude;
  float course;
  float speed;
  float hdop;
  int satellites;
  float temperature = 0;
  float humidity = 0;
  float pressure = 0;
  float batteryVoltage;
  float transmissionLatitude;
  float transmissionLongitude;
  time_t transmissionTime;
} record;

CircularBuffer<record, 50> buffer;

static uint16 redLed = 0;
static uint16 greenLed = 0;
static uint16 blueLed = 0;

uint8_t appDataSize = 0;
//uint8_t appData[LORAWAN_APP_DATA_MAX_SIZE];

enum eDeviceState
{
  DEVICE_STATE_INIT,
  DEVICE_STATE_JOIN,
  DEVICE_STATE_SEND,
  DEVICE_STATE_CYCLE,
  DEVICE_STATE_SLEEP
};

enum eDeviceState deviceState;

void displayVersionAndName()
{
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, chipIdNameLookup());
  display.drawString(0, 18, String("FW ver: ") + VERSION);
  display.setFont(Dialog_plain_8);
  display.drawString(0, 36, BUILD_TIMESTAMP);
  display.display();

  pixels.setPixelColor(0, pixels.Color(1, 0, 0));
  pixels.show(); // Send the updated pixel colors to the hardware.
  delay(500);
  pixels.setPixelColor(0, pixels.Color(0, 1, 0));
  pixels.show(); // Send the updated pixel colors to the hardware.
  delay(500);
  pixels.setPixelColor(0, pixels.Color(0, 0, 1));
  pixels.show(); // Send the updated pixel colors to the hardware.
  delay(500);

  pixels.clear(); // Set all pixel colors to 'off'
  pixels.show();  // Send the updated pixel colors to the hardware.

  display.clear();
}
///////////////////////////////////////////////////
//Some utilities for going into low power mode
TimerEvent_t sleepTimer;
//Records whether our sleep/low power timer expired
bool sleepTimerExpired;
static void wakeUp()
{
  Serial.println();
  Serial.print("Woke Up!!");
  delay(10);
  if (digitalRead(INT_GPIO) == 0)
  {
    Serial.print(" by GPIO.");
  }
  Serial.println();
  Serial.println();
  sleepTimerExpired = true;
  detachInterrupt(INT_GPIO);
}
void printDigits(int digits)
{
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void digitalClockDisplay()
{
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year());
  Serial.println();
}
int32_t fracPart(double val, int n)
{
  int32_t interim = (int32_t)((val - (int32_t)(val)) * pow(10, n));
  if (interim < 0)
  {
    interim = interim * -1;
  }
  return interim;
}
void displayInfo()
{
  Serial.print("Date/Time: ");
  if (Air530.date.isValid())
  {
    Serial.printf("%d/%02d/%02d", Air530.date.year(), Air530.date.day(), Air530.date.month());
  }
  else
  {
    Serial.print("INVALID");
  }

  if (Air530.time.isValid())
  {
    Serial.printf(" %02d:%02d:%02d.%02d", Air530.time.hour(), Air530.time.minute(), Air530.time.second(), Air530.time.centisecond());
  }
  else
  {
    Serial.print(" INVALID");
  }
  Serial.println();

  Serial.print("LAT: ");
  Serial.print(Air530.location.lat(), 6);
  Serial.print(", LON: ");
  Serial.print(Air530.location.lng(), 6);
  Serial.print(", ALT: ");
  Serial.print(Air530.altitude.meters());

  Serial.println();

  Serial.print("SATS: ");
  Serial.print(Air530.satellites.value());
  Serial.print(", HDOP: ");
  Serial.print(Air530.hdop.hdop());
  Serial.print(", AGE: ");
  Serial.print(Air530.location.age());
  Serial.print(", COURSE: ");
  Serial.print(Air530.course.deg());
  Serial.print(", SPEED: ");
  Serial.println(Air530.speed.kmph());
  Serial.println();
}

void displayOled(boolean loraTransmitting)
{
  display.clear();
  char str[30];

  int index = sprintf(str, "%02d-%02d-%02d", year(), day(), month());
  str[index] = 0;
  display.drawString(0, 0, str);
  index = sprintf(str, "%02d:%02d:%02d", hour(), minute(), second());
  str[index] = 0;
  display.drawString(55, 0, str);

  // Serial.print ("Raw voltage : ");
  // Serial.println(getBatteryVoltage());
  double batteryVoltage = getBatteryVoltage() / 1000.0;
  // Serial.print ("double voltage : ");
  // Serial.println(batteryVoltage);
  // Serial.print ("fracPart(batteryVoltage, 1)  : ");
  // Serial.println(fracPart(batteryVoltage, 1));
  index = sprintf(str, "%d.%dV", (int)batteryVoltage, fracPart(batteryVoltage, 1));
  str[index] = 0;
  display.drawString(105, 30, str);

  display.drawString(0, 10, "Last GPS:");
  if (lastGpsFixTime > 0)
  {
    index = sprintf(str, "-%ds", now() - lastGpsFixTime);
    str[index] = 0;
    display.drawString(50, 10, str);
  }

  display.drawString(0, 20, "Last ACK:");
  if (lastGpsFixTime > 0)
  {
    index = sprintf(str, "-%ds", now() - lastLoRaWanAck);
    str[index] = 0;
    display.drawString(50, 20, str);
  }

  display.drawString(0, 30, "Last RSSI:");
  index = sprintf(str, "-%d dBm", lastRssi);
  str[index] = 0;
  display.drawString(50, 30, str);

  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  if (loraTransmitting)
  {
    index = sprintf(str, "#%d Lora", buffer.size());
    str[index] = 0;
  }
  else
  {
    index = sprintf(str, "#%d GPS", buffer.size());
    str[index] = 0;
  }
  display.drawString(128, 10, str);
  if (lastGpsFix.isValid() && !loraTransmitting)
  {
    double distance = TinyGPSPlus::distanceBetween(lastGpsFix.lat(), lastGpsFix.lng(), Air530.location.lat(), Air530.location.lng());
    index = sprintf(str, "%dm", (int)distance);
    str[index] = 0;
    //Serial.println(str);
    display.drawString(116, 20, str);
    display.drawXbm(display.getWidth() - delta_width, 22, delta_width, delta_height, delta_bits);
  }
  display.drawString(display.getWidth() - SATELLITE_IMAGE_WIDTH - 4, 0, String(Air530.satellites.value()));
  display.drawXbm(display.getWidth() - SATELLITE_IMAGE_WIDTH, 0, SATELLITE_IMAGE_WIDTH, SATELLITE_IMAGE_HEIGHT, SATELLITE_IMAGE);
  display.setTextAlignment(TEXT_ALIGN_LEFT);

  index = sprintf(str, "alt: %d.%d", (int)Air530.altitude.meters(), fracPart(Air530.altitude.meters(), 2));
  str[index] = 0;
  display.drawString(70, 42, str);

  index = sprintf(str, "hdop: %d.%d", (int)Air530.hdop.hdop(), fracPart(Air530.hdop.hdop(), 2));
  str[index] = 0;
  display.drawString(70, 54, str);

  index = sprintf(str, "lat : %d.%d", (int)Air530.location.lat(), fracPart(Air530.location.lat(), 4));
  str[index] = 0;
  display.drawString(0, 42, str);

  index = sprintf(str, "lon : %d.%d", (int)Air530.location.lng(), fracPart(Air530.location.lng(), 4));
  str[index] = 0;
  display.drawString(0, 54, str);

  display.display();
}

void VextON(void)
{
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
}
void VextOFF(void) //Vext default OFF
{
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, HIGH);
}
void turnOnDisplay()
{
  VextON(); // oled power on;
  delay(500);
  display.init();
  display.setI2cAutoInit(true);
  display.clear();
  display.display();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setBrightness(64);
  pixels.begin(); // INITIALIZE RGB strip object (REQUIRED)
  pixels.clear(); // Set all pixel colors to 'off'
  pixels.show();  // Send the updated pixel colors to the hardware.
  displayVersionAndName();
  display.setFont(ArialMT_Plain_10);
}

static void lowPowerSleep(uint32_t sleeptime)
{
  attachInterrupt(INT_GPIO, wakeUp, RISING);
  display.clear();
  display.display();
  display.stop();
  pixels.clear();
  pixels.show();
  VextOFF(); //oled power off
  // displayDateTime(false);
  // displayInfo();
  sleepTimerExpired = false;
  TimerInit(&sleepTimer, &wakeUp);
  TimerSetValue(&sleepTimer, sleeptime);
  TimerStart(&sleepTimer);
  //Low power handler also gets interrupted by other timers
  //So wait until our timer had expired
  Serial.println();
  Serial.flush();
  Serial.end();
  while (!sleepTimerExpired)
  {
    //Serial.print("-");
    lowPowerHandler();
  }
  TimerStop(&sleepTimer);
  Serial.begin(115200);

  turnOnDisplay();
}

static CayenneLPP prepareTxFrame(record data)
{
  /*appData size is LoRaWan_APP_Custom_DATA_MAX_SIZE which is defined in "commissioning.h".
    appDataSize max value is LoRaWan_APP_Custom_DATA_MAX_SIZE.
    if enabled AT, don't modify LoRaWan_APP_Custom_DATA_MAX_SIZE, it may cause system hanging or failure.
    if disabled AT, LoRaWan_APP_Custom_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
    for example, if use REGION_CN470,
    the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
  */
  CayenneLPP lpp(LORAWAN_APP_DATA_MAX_SIZE); // or 160?

  lpp.reset();
  lpp.addUnixTime(1, data.time);
  lpp.addGPS(1, data.latitude, data.longitude, data.altitude);
  lpp.addDirection(1, data.course);
  lpp.addDistance(1, data.speed);
  lpp.addAnalogInput(1, data.hdop);
  lpp.addVoltage(1, data.batteryVoltage / 1000.0f);

  lpp.addTemperature(1, data.temperature);
  lpp.addRelativeHumidity(1, data.humidity);
  lpp.addBarometricPressure(1, data.pressure);

  lpp.addGPS(2, data.transmissionLatitude, data.transmissionLongitude, data.altitude);
  lpp.addUnixTime(2, data.transmissionTime);

  Serial.print("lpp data size: ");
  Serial.print(lpp.getSize());
  Serial.println();

  return lpp;
}

///////////////////////////////////////////////////
void join()
{
  turnOnDisplay();
  if (ACTIVE_REGION == LORAMAC_REGION_AU915)
  {
    //TTN uses sub-band 2 in AU915
    LoRaWAN.setSubBand2();
  }

  LoRaWAN.begin(LORAWAN_CLASS, ACTIVE_REGION);

  //Enable ADR
  LoRaWAN.setAdaptiveDR(true);
  display.clear();
  display.display();

  display.drawString(0, 0, "LORAWAN Joining");
  display.display();

  while (1)
  {
    Serial.print("Joining... ");
    LoRaWAN.joinOTAA(appEui, appKey);
    if (!LoRaWAN.isJoined())
    {
      //In this example we just loop until we're joined, but you could
      //also go and start doing other things and try again later
      display.clear();
      display.display();

      display.drawString(0, 0, "LORAWAN JOIN FAILED! Sleeping for 30 seconds...");
      display.display();
      Serial.println("JOIN FAILED! Sleeping for 30 minutes");
      lowPowerSleep(1800000);
    }
    else
    {
      display.clear();
      display.display();

      display.drawString(0, 0, "LORAWAN JOINED...");
      display.display();
      Serial.println("JOINED");
      break;
    }
  }
}

//A 'do nothing' function for timer callbacks
static void wakeUpDummy() {}

void displayRgb()
{

  if (lastLoRaWanAck == 0)
  {
    return;
  }
  if (lorawanClass == CLASS_A) // don't use RGB for tracking, just coverage.
  {
    return;
  }

  time_t test = now();
  uint32_t redLed = (test - lastLoRaWanAck) / 10;
  if (redLed > 255)
  {
    redLed = 255;
  }

  uint32_t greenLed = lastRssi;

  uint32_t blueLed = 0;
  blueLed = buffer.size() * 5;
  pixels.begin(); // INITIALIZE RGB strip object (REQUIRED)

  pixels.setPixelColor(0, pixels.Color(redLed, 0, blueLed));
  //pixels.setPixelColor(0, pixels.Color(redLed, greenLed, blueLed));
  pixels.show(); // Send the updated pixel colors to the hardware.
}

void transmitRecord()
{
  lastLoRaWanHeartbeat = now();
  time_t startTransmitting = now();
  if (buffer.isEmpty())
  {
    return;
  }

  Serial.print("Buffer size = ");
  Serial.println(buffer.size());
  float currentLongitude = buffer.last().longitude;
  float currentLatitude = buffer.last().latitude;
  time_t currentTime = buffer.last().time;

  while (!buffer.isEmpty())
  {
    displayOled(true);
    CayenneLPP lpp = prepareTxFrame(buffer.last());

    if (LoRaWAN.send(lpp.getSize(), lpp.getBuffer(), 2, true))
    {
      Serial.println("Send OK");

      if (lastLoRaWanAck > startTransmitting)
      {
        buffer.pop();
        while (buffer.first().transmissionTime != currentTime)
        {
          record aRecord = buffer.shift();
          aRecord.transmissionLongitude = currentLongitude;
          aRecord.transmissionLatitude = currentLatitude;
          aRecord.transmissionTime = currentTime;
          buffer.push(aRecord);
        }
      }
      else
      {
        // Didn't get an ACK
        break;
      }
    }

    else
    {
      Serial.println("Send FAILED");
      break;
    }
  }

  if (lorawanClass == CLASS_A)
  {
    Serial.println("lowPowerSleep");
    //lowPowerSleep(30000);
    lowPowerSleep(1800000); //30 minutes
  }
}

boolean CheckVoltage()
{
  double batteryVoltage = getBatteryVoltage() / 1000.0;
  Serial.print(ESC_FG_RED);
  Serial.print("\r");
  Serial.printf("%d.%dV ", (int)batteryVoltage, fracPart(batteryVoltage, 2));
  Serial.print(ESC_FG_DEFAULT);

  if (batteryVoltage < 2.85)
  {
    char str[30];
    int index = sprintf(str, "%d.%dV", (int)batteryVoltage, fracPart(batteryVoltage, 2));
    str[index] = 0;
    display.clear();
    display.setFont(ArialMT_Plain_24);
    display.drawString(0, 0, str);
    display.display();

    delay(1000);
    Serial.println("Let the solar panel charge a bit more");

    lowPowerSleep(7200000); // 2 hours
    return false;
  }
  return true;
}
///////////////////////////////////////////////////
void setup()
{
  boardInitMcu();
  Serial.begin(115200);
  pinMode(INT_GPIO, INPUT);

  turnOnDisplay();
  CheckVoltage();
  pinMode(GPIO14, OUTPUT);
  digitalWrite(GPIO14, HIGH);
}

void loop()
{
  if (!CheckVoltage())
  {
    return;
  }
  switch (deviceState)
  {
  case DEVICE_STATE_INIT:
  {
  }
  }
  if (!LoRaWAN.isJoined())
  {
    join();
    delay(1000);
    display.clear();
    display.display();
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "GPS initing...");
    display.display();
  }

  int gpsState = digitalRead(GPIO14);
  // Serial.print("GPS pin state :");
  // Serial.println(gpsState);
  if (gpsState == PINLEVEL::HIGH)
  {

    Air530.begin();
    Air530.setmode(MODE_GPS_GLONASS);
    Air530.setPPS(3, 500);
    display.clear();
    display.display();
    display.drawString(0, 0, "GPS initialised");
    display.display();
    delay(1000);
  }
  displayRgb();

  // if (LoRaWAN.busy())
  // {
  //   displayOled(true);
  //   TimerEvent_t pollStateTimer;
  //   TimerInit(&pollStateTimer, wakeUpDummy);
  //   TimerSetValue(&pollStateTimer, 100);
  //   //Serial.println("LORAWAN BUSY");
  //   TimerStart(&pollStateTimer);
  //   lowPowerHandler();
  //   TimerStop(&pollStateTimer);
  //   Radio.IrqProcess();
  //   return;
  // }
  serialPos++;
  if (serialPos > 50)
  {
    serialPos = 0;
  }
  char str[12];
  sprintf(str, "\33[%dC", serialPos);
  Serial.print(str);

  uint32_t starttime = millis();
  while ((millis() - starttime) < 1000)
  {
    while (Air530.available() > 0)
    {
      Air530.encode(Air530.read());
    }
  }
  if (Air530.time.isValid())
  {
    setTime(Air530.time.hour(),
            Air530.time.minute(),
            Air530.time.second(),
            Air530.date.day(),
            Air530.date.month(),
            Air530.date.year());
    if (lastGpsFixTime == 0)
    { // stop displaying massive negative numbers
      lastGpsFixTime = now();
      lastLoRaWanAck = now();
    }
  }
  displayOled(false);
  if (!Air530.location.isValid())
  {
    Serial.print("i ");
    return;
  }

  if (Air530.hdop.hdop() > 1)
  {
    Serial.print("h ");
    return;
  }
  if (Air530.hdop.hdop() <= 0)
  {
    Serial.print("h ");
    return;
  }
  uint32_t currentAge = Air530.location.age();

  if (currentAge > 1500)
  {
    // GPS doesn't have a new 'fix'
    Serial.print("a");
    Serial.print(currentAge);
    Serial.print(" ");
    return;
  }

  lastGpsFixTime = now();

  if (lastGpsFix.isValid() &&
      Air530.location.isValid() &&
      TinyGPSPlus::distanceBetween(lastGpsFix.lat(), lastGpsFix.lng(), Air530.location.lat(), Air530.location.lng()) < 5)
  {
    // Serial.println("Not moved much");
    // Serial.printf("Last ACK: %02d:%02d:%02d", hour(lastLoRaWanAck), minute(lastLoRaWanAck), second(lastLoRaWanAck));
    // Serial.println();
    // Serial.printf("Now     : %02d:%02d:%02d", hour(), minute(), second());
    // Serial.println();

    time_t diff = now() - lastLoRaWanHeartbeat;
    // Serial.printf("Diff    : %02d:%02d:%02d", hour(diff), minute(diff), second(diff));
    // Serial.println();

    if (minute(diff) < 30 && lastLoRaWanHeartbeat != 0)
    {
      // Serial.println("Not moved much and have Ack'd recently, skip");
      Serial.print(". ");
      return;
    }
    else
    {
      lastLoRaWanHeartbeat = now();
      Serial.println();
      Serial.println();
      Serial.println("30 minutes since last ACK");
    }
  }
  else
  {
    Serial.println();
    Serial.println();
    Serial.println("Moved more than 5 meters");
  }
  displayInfo();

  record newData;
  newData.time = now();
  newData.latitude = Air530.location.lat();
  newData.longitude = Air530.location.lng();
  newData.altitude = Air530.altitude.meters();
  newData.course = Air530.course.deg();
  newData.speed = Air530.speed.kmph();
  newData.satellites = Air530.satellites.value();
  newData.hdop = Air530.hdop.hdop();
  lastGpsFix = Air530.location;

  Air530.end();

  newData.batteryVoltage = getBatteryVoltage();

  newData.transmissionLatitude = newData.latitude;
  newData.transmissionLongitude = newData.longitude;
  newData.transmissionTime = newData.time;

  Wire.beginTransmission(0x76);
  if (Wire.endTransmission() == 0)
  {

    BME280 bme280;
    Wire.begin();
    if (!bme280.init())
    {
      Serial.println("Device error!");
    }
    else
    {
      delay(100); // To let Sensor settle

      newData.temperature = bme280.getTemperature();
      newData.humidity = bme280.getHumidity();
      newData.pressure = bme280.getPressure() / 100;
    }
    Wire.end();
  }
  VextOFF();
  Serial.print("Temperature: ");
  Serial.print(newData.temperature);
  Serial.print(", Humidity: ");
  Serial.print(newData.humidity);
  Serial.print(" and Pressure: ");
  Serial.print(newData.pressure);
  Serial.println();

  buffer.push(newData);
  // newData.time = newData.time - 10000;
  // buffer.push(newData);

  transmitRecord();
}

///////////////////////////////////////////////////
//Example of handling downlink data
void downLinkDataHandle(McpsIndication_t *mcpsIndication)
{
  Serial.printf("Received downlink: %s, RXSIZE %d, PORT %d, DATA: ", mcpsIndication->RxSlot ? "RXWIN2" : "RXWIN1", mcpsIndication->BufferSize, mcpsIndication->Port);
  for (uint8_t i = 0; i < mcpsIndication->BufferSize; i++)
  {
    Serial.printf("%02X", mcpsIndication->Buffer[i]);
  }
  Serial.println();
}

/*
http://community.heltec.cn/t/solved-cubecell-ab01-and-lorawan-frame-counter-is-not-incrementing-after-few-days-of-running-time/2979/39?u=bigwave
.platformio\packages\framework-arduinoasrmicro650x\libraries\LoRaWanMinimal\src\LoRaWanMinimal_APP.cpp

extern void myLoRaWanFCNCheck(bool ackReceived, uint8_t rssi);

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] mcpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 *
static void McpsIndication( McpsIndication_t *mcpsIndication )
{
	if( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
	{
		return;
	}
	printf( "Downlink/ACK received: rssi = %d, snr = %d, datarate = %d\r\n", mcpsIndication->Rssi, (int)mcpsIndication->Snr,(int)mcpsIndication->RxDatarate);
  myLoRaWanFCNCheck( mcpsIndication->AckReceived, mcpsIndication->Rssi);
*/
void myLoRaWanFCNCheck(bool ackReceived, uint8_t rssi)
{
  Serial.println("ACK RECEIVED");
  if (ackReceived)
  {
    lastLoRaWanAck = now();
    lastRssi = rssi;
  }
}
