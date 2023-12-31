#include <Arduino.h>

#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <DFRobot_MICS.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <DFRobot_ENS160.h>
#include <SPI.h>
#include "Plantower_PMS7003.h"
#include "PMS.h"
#include <AHT10.h>

#include <Adafruit_Sensor.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//*********************************************** TEMT6000 ***********************************************************
int temt6000Pin = A1; // define the temt6000Pin

//*********************************************** OLED ***********************************************************
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

//*********************************************** Wifi and MQTT ***********************************************************
// WiFi
const char *ssid = "DIGIFIBRA-cF5T";
const char *password = "P92sKt3FGfsy";

// MQTT Broker
const char *mqtt_broker = "192.168.1.43";
const char *topic = "emqx/weatherstation";
const char *mqtt_username = "root";
const char *mqtt_password = "orangepi.hass";
const int mqtt_port = 1883;

String client_id = "weatherstation-client-";

// *********************************************************************************************************;

WiFiClient espClient;
PubSubClient client(espClient);

void callback(char *topic, byte *payload, unsigned int length);
void reconnect();

// How many internal neopixels do we have? some boards have more than one!
#define NUMPIXELS 1
#define PIN_NEOPIXEL 7
Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

//*********************************************** AHT10 ***********************************************************
AHT10 myAHT10(AHT10_ADDRESS_0X39);
//*********************************************** AHT20 ***********************************************************
Adafruit_AHTX0 aht;
// *********************************************** BME280 ***********************************************************

Adafruit_BMP280 bmp;

//*********************************************** PMS***********************************************************
HardwareSerial SerialPMS(1);
PMS pms(SerialPMS);
PMS::DATA data;

#define RXD0 20
#define TXD0 21

char output[256];
Plantower_PMS7003 pms7003 = Plantower_PMS7003();

//*********************************************  MICS *************************************************************

// MiCS5524
#define CALIBRATION_TIME 3 // Default calibration time is three minutes
#define ADC_PIN A0
#define POWER_PIN 6
DFRobot_MICS_ADC mics(/*adcPin*/ ADC_PIN, /*powerPin*/ POWER_PIN);

//*************************************************  ENS160  *******************************************************
// ENS160
uint8_t csPin = 5;
DFRobot_ENS160_SPI ENS160(&SPI, csPin);
void displayAHT10(float newTempAHT10, float newHumAHT10);
void displayAHT20(float newTempAHT20, float newHumAHT20);
void displayBMP280(float newTempBMP, float newPres, float newBar, float newAlt);
void displayENS160(uint8_t Status, uint8_t AQI, uint16_t TVOC, uint16_t ECO2);
void displayPMS7003(float PM1_0, float PM2_5, float PM10_0, float PM1_0_atmos, float PM2_5_atmos, float PM10_0_atmos);
void displayPMS7003Greater(float RawGreaterThan_0_5, float RawGreaterThan_1_0, float RawGreaterThan_2_5, float RawGreaterThan_5_0, float RawGreaterThan_10_0);
void displayMQTTerror(int mqttState);
void displayLUZ(float light, int light_value, float lux);

void loop2(void *pcParameters);

void setupAHT10()
{
  // AHT10
  while (myAHT10.begin() != true)
  {
    Serial.println(F("AHT10 not connected or fail to load calibration coefficient")); //(F()) save string to flash & keeps dynamic memory free
    delay(5000);
  }
  Serial.println(F("AHT10 OK"));
}
void setupAHT20()
{
  // AHT20
  if (aht.begin())
  {
    Serial.println("Found AHT20");
  }
  else
  {
    Serial.println("Didn't find AHT20");
  }
}
void setupBMP280()
{
  // BMP280
  if (!bmp.begin())
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1)
      ;
  }
  else
  {
    Serial.println(F("BMP280 OK"));
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Modo de operación */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Presion oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtrado. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Tiempo Standby. */
}
void setupPMS7003()
{
  // PMS7003
  SerialPMS.begin(9600, SERIAL_8N1, RXD0, TXD0);
  pms7003.init(&SerialPMS);
}
void setupENS160()
{ // CO2 ENS160
  //  Init the sensor
  ENS160.begin();
  Serial.println("Begin ok!");
  /**
   * Set power mode
   * mode Configurable power mode:
   *   ENS160_SLEEP_MODE: DEEP SLEEP mode (low power standby)
   *   ENS160_IDLE_MODE: IDLE mode (low-power)
   *   ENS160_STANDARD_MODE: STANDARD Gas Sensing Modes
   */
  ENS160.setPWRMode(ENS160_STANDARD_MODE);

  /**
   * Users write ambient temperature and relative humidity into ENS160 for calibration and compensation of the measured gas data.
   * ambientTemp Compensate the current ambient temperature, float type, unit: C
   * relativeHumidity Compensate the current ambient temperature, float type, unit: %rH
   */
  ENS160.setTempAndHum(/*temperature=*/25.0, /*humidity=*/50.0);
}
void setupWifi()
{
  //**********************************************************************************************************
  WiFi.mode(WIFI_STA); // Optional
  WiFi.begin(ssid, password);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  Serial.println("\nConnecting");
  // adjust the brightness of the led
  pixels.setBrightness(10);

  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(100);
    // set the color of the led to red
    pixels.setPixelColor(0, pixels.Color(0, 255, 0));
    pixels.show();
    delay(100);
    // set the color of the led to off
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.show();
    delay(100);
  }
}
void setupMICS()
{
  //**********************************************************************************************************
  // MiCS5524
  while (!mics.begin())
  {
    Serial.println("NO Deivces !");
    delay(1000);
  }
  Serial.println("Device connected successfully !");

  /**!
    Gets the power mode of the sensor
    The sensor is in sleep mode when power is on,so it needs to wake up the sensor.
    The data obtained in sleep mode is wrong
   */
  uint8_t mode = mics.getPowerState();
  Serial.print(mode);
  if (mode == SLEEP_MODE)
  {
    mics.wakeUpMode();
    Serial.println("wake up sensor success!");
  }
  else
  {
    Serial.println("The sensor is wake up mode");
  }
  int a = 0;
  Serial.println("Please wait until the warm-up time is over!");
  while (!mics.warmUpTime(CALIBRATION_TIME))
  {
    a = a + 1;
    Serial.print("Warm-up time:");
    Serial.print(a);
    Serial.println("s");
    delay(1000);
  }
}

// variable para mostrar en pantalla
float lastTempAHT10 = 0;
float lastHumAHT10 = 0;
float lastTempAHT20 = 0;
float lastHumAHT20 = 0;
float lastTempBMP = 0;
float lastPres = 0;
float lastBar = 0;
float lastAlt = 0;
float lastStatus = 0;
float lastAQI = 0;
float lastTVOC = 0;
float lastECO2 = 0;

float lastPM1_0 = 0;
float lastPM2_5 = 0;
float lastPM10_0 = 0;
float lastPM1_0_atmos = 0;
float lastPM2_5_atmos = 0;
float lastPM10_0_atmos = 0;
float lastRawGreaterThan_0_5 = 0;
float lastRawGreaterThan_1_0 = 0;
float lastRawGreaterThan_2_5 = 0;
float lastRawGreaterThan_5_0 = 0;
float lastRawGreaterThan_10_0 = 0;
float light = 0;

// Temt6000 light sensor
int light_value = 0;
float volts = 0;
float amps = 0;
float microamps = 0;
float lux = 0;

// AHT10
float newTempAHT10 = 0;
float newHumAHT10 = 0;

// AHT20
sensors_event_t humidity, temp;

float newTempAHT20 = 0;
float newHumHT20 = 0;

// BMP280
float newTempBMP = 0;
float newPres = 0;
float newBar = 0;
float newAlt = 0;

// ENS160
uint8_t Status = 0;
uint8_t AQI = 0;
uint16_t TVOC = 0;
uint16_t ECO2 = 0;

float PM1_0 = 0;
float PM2_5 = 0;
float PM10_0 = 0;
float PM1_0_atmos = 0;
float PM2_5_atmos = 0;
float PM10_0_atmos = 0;
float RawGreaterThan_0_5 = 0;
float RawGreaterThan_1_0 = 0;
float RawGreaterThan_2_5 = 0;
float RawGreaterThan_5_0 = 0;
float RawGreaterThan_10_0 = 0;

//*************************************************  SETUP  *******************************************************
void setup()
{
  pinMode(temt6000Pin, INPUT); // use a input pin to read the data
  pixels.setBrightness(5);

  // Set software serial baud to 115200;
  Serial.begin(115200);
  // Connecting to a WiFi network
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }
  delay(2000);
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 5);
  // Display static text
  display.println("Weather Station");
  display.display();

  // WiFi
  // WiFi.mode(WIFI_STA); // Optional
  WiFi.begin(ssid, password);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  Serial.println("\nConnecting");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println("Connecting to WiFi..");
    // show in display oled
    display.setCursor(0, 20);
    display.println("Connecting to WiFi..");
    display.display();
    // create pixel blink in red color
    pixels.setPixelColor(0, pixels.Color(0, 255, 0));
    pixels.show();
    delay(500);
    // set the color of the led to off
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.show();
  }
  Serial.println("Connected to the Wi-Fi network");
  // connecting to a mqtt broker
  client.setKeepAlive(20860);
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);

  while (!client.connected())
  {
    // String client_id = "weatherstation-client-";
    client_id = client_id + String(WiFi.macAddress());
    Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password))
    {
      Serial.println("Public EMQX MQTT broker connected");
    }
    else
    {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
  // Publish and subscribe
  client.publish(topic, "Hi, I'm WEATHERSTATION ^^");

  Serial.println("TEMT6000: OK");

  setupAHT10();
  setupAHT20();
  setupBMP280();
  setupENS160();
  setupPMS7003();

  // client.subscribe(topic);
  display.clearDisplay();
  display.setCursor(0, 5);
  // Display static text
  display.println("Weather Station");
  display.display();
  display.setCursor(0, 20);
  // Display static text
  display.println("IP: " + WiFi.localIP().toString());
  // display broker mqtt information
  display.setCursor(0, 30);
  display.println("MQTT: " + String(mqtt_broker));
  display.setCursor(0, 40);

  // display mqtt status
  if (client.connected())
  {
    display.println("MQTT: Connected");
  }
  else
  {
    display.println("MQTT: Disconnected");
  }
  display.display();
  delay(5000);

  xTaskCreatePinnedToCore(
      loop2,  /* Function to implement the task */
      "loop", /* Name of the task */
      1000,   /* Stack size in words */
      NULL,   /* Task input parameter */
      0,      /* Priority of the task */
      NULL,   /* Task handle. */
      1);     /* Core where the task should run */
}

// create state machine to show the function to display in oled
enum State
{
  AHT10s,
  AHT20,
  BMP280,
  ENS160state,
  PMS7003,
  PMS7003G,
  TEMT6000,
};

State currentState = AHT10s;
unsigned long lastDisplayTime = 0;
const unsigned long displayInterval = 2000; // Intervalo de 5 segundos

void loop2(void *pcParameters)
{
  while (true)
  {
    long now = millis();

    // Verificar si ha pasado el tiempo necesario desde la última visualización
    if (now - lastDisplayTime >= displayInterval)
    {
      lastDisplayTime = now;

      // Actualizar el estado de la máquina de estados
      switch (currentState)
      {
      case AHT10s:
        displayAHT10(newTempAHT10, newHumAHT10);
        currentState = AHT20;
        break;
      case AHT20:
        displayAHT20(newTempAHT20, newHumHT20);
        currentState = BMP280;
        break;
      case BMP280:
        displayBMP280(newTempBMP, newPres, newBar, newAlt);
        currentState = ENS160state;
        break;
      case ENS160state:
        displayENS160(lastStatus, lastAQI, lastTVOC, lastECO2);
        currentState = PMS7003;
        break;
      case PMS7003:
        displayPMS7003(lastPM1_0, lastPM2_5, lastPM10_0, lastPM1_0_atmos, lastPM2_5_atmos, lastPM10_0_atmos);
        currentState = PMS7003G;
        break;
      case PMS7003G:
        displayPMS7003Greater(lastRawGreaterThan_0_5, lastRawGreaterThan_1_0, lastRawGreaterThan_2_5, lastRawGreaterThan_5_0, lastRawGreaterThan_10_0);
        currentState = TEMT6000;
        break;
      case TEMT6000:
        displayLUZ(light, light_value, lux);
        currentState = AHT10s;
        break;
      default:
        break;
      }
    }
  }
}
void loop()
{
  // Temt6000 light sensor
  light_value = analogRead(temt6000Pin);
  light = light_value * 0.0976; // percentage calculation
  volts = light_value * (3.3 / 1024.0);
  amps = volts / 10000.0;
  microamps = amps * 1000000;
  lux = microamps * 2.0;

  // AHT10
  newTempAHT10 = myAHT10.readTemperature(AHT10_FORCE_READ_DATA) - 3;
  newHumAHT10 = myAHT10.readHumidity(AHT10_USE_READ_DATA);

  // AHT20
  // sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);

  newTempAHT20 = temp.temperature - 3;
  newHumHT20 = humidity.relative_humidity;

  // BMP280
  newTempBMP = bmp.readTemperature() - 3;
  newPres = bmp.readPressure();
  newBar = bmp.readPressure() / 100;
  newAlt = bmp.readAltitude(1013.25);

  // ENS160
  Status = ENS160.getENS160Status();
  AQI = ENS160.getAQI();
  TVOC = ENS160.getTVOC();
  ECO2 = ENS160.getECO2();

  PM1_0 = 0;
  PM2_5 = 0;
  PM10_0 = 0;
  PM1_0_atmos = 0;
  PM2_5_atmos = 0;
  PM10_0_atmos = 0;
  RawGreaterThan_0_5 = 0;
  RawGreaterThan_1_0 = 0;
  RawGreaterThan_2_5 = 0;
  RawGreaterThan_5_0 = 0;
  RawGreaterThan_10_0 = 0;

  pms7003.updateFrame();

  if (pms7003.hasNewData())
  {
    PM1_0 = pms7003.getPM_1_0();
    PM2_5 = pms7003.getPM_2_5();
    PM10_0 = pms7003.getPM_10_0();
    PM1_0_atmos = pms7003.getPM_1_0_atmos();
    PM2_5_atmos = pms7003.getPM_2_5_atmos();
    PM10_0_atmos = pms7003.getPM_10_0_atmos();
    RawGreaterThan_0_5 = pms7003.getRawGreaterThan_0_5();
    RawGreaterThan_1_0 = pms7003.getRawGreaterThan_1_0();
    RawGreaterThan_2_5 = pms7003.getRawGreaterThan_2_5();
    RawGreaterThan_5_0 = pms7003.getRawGreaterThan_5_0();
    RawGreaterThan_10_0 = pms7003.getRawGreaterThan_10_0();

    lastPM1_0 = PM1_0;
    lastPM2_5 = PM2_5;
    lastPM10_0 = PM10_0;
    lastPM1_0_atmos = PM1_0_atmos;
    lastPM2_5_atmos = PM2_5_atmos;
    lastPM10_0_atmos = PM10_0_atmos;
    lastRawGreaterThan_0_5 = RawGreaterThan_0_5;
    lastRawGreaterThan_1_0 = RawGreaterThan_1_0;
    lastRawGreaterThan_2_5 = RawGreaterThan_2_5;
    lastRawGreaterThan_5_0 = RawGreaterThan_5_0;
    lastRawGreaterThan_10_0 = RawGreaterThan_10_0;
  }

  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  long now = millis();
  static long lastMsg = 0;

  if (now - lastMsg > 1000)
  { // Enviar telemetría cada 10 segundos
    lastMsg = now;
    // disable pixels by night usin lux
    if (lux < 15)
    {
      pixels.setBrightness(0);
    }
    else
    {
      pixels.setBrightness(10);
    }
    pixels.setPixelColor(0, pixels.Color(255, 20, 50));
    pixels.show();

    Serial.println("********************  TEMT6000  **********************");
    char msgl[50];
    snprintf(msgl, 50, "%.2f", light);
    Serial.print("Luz: ");
    Serial.println(msgl);
    // Publicar el mensaje
    client.publish("weatherstation/luz", msgl);

    char msglRaw[50];
    snprintf(msglRaw, 50, "%d", light_value);
    Serial.print("Light Raw: ");
    Serial.println(light_value);
    // Publicar el mensaje
    client.publish("weatherstation/luzRaw", msglRaw);

    char msglux[50];
    snprintf(msglux, 50, "%.2f", lux);
    Serial.print("Luminance: ");
    Serial.print(lux);
    Serial.println(" lux");
    // Publicar el mensaje
    client.publish("weatherstation/luminance", msglux);

    //

    // Aquí, pon el código para leer tu sensor
    Serial.println("********************  AHT10  **********************");
    char msgt[50];
    snprintf(msgt, 50, "%.2f", newTempAHT10);
    Serial.print("Temperatura AHT10: ");
    Serial.println(msgt);

    // Publicar el mensaje
    client.publish("weatherstation/temperaturaAHT10", msgt);

    char msgh[50];
    snprintf(msgh, 50, "%.2f", newHumAHT10);
    Serial.print("Humedad AHT10: ");
    Serial.println(msgh);

    // Publicar el mensaje
    client.publish("weatherstation/humedadAHT10", msgh);
    // displayAHT10(newTempAHT10, newHumAHT10);

    Serial.println("********************  AHT20  **********************");
    char msgtAHT20[50];
    snprintf(msgtAHT20, 50, "%.2f", newTempAHT20);
    Serial.print("Temperatura AHT20: ");
    Serial.println(msgtAHT20);
    client.publish("weatherstation/temperaturaAHT20", msgtAHT20);

    char msghAHT20[50];
    snprintf(msghAHT20, 50, "%.2f", newHumHT20);
    Serial.print("Humedad AHT20: ");
    Serial.println(msghAHT20);
    client.publish("weatherstation/humedadAHT20", msghAHT20);

    // displayAHT20(newTempAHT20, newHumHT20);

    Serial.println("********************  BMP280  **********************");
    char msgtBMP[50];
    snprintf(msgtBMP, 50, "%.2f", newTempBMP);
    Serial.print("Temperatura BMP280: ");
    Serial.println(msgtBMP);

    client.publish("weatherstation/temperaturaBMP", msgtBMP);

    char msgp[50];
    snprintf(msgp, 50, "%.2f", newPres);
    Serial.print("Presion BMP280: ");
    Serial.println(msgp);
    client.publish("weatherstation/presionBMP", msgp);

    char msgb[50];
    snprintf(msgb, 50, "%.2f", newBar);
    Serial.print("Presion Bar BMP280: ");
    Serial.println(msgb);
    client.publish("weatherstation/barometroBMP", msgb);

    char msga[50];
    snprintf(msga, 50, "%.2f", newAlt);
    Serial.print("Altitud BMP280: ");
    Serial.println(msga);
    client.publish("weatherstation/altitudBMP", msga);

    // displayBMP280(newTempBMP, newPres, newBar, newAlt);

    Serial.println("********************  ENS160  **********************");
    if (Status == 0 && ((AQI != 0 && AQI < 6) || TVOC != 0 || ECO2 != 0))
    {
      pixels.setPixelColor(0, pixels.Color(0, 0, 255));
      pixels.show();

      // ENS160
      char msgStatus[50];
      snprintf(msgStatus, 50, "%d", Status);
      Serial.print("Status ENS160: ");
      Serial.println(msgStatus);
      client.publish("weatherstation/ENS160/statusENS160", msgStatus);
      lastStatus = Status;

      char msgAQI[50];
      snprintf(msgAQI, 50, "%d", AQI);
      Serial.print("AQI ENS160: ");
      Serial.println(msgAQI);
      client.publish("weatherstation/ENS160/AQIENS160", msgAQI);
      lastAQI = AQI;

      char msgTVOC[50];
      snprintf(msgTVOC, 50, "%d", TVOC);
      Serial.print("TVOC ENS160: ");
      Serial.println(msgTVOC);
      client.publish("weatherstation/ENS160/TVOCENS160", msgTVOC);
      lastTVOC = TVOC;

      char msgECO2[50];
      snprintf(msgECO2, 50, "%d", ECO2);
      Serial.print("ECO2 ENS160: ");
      Serial.println(msgECO2);
      client.publish("weatherstation/ENS160/ECO2ENS160", msgECO2);
      lastECO2 = ECO2;
      // displayENS160(Status, AQI, TVOC, ECO2);
    }
    else
    {
      Serial.println("No hay datos de ENS160");
    }

    Serial.println("********************  PMS7003  **********************");

    if (PM10_0 > 0 || PM2_5 > 0 || PM1_0 > 0 || PM10_0_atmos > 0 || PM2_5_atmos > 0 || PM1_0_atmos > 0 || RawGreaterThan_0_5 > 0 || RawGreaterThan_1_0 > 0 || RawGreaterThan_2_5 > 0 || RawGreaterThan_5_0 > 0 || RawGreaterThan_10_0 > 0)
    {
      pixels.setPixelColor(0, pixels.Color(0, 255, 0));
      pixels.show();
      // PMS7003
      char msgPM1_0[50];
      snprintf(msgPM1_0, 50, "%.2f", PM1_0);
      Serial.print("PM1_0 PMS7003: ");
      Serial.println(msgPM1_0);
      client.publish("weatherstation/PMS7003/PM1_0", msgPM1_0);

      char msgPM2_5[50];
      snprintf(msgPM2_5, 50, "%.2f", PM2_5);
      Serial.print("PM2_5 PMS7003: ");
      Serial.println(msgPM2_5);
      client.publish("weatherstation/PMS7003/PM2_5", msgPM2_5);

      char msgPM10_0[50];
      snprintf(msgPM10_0, 50, "%.2f", PM10_0);
      Serial.print("PM10_0 PMS7003: ");
      Serial.println(msgPM10_0);
      client.publish("weatherstation/PMS7003/PM10_0", msgPM10_0);

      char msgPM1_0_atmos[50];
      snprintf(msgPM1_0_atmos, 50, "%.2f", PM1_0_atmos);
      Serial.print("PM1_0_atmos PMS7003: ");
      Serial.println(msgPM1_0_atmos);
      client.publish("weatherstation/PMS7003/PM1_0_atmos", msgPM1_0_atmos);

      char msgPM2_5_atmos[50];
      snprintf(msgPM2_5_atmos, 50, "%.2f", PM2_5_atmos);
      Serial.print("PM2_5_atmos PMS7003: ");
      Serial.println(msgPM2_5_atmos);
      client.publish("weatherstation/PMS7003/PM2_5_atmos", msgPM2_5_atmos);

      char msgPM10_0_atmos[50];
      snprintf(msgPM10_0_atmos, 50, "%.2f", PM10_0_atmos);
      Serial.print("PM10_0_atmos PMS7003: ");
      Serial.println(msgPM10_0_atmos);
      client.publish("weatherstation/PMS7003/PM10_0_atmos", msgPM10_0_atmos);

      char msgRawGreaterThan_0_5[50];
      snprintf(msgRawGreaterThan_0_5, 50, "%.0f", RawGreaterThan_0_5);
      Serial.print("RawGreaterThan_0_5 PMS7003: ");
      Serial.println(msgRawGreaterThan_0_5);
      client.publish("weatherstation/PMS7003/RawGreaterThan_0_5", msgRawGreaterThan_0_5);

      char msgRawGreaterThan_1_0[50];
      snprintf(msgRawGreaterThan_1_0, 50, "%.0f", RawGreaterThan_1_0);
      Serial.print("RawGreaterThan_1_0 PMS7003: ");
      Serial.println(msgRawGreaterThan_1_0);
      client.publish("weatherstation/PMS7003/RawGreaterThan_1_0", msgRawGreaterThan_1_0);

      char msgRawGreaterThan_2_5[50];
      snprintf(msgRawGreaterThan_2_5, 50, "%.0f", RawGreaterThan_2_5);
      Serial.print("RawGreaterThan_2_5 PMS7003: ");
      Serial.println(msgRawGreaterThan_2_5);
      client.publish("weatherstation/PMS7003/RawGreaterThan_2_5", msgRawGreaterThan_2_5);

      char msgRawGreaterThan_5_0[50];
      snprintf(msgRawGreaterThan_5_0, 50, "%.0f", RawGreaterThan_5_0);
      Serial.print("RawGreaterThan_5_0 PMS7003: ");
      Serial.println(msgRawGreaterThan_5_0);
      client.publish("weatherstation/PMS7003/RawGreaterThan_5_0", msgRawGreaterThan_5_0);

      char msgRawGreaterThan_10_0[50];
      snprintf(msgRawGreaterThan_10_0, 50, "%.0f", RawGreaterThan_10_0);
      Serial.print("RawGreaterThan_10_0 PMS7003: ");
      Serial.println(msgRawGreaterThan_10_0);
      client.publish("weatherstation/PMS7003/RawGreaterThan_10", msgRawGreaterThan_10_0);

      // displayPMS7003(PM1_0, PM2_5, PM10_0, PM1_0_atmos, PM2_5_atmos, PM10_0_atmos, RawGreaterThan_0_5, RawGreaterThan_1_0, RawGreaterThan_2_5, RawGreaterThan_5_0, RawGreaterThan_10_0);
    }
    else
    {
      Serial.println("No hay datos de PMS7003");
    }
    // display data in oled
    // use millis to display in oled
    // displayLUZ(light, light_value, lux);
  }
  // delay(1000);
}
void reconnect()
{
  int contador_error = 0;
  // Loop hasta que estemos reconectados
  while (!client.connected())
  {
    Serial.print("Intentando conexión MQTT...");
    // Intentar conectar
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password))
    {
      Serial.println("conectado");
    }
    else
    {
      Serial.print("COmunicacion MQTT falló, rc=");
      Serial.println(client.state());
      Serial.println("Intentar de nuevo en 5 segundos");

      pixels.setBrightness(10);
      pixels.setPixelColor(0, pixels.Color(0, 255, 0));
      pixels.show();
      delay(500);
      // set the color of the led to off
      pixels.setPixelColor(0, pixels.Color(0, 0, 0));
      pixels.show();
      // Esperar 5 segundos antes de volver a intentar
      displayMQTTerror(client.state());

      delay(5000);
      contador_error++;
      if (contador_error > 10)
      {
        Serial.println("Reiniciando ESP");

        ESP.restart();
      }
      Serial.println("Intento " + String(contador_error));
    }
  }
}
bool checkBound(float newValue, float prevValue, float maxDiff)
{
  return !isnan(newValue) && (newValue < prevValue - maxDiff || newValue > prevValue + maxDiff);
}
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  Serial.println("-----------------------");
}

// Display every data in oled
void displayAHT10(float newTempAHT10, float newHumAHT10)
{
  display.clearDisplay();
  // display AHT10 data
  display.setCursor(0, 5);
  display.println("AHT10: ");
  if (checkBound(newTempAHT10, lastTempAHT10, 1.0))
  {
    lastTempAHT10 = newTempAHT10;
    display.setCursor(0, 20);
    display.println("Temp: " + String(lastTempAHT10) + " C");
    display.display();
  }
  else
  {
    display.setCursor(0, 20);
    display.println("Temp: " + String(newTempAHT10)) + " C";
    display.display();
  }
  if (checkBound(newHumAHT10, lastHumAHT10, 1.0))
  {
    lastHumAHT10 = newHumAHT10;
    display.setCursor(0, 35);
    display.println("Hum: " + String(lastHumAHT10)) + " %";
    display.display();
  }
  else
  {
    display.setCursor(0, 35);
    display.println("Hum: " + String(newHumAHT10)) + " %";
    display.display();
  }
  // delay(1000);
}
void displayAHT20(float newTempAHT20, float newHumAHT20)
{
  display.clearDisplay();
  // display AHT20 data
  display.setCursor(0, 5);
  display.println("AHT20: ");
  if (checkBound(newTempAHT20, lastTempAHT20, 1.0))
  {
    lastTempAHT20 = newTempAHT20;
    display.setCursor(0, 20);
    display.println("Temperatura: " + String(lastTempAHT20) + " C");
    display.display();
  }
  else
  {
    display.setCursor(0, 20);
    display.println("Temperatura: " + String(newTempAHT20)) + " C";
    display.display();
  }
  if (checkBound(newHumAHT20, lastHumAHT20, 1.0))
  {
    lastHumAHT20 = newHumAHT20;
    display.setCursor(0, 35);
    display.println("Humedad: " + String(lastHumAHT20)) + " %";
    display.display();
  }
  else
  {
    display.setCursor(0, 35);
    display.println("Humedad: " + String(newHumAHT20)) + " %";
    display.display();
  }
  // delay(1000);
}
void displayBMP280(float newTempBMP, float newPres, float newBar, float newAlt)
{
  display.clearDisplay();
  // display BMP280 data
  display.setCursor(0, 5);
  display.println("BMP280: ");
  if (checkBound(newTempBMP, lastTempBMP, 1.0))
  {
    lastTempBMP = newTempBMP;
    display.setCursor(0, 20);
    display.println("Temp: " + String(lastTempBMP) + " C");
    display.display();
  }
  else
  {
    display.setCursor(0, 20);
    display.println("Temp: " + String(newTempBMP)) + " C";
    display.display();
  }
  if (checkBound(newPres, lastPres, 1.0))
  {
    lastPres = newPres;
    display.setCursor(0, 30);
    display.println("Presion: " + String(lastPres)) + " Pa";
    display.display();
  }
  else
  {
    display.setCursor(0, 30);
    display.println("Presion: " + String(newPres)) + " Pa";
    display.display();
  }
  if (checkBound(newBar, lastBar, 1.0))
  {
    lastBar = newBar;
    display.setCursor(0, 40);
    display.println("Barometro: " + String(lastBar)) + " hPa";
    display.display();
  }
  else
  {
    display.setCursor(0, 40);
    display.println("Barometro: " + String(newBar)) + " hPa";
    display.display();
  }
  if (checkBound(newAlt, lastAlt, 1.0))
  {
    lastAlt = newAlt;
    display.setCursor(0, 50);
    display.println("Altitud: " + String(lastAlt)) + " m";
    display.display();
  }
  else
  {
    display.setCursor(0, 50);
    display.println("Altitud: " + String(newAlt)) + " m";
    display.display();
  }
  // delay(1000);
}
void displayENS160(uint8_t Status, uint8_t AQI, uint16_t TVOC, uint16_t ECO2)
{
  display.clearDisplay();
  // display ENS160 data
  display.setCursor(0, 5);
  display.println("ENS160: ");
  if (checkBound(Status, lastStatus, 1.0))
  {
    lastStatus = Status;
    display.setCursor(0, 20);
    display.println("Status: " + String(lastStatus));
    display.display();
  }
  else
  {
    display.setCursor(0, 20);
    display.println("Status: " + String(Status));
    display.display();
  }
  if (checkBound(AQI, lastAQI, 1.0))
  {
    lastAQI = AQI;
    display.setCursor(0, 30);
    display.println("AQI: " + String(lastAQI));
    display.display();
  }
  else
  {
    display.setCursor(0, 30);
    display.println("AQI: " + String(AQI));
    display.display();
  }
  if (checkBound(TVOC, lastTVOC, 1.0))
  {
    lastTVOC = TVOC;
    display.setCursor(0, 40);
    display.println("TVOC: " + String((int)lastTVOC));
    display.display();
  }
  else
  {
    display.setCursor(0, 40);
    display.println("TVOC: " + String((int)TVOC));
    display.display();
  }
  if (checkBound(ECO2, lastECO2, 1.0))
  {
    lastECO2 = ECO2;
    display.setCursor(0, 50);
    display.println("ECO2: " + String((int)lastECO2));
    display.display();
  }
  else
  {
    display.setCursor(0, 50);
    display.println("ECO2: " + String((int)ECO2));
    display.display();
  }
  // delay(1000);
}
void displayPMS7003(float PM1_0, float PM2_5, float PM10_0, float PM1_0_atmos, float PM2_5_atmos, float PM10_0_atmos)
{
  display.clearDisplay();
  // display PMS7003 data
  display.setCursor(0, 5);
  display.println("PMS7003: ");
  if (checkBound(PM1_0, lastPM1_0, 1.0))
  {
    lastPM1_0 = PM1_0;
    display.setCursor(0, 20);
    display.println("PM 1.0: " + String((int)lastPM1_0) + " ug/m3");
    display.display();
  }
  else
  {
    display.setCursor(0, 20);
    display.println("PM 1.0: " + String((int)PM1_0) + " ug/m3");
    display.display();
  }
  if (checkBound(PM2_5, lastPM2_5, 1.0))
  {
    lastPM2_5 = PM2_5;
    display.setCursor(0, 30);
    display.println("PM 2.5: " + String((int)lastPM2_5) + " ug/m3");
    display.display();
  }
  else
  {
    display.setCursor(0, 30);
    display.println("PM 2.5: " + String((int)PM2_5) + " ug/m3");
    display.display();
  }
  if (checkBound(PM10_0, lastPM10_0, 1.0))
  {
    lastPM10_0 = PM10_0;
    display.setCursor(0, 40);
    display.println("PM 10: " + String((int)lastPM10_0) + " ug/m3");
    display.display();
  }
  else
  {
    display.setCursor(0, 40);
    display.println("PM 10: " + String((int)PM10_0) + " ug/m3");
    display.display();
  }
}
void displayPMS7003Greater(float RawGreaterThan_0_5, float RawGreaterThan_1_0, float RawGreaterThan_2_5, float RawGreaterThan_5_0, float RawGreaterThan_10_0)
{
  display.clearDisplay();
  // delay(1000);
  if (checkBound(RawGreaterThan_0_5, lastRawGreaterThan_0_5, 1.0))
  {
    lastRawGreaterThan_0_5 = RawGreaterThan_0_5;
    display.setCursor(0, 10);
    display.println("GT>0.5: " + String(lastRawGreaterThan_0_5) + " ppl/m3");
    display.display();
  }
  else
  {
    display.setCursor(0, 10);
    display.println("GT>0.5: " + String(RawGreaterThan_0_5) + " ppl/m3");
    display.display();
  }
  if (checkBound(RawGreaterThan_1_0, lastRawGreaterThan_1_0, 1.0))
  {
    lastRawGreaterThan_1_0 = RawGreaterThan_1_0;
    display.setCursor(0, 20);
    display.println("GT>1: " + String(lastRawGreaterThan_1_0) + " ppl/m3");
    display.display();
  }
  else
  {
    display.setCursor(0, 20);
    display.println("GT>1: " + String(RawGreaterThan_1_0) + " ppl/m3");
    display.display();
  }
  if (checkBound(RawGreaterThan_2_5, lastRawGreaterThan_2_5, 1.0))
  {
    lastRawGreaterThan_2_5 = RawGreaterThan_2_5;
    display.setCursor(0, 30);
    display.println("GT>2.5: " + String(lastRawGreaterThan_2_5) + " ppl/m3");
    display.display();
  }
  else
  {
    display.setCursor(0, 30);
    display.println("GT>2.5: " + String(RawGreaterThan_2_5) + " ppl/m3");
    display.display();
  }
  if (checkBound(RawGreaterThan_5_0, lastRawGreaterThan_5_0, 1.0))
  {
    lastRawGreaterThan_5_0 = RawGreaterThan_5_0;
    display.setCursor(0, 40);
    display.println("GT>5: " + String(lastRawGreaterThan_5_0) + " ppl/m3");
    display.display();
  }
  else
  {
    display.setCursor(0, 40);
    display.println("GT>5: " + String(RawGreaterThan_5_0) + " ppl/m3");
    display.display();
  }
  if (checkBound(RawGreaterThan_10_0, lastRawGreaterThan_10_0, 1.0))
  {
    lastRawGreaterThan_10_0 = RawGreaterThan_10_0;
    display.setCursor(0, 50);
    display.println("GT>10: " + String(lastRawGreaterThan_10_0) + " ppl/m3");
    display.display();
  }
  else
  {
    display.setCursor(0, 50);
    display.println("GT>10: " + String(RawGreaterThan_10_0) + " ppl/m3");
    display.display();
  }
  // delay(1000);
}
void displayLUZ(float light, int light_value, float lux)
{
  display.clearDisplay();
  // display LUZ data
  display.setCursor(0, 5);
  display.println("LUZ: ");
  display.setCursor(0, 25);
  display.println("Luz: " + String(light) + " %");
  display.setCursor(0, 35);
  display.println("Luz Raw: " + String(light_value));
  display.setCursor(0, 45);
  display.println("Luminance: " + String(lux) + " lux");
  display.display();
  // delay(1000);
}
void displayMQTTerror(int mqtterror)
{
  display.clearDisplay();
  display.setCursor(0, 5);
  display.println("MQTT: ");
  display.setCursor(0, 20);
  display.println("Error: " + String(mqtterror));
  display.setCursor(0, 30);
  display.println("Reintentando...");
  display.display();
  delay(2000);
}