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

// WiFi
const char *ssid = "DIGIFIBRA-cF5T";
const char *password = "P92sKt3FGfsy";

// MQTT Broker
const char *mqtt_broker = "192.168.1.43";
const char *topic = "emqx/weatherstation";
const char *mqtt_username = "root";
const char *mqtt_password = "orangepi.hass";
const int mqtt_port = 1883;

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

//*************************************************  SETUP  *******************************************************
void setup()
{
  pixels.setBrightness(10);

  // Set software serial baud to 115200;
  Serial.begin(115200);
  // Connecting to a WiFi network
  WiFi.mode(WIFI_STA); // Optional
  WiFi.begin(ssid, password);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  Serial.println("\nConnecting");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the Wi-Fi network");
  // connecting to a mqtt broker
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);

  while (!client.connected())
  {
    String client_id = "weatherstation-client-";
    client_id += String(WiFi.macAddress());
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

  setupAHT10();
  setupAHT20();
  setupBMP280();
  setupENS160();
  setupPMS7003();

  // client.subscribe(topic);
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

void loop()
{
  // AHT10
  float newTempAHT10 = myAHT10.readTemperature(AHT10_FORCE_READ_DATA)-3;
  float newHumAHT10 = myAHT10.readHumidity(AHT10_USE_READ_DATA);

  // AHT20
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);

  float newTempAHT20 = temp.temperature - 3;
  float newHumHT20 = humidity.relative_humidity;

  // BMP280
  float newTempBMP = bmp.readTemperature() - 3;
  float newPres = bmp.readPressure();
  float newBar = bmp.readPressure() / 100;
  float newAlt = bmp.readAltitude(1013.25);

  // ENS160
  uint8_t Status = ENS160.getENS160Status();
  uint8_t AQI = ENS160.getAQI();
  uint16_t TVOC = ENS160.getTVOC();
  uint16_t ECO2 = ENS160.getECO2();

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

  pms7003.updateFrame();

  if (pms7003.hasNewData())
  {
    float newPM1_0 = pms7003.getPM_1_0();
    float newPM2_5 = pms7003.getPM_2_5();
    float newPM10_0 = pms7003.getPM_10_0();
    float newPM1_0_atmos = pms7003.getPM_1_0_atmos();
    float newPM2_5_atmos = pms7003.getPM_2_5_atmos();
    float newPM10_0_atmos = pms7003.getPM_10_0_atmos();
    float newRawGreaterThan_0_5 = pms7003.getRawGreaterThan_0_5();
    float newRawGreaterThan_1_0 = pms7003.getRawGreaterThan_1_0();
    float newRawGreaterThan_2_5 = pms7003.getRawGreaterThan_2_5();
    float newRawGreaterThan_5_0 = pms7003.getRawGreaterThan_5_0();
    float newRawGreaterThan_10_0 = pms7003.getRawGreaterThan_10_0();

    PM1_0 = newPM1_0;
    PM2_5 = newPM2_5;
    PM10_0 = newPM10_0;
    PM1_0_atmos = newPM1_0_atmos;
    PM2_5_atmos = newPM2_5_atmos;
    PM10_0_atmos = newPM10_0_atmos;
    RawGreaterThan_0_5 = newRawGreaterThan_0_5;
    RawGreaterThan_1_0 = newRawGreaterThan_1_0;
    RawGreaterThan_2_5 = newRawGreaterThan_2_5;
    RawGreaterThan_5_0 = newRawGreaterThan_5_0;
    RawGreaterThan_10_0 = newRawGreaterThan_10_0;
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

    Serial.println("********************  ENS160  **********************");
    if (Status == 0 || (AQI < 6 || TVOC > 0 || ECO2 > 0))
    {
      pixels.setPixelColor(0, pixels.Color(255, 0, 0));
      pixels.show();

      // ENS160
      char msgStatus[50];
      snprintf(msgStatus, 50, "%d", Status);
      Serial.print("Status ENS160: ");
      Serial.println(msgStatus);
      client.publish("weatherstation/ENS160/statusENS160", msgStatus);

      char msgAQI[50];
      snprintf(msgAQI, 50, "%d", AQI);
      Serial.print("AQI ENS160: ");
      Serial.println(msgAQI);
      client.publish("weatherstation/ENS160/AQIENS160", msgAQI);

      char msgTVOC[50];
      snprintf(msgTVOC, 50, "%d", TVOC);
      Serial.print("TVOC ENS160: ");
      Serial.println(msgTVOC);
      client.publish("weatherstation/ENS160/TVOCENS160", msgTVOC);

      char msgECO2[50];
      snprintf(msgECO2, 50, "%d", ECO2);
      Serial.print("ECO2 ENS160: ");
      Serial.println(msgECO2);
      client.publish("weatherstation/ENS160/ECO2ENS160", msgECO2);
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
    }
    else
    {
      Serial.println("No hay datos de PMS7003");
    }
  }

  delay(1000);
}

void reconnect()
{
  // Loop hasta que estemos reconectados
  while (!client.connected())
  {
    Serial.print("Intentando conexión MQTT...");
    // Intentar conectar
    if (client.connect("WEATHERSTATIONClient"))
    {
      Serial.println("conectado");
    }
    else
    {
      Serial.print("falló, rc=");
      Serial.print(client.state());
      Serial.println(" intentar de nuevo en 5 segundos");
      // Esperar 5 segundos antes de volver a intentar
      delay(5000);
    }
  }
}

bool checkBound(float newValue, float prevValue, float maxDiff)
{
  return !isnan(newValue) && (newValue < prevValue - maxDiff || newValue > prevValue + maxDiff);
}