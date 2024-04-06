#define BLYNK_TEMPLATE_ID "TMPL60VMq3AXH"
#define BLYNK_TEMPLATE_NAME "proj"
#define BLYNK_AUTH_TOKEN "Fhh5hoCb_iWpPRe1cFCu236J_NhaGEuA"
#include <BlynkSimpleEsp32.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "DHTesp.h" 
#include <Ticker.h>

#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);



char auth[] = "Fhh5hoCb_iWpPRe1cFCu236J_NhaGEuA";
char ssid[] = "Redmi13C";
char pass[] = "iml36912";




//relay
int fan =  4;
int light = 16;
int pump = 17;

WiFiClient espClient;
PubSubClient client(espClient);

#ifndef ESP32
#pragma message(THIS EXAMPLE IS FOR ESP32 ONLY!)
#error Select ESP32 board.
#endif

DHTesp dht;

void tempTask(void *pvParameters);
bool getTemperature();
void triggerGetTemp();

TaskHandle_t tempTaskHandle = NULL;
Ticker tempTicker;
ComfortState cf;

bool tasksEnabled = false;
int dhtPin = 22;

bool initTemp() {
  byte resultValue = 0;
  // Initialize temperature sensor
  dht.setup(dhtPin, DHTesp::DHT11);
  Serial.println("DHT initiated");

  // Start task to get temperature
  xTaskCreatePinnedToCore(
    tempTask,                       /* Function to implement the task */
    "tempTask ",                    /* Name of the task */
    4000,                           /* Stack size in words */
    NULL,                           /* Task input parameter */
    5,                              /* Priority of the task */
    &tempTaskHandle,                /* Task handle. */
    1);                             /* Core where the task should run */

  if (tempTaskHandle == NULL) {
    Serial.println("Failed to start task for temperature update");
    return false;
  } else {
    // Start update of environment data every 20 seconds
    tempTicker.attach(1, triggerGetTemp);
  }
  return true;
}

void triggerGetTemp() {
  if (tempTaskHandle != NULL) {
    xTaskResumeFromISR(tempTaskHandle);
  }
}

void tempTask(void *pvParameters) {
  Serial.println("tempTask loop started");
  while (1) // tempTask loop
  {
    if (tasksEnabled) {
      // Get temperature values
      getTemperature();
    }
    // Got sleep again
    vTaskSuspend(NULL);
  }
}

bool getTemperature() {
  // Reading temperature for humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
  TempAndHumidity newValues = dht.getTempAndHumidity();
  // Check if any reads failed and exit early (to try again).
  if (dht.getStatus() != 0) {
    Serial.println("DHT11 error status: " + String(dht.getStatusString()));
    return false;
  }

  float heatIndex = dht.computeHeatIndex(newValues.temperature, newValues.humidity);
  float dewPoint = dht.computeDewPoint(newValues.temperature, newValues.humidity);
  float cr = dht.getComfortRatio(cf, newValues.temperature, newValues.humidity);

  Serial.println("Temperature: " + String(newValues.temperature) + " °C, Humidity: " + String(newValues.humidity) + " %");

  // Publish temperature and humidity to MQTT topics
  client.publish("esp32/temperature", String(newValues.temperature).c_str());
  client.publish("esp32/humidity", String(newValues.humidity).c_str());

  return true;
}

#define TdsSensorPin 33
#define VREF 1 // analog reference voltage(Volt) of the ADC
#define SCOUNT 30 // sum of sample point
int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, temperature = 25;

int analogPin = A3; // ประกาศตัวแปร ให้ analogPin แทนขา analog ขาที่ 5
int val = 0;

//flowsensor
const int sensorPin = 35; // GPIO pin connected to sensor output
volatile int pulseCount; // Volatile because it's accessed within an interrupt

void IRAM_ATTR pulseCounter() {
  pulseCount++;
}


void setup() {
  Serial.begin(115200);
  pinMode(fan,OUTPUT);
  pinMode(light,OUTPUT);
  pinMode(pump,OUTPUT);
  pinMode(analogPin, INPUT);
  pinMode(TdsSensorPin, INPUT);
  pinMode(sensorPin, INPUT_PULLUP); // Assuming the sensor pulls the line low when a pulse is detected
  attachInterrupt(digitalPinToInterrupt(sensorPin), pulseCounter, FALLING);
  sensors.begin();
  Serial.println();
  Serial.println("DHT ESP32 example with tasks");
  initTemp();
  // Signal end of setup() to tasks
  tasksEnabled = true;

  Blynk.begin(auth,ssid,pass);
}

void loop() {

   Blynk.run();

  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U) //every 40 milliseconds, read the analog value from the ADC
  {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin); //read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
  }
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U)
  {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = averageVoltage / compensationCoefficient; //temperature compensation
    tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5; //convert voltage value to tds value

    Serial.print("TDS Value:");
    Serial.print(tdsValue, 0);
    Serial.println("ppm");

    // Publish TDS value to MQTT
    client.publish("esp32/tds", String(tdsValue).c_str());
  }
  //flow sensor
  detachInterrupt(sensorPin); // Disable interrupts while reading the count
  int pulses = pulseCount;
  pulseCount = 0; // Reset the count
  attachInterrupt(digitalPinToInterrupt(sensorPin), pulseCounter, FALLING); // Re-enable interrupts
  float flowRate = pulses / 7.5; // Assuming 7.5 pulses per liter, adjust according to your sensor's specifications
  
  Serial.print("Flow rate: ");
  Serial.print(flowRate);
  Serial.println(" L/min");

  // Publish flow rate to MQTT
  client.publish("esp32/flow_rate", String(flowRate).c_str());

  // ds18b20
  sensors.requestTemperatures();

  // Get the temperature in Celsius
  float temperatureC = sensors.getTempCByIndex(0);

  Serial.print("Temperature: ");
  Serial.print(temperatureC);
  Serial.println(" °C");

  // Publish temperature to MQTT
  client.publish("esp32/temperaturewather", String(temperatureC).c_str());

  int sensorValue = analogRead(A3);
  val = analogRead(analogPin); // Read analog value from LDR Photoresistor Sensor Module
  
  Serial.print("ldrval = ");
  Serial.println(val);
  client.publish("esp32/ldr", String(val).c_str());

  if (!tasksEnabled) {
    // Wait 2 seconds to let system settle down
    delay(2000);
    // Enable task that will read values from the DHT sensor
    tasksEnabled = true;
    if (tempTaskHandle != NULL) {
      vTaskResume(tempTaskHandle);
    }
  }
  yield();

  delay(1000);
}

int getMedianNum(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}
