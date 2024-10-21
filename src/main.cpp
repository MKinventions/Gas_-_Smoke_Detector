
/****************************************************************/
/*********MKinventions Robotics and Electronics******************/
/****************************************************************/
#include <Arduino.h>
#include "Arduino_FreeRTOS.h"
#include "timers.h"

#define MQ135_PIN A0 // MQ-135 connected to analog pin A0
#define MQ3_PIN A1   // MQ-3 connected to analog pin A1
#define BUZZER 5    // Buzzer connected to digital pin 12
#define AIR_LED 7
#define SMOKE_LED 6

// Parameters for calibration
float R0_MQ135 = 10.0; // Base resistance for MQ-135 in clean air
float R0_MQ3 = 10.0;   // Base resistance for MQ-3 in clean air
float ppm_MQ135 = 0; 
float alcoholConcentration = 0;

TimerHandle_t xBuzzerTimer;
TimerHandle_t xAirLedTimer;
TimerHandle_t xSmokeLedTimer;

TaskHandle_t xSmokeSensorHandle;
TaskHandle_t xAirSensorHandle;

bool isMQ135_Detected = false;   
bool isMQ3_Detected = false;
bool buzzerState = false;
bool airLedState = false;
bool smokeLedState = false;

const int gasSensorThreshold_MQ135 = 800; // Threshold for MQ-135
const int smokeSensorThreshold_MQ3 = 300; // Threshold for MQ-3


void buzzerTimer_callback(TimerHandle_t xTimer)
{
  buzzerState = !buzzerState;                     // Toggle buzzer state
  digitalWrite(BUZZER, buzzerState ? HIGH : LOW); // Update buzzer based on the state
  // Serial.print("Buzzer state: ");
  // Serial.println(buzzerState ? "ON" : "OFF");     // Debugging message
}

void airLedBlinkTimer_callback(TimerHandle_t xTimer)
{
  airLedState = !airLedState;                      // Toggle air led state
  digitalWrite(AIR_LED, airLedState ? HIGH : LOW); // Update air led based on the state
  Serial.print("Poor Quality Air (MQ-135) =>[");     
  Serial.print(ppm_MQ135);
  Serial.println("]");
}

void smokeLedBlinkTimer_callback(TimerHandle_t xTimer)
{
  smokeLedState = !smokeLedState;                      // Toggle smoke led state
  digitalWrite(SMOKE_LED, smokeLedState ? HIGH : LOW); // Update smoke led based on the state
  Serial.print("Danger Smoke/Gas Leakage (MQ-3) =>[");
  Serial.print(alcoholConcentration);
  Serial.println("]");  

}

// Task for the MQ-135 sensor
void sensorTask_MQ135(void *pvParameters)
{
  while (1)
  {
    // Read analog value from the MQ-135 sensor
    int readRaw_MQ135 = analogRead(MQ135_PIN);

    float voltage = readRaw_MQ135 * (5.0 / 1023.0);  
  // Calculate the sensor resistance using the voltage divider formula
   float RS_MQ135 = (5.0 - voltage) / voltage; // RS in kOhms

    // Calculate the RS/R0 ratio for MQ-135
    float RS_R0_ratio_MQ135 = RS_MQ135 / R0_MQ135;

    // Estimate gas concentration in ppm for MQ-135
    ppm_MQ135 = 116.6020682 * pow(RS_R0_ratio_MQ135, -2.769034857);


    // Check air quality for MQ-135
    if (ppm_MQ135 >= gasSensorThreshold_MQ135)
    {
      vTaskSuspend(xSmokeSensorHandle);
      if (!xTimerIsTimerActive(xBuzzerTimer))
      {
        xTimerStart(xBuzzerTimer, 0);
      }
      if (!xTimerIsTimerActive(xAirLedTimer))
      {
        xTimerStart(xAirLedTimer, 0);
      }
    }
    else
    {
      // Print result to the Serial Monitor
      Serial.print("MQ-135 Sensor Value: [");
      Serial.print(ppm_MQ135);
      Serial.println("], Good Quality Air");

      xTimerStop(xBuzzerTimer, 0);
      xTimerStop(xAirLedTimer, 0);
      vTaskResume(xSmokeSensorHandle);
      digitalWrite(BUZZER, LOW);  // Ensure buzzer is off
      digitalWrite(AIR_LED, LOW); // Ensure air LED is off
    }

    vTaskDelay(pdMS_TO_TICKS(2000)); // Check every 2000ms to reduce CPU load
  }
}

// Task for the MQ-3 sensor
void sensorTask_MQ3(void *pvParameters)
{
  while (1)
  {
    // Read analog value from the MQ-3 sensor
    int readRaw_MQ3 = analogRead(MQ3_PIN);

    // Convert the analog value to voltage (Arduino Uno runs on 5V logic)
    float voltage = readRaw_MQ3 * (5.0 / 1023.0);

    // Calculate the alcohol concentration in PPM
    // The conversion factor would depend on the calibration from the datasheet or empirical testing
    alcoholConcentration = voltage * 50.0; // Example conversion factor

    // Check air quality for MQ-3
    if (alcoholConcentration > smokeSensorThreshold_MQ3)
    {
      vTaskSuspend(xAirSensorHandle);
      // Smoke/gas detected, start the timers
      if (!xTimerIsTimerActive(xBuzzerTimer))
      {
        xTimerStart(xBuzzerTimer, 0);
      }
      if (!xTimerIsTimerActive(xSmokeLedTimer))
      {
        xTimerStart(xSmokeLedTimer, 0);
      }
    }
    else
    {
      // Print result to the Serial Monitor
      Serial.print("MQ-3 Sensor Value: [");
      Serial.print(alcoholConcentration);
      Serial.println("], No Smoke/Gas Leakage");
      // No smoke/gas detected, stop the timers
      xTimerStop(xBuzzerTimer, 0);
      xTimerStop(xSmokeLedTimer, 0);
      vTaskResume(xAirSensorHandle);
      // digitalWrite(BUZZER, LOW);    // Ensure buzzer is off
      digitalWrite(SMOKE_LED, LOW); // Ensure smoke LED is off
    }

    vTaskDelay(pdMS_TO_TICKS(2000)); // Check every 2000ms to reduce CPU load
  }
}

void setup()
{
  Serial.begin(9600);
  pinMode(MQ135_PIN, INPUT);
  pinMode(MQ3_PIN, INPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(SMOKE_LED, OUTPUT);
  pinMode(AIR_LED, OUTPUT);

  // Create the buzzer timer
  xBuzzerTimer = xTimerCreate("Buzzer", pdMS_TO_TICKS(100), pdTRUE, 0, buzzerTimer_callback);
  if (xBuzzerTimer == NULL)
  {
    Serial.println("Timer Create Failed!");
    while (1)
      ;
  }
  // Create the Air sensor led timer
  xAirLedTimer = xTimerCreate("Air Sensor", pdMS_TO_TICKS(100), pdTRUE, 0, airLedBlinkTimer_callback);
  if (xAirLedTimer == NULL)
  {
    Serial.println("Timer Create Failed for Air Led!");
    while (1)
      ;
  }

  // Create the Smoke sensor led timer
  xSmokeLedTimer = xTimerCreate("Gas/Smoke Sensor", pdMS_TO_TICKS(100), pdTRUE, 0, smokeLedBlinkTimer_callback);
  if (xSmokeLedTimer == NULL)
  {
    Serial.println("Timer Create Failed for Smoke Led!");
    while (1)
      ;
  }

  // Create the gas sensor tasks
  xTaskCreate(sensorTask_MQ135, "Air Quality Sensor MQ-135", 255, NULL, 1, &xAirSensorHandle);
  xTaskCreate(sensorTask_MQ3, "Gas/Smoke Sensor MQ-3", 255, NULL, 2, &xSmokeSensorHandle);
}

void loop()
{
}
