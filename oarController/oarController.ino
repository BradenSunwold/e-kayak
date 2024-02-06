#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <FreeRTOS_SAMD21.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_NeoPixel.h>
#include "LedPixelMaps.hpp"

unsigned long beginTime = 0;
unsigned long taskTime = 0;

uint32_t count = 0;

// Define rf24 radio
const int rf24CE = 6;
const int rf24CSN = 5;
RF24 radio(rf24CE, rf24CSN, 4000000);
// const byte address[6] = "2Node";
uint8_t address[][6] = { "1Node", "2Node" };

// Define BNO IMU
#define BNO08X_CS 10
#define BNO08X_INT 9
#define BNO08X_RESET -1
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// #define PIXEL_PIN 10
// #define LED_COUNT 12
// Adafruit_NeoPixel strip(LED_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

#define BUTTON_PIN 12
#define POWER_HOLD 13

#define VBATPIN A7

// #define configUSE_IDLE_HOOK 1

// Initialize global queues
size_t msgQueueLength = 10;
static QueueHandle_t rfInMsgQueue;
static QueueHandle_t imuDataQueue;
static QueueHandle_t kayakStatusQueue;
static QueueHandle_t buttonQueue;
static QueueHandle_t currentStateQueue;
static QueueHandle_t ledAnnimationQueue;
static QueueHandle_t oarBatteryQueue; 
static QueueHandle_t rfOutMsgQueue;
static QueueHandle_t ledPixelMapQueue;

// Structs / Enums
typedef enum
{
  eBatteryVolt,
  eStartup,
  eLowBattery, 
  eHighCurrent, 
  eComsLoss,
  eUnknownFault,
  eFaultCleared
} StatusType_t;

typedef enum
{
  eConnecting,
  eConnected,
  eModeUpdate,
  eSpeedUpdate, 
  eBatteryUpdate,
  eError
} LedAnnimateType_t;

typedef struct
{
  LedAnnimateType_t fAnnimation;
  uint32_t fColor;
  int fDelay;
  uint32_t fData;
} LedMsg_t;

typedef struct 
{
  StatusType_t fStatusType;
  float fStatusData;
} StatusMsg_t;

typedef struct
{
  bool fAutoMode;
  uint8_t fSpeed;     // 0 = off, 1 = low speed, 2 = med speed, 3 = high speed
} StateMsg_t;

typedef struct
{
  float fRoll;
  float fPitch; 
  float fYaw;
} ImuDataType_t;

typedef struct
{
  float fAccel;
  float fGyro; 
} ImuAdditionalReports_t;

typedef struct 
{
  ImuAdditionalReports_t fX;
  ImuAdditionalReports_t fY;
  ImuAdditionalReports_t fZ;
} ImuAddReportsVector_t;

typedef struct
{
  ImuDataType_t fEulerData;
  ImuAddReportsVector_t fAddReportsVect;
} FullImuDataSet_t;

typedef struct 
{
  StateMsg_t fOutputState;
  FullImuDataSet_t fOutputImu;
} RfOutputMsg_t;


void CirclingAnnimation(uint32_t color, int wait);
void PulseAnnimation(uint32_t color, int wait);
void BatteryUpdateAnnimation(uint32_t newBatteryPercentage);
void StartupAnnimation(int wait);
void SpeedUpdateAnnimation(uint32_t newSpeed);

//######################** Support functions ****************************//
void SetImuReports()
{
  long reportIntervalUs = 50000;

  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_ARVR_STABILIZED_RV, reportIntervalUs)) 
  {
    Serial.println("Could not enable stabilized remote vector");
  }

  if (!bno08x.enableReport(SH2_ACCELEROMETER, reportIntervalUs)) 
  {
    Serial.println("Could not enable accelerometer");
  }

  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, reportIntervalUs)) 
  {
    Serial.println("Could not enable gyroscope");
  }
}

void QuaternionToEuler(float qr, float qi, float qj, float qk, ImuDataType_t* dataPtr, bool degrees = false) 
{
    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    dataPtr->fYaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    dataPtr->fPitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    dataPtr->fRoll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      dataPtr->fYaw *= RAD_TO_DEG;
      dataPtr->fPitch *= RAD_TO_DEG;
      dataPtr->fRoll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotationalVector, ImuDataType_t* dataPtr, bool degrees = false) 
{
    QuaternionToEuler(rotationalVector->real, rotationalVector->i, rotationalVector->j, rotationalVector->k, dataPtr, degrees);
}

//****************************** Tasks **********************************//
// Task globals
// Input tasks
TaskHandle_t Handle_RfInputTask;
TaskHandle_t Handle_ImuInputTask;
TaskHandle_t Handle_KayakStatusManagerTask;
TaskHandle_t Handle_ProcessOutputsTask;
TaskHandle_t Handle_LedDriverTask;
TaskHandle_t Handle_ButtonInputTask;
TaskHandle_t Handle_StateManagerTask;
TaskHandle_t Handle_OarBatteryMonitorTask;
TaskHandle_t Handle_RfOutputTask;
TaskHandle_t Handle_LedPixelUpdaterTask;

//**************************************************************************
// Can use these function for RTOS delays
// Takes into account processor speed
// Use these instead of delay(...) in rtos tasks
//**************************************************************************
void myDelayUs(int us)
{
  vTaskDelay( us / portTICK_PERIOD_US );  
}

void myDelayMs(int ms)
{
  vTaskDelay( (ms * 1000) / portTICK_PERIOD_US );  
}

void myDelayMsUntil(TickType_t *previousWakeTime, int ms)
{
  vTaskDelayUntil( previousWakeTime, (ms * 1000) / portTICK_PERIOD_US );  
}

// RF input task
static void RfInputTask( void *pvParameters ) 
{
  // Struct to locally store incoming RF data
  StatusMsg_t incomingData;
  // radio.printDetails();       // (smaller) function that prints raw register values

  while(1)
  {
    radio.setPayloadSize(sizeof(incomingData));  // float datatype occupies 4 bytes    // Read incoming rf24 radio
    radio.startListening();

    if (radio.available()) 
    {
      // Store incoming data to local buffer
      radio.read(&incomingData, sizeof(incomingData));
      // Serial.println(incomingData.statusData);

      // Store relevant data to queue
      xQueueSend(rfInMsgQueue, (void *)&incomingData, 1);

    }
    // beginTime = millis();
    count = 0;
    myDelayMs(50);    // execute task at 20Hz
    // taskTime = millis() - beginTime;
    // Serial.println(taskTime);
  }

  Serial.println("Task Monitor: Deleting");
  vTaskDelete( NULL );
}

// IMU input task
static void ImuInputTask( void *pvParameters ) 
{

  sh2_SensorValue_t sensorData;   // local variables to cpature reported data
  // ImuDataType_t eulerData;

  FullImuDataSet_t fullImuDataSet;

  while(1)
  {
    // Check for IMU reset
    if (bno08x.wasReset()) 
    {
      Serial.print("sensor was reset ");
      SetImuReports();
    }

    // Read incoming sensor data
    while (bno08x.getSensorEvent(&sensorData)) 
    {
      // in this demo only one report type will be received depending on FAST_MODE define (above)
      switch (sensorData.sensorId) 
      {
        case SH2_ARVR_STABILIZED_RV :
          quaternionToEulerRV(&sensorData.un.arvrStabilizedRV, &fullImuDataSet.fEulerData, true);
          // Serial.print("Pitch: ");
          // Serial.println(fullImuDataSet.fEulerData.fPitch);
          // Serial.println("Roll: ");
          // Serial.println(fullImuDataSet.fEulerData.fRoll);
          // Serial.print("Yaw: ");
          // Serial.println(fullImuDataSet.fEulerData.fYaw);
          break;
        case SH2_ACCELEROMETER :
          fullImuDataSet.fAddReportsVect.fX.fAccel = sensorData.un.accelerometer.x;
          fullImuDataSet.fAddReportsVect.fY.fAccel = sensorData.un.accelerometer.y;
          fullImuDataSet.fAddReportsVect.fZ.fAccel = sensorData.un.accelerometer.z;
          break;
        case SH2_GYROSCOPE_CALIBRATED :
          fullImuDataSet.fAddReportsVect.fX.fGyro = sensorData.un.gyroscope.x;
          fullImuDataSet.fAddReportsVect.fY.fGyro = sensorData.un.gyroscope.y;
          fullImuDataSet.fAddReportsVect.fZ.fGyro = sensorData.un.gyroscope.z;
          break;
        default : 
          Serial.println("Nothing recieved from IMU");
      }
      // Push data to queue
      xQueueSend(imuDataQueue, (void*)&fullImuDataSet, 1);
    }
    
    // beginTime = millis();
    count = 0;
    myDelayMs(50);    // execute task at 20Hz
    // taskTime = millis() - beginTime;
    // Serial.println(taskTime);
  }

  Serial.println("Task Monitor: Deleting");
  vTaskDelete( NULL );
}

static void ButtonInputTask( void *pvParameters )
{
  // Debounce button inputs
  int buttonState = 0; 
  int lastButtonState = 0;
  uint32_t buttonStateChangeCount = 0;

  int buttonReport = 0;

  unsigned long lastDebounceTime = 0;   // the last time the output pin was toggled
  unsigned long debounceDelay = 20;     // the debounce time; increase if the output flickers
  unsigned long doubleClickTime = 0;
  unsigned long doubleClickDelay = 300; // the double click time frame, lower if double click seem laggy


  while(1)
  {
    int reading = digitalRead(BUTTON_PIN);

    // If the switch changed, due to noise or pressing:
    if (reading != lastButtonState) 
    {
      // reset the debouncing timer
      lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) 
    {
      // whatever the reading is at, it's been there for longer than the debounce
      // delay, so take it as the actual current state:

      // if the button state has changed:
      if (reading != buttonState) 
      {
        buttonState = reading;

        buttonStateChangeCount++;
        if(buttonStateChangeCount <= 1)
        {
          doubleClickTime = millis();   // Capture time of first button state change
        }
      }
    }

    lastButtonState = reading;

    if((millis() - doubleClickTime) > doubleClickDelay)
    {
      if(buttonStateChangeCount > 2)
      {
        // Found double click? 
        Serial.println("DOUBLE");
        buttonReport = 2;
        xQueueSend(buttonQueue, (void*)&buttonReport, 1);
      }
      else if(buttonStateChangeCount > 1)
      {
        // Found single click
        Serial.println("SINGLE");
        buttonReport = 1;
        xQueueSend(buttonQueue, (void*)&buttonReport, 1);
      }
      buttonStateChangeCount = 0;
      buttonReport = 0;
    }

    // beginTime = millis();
    myDelayMs(50);   // execute task at 20Hz
    // taskTime = millis() - beginTime;
    // Serial.println(taskTime);
  }

  Serial.println("Task Monitor: Deleting");
  vTaskDelete( NULL );
}

static void KayakStsManagerTask( void *pvParameters ) 
{
  StatusMsg_t rfReceiverMsg;
  rfReceiverMsg.fStatusType = eStartup;    // Initialize status type to connecting on startup
  rfReceiverMsg.fStatusData = 0.0;
  xQueueSend(kayakStatusQueue, (void *)&rfReceiverMsg, 1);

  unsigned long lastReceivedMsg = 0;
  unsigned long comsReconnectTime = 3000;     // If lose signal for 2 seconds, signal re connecting annimation
  unsigned long comsErrorTime = 8000;         // 8 seconds till hard fault unless re-connects

  bool startupFlag = true;
  bool comsTimeoutFlag = false;

  // myDelayMs(500);    // Give 1 second for connecting annimation to play out

  while(1)
  {
    // Check for a message in the RF input queue
    if (xQueueReceive(rfInMsgQueue, (void *)&rfReceiverMsg, 0) == pdTRUE) 
    {
      // Pass message through to output processor
      xQueueSend(kayakStatusQueue, (void *)&rfReceiverMsg, 1);
      lastReceivedMsg = millis();   // Record last recieved message from kayak
      comsTimeoutFlag = false;
    }
    // If have not recieved message within X seconds - signal coms loss 
    else if((millis() - lastReceivedMsg) > comsReconnectTime)
    {
      if(startupFlag)
      {
        rfReceiverMsg.fStatusType = eStartup;
        xQueueSend(kayakStatusQueue, (void *)&rfReceiverMsg, 1);
        startupFlag = false;
      }
      else if((millis() - lastReceivedMsg) > comsErrorTime && !comsTimeoutFlag)
      {
        rfReceiverMsg.fStatusType = eComsLoss;
        xQueueSend(kayakStatusQueue, (void *)&rfReceiverMsg, 1);
        comsTimeoutFlag = true;
      }
    }

    // beginTime = millis();
    myDelayMs(50);    // execute task at 20Hz
    // taskTime = millis() - beginTime;
    // Serial.println(taskTime);
  }

  Serial.println("Task Monitor: Deleting");
  vTaskDelete( NULL );
}

static void OarBatteryMonitorTask( void *pvParameters ) 
{

  while(1)
  {
    // Read battery pin 
    float batteryVoltage = analogRead(VBATPIN);
    batteryVoltage *= 2;        // Account for voltage divider
    batteryVoltage *= 3.3;      // Multiply by 3.3V - reference voltage
    batteryVoltage /= 1024;     // convert to voltage
    // Serial.print("VBat: " ); Serial.println(batteryVoltage);

    // Send to output processor
    xQueueSend(oarBatteryQueue, (void *)&batteryVoltage, 1);

    // beginTime = millis();
    myDelayMs(1000);    // Execute task at 20Hz
    // taskTime = millis() - beginTime;
    // Serial.println(taskTime);
  }

  Serial.println("Task Monitor: Deleting");
  vTaskDelete( NULL );  
}

static void StateManagerTask( void *pvParameters )
{
  // Initialize state
  StateMsg_t currState;
  currState.fAutoMode = false;
  currState.fSpeed = 0;

  uint8_t buttonPress = 0;

  while(1)
  {
    if(xQueueReceive(buttonQueue, (void *)&buttonPress, 0) == pdTRUE)
    {
      // Received button update 
      if(buttonPress == 1)
      {
        // Single click - Increment speed and check for roll over
        if(currState.fSpeed == 3)
        {
          currState.fSpeed = 0;
        }
        else
        {
          currState.fSpeed++;
        }
      }
      else if(buttonPress == 2)
      {
        // Double click - Toggle mode
        currState.fAutoMode = !currState.fAutoMode;
      }

      // Send current state to output proceesor
      xQueueSend(currentStateQueue, (void *)&currState, 1);
    }

    // beginTime = millis();
    myDelayMs(50);    // execute task at 20Hz
    // taskTime = millis() - beginTime;
    // Serial.println(taskTime);
  }

  Serial.println("Task Monitor: Deleting");
  vTaskDelete( NULL );  
}

static void ProcessOutputsTask( void *pvParameters ) 
{
  bool fault = false;
  bool faultCleared = false;
  bool startup = true;

  bool autoMode = false;
  uint32_t speed = 0;

  StatusMsg_t kayakStatusMsg;
  StateMsg_t stateMsg;
  LedMsg_t ledMsg;

  float kayakBatteryVolt = 26;
  float oarbatteryVolt = 4.2;

  const TickType_t xTicksToWait = 5 / portTICK_PERIOD_MS;

  while(1)
  {
    // First check kayak status
    if(xQueueReceive(kayakStatusQueue, (void *)&kayakStatusMsg, 0) == pdTRUE)
    {
      // If received new status
      switch(kayakStatusMsg.fStatusType)
      {
        // first check for errors
        case eLowBattery : 
        case eHighCurrent :
        case eComsLoss :
        case eUnknownFault :
          // faults take priority
          ledMsg.fAnnimation = eError;
          ledMsg.fColor = strip.Color(255, 0, 0);
          ledMsg.fDelay = 5;

          // Add to led queue
          // Serial.println("Faulted");
          xQueueSend(ledAnnimationQueue, (void *)&ledMsg, 1);
          break;

        // Then check for fault cleared
        case eFaultCleared :
          ledMsg.fAnnimation = eConnected;
          ledMsg.fColor = strip.Color(0, 255, 0);
          ledMsg.fDelay = 5;

          // Add to led queue
          // Serial.println("Fault Cleared");
          xQueueSend(ledAnnimationQueue, (void *)&ledMsg, 1);
          break;

        // Then check for battery status
        case eBatteryVolt :
          // Package kayak battery voltage for output processor
          kayakBatteryVolt = kayakStatusMsg.fStatusData;
          if(startup == true)
          {
            // Send eConnected annimation
            ledMsg.fAnnimation = eConnected;
            ledMsg.fColor = strip.Color(0, 255, 0);
            ledMsg.fDelay = 5;

            // Add to led queue
            // Serial.println("Connected");
            xQueueSend(ledAnnimationQueue, (void *)&ledMsg, 1);
            startup = false;    // Signal output processor that startup is over
          }
          break;
        
        case eStartup :
          startup = true;
          // Send eConnecting annimation
          ledMsg.fAnnimation = eConnecting;
          ledMsg.fColor = strip.Color(0, 255, 0);
          ledMsg.fDelay = 50;

          // Add to led queue
          // Serial.println("Startup");
          xQueueSend(ledAnnimationQueue, (void *)&ledMsg, 1);
          break;

        default :
          Serial.println("Did not recieve valid RF message");
          break;
      }
    }

    // Next check current state and report to LED driver / RF out any changes
    if(xQueueReceive(currentStateQueue, (void *)&stateMsg, 0) == pdTRUE)
    {
      if((stateMsg.fAutoMode && !autoMode) || (!stateMsg.fAutoMode && autoMode))
      {
        // Report state change to LED driver
        ledMsg.fAnnimation = eModeUpdate;
        ledMsg.fColor = strip.Color(120, 120, 255);
        ledMsg.fDelay = 5;

        // Toggle autoMode
        autoMode = stateMsg.fAutoMode;
        // Serial.println(autoMode);

        // Add to queue
        Serial.println("State Change");
        xQueueSend(ledAnnimationQueue, (void *)&ledMsg, 1);
        xQueueSend(rfOutMsgQueue, (void *)&stateMsg, 1);
      }
      else if(stateMsg.fSpeed != speed)
      {
        // Package current speed and send to LED driver
        ledMsg.fAnnimation = eSpeedUpdate;
        ledMsg.fColor = strip.Color(120, 120, 255);
        ledMsg.fData = stateMsg.fSpeed;
        speed = stateMsg.fSpeed;    // Update local speed checker variable
        // Serial.println(speed);

        // Add to queues
        Serial.println("Speed change");
        xQueueSend(ledAnnimationQueue, (void *)&ledMsg, 1);
        xQueueSend(rfOutMsgQueue, (void *)&stateMsg, 1);
      }
    }

    // Finally read oar battery
    if(xQueueReceive(oarBatteryQueue, (void *)&oarbatteryVolt, 0) == pdTRUE)
    {
      // Normalize battery voltages between 0 - 100% - equation based on voltage curves
      float kayakBatteryPercentage = (2.37 * (kayakBatteryVolt * kayakBatteryVolt)) - (87.33 * kayakBatteryVolt) + 802.79;
      if(kayakBatteryPercentage > 100)
      {
        kayakBatteryPercentage = 100;
      }
      else if(kayakBatteryPercentage < 0)
      {
        kayakBatteryPercentage = 0;
      }

      float oarBatteryPercentage = (-114.3 * (oarbatteryVolt * oarbatteryVolt)) + (959.22 * oarbatteryVolt) - 1909.5;
      if(oarBatteryPercentage > 100)
      {
        oarBatteryPercentage = 100;
      }
      else if (oarBatteryPercentage < 0)
      {
        oarBatteryPercentage = 0;
      }

      uint32_t batteryPercentageReport;
      if(kayakBatteryPercentage > oarBatteryPercentage)
      {
        batteryPercentageReport = (uint32_t) oarBatteryPercentage;
      }
      else
      {
        batteryPercentageReport = (uint32_t) kayakBatteryPercentage;
      }

      // Add battery voltage to queue
      ledMsg.fAnnimation = eBatteryUpdate;
      ledMsg.fData = batteryPercentageReport;
      xQueueSend(ledAnnimationQueue, (void *)&ledMsg, 1);
    }

    // Serial.println(currTime - prevTime);
    // beginTime = millis();
    myDelayMs(50);    // execute task at 20Hz
    // taskTime = millis() - beginTime;
    // Serial.println(taskTime);
  }

  Serial.println("Task Monitor: Deleting");
  vTaskDelete( NULL );
}

static void LedDriverTask( void *pvParameters )
{
  LedMsg_t annimateMsg;  
  bool fault = false;
  bool connecting = true;
  uint32_t faultColor = strip.Color(255, 0, 0);
  int faultDelay = 6;

  bool autoMode = false;
  uint32_t speed = 0;
  uint32_t batteryPercent = 100;

  while(1)
  {    
    int faultCount = 0;
    // If faulted - loop until fault clears 
    while(fault)
    {
      if(faultCount % 40 == 0)
      {
        PulseAnnimation(faultColor, faultDelay);
      }

      // pop from queue until we get a connected annimation msg meaning the fault has cleared
      if(xQueueReceive(ledAnnimationQueue, (void *)&annimateMsg, 0) == pdTRUE)
      {
        // Serial.println(annimateMsg.fAnnimation);
        if(annimateMsg.fAnnimation == eConnected)
        {
          PulseAnnimation(annimateMsg.fColor, annimateMsg.fDelay);
          StartupAnnimation(50);
          fault = false;
        }
      }
      faultCount++;

      beginTime = millis();
      myDelayMs(500);
      taskTime = millis() - beginTime;
      Serial.println(taskTime);
    }

    while(connecting)
    {
      CirclingAnnimation(strip.Color(0, 255, 0), 50);

      // pop from queue until we get a connected or fault annimation msg meaning the fault has 
      if(xQueueReceive(ledAnnimationQueue, (void *)&annimateMsg, 0) == pdTRUE)
      {
        // Serial.println(annimateMsg.fAnnimation);
        if(annimateMsg.fAnnimation == eConnected)
        {
          PulseAnnimation(annimateMsg.fColor, annimateMsg.fDelay);
          StartupAnnimation(50);
          connecting = false;
          fault = false;
        }
        else if(annimateMsg.fAnnimation == eError)
        {
          connecting = false;
          fault = true;
        }
      }

      beginTime = millis();
      myDelayMs(500);
      taskTime = millis() - beginTime;
      Serial.println(taskTime);
    }

    // Read from annimation queue
    if(xQueueReceive(ledAnnimationQueue, (void *)&annimateMsg, 0) == pdTRUE)
    {
      // Serial.println(annimateMsg.fAnnimation);

      switch(annimateMsg.fAnnimation)
      {
        case eError :
          fault = true;
          connecting = false;
          break;
        case eConnecting :
          // Call circle annimate
          connecting = true;
          CirclingAnnimation(annimateMsg.fColor, annimateMsg.fDelay);
          break;
        // case eConnected :
        //   // Call flash annimate 
        //   connecting = false;
        //   PulseAnnimation(annimateMsg.fColor, annimateMsg.fDelay);
        //   StartupAnnimation(50);
        //   break;
        case eModeUpdate :
          // Call flash update twice
          autoMode != autoMode;
          PulseAnnimation(annimateMsg.fColor, annimateMsg.fDelay);
          PulseAnnimation(annimateMsg.fColor, annimateMsg.fDelay);
          SpeedUpdateAnnimation(speed);
          BatteryUpdateAnnimation(batteryPercent);
          break;
        case eSpeedUpdate :
          // Call speed annimate
          speed = annimateMsg.fData;
          SpeedUpdateAnnimation(speed);
          break;
        case eBatteryUpdate :
          // Call battery annimate
          batteryPercent = annimateMsg.fData;
          BatteryUpdateAnnimation(batteryPercent);
          break;
        default :
          break;
      }
    }
    myDelayMs(30);   // execute task at 60Hz
  }

  Serial.println("Task Monitor: Deleting");
  vTaskDelete( NULL );
}

static void LedPixelUpdaterTask( void *pvParameters )
{
  // Pixel map
  LedMap_t pixelMap;

  // Set default pixel map to connecting annimation sequence
  // memcpy(&pixelMap.fPixelColor[0][0], &connectSequenceZero, sizeof(connectSequenceZero));
  // memcpy(&pixelMap.fPixelColor[1][0], &connectSequenceOne, sizeof(connectSequenceOne));
  // memcpy(&pixelMap.fPixelColor[2][0], &connectSequenceTwo, sizeof(connectSequenceTwo));
  // memcpy(&pixelMap.fPixelColor[3][0], &connectSequenceThree, sizeof(connectSequenceThree));
  // memcpy(&pixelMap.fPixelColor[4][0], &connectSequenceFour, sizeof(connectSequenceFour));
  // memcpy(&pixelMap.fPixelColor[5][0], &connectSequenceFive, sizeof(connectSequenceFive));
  // memcpy(&pixelMap.fPixelColor[6][0], &connectSequenceSix, sizeof(connectSequenceSix));
  // memcpy(&pixelMap.fPixelColor[7][0], &connectSequenceSeven, sizeof(connectSequenceSeven));
  // memcpy(&pixelMap.fPixelColor[8][0], &connectSequenceEight, sizeof(connectSequenceEight));
  // memcpy(&pixelMap.fPixelColor[9][0], &connectSequenceNine, sizeof(connectSequenceNine));
  // memcpy(&pixelMap.fPixelColor[10][0], &connectSequenceTen, sizeof(connectSequenceTen));
  // memcpy(&pixelMap.fPixelColor[11][0], &connectSequenceEleven, sizeof(connectSequenceEleven));

  memcpy(&pixelMap.fPixelColor[0][0], &connectedSequenceZero, sizeof(connectedSequenceZero));
  memcpy(&pixelMap.fPixelColor[1][0], &connectedSequenceOne, sizeof(connectedSequenceOne));
  memcpy(&pixelMap.fPixelColor[2][0], &connectedSequenceTwo, sizeof(connectedSequenceTwo));
  memcpy(&pixelMap.fPixelColor[3][0], &connectedSequenceThree, sizeof(connectedSequenceThree));
  memcpy(&pixelMap.fPixelColor[4][0], &connectedSequenceFour, sizeof(connectedSequenceFour));
  memcpy(&pixelMap.fPixelColor[5][0], &connectedSequenceFive, sizeof(connectedSequenceFive));
  memcpy(&pixelMap.fPixelColor[6][0], &connectedSequenceSix, sizeof(connectedSequenceSix));
  memcpy(&pixelMap.fPixelColor[7][0], &connectedSequenceSeven, sizeof(connectedSequenceSeven));
  memcpy(&pixelMap.fPixelColor[8][0], &connectedSequenceEight, sizeof(connectedSequenceEight));
  memcpy(&pixelMap.fPixelColor[9][0], &connectedSequenceNine, sizeof(connectedSequenceNine));
  memcpy(&pixelMap.fPixelColor[10][0], &connectedSequenceTen, sizeof(connectedSequenceTen));
  memcpy(&pixelMap.fPixelColor[11][0], &connectedSequenceEleven, sizeof(connectedSequenceEleven));

  pixelMap.fDelay = 75;
  pixelMap.fNumCyclesBlock = 0;

  volatile int delayCount = 0;    // Counter to track annimation delay times
  volatile int cycleCount = 0;    // Counter to track 12 cycle counter
  volatile int taskDelay = 10;
  
  while(1)
  {
    // Serial.print("Cycle Count: ");
    // Serial.println(cycleCount);
    // Serial.print("Delay count: ");
    // Serial.println(delayCount);    
    if(pixelMap.fNumCyclesBlock <= 0)
    {
      // Only read in next annimation if pixel map unblocked
      if(xQueueReceive(ledPixelMapQueue, (void *)&pixelMap, 0) == pdTRUE)
      {
        // Read from pixel map queue and update annimation struct
        delayCount = 0;
        cycleCount = 0;
      }
    }

    if(cycleCount >= strip.numPixels())
    {
      cycleCount = 0;

      // Check if annimation is done blocking
      if(pixelMap.fNumCyclesBlock > 0)
      {
        pixelMap.fNumCyclesBlock--;
      }
      else
      {
        pixelMap.fNumCyclesBlock = 0;
      }
    }

    if((delayCount * taskDelay) >= pixelMap.fDelay)
    {
      // Serial.println("tock");
      // Set each pixel for the current frame
      for(int i = 0; i < strip.numPixels(); i++)
      {
        strip.setPixelColor(i, pixelMap.fPixelColor[cycleCount][i]);
        strip.show();
      }

      delayCount = 0;
      cycleCount++;
    }

    delayCount++;
    count = 0;
    myDelayMs(10);    // Execute task at 100Hz
  }

  Serial.println("Task Monitor: Deleting");
  vTaskDelete( NULL );
}

static void RfOutputTask( void *pvParameters )
{
  StateMsg_t stateDataOut;
  ImuDataType_t imuDataOut;

  RfOutputMsg_t outputMsg;

  while(1)
  {
    radio.setPayloadSize(sizeof(outputMsg)); 
    radio.stopListening();  // put radio in TX mode

    // Read in IMU data
    if (xQueueReceive(imuDataQueue, (void *)&imuDataOut, 0) == pdTRUE)
    {
      // Package IMU data into RF output struct
      memcpy(&outputMsg.fOutputImu, &imuDataOut, sizeof(imuDataOut));
    }

    if(xQueueReceive(rfOutMsgQueue, (void *)&stateDataOut, 0) == pdTRUE)
    {
      // Package state into RF output struct
      memcpy(&outputMsg.fOutputState, &stateDataOut, sizeof(stateDataOut));
    }

    // Send RF data out
    unsigned long start_timer = micros();                // start the timer
    bool report = radio.write(&outputMsg, sizeof(outputMsg));  // transmit & save the report
    unsigned long end_timer = micros();                  // end the timer

    // if (report) 
    // {
    //   Serial.print(F("Transmission successful! "));  // payload was delivered
    //   Serial.print(F("Time to transmit = "));
    //   Serial.print(end_timer - start_timer);  // print the timer result
    //   Serial.print(F(" us. Sent: "));
    // } 
    // else 
    // {
    //   Serial.println(F("Transmission failed or timed out"));  // payload was not delivered
    // }

    count = 0;
    myDelayMs(50);   // execute task at 20Hz
  }

  Serial.println("Task Monitor: Deleting");
  vTaskDelete( NULL );
}


void setup() 
{
  pinMode(POWER_HOLD, OUTPUT);
  digitalWrite(POWER_HOLD, HIGH);

  int serialResetCount = 0;
  Serial.begin(9600);
  while (!Serial)// && serialResetCount < 10) 
  {
    delay(10);     // will pause until serial console opens
    serialResetCount++;
  }
  Serial.println("Begin Setup");

  // SETUP RF24 RADIO
  radio.begin();
  radio.openReadingPipe(1, address[1]);
  radio.openWritingPipe(address[0]); 
  radio.setPALevel(RF24_PA_MIN);

  // SETUP Button
  pinMode(BUTTON_PIN, INPUT);

  // SETUP BNO
  if (!bno08x.begin_I2C()) 
  {
    Serial.println("Failed to find BNO08x chip");
    while (1) 
    {   
      delay(10); 
    }
  }
  Serial.println("BNO08x Found!");
  SetImuReports();

  // SETUP NEOPIXELS
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(25); // Set BRIGHTNESS to about 1/5 (max = 255)

  // Create queues
  rfInMsgQueue = xQueueCreate(msgQueueLength, sizeof(StatusMsg_t));
  imuDataQueue = xQueueCreate(msgQueueLength, sizeof(FullImuDataSet_t));
  kayakStatusQueue = xQueueCreate(msgQueueLength, sizeof(StatusMsg_t));
  buttonQueue = xQueueCreate(msgQueueLength, sizeof(uint8_t));
  currentStateQueue = xQueueCreate(msgQueueLength, sizeof(StateMsg_t));
  ledAnnimationQueue = xQueueCreate(msgQueueLength * 3, sizeof(LedMsg_t));
  oarBatteryQueue = xQueueCreate(msgQueueLength * 3, sizeof(float));
  rfOutMsgQueue = xQueueCreate(msgQueueLength, sizeof(StateMsg_t));
  ledPixelMapQueue = xQueueCreate(msgQueueLength, sizeof(LedMap_t));

  // Create tasks
  xTaskCreate(RfInputTask, "RF In", 256, NULL, tskIDLE_PRIORITY + 2, &Handle_RfInputTask);
  xTaskCreate(ImuInputTask, "IMU In", 256, NULL, tskIDLE_PRIORITY + 3, &Handle_RfInputTask);
  // xTaskCreate(ButtonInputTask, "Button In",  256, NULL, tskIDLE_PRIORITY + 4, &Handle_ButtonInputTask);
  // xTaskCreate(KayakStsManagerTask, "Kayak Status", 256, NULL, tskIDLE_PRIORITY + 5, &Handle_KayakStatusManagerTask);
  // xTaskCreate(StateManagerTask, "Kayak State", 256, NULL, tskIDLE_PRIORITY + 6, &Handle_StateManagerTask);
  // xTaskCreate(ProcessOutputsTask, "Process Outputs", 256, NULL, tskIDLE_PRIORITY + 7, &Handle_ProcessOutputsTask);
  // // xTaskCreate(LedDriverTask, "LED Driver", 256, NULL, tskIDLE_PRIORITY + 1, &Handle_LedDriverTask);
  // xTaskCreate(OarBatteryMonitorTask, "Battery Monitor", 256, NULL, tskIDLE_PRIORITY + 9, &Handle_OarBatteryMonitorTask);
  // xTaskCreate(RfOutputTask, "RF Out", 256, NULL, tskIDLE_PRIORITY + 8, &Handle_RfOutputTask);
  xTaskCreate(LedPixelUpdaterTask, "Pixel updater", 512, NULL, tskIDLE_PRIORITY + 1, &Handle_LedPixelUpdaterTask);

  Serial.println("");
  Serial.println("******************************");
  Serial.println("        Program start         ");
  Serial.println("******************************");
  Serial.flush();

  // Start the RTOS, this function will never return and will schedule the tasks.
  vTaskStartScheduler();

  // error scheduler failed to start
  // should never get here
  while(1)
  {
	  Serial.println("Scheduler Failed! \n");
	  Serial.flush();
	  delay(1000);
  }

}

void loop() 
{
  // put your main code here, to run repeatedly:
  Serial.println(count);
  count++;
}

// Support functions
void CirclingAnnimation(uint32_t color, int wait) 
{

  // Initialize first three pixels on
  strip.setPixelColor(0, color); 
  strip.setPixelColor(1, color);
  strip.setPixelColor(2, color);            

  for(int i=0; i<strip.numPixels(); i++) {      
    // Turn off first of three pixels
    strip.setPixelColor(i, strip.Color(0, 0, 0));             

    // Turn on next pixel in set of three
    if(i > strip.numPixels() - 4)
    {
      // Wrap around case (i = 9)
      int j = i + 3;
      if(j >= strip.numPixels())
      {
        j = j - strip.numPixels();
      }
      strip.setPixelColor(j, color);
    }
    else
    {
      strip.setPixelColor(i+3, color);
    }
    strip.show();                               //  Update strip to match
    myDelayMs(wait);                            //  Pause for a moment
  }
}

void PulseAnnimation(uint32_t color, int wait)
{
  uint8_t largestColorValue = 0;
  uint8_t red = (color >> 16) & 0xFF;
  uint8_t green = (color >> 8) & 0xFF;
  uint8_t blue = color & 0xFF;

  if(red > largestColorValue)
  {
    largestColorValue = red;
  }
  if(green > largestColorValue)
  {
    largestColorValue = green;
  }
  if(blue > largestColorValue)
  {
    largestColorValue = blue;
  }

  uint8_t leastCommonMultipleRed = red / largestColorValue;     
  uint8_t leastCommonMultipleGreen = green / largestColorValue;
  uint8_t leastCommonMultipleBlue = blue / largestColorValue;

  // Fade all pixels into chosen color
  for(int i = 0; i < largestColorValue / 4; i++)
  {
    uint32_t colorAnnimate = strip.Color((i * (leastCommonMultipleRed * 2)), (i * (leastCommonMultipleGreen * 2)), (i * (leastCommonMultipleBlue * 2)));
    strip.fill(colorAnnimate);
    strip.show();
    myDelayMs(wait);
  }

  // Turn all LEDs back to off
  for(int i = largestColorValue / 4; i >= 0; i--)
  {
    uint32_t colorAnnimate = strip.Color((i * (leastCommonMultipleRed * 2)), (i * (leastCommonMultipleGreen * 2)), (i * (leastCommonMultipleBlue* 2)));
    strip.fill(colorAnnimate);
    strip.show();
    myDelayMs(wait);  
  }
}

void StartupAnnimation(int wait)
{
  
  uint32_t batteryColorIndex[] = {strip.Color(255, 0, 0), strip.Color(255, 50, 0), strip.Color(255, 170, 0), strip.Color(200, 255, 0),
                                          strip.Color(75, 255, 0), strip.Color(0, 255, 0)};
  // Clear color
  strip.fill(strip.Color(0, 0, 0));
  strip.show();

  // Startup annimation will be ramp up on each half circle of pixel ring
  for(int i = 0; i < (strip.numPixels() / 2); i++)
  {
    // 6 pixels on each side - Speed side want all green / Battery side want red, orange, yellow, yellow, green, green ?
    
    // Set speed side
    strip.setPixelColor(i, strip.Color(120, 120, 255)); 

    // Set battery side
    strip.setPixelColor(strip.numPixels() - i - 1, batteryColorIndex[i]);

    strip.show();
    myDelayMs(wait);
  }

  // Ramp speed half back down since not yet moving
  for(int i = (strip.numPixels() / 2) - 1; i >= 0; i--)
  {
    strip.setPixelColor(i, strip.Color(0, 0, 0)); 

    strip.show();
    myDelayMs(wait);
  }
}

void BatteryUpdateAnnimation(uint32_t newBatteryPercentage)
{
  // Set battery LED index
  uint32_t batteryColorIndex[] = {strip.Color(255, 0, 0), strip.Color(255, 50, 0), strip.Color(255, 170, 0), strip.Color(200, 255, 0),
                                          strip.Color(75, 255, 0), strip.Color(0, 255, 0)};

  // 100% battery / 6 LEDs = 16.666 percent per LED
  float numLeds = round(newBatteryPercentage / (100.0 / ((float)strip.numPixels() / 2.0)));

  // Set / turn off LED colors
  for(int i = 0; i < (strip.numPixels() / 2); i++)
  {
    if (i < numLeds)
    {
      strip.setPixelColor(strip.numPixels() - i - 1, batteryColorIndex[i]);   // Turn on used LEDs
    }
    else
    {
      strip.setPixelColor(strip.numPixels() - i - 1, strip.Color(0, 0, 0));   // Turn off unused LEDs
    }
  }

  // Enable new LED sequence
  strip.show();
}

void SpeedUpdateAnnimation(uint32_t newSpeed)
{
  uint32_t speedColor = strip.Color(120, 120, 255); 

  // 6 LEDs / 3 speed states = 2
  int ledMultiplier = round((strip.numPixels() / 2) / 3);

  if(newSpeed == 0)
  {
    // Turn off all LEDs
    for(int i = 0; i < 6; i++)
    {
      strip.setPixelColor(i, strip.Color(0, 0, 0));
    }
  }
  else
  {
    // Set LED colors
    for(int i = 0; i < (newSpeed * ledMultiplier); i++)
    {
      // What happens if newSpeed = 0 so i = 0 (is 0 < 0)?
      strip.setPixelColor(i, speedColor); 
    }
  }
  strip.show();
}



