#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <FreeRTOS_SAMD21.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_NeoPixel.h>

#include "LedPixelMaps.hpp"
#include "TaskMetaData.hpp"

// Global flags
bool gMotorFaultFlag = false;
bool gComsTimeoutFlag = true;   // Initialize to true on boot until RF modules connect

// Define rf24 radio
const int rf24CE = 6;
const int rf24CSN = 5;
RF24 radio(rf24CE, rf24CSN, 4000000);
// const byte address[6] = "2Node";
uint8_t address[][6] = { "1Node", "2Node" };

// Define mutex for sharing radio resource between Rx and Tx threads
SemaphoreHandle_t radioSemaphore = NULL;

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

#define VBATPIN A0

// #define configUSE_IDLE_HOOK 1

// Initialize global queues
size_t msgQueueLength = 10;
static QueueHandle_t rfInMsgQueue;
static QueueHandle_t imuDataQueue;
static QueueHandle_t kayakStatusQueue;
static QueueHandle_t buttonQueue;
static QueueHandle_t currentStateQueue;
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

typedef struct 
{
  uint8_t fMessageIndex_1;
  StateMsg_t fOutputState;
  ImuDataType_t fOutputImuEuler;
  float fPad[3];
} RfOutputMsgFirstHalf_t;

typedef struct 
{
  uint8_t fMessageIndex_2;
  ImuAddReportsVector_t fOutputImuAddReports;
} RfOutputMsgSecondHalf_t;



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
TaskHandle_t Handle_ReadRfTask;
TaskHandle_t Handle_ReadImuTask;
TaskHandle_t Handle_ProcessOutputsTask;
TaskHandle_t Handle_ButtonInputTask;
TaskHandle_t Handle_RfOutputTask;
TaskHandle_t Handle_LedPixelUpdaterTask;

// Test tasks
TaskHandle_t Handle_DumpTaskMetaData;
TaskHandle_t Handle_LedPixelUpdaterTester;

// Global vars for tracking task meta data

TaskMetaData readRfMetaData(Handle_ReadRfTask);
TaskMetaData readImuMetaData(Handle_ReadRfTask);
TaskMetaData processOutputsMetaData(Handle_ProcessOutputsTask);
TaskMetaData buttonInputMetaData(Handle_ButtonInputTask);
TaskMetaData writeRfMetaData(Handle_RfOutputTask);
TaskMetaData ledDriverMetaData(Handle_LedPixelUpdaterTask);

// test tasks 
TaskMetaData diagTaskMetaData(Handle_DumpTaskMetaData);

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
static void ReadRfTask( void *pvParameters ) 
{
  // Struct to locally store incoming RF data
  StatusMsg_t incomingData;

  // Vars to track coms timeout to motor
  volatile TickType_t lastReceivedMsgTimeInTicks = xTaskGetTickCount();
  double comsDroppedTimeInMs = 3000.0;     // If lose signal for 3 seconds, signal re connecting annimation

  radioSemaphore = xSemaphoreCreateMutex();     // Create mutex
  
  while(1)
  {
    readRfMetaData.GetMetaData().UpdateTimestamp();   // Meta data tracks rate of task call
    readRfMetaData.GetExecutionTimer().Start();       // Execution timer tracks task execution time

    if( radioSemaphore != NULL )
    {
      /* See if we can obtain the semaphore.  If the semaphore is not
      available wait 5 ticks to see if it becomes free. */
      if( xSemaphoreTake( radioSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {
        radio.setPayloadSize(sizeof(incomingData)); 
        if (radio.available()) 
        {
          // Store incoming data to local buffer
          radio.read(&incomingData, sizeof(incomingData));

          // Update coms watchdog
          lastReceivedMsgTimeInTicks = xTaskGetTickCount();

          // Store relevant data to queue for output processing
          xQueueSend(kayakStatusQueue, (void *)&incomingData, 1);
        }

          xSemaphoreGive( radioSemaphore );
      }
      else
      {
          // Serial.println("Could not take mutex");
      }
    }

    // Check on coms watchdog
    if(((static_cast<double>(xTaskGetTickCount()) / static_cast<double>(portTICK_PERIOD_MS)) - 
                       (static_cast<double>(lastReceivedMsgTimeInTicks) / static_cast<double>(portTICK_PERIOD_MS))) > comsDroppedTimeInMs)
    {
      // Trigger coms fault - set global flag
      gComsTimeoutFlag = true;
    }
    else
    {
      // Clear flag
      gComsTimeoutFlag = false;
    }

    readRfMetaData.GetExecutionTimer().Stop();

    myDelayMs(100);    // execute task at 20Hz
  }

  Serial.println("Task Monitor: Deleting");
  vTaskDelete( NULL );
}

static void ReadImuTask( void *pvParameters ) 
{
  sh2_SensorValue_t sensorData;   // local variables to cpature reported data
  FullImuDataSet_t fullImuDataSet;

  while(1)
  {
    readImuMetaData.GetMetaData().UpdateTimestamp();   // Meta data tracks rate of task call
    readImuMetaData.GetExecutionTimer().Start();       // Execution timer tracks task execution time

    // Check for IMU reset
    if (bno08x.wasReset()) 
    {
      Serial.print("sensor was reset ");
      SetImuReports();
    }

    // Read incoming sensor data
    if (bno08x.getSensorEvent(&sensorData)) 
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

    readImuMetaData.GetExecutionTimer().Stop();

    myDelayMs(50);    // execute task at 20Hz
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

  // Initialize button state message
  StateMsg_t currButtonState;
  currButtonState.fAutoMode = false;
  currButtonState.fSpeed = 0;

  volatile TickType_t lastDebounceTimeInMs = 0;   // the last time the output pin was toggled
  volatile TickType_t debounceDelayInMs = 20;     // the debounce time; increase if the output flickers
  volatile TickType_t doubleClickTimeInMs = 0;
  volatile TickType_t doubleClickDelayInMs = 300; // the double click time frame, lower if double click seem laggy


  while(1)
  {

    buttonInputMetaData.GetMetaData().UpdateTimestamp();   // Meta data tracks rate of task call
    buttonInputMetaData.GetExecutionTimer().Start();       // Execution timer tracks task execution time

    int reading = digitalRead(BUTTON_PIN);

    // If the switch changed, due to noise or pressing:
    if (reading != lastButtonState) 
    {
      // reset the debouncing timer
      lastDebounceTimeInMs = xTaskGetTickCount() / portTICK_PERIOD_MS;
    }

    if (((xTaskGetTickCount() / portTICK_PERIOD_MS) - lastDebounceTimeInMs) > debounceDelayInMs) 
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
          doubleClickTimeInMs = xTaskGetTickCount() / portTICK_PERIOD_MS;   // Capture time of first button state change
        }
      }
    }

    lastButtonState = reading;

    if(((xTaskGetTickCount() / portTICK_PERIOD_MS) - doubleClickTimeInMs) > doubleClickDelayInMs)
    {
      if(buttonStateChangeCount > 2)
      {
        // Found double click
        Serial.println("DOUBLE");
        buttonReport = 2;

        // Update the current state
        currButtonState.fAutoMode = !currButtonState.fAutoMode;

        xQueueSend(buttonQueue, (void*)&buttonReport, 1);
      }
      else if(buttonStateChangeCount > 1)
      {
        // Found single click
        Serial.println("SINGLE");
        buttonReport = 1;

        // Update the current state
        if(currButtonState.fSpeed == 3)
        {
          currButtonState.fSpeed = 0;
        }
        else
        {
          currButtonState.fSpeed++;
        }

        xQueueSend(buttonQueue, (void*)&buttonReport, 1);
      }

      // If coms timeout or motor fault, latch all to defaults
      if(gMotorFaultFlag || gComsTimeoutFlag)
      {
        currButtonState.fSpeed = 0;
        currButtonState.fAutoMode = false;
      }

      // Send current state to output proceesor
      xQueueSend(currentStateQueue, (void *)&currButtonState, 1);

      // Reset local flags
      buttonStateChangeCount = 0;
      buttonReport = 0;
    }

    buttonInputMetaData.GetExecutionTimer().Stop();

    myDelayMs(50);   // execute task at 20Hz
  }

  Serial.println("Task Monitor: Deleting");
  vTaskDelete( NULL );
}

static void ProcessOutputsTask( void *pvParameters ) 
{
  // bool fault = false;
  // bool startup = true;
  bool startupConnected = false;

  bool autoMode = false;
  uint32_t speed = 0;

  StatusMsg_t kayakStatusMsg;
  StateMsg_t stateMsg;
  LedMap_t ledMsg;

  float kayakBatteryVolt = 26;
  float oarbatteryVolt = 4.2;
  uint32_t batteryPercentageReport = 100;
  uint32_t prevBatteryPercentage = 99;    // Set prev percentage 99 to force an animation update

  const TickType_t xTicksToWait = 5 / portTICK_PERIOD_MS;

  Serial.println("Connecting");

  while(1)
  {

    processOutputsMetaData.GetMetaData().UpdateTimestamp();   // Meta data tracks rate of task call
    processOutputsMetaData.GetExecutionTimer().Start();       // Execution timer tracks task execution time

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
          Serial.println("FAULT");
          LoadErrorAnnimation(ledMsg, 75, 0);  // Set pixel map

          // Add to led queue
          xQueueSend(ledPixelMapQueue, (void *)&ledMsg, 1);
          fault = true;
          break;

        // Then check for fault cleared
        case eFaultCleared :
          LoadConnectedAnnimation(ledMsg, 75, 1);  // Set pixel map

          // Add to led queue
          xQueueSend(ledPixelMapQueue, (void *)&ledMsg, 1);
          fault = false;
          break;

        // Then check for battery status
        case eBatteryVolt :
          // Package kayak battery voltage for output processor
          kayakBatteryVolt = kayakStatusMsg.fStatusData;
          if(startup)
          {
            if(!startupConnected)
            {
              // First load connnected annimation before startup annimation
              LoadConnectedAnnimation(ledMsg, 75, 1);
              Serial.println("Connected");

              xQueueSend(ledPixelMapQueue, (void *)&ledMsg, 1);
              startupConnected = true;
            }
            else
            {
              // Now load startup annimation after connected annimation
              LoadStartupAnnimation(ledMsg, 80, 1);
              Serial.println("Startup");

              xQueueSend(ledPixelMapQueue, (void *)&ledMsg, 1);
              startup = false;    // Signal output processor that startup is over

            }

          }
          break;
        
        case eStartup :
          startup = true;
          startupConnected = false;
          LoadConnectingAnnimation(ledMsg, 50, 1);

          // Add to led queue
          xQueueSend(ledPixelMapQueue, (void *)&ledMsg, 1);
          break;

        default :
          Serial.println("Did not recieve valid RF message");
          break;
      }
    }

    //  Read the batteries
    float oarBatteryVoltage = analogRead(VBATPIN);
    oarBatteryVoltage *= 2;        // Account for voltage divider
    oarBatteryVoltage *= 3.3;      // Multiply by 3.3V - reference voltage
    oarBatteryVoltage /= 1024;     // convert to voltage

    // Normalize battery voltages between 0 - 100% - equation based on voltage curves
    float kayakBatteryPercentage = (1.0234 * (kayakBatteryVolt * kayakBatteryVolt * kayakBatteryVolt)) - (69.144 * (kayakBatteryVolt * kayakBatteryVolt)) 
                                                  + (1556 * kayakBatteryVolt) - 11654;
    if(kayakBatteryPercentage > 100)
    {
      kayakBatteryPercentage = 100;
    }
    else if(kayakBatteryPercentage < 0)
    {
      kayakBatteryPercentage = 0;
    }

    float oarBatteryPercentage = (-114.3 * (oarBatteryVoltage * oarBatteryVoltage)) + (959.22 * oarBatteryVoltage) - 1909.5;
    if(oarBatteryPercentage > 100)
    {
      oarBatteryPercentage = 100;
    }
    else if (oarBatteryPercentage < 0)
    {
      oarBatteryPercentage = 0;
    }

    if(kayakBatteryPercentage > oarBatteryPercentage)
    {
      batteryPercentageReport = (uint32_t) oarBatteryPercentage;
    }
    else
    {
      batteryPercentageReport = (uint32_t) kayakBatteryPercentage;
    }

    if(!fault && !startup && batteryPercentageReport != prevBatteryPercentage)
    {
      // Only send new battery status if battery level has changed
      LoadGaugeUpdateAnnimation(ledMsg, 200, 0, speed, batteryPercentageReport);
      // Serial.println("Sending battery Update to LED");

      xQueueSend(ledPixelMapQueue, (void *)&ledMsg, 1);
      prevBatteryPercentage = batteryPercentageReport;
    }

    // Next check current state and report to LED driver / RF out any changes
    if(xQueueReceive(currentStateQueue, (void *)&stateMsg, 0) == pdTRUE)
    {
      if((stateMsg.fAutoMode && !autoMode) || (!stateMsg.fAutoMode && autoMode))
      {
        // Toggle autoMode
        autoMode = stateMsg.fAutoMode;

        if(!fault && !startup)
        {
          // Only report to LED driver / RF output if we arn't currently faulted
          LoadModeChangeAnnimation(ledMsg, 75, 2);
          Serial.print("Mode: ");
          Serial.println(stateMsg.fAutoMode);
          xQueueSend(ledPixelMapQueue, (void *)&ledMsg, 1);
          xQueueSend(rfOutMsgQueue, (void *)&stateMsg, 1);
        }

      }
      else if(stateMsg.fSpeed != speed)
      {
        speed = stateMsg.fSpeed;                            // Update local speed checker variable

        if(!fault && !startup)
        {
          // Only report to LED driver / RF output if we arn't currently faulted
          LoadGaugeUpdateAnnimation(ledMsg, 200, 0, speed, batteryPercentageReport);
          Serial.print("Speed: ");
          Serial.println(stateMsg.fSpeed);
          xQueueSend(ledPixelMapQueue, (void *)&ledMsg, 1);
          xQueueSend(rfOutMsgQueue, (void *)&stateMsg, 1);
        }
      }
    }

    processOutputsMetaData.GetExecutionTimer().Stop();

    myDelayMs(100);    // execute task at 20Hz

  }

  Serial.println("Task Monitor: Deleting");
  vTaskDelete( NULL );
}

static void LedPixelUpdaterTask( void *pvParameters )
{
  // Pixel map
  LedMap_t pixelMap;

  // Set default pixel map to connecting annimation sequence
  LoadConnectingAnnimation(pixelMap, 50, 1);

  // Mechanism to track when next annimation update should be  
  volatile TickType_t currPixelTimeout = xTaskGetTickCount();
  volatile TickType_t nextPixelTimeout = xTaskGetTickCount() + (pixelMap.fDelayInMs / portTICK_PERIOD_MS);  

  volatile int cycleCount = 0;    // Counter to track 12 cycle counter
  volatile int taskDelay = 25;
  
  while(1)
  {

    ledDriverMetaData.GetMetaData().UpdateTimestamp();   // Meta data tracks rate of task call
    ledDriverMetaData.GetExecutionTimer().Start();       // Execution timer tracks task execution time

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
        nextPixelTimeout = xTaskGetTickCount() + (pixelMap.fDelayInMs / portTICK_PERIOD_MS);    // Reset next annimation timeout
        cycleCount = 0;
        Serial.println("New pixel map command");
      }
    }

    // Update Cycle counters
    if(cycleCount >= strip.numPixels())
    {
      Serial.println(cycleCount);
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

    // Update strip if time 
    if(currPixelTimeout = xTaskGetTickCount() >= nextPixelTimeout)
    {
      // Serial.println("tock");
      // Set each pixel for the current frame
      for(int i = 0; i < strip.numPixels(); i++)
      {
        strip.setPixelColor(i, pixelMap.fPixelColor[cycleCount][i]);
        strip.show();
      }

      nextPixelTimeout = xTaskGetTickCount() + (pixelMap.fDelayInMs / portTICK_PERIOD_MS);  // Reset next annimation timeout
      cycleCount++;
    }

    ledDriverMetaData.GetExecutionTimer().Stop();

    myDelayMs(taskDelay);    // Execute task at 100Hz

  }

  Serial.println("Task Monitor: Deleting");
  vTaskDelete( NULL );
}

static void LedPixelUpdaterTester( void *pvParameters )
{
  // Mechanisms to track when to push Pixel maps into queues
  uint32_t queueInsertDelayInMs = 500;   // Delay between pushing each pixel map to queue
  uint32_t loadNextSetDelayInMs = 7000;   // Delay between loading full next set of pixel map transitions - should be greater than queueInsertDelay * totalAnimations

  volatile TickType_t nextQueueInsertTimeout = xTaskGetTickCount() + (queueInsertDelayInMs / portTICK_PERIOD_MS); 
  volatile TickType_t nextSetLoadTimeout = xTaskGetTickCount() + (loadNextSetDelayInMs / portTICK_PERIOD_MS); 

  // Load pixel maps
  LoadConnectingAnnimation(pixelMapList[0], 50, 1);
  LoadConnectedAnnimation(pixelMapList[1], 75, 1);
  LoadErrorAnnimation(pixelMapList[2], 75, 0);
  LoadModeChangeAnnimation(pixelMapList[3], 75, 2);
  LoadStartupAnnimation(pixelMapList[4], 80, 1);
  LoadGaugeUpdateAnnimation(pixelMapList[5], 200, 0, 1, 50);

  // index to track map transitions
  uint32_t index = 0;
  bool insertFlag = false;

  while(1)
  {
    // If ready for next set of pixel map transitions
    if(xTaskGetTickCount() >= nextSetLoadTimeout)
    {
      // Serial.println("Set Insert");
      insertFlag = true;

      // Reset load timeout
      nextSetLoadTimeout = xTaskGetTickCount() + (loadNextSetDelayInMs / portTICK_PERIOD_MS); 
    }

    // If ready to insert, insert next index once delay has been met
    if(insertFlag)
    {
      if(xTaskGetTickCount() >= nextQueueInsertTimeout)
      {
        // Insert
        // Serial.println("Queue Insert");
        xQueueSend(ledPixelMapQueue, (void *)&pixelMapList[index], 1);

        index++;

        // Reset next queue timeout
        nextQueueInsertTimeout = xTaskGetTickCount() + (queueInsertDelayInMs / portTICK_PERIOD_MS); 
      }

      // If we have inserted full queue, lower flag
      if(index >= totalAnimations)
      {
        index = 0;
        insertFlag = false;
      }
    }

    // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    // Serial.println(uxHighWaterMark);
    
    // count = 0;

    // beginTime = millis();
    myDelayMs(50);   // execute task at 10 Hz
    // taskTime = millis() - beginTime;
    // Serial.println(taskTime);
  }

  Serial.println("Task Monitor: Deleting");
  vTaskDelete( NULL );
}

static void RfOutputTask( void *pvParameters )
{
  StateMsg_t stateDataOut;

  // Initialize state data
  stateDataOut.fAutoMode = false;
  stateDataOut.fSpeed = 0;

  FullImuDataSet_t imuDataOut;
  RfOutputMsgFirstHalf_t outputMsgFirstHalf;
  RfOutputMsgSecondHalf_t outputMsgSecondHalf;

  // initialize IMU data
  imuDataOut.fEulerData.fRoll = 0.0;
  imuDataOut.fEulerData.fPitch = 0.0;
  imuDataOut.fEulerData.fYaw = 0.0;
  imuDataOut.fAddReportsVect.fX.fAccel = 0.0;
  imuDataOut.fAddReportsVect.fX.fGyro = 0.0;
  imuDataOut.fAddReportsVect.fY.fAccel = 0.0;
  imuDataOut.fAddReportsVect.fY.fGyro = 0.0;
  imuDataOut.fAddReportsVect.fZ.fAccel = 0.0;
  imuDataOut.fAddReportsVect.fZ.fGyro = 0.0;

  outputMsgFirstHalf.fMessageIndex_1 = 1;
  outputMsgFirstHalf.fOutputState = stateDataOut;
  outputMsgFirstHalf.fOutputImuEuler = imuDataOut.fEulerData;

  outputMsgSecondHalf.fMessageIndex_2 = 2;
  outputMsgSecondHalf.fOutputImuAddReports = imuDataOut.fAddReportsVect;


  while(1)
  {
    writeRfMetaData.GetMetaData().UpdateTimestamp();   // Meta data tracks rate of task call
    writeRfMetaData.GetExecutionTimer().Start();       // Execution timer tracks task execution time

    // Read in IMU / state data
    if (xQueueReceive(imuDataQueue, (void *)&imuDataOut, 0) == pdTRUE)
    {
      // Package IMU data into RF output struct
      memcpy(&outputMsgFirstHalf.fOutputImuEuler, &imuDataOut.fEulerData, sizeof(imuDataOut.fEulerData));
      memcpy(&outputMsgSecondHalf.fOutputImuAddReports, &imuDataOut.fAddReportsVect, sizeof(imuDataOut.fAddReportsVect));
    }
    if(xQueueReceive(rfOutMsgQueue, (void *)&stateDataOut, 0) == pdTRUE)
    {
      // Package state into RF output struct
      memcpy(&outputMsgFirstHalf.fOutputState, &stateDataOut, sizeof(stateDataOut));
      Serial.println(outputMsgFirstHalf.fOutputState.fAutoMode);
      Serial.println(outputMsgFirstHalf.fOutputState.fSpeed);
    }

    if( radioSemaphore != NULL )
    {
      if( xSemaphoreTake( radioSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {
        // radio.setPayloadSize(sizeof(outputMsg)); 
        radio.setPayloadSize(sizeof(outputMsgFirstHalf)); 
        radio.stopListening();  // put radio in TX mode

        // Send RF data out
        unsigned long start_timer = millis();                // start the timer
        bool report = radio.write(&outputMsgFirstHalf, sizeof(outputMsgFirstHalf));  // transmit & save the report

        radio.setPayloadSize(sizeof(outputMsgSecondHalf));
        report = radio.write(&outputMsgSecondHalf, sizeof(outputMsgSecondHalf));  // transmit & save the report
        unsigned long end_timer = millis();                  // end the timer

        // Serial.println(end_timer - start_timer);

        radio.startListening();  // put radio back in Rx mode

        xSemaphoreGive( radioSemaphore );
      }
    }
    else
    {
      // Serial.println("Could not take mutex");
    }

    // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    // Serial.println(uxHighWaterMark);
    
    // count = 0;

    writeRfMetaData.GetExecutionTimer().Stop(); 

    myDelayMs(50);   // execute task at 20Hz

  }

  Serial.println("Task Monitor: Deleting");
  vTaskDelete( NULL );
}

static void DumpTaskMetaDataTask( void *pvParameters )
{
  while(1)
  {

    diagTaskMetaData.GetMetaData().UpdateTimestamp();   // Meta data tracks rate of task call
    diagTaskMetaData.GetExecutionTimer().Start();       // Execution timer tracks task execution time

    // Print read rf diag
    Serial.print("ReadRF max call rate: ");
    Serial.println(readRfMetaData.GetMetaData().GetUpdateRateStats().GetMaxTimeInTicks());
    Serial.print("ReadRF max execution rate: ");
    Serial.println(readRfMetaData.GetTaskExecutionTimeStats().GetMaxTimeInMs());
    Serial.print("ReadRF stack use: ");
    Serial.println(readRfMetaData.GetStackUsageHighWaterMark());
    Serial.println();

    // Print read IMU diag
    Serial.print("ReadImu max call rate: ");
    Serial.println(readImuMetaData.GetMetaData().GetUpdateRateStats().GetMaxTimeInMs());
    Serial.print("ReadImu max execution rate: ");
    Serial.println(readImuMetaData.GetTaskExecutionTimeStats().GetMaxTimeInMs());
    Serial.print("ReadImu stack use: ");
    Serial.println(readImuMetaData.GetStackUsageHighWaterMark());
    Serial.println();


    // Print output processor diag
    Serial.print("outProcessor max call rate: ");
    Serial.println(processOutputsMetaData.GetMetaData().GetUpdateRateStats().GetMaxTimeInMs());
    Serial.print("outProcessor max execution rate: ");
    Serial.println(processOutputsMetaData.GetTaskExecutionTimeStats().GetMaxTimeInMs());
    Serial.print("outProcessor stack use: ");
    Serial.println(processOutputsMetaData.GetStackUsageHighWaterMark());
    Serial.println();


    // Print button input diag
    Serial.print("buttonIn max call rate: ");
    Serial.println(buttonInputMetaData.GetMetaData().GetUpdateRateStats().GetMaxTimeInMs());
    Serial.print("buttonIn max execution rate: ");
    Serial.println(buttonInputMetaData.GetTaskExecutionTimeStats().GetMaxTimeInMs());
    Serial.print("buttonIn stack use: ");
    Serial.println(buttonInputMetaData.GetStackUsageHighWaterMark());
    Serial.println();


    // Print write RF diag
    Serial.print("WriteRf max call rate: ");
    Serial.println(writeRfMetaData.GetMetaData().GetUpdateRateStats().GetMaxTimeInMs());
    Serial.print("WriteRf max execution rate: ");
    Serial.println(writeRfMetaData.GetTaskExecutionTimeStats().GetMaxTimeInMs());
    Serial.print("WriteRf stack use: ");
    Serial.println(writeRfMetaData.GetStackUsageHighWaterMark());
    Serial.println();


    // Print Led driver diag
    Serial.print("LedDriver max call rate: ");
    Serial.println(ledDriverMetaData.GetMetaData().GetUpdateRateStats().GetMaxTimeInMs());
    Serial.print("LedDriver max execution rate: ");
    Serial.println(ledDriverMetaData.GetTaskExecutionTimeStats().GetMaxTimeInMs());
    Serial.print("LedDriver stack use: ");
    Serial.println(ledDriverMetaData.GetStackUsageHighWaterMark());
    Serial.println();

    // Print task diag dump diag
    Serial.print("DiagDump max call rate: ");
    Serial.println(diagTaskMetaData.GetMetaData().GetUpdateRateStats().GetMaxTimeInMs());
    Serial.print("DiagDump max execution rate: ");
    Serial.println(diagTaskMetaData.GetTaskExecutionTimeStats().GetMaxTimeInMs());
    Serial.print("DiagDump stack use: ");
    Serial.println(diagTaskMetaData.GetStackUsageHighWaterMark());
    Serial.println();

    Serial.println();

    diagTaskMetaData.GetExecutionTimer().Stop(); 

    myDelayMs(2000);   // execute task at .5Hz
  }

  Serial.println("Task Monitor: Deleting");
  vTaskDelete( NULL );
}


void setup() 
{
  pinMode(POWER_HOLD, OUTPUT);
  digitalWrite(POWER_HOLD, HIGH);

  int serialResetCount = 0;
  Serial.begin(115200);
  while (!Serial && serialResetCount < 100) 
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
  strip.setBrightness(45); // Set BRIGHTNESS to about 1/5 (max = 255)

  // Create queues
  rfInMsgQueue = xQueueCreate(msgQueueLength, sizeof(StatusMsg_t));
  imuDataQueue = xQueueCreate(msgQueueLength, sizeof(FullImuDataSet_t));
  kayakStatusQueue = xQueueCreate(msgQueueLength, sizeof(StatusMsg_t));
  currentStateQueue = xQueueCreate(msgQueueLength, sizeof(StateMsg_t));
  rfOutMsgQueue = xQueueCreate(msgQueueLength, sizeof(StateMsg_t));
  ledPixelMapQueue = xQueueCreate(msgQueueLength, sizeof(LedMap_t));

  // Create tasks                                                                                                         // Total RAM usage: ~4KB RAM
  xTaskCreate(ReadRfTask, "Read in", 90, NULL, tskIDLE_PRIORITY + 5, &Handle_ReadRfTask);                                 // 352 bytes RAM
  xTaskCreate(ReadImuTask, "Read in", 187, NULL, tskIDLE_PRIORITY + 6, &Handle_ReadImuTask);                              // 736 bytes
  xTaskCreate(ButtonInputTask, "Button In",  75, NULL, tskIDLE_PRIORITY + 7, &Handle_ButtonInputTask);                    // 336 bytes
  xTaskCreate(ProcessOutputsTask, "Process Outputs", 234, NULL, tskIDLE_PRIORITY + 3, &Handle_ProcessOutputsTask);        // 936 bytes
  xTaskCreate(RfOutputTask, "RF Out", 125, NULL, tskIDLE_PRIORITY + 6, &Handle_RfOutputTask);                             // 432 bytes
  xTaskCreate(LedPixelUpdaterTask, "Pixel updater", 250, NULL, tskIDLE_PRIORITY + 5, &Handle_LedPixelUpdaterTask);        // 928 bytes

  // Test tasks
  xTaskCreate(DumpTaskMetaDataTask, "Diagnostics Dump", 100, NULL, tskIDLE_PRIORITY + 1, &Handle_LedPixelUpdaterTester);
  //  xTaskCreate(LedPixelUpdaterTester, "Pixel tester", 500, NULL, tskIDLE_PRIORITY + 5, &Handle_LedPixelUpdaterTester);
  

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
}

// Support annimation functions
void LoadConnectingAnnimation(LedMap_t &pixelMap, uint32_t delayTime, int blockingCycles)
{
  memcpy(&pixelMap.fPixelColor[0][0], &connectSequenceZero, sizeof(connectSequenceZero));
  memcpy(&pixelMap.fPixelColor[1][0], &connectSequenceOne, sizeof(connectSequenceOne));
  memcpy(&pixelMap.fPixelColor[2][0], &connectSequenceTwo, sizeof(connectSequenceTwo));
  memcpy(&pixelMap.fPixelColor[3][0], &connectSequenceThree, sizeof(connectSequenceThree));
  memcpy(&pixelMap.fPixelColor[4][0], &connectSequenceFour, sizeof(connectSequenceFour));
  memcpy(&pixelMap.fPixelColor[5][0], &connectSequenceFive, sizeof(connectSequenceFive));
  memcpy(&pixelMap.fPixelColor[6][0], &connectSequenceSix, sizeof(connectSequenceSix));
  memcpy(&pixelMap.fPixelColor[7][0], &connectSequenceSeven, sizeof(connectSequenceSeven));
  memcpy(&pixelMap.fPixelColor[8][0], &connectSequenceEight, sizeof(connectSequenceEight));
  memcpy(&pixelMap.fPixelColor[9][0], &connectSequenceNine, sizeof(connectSequenceNine));
  memcpy(&pixelMap.fPixelColor[10][0], &connectSequenceTen, sizeof(connectSequenceTen));
  memcpy(&pixelMap.fPixelColor[11][0], &connectSequenceEleven, sizeof(connectSequenceEleven));

  pixelMap.fDelayInMs = delayTime; // 50
  pixelMap.fNumCyclesBlock = blockingCycles;  // 1
}

void LoadConnectedAnnimation(LedMap_t &pixelMap, uint32_t delayTime, int blockingCycles)
{
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

  pixelMap.fDelayInMs = delayTime;  //75 
  pixelMap.fNumCyclesBlock = blockingCycles; // 1
}

void LoadErrorAnnimation(LedMap_t &pixelMap, uint32_t delayTime, int blockingCycles)
{
  memcpy(&pixelMap.fPixelColor[0][0], &errorSequenceZero, sizeof(errorSequenceZero));
  memcpy(&pixelMap.fPixelColor[1][0], &errorSequenceOne, sizeof(errorSequenceOne));
  memcpy(&pixelMap.fPixelColor[2][0], &errorSequenceTwo, sizeof(errorSequenceTwo));
  memcpy(&pixelMap.fPixelColor[3][0], &errorSequenceThree, sizeof(errorSequenceThree));
  memcpy(&pixelMap.fPixelColor[4][0], &errorSequenceFour, sizeof(errorSequenceFour));
  memcpy(&pixelMap.fPixelColor[5][0], &errorSequenceFive, sizeof(errorSequenceFive));
  memcpy(&pixelMap.fPixelColor[6][0], &errorSequenceSix, sizeof(errorSequenceSix));
  memcpy(&pixelMap.fPixelColor[7][0], &errorSequenceSeven, sizeof(errorSequenceSeven));
  memcpy(&pixelMap.fPixelColor[8][0], &errorSequenceEight, sizeof(errorSequenceEight));
  memcpy(&pixelMap.fPixelColor[9][0], &errorSequenceNine, sizeof(errorSequenceNine));
  memcpy(&pixelMap.fPixelColor[10][0], &errorSequenceTen, sizeof(errorSequenceTen));
  memcpy(&pixelMap.fPixelColor[11][0], &errorSequenceEleven, sizeof(errorSequenceEleven));

  pixelMap.fDelayInMs = delayTime;  // 75
  pixelMap.fNumCyclesBlock = blockingCycles; //0
}

void LoadModeChangeAnnimation(LedMap_t &pixelMap, uint32_t delayTime, int blockingCycles)
{
  memcpy(&pixelMap.fPixelColor[0][0], &modeChangeSequenceZero, sizeof(modeChangeSequenceZero));
  memcpy(&pixelMap.fPixelColor[1][0], &modeChangeSequenceOne, sizeof(modeChangeSequenceOne));
  memcpy(&pixelMap.fPixelColor[2][0], &modeChangeSequenceTwo, sizeof(modeChangeSequenceTwo));
  memcpy(&pixelMap.fPixelColor[3][0], &modeChangeSequenceThree, sizeof(modeChangeSequenceThree));
  memcpy(&pixelMap.fPixelColor[4][0], &modeChangeSequenceFour, sizeof(modeChangeSequenceFour));
  memcpy(&pixelMap.fPixelColor[5][0], &modeChangeSequenceFive, sizeof(modeChangeSequenceFive));
  memcpy(&pixelMap.fPixelColor[6][0], &modeChangeSequenceSix, sizeof(modeChangeSequenceSix));
  memcpy(&pixelMap.fPixelColor[7][0], &modeChangeSequenceSeven, sizeof(modeChangeSequenceSeven));
  memcpy(&pixelMap.fPixelColor[8][0], &modeChangeSequenceEight, sizeof(modeChangeSequenceEight));
  memcpy(&pixelMap.fPixelColor[9][0], &modeChangeSequenceNine, sizeof(modeChangeSequenceNine));
  memcpy(&pixelMap.fPixelColor[10][0], &modeChangeSequenceTen, sizeof(modeChangeSequenceTen));
  memcpy(&pixelMap.fPixelColor[11][0], &modeChangeSequenceEleven, sizeof(modeChangeSequenceEleven));

  pixelMap.fDelayInMs = delayTime;  // 75
  pixelMap.fNumCyclesBlock = blockingCycles;  // 2
}

void LoadStartupAnnimation(LedMap_t &pixelMap, uint32_t delayTime, int blockingCycles)
{
  memcpy(&pixelMap.fPixelColor[0][0], &startSequenceZero, sizeof(startSequenceZero));
  memcpy(&pixelMap.fPixelColor[1][0], &startSequenceOne, sizeof(startSequenceOne));
  memcpy(&pixelMap.fPixelColor[2][0], &startSequenceTwo, sizeof(startSequenceTwo));
  memcpy(&pixelMap.fPixelColor[3][0], &startSequenceThree, sizeof(startSequenceThree));
  memcpy(&pixelMap.fPixelColor[4][0], &startSequenceFour, sizeof(startSequenceFour));
  memcpy(&pixelMap.fPixelColor[5][0], &startSequenceFive, sizeof(startSequenceFive));
  memcpy(&pixelMap.fPixelColor[6][0], &startSequenceSix, sizeof(startSequenceSix));
  memcpy(&pixelMap.fPixelColor[7][0], &startSequenceSeven, sizeof(startSequenceSeven));
  memcpy(&pixelMap.fPixelColor[8][0], &startSequenceEight, sizeof(startSequenceEight));
  memcpy(&pixelMap.fPixelColor[9][0], &startSequenceNine, sizeof(startSequenceNine));
  memcpy(&pixelMap.fPixelColor[10][0], &startSequenceTen, sizeof(startSequenceTen));
  memcpy(&pixelMap.fPixelColor[11][0], &startSequenceEleven, sizeof(startSequenceEleven));

  pixelMap.fDelayInMs = delayTime;  // 80
  pixelMap.fNumCyclesBlock = blockingCycles; // 1
}

// Speed settings 0 - 3, battery in percentage from 0 - 100%
void LoadGaugeUpdateAnnimation(LedMap_t &pixelMap, uint32_t delayTime, int blockingCycles, uint32_t speed, uint32_t batteryPercentage)
{
  uint32_t speedColor = strip.Color(120, 120, 255); 
  int ledMultiplier = round((strip.numPixels() / 2) / 3);

  uint32_t speedPixelsOn = speed * ledMultiplier;     // Determines how many speed pixels should be on to represent new speed
  // Serial.print("num speed pixels: ");
  // Serial.println(speedPixelsOn);

  uint32_t batteryPixelsOn = round(batteryPercentage / (100.0 / ((float)strip.numPixels() / 2.0)));
  // Serial.println("Num bat pixels on:");
  // Serial.println(batteryPercentage);
  // Serial.println(batteryPixelsOn);

  int speedCount = 0;
  int batteryCount = 0;
  for(int i = (strip.numPixels() / 2) - 1; i >= 0; i--)
  {
    if(speedCount < speedPixelsOn)
    {
      // Keep turning on speed LEDs until reached number of on
      pixelMap.fPixelColor[0][i] = speedColor;
    }
    else
    {
      pixelMap.fPixelColor[0][i] = 0;  // Turn off unused LEDs
    }
    speedCount++;
  }

  for(int i = strip.numPixels() / 2; i < strip.numPixels(); i++)
  {
    if(batteryCount < batteryPixelsOn)
    {
      // Keep turning on speed LEDs until reached number of on
      pixelMap.fPixelColor[0][i] = batteryColorIndex[batteryCount];
    }
    else
    {
      pixelMap.fPixelColor[0][i] = 0;  // Turn off unused LEDs
    }
    batteryCount++;
  }

  // Copy oth index with correct color setting to rest of pixel map
  for(int i = 1; i < 12; i++)
  {
    memcpy(&pixelMap.fPixelColor[i], &pixelMap.fPixelColor[0], sizeof(pixelMap.fPixelColor[0]));
  }


  // for(int i = 0; i < strip.numPixels() / 2; i++)
  // {
  //   for(int j = 0; j < strip.numPixels() / 2; j++)
  //   {
  //     // First 6 pixels are speed, set all 12 pixel sets to same speed #
  //     if(j < speedPixelsOn)
  //     {
  //       // Set current speed pixel on
  //       pixelMap.fPixelColor[i][j] = speedColor;
  //     }
  //     else
  //     {
  //       pixelMap.fPixelColor[i][j] = 0;     // Make sure other pixels are off
  //     }

  //     // Last 6 pixels are battery, set all 12 pixel sets to same battery #
  //     if(j < batteryPixelsOn)
  //     {
  //       // Set current battery pixel on
  //       pixelMap.fPixelColor[i][strip.numPixels() - j - 1] = batteryColorIndex[j];
  //     }
  //     else
  //     {
  //       pixelMap.fPixelColor[i][strip.numPixels() - j - 1] = 0;     // Make sure other pixels are off
  //     }
  //   }
  // }
  pixelMap.fDelayInMs = delayTime;  //200
  pixelMap.fNumCyclesBlock = blockingCycles;  // 
}





