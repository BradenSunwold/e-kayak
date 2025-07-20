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

// Structs / Enums
typedef enum
{
  eBatteryVolt,
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

// Forward decs
std::function<void(int, uint32_t*)> GaugeFrameGenerator(uint8_t speed, uint32_t batteryPercentage);
std::function<void(int, uint32_t*)> StartupFrameGenerator(const uint32_t* leftHalfColors, const uint32_t* rightHalfColors);



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
// Initialize global queues
size_t msgQueueLength = 10;
static QueueHandle_t rfInMsgQueue;
static QueueHandle_t imuDataQueue;
static QueueHandle_t kayakStatusQueue;
static QueueHandle_t currentStateQueue;
static QueueHandle_t rfOutMsgQueue;
static QueueHandle_t ledPixelMapQueue;

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

// Global variables for setting task rates
int gReadRfTaskRateInMs = 0xFFFFFFFF; 
int gReadImuTaskRateInMs = 0xFFFFFFFF; 
int gProcessOutTaskRateInMs = 0xFFFFFFFF; 
int gButtonInTaskRateInMs = 0xFFFFFFFF; 
int gWriteRfTaskRateInMs = 0xFFFFFFFF; 
int gLedDriverTaskRateInMs = 0xFFFFFFFF; 

// Test task rates
int gLedTesterTaskRateInMs = 0xFFFFFFFF; 
int gDiagDumpTaskRateInMs = 0xFFFFFFFF; 

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
  bool firstMessageReceived = false;

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
          firstMessageReceived = true;

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
                       (static_cast<double>(lastReceivedMsgTimeInTicks) / static_cast<double>(portTICK_PERIOD_MS))) > comsDroppedTimeInMs || !firstMessageReceived)
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

    myDelayMs(gReadRfTaskRateInMs);    // execute task at 20Hz
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
      xQueueOverwrite(imuDataQueue, (void*)&fullImuDataSet);
    }

    readImuMetaData.GetExecutionTimer().Stop();

    myDelayMs(gReadImuTaskRateInMs);    // execute task at 20Hz
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

  // Initialize button state message
  StateMsg_t currButtonState;
  currButtonState.fAutoMode = false;
  currButtonState.fSpeed = 0;

  volatile TickType_t lastDebounceTimeInMs = 0;     // the last time the output pin was toggled
  volatile TickType_t debounceDelayInMs = 0;        // debounce time; increase if the output flickers - set to 0 since task running at 50ms period
  volatile TickType_t doubleClickTimeInMs = 0;
  volatile TickType_t doubleClickDelayInMs = 300;   // the double click time frame, lower if double click seem laggy


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
      // Double button press = 4 state changes (on / off, on / off)
      if(buttonStateChangeCount > 2)
      {
        // Found double click
        Serial.println("DOUBLE");

        // Update the current state
        currButtonState.fAutoMode = !currButtonState.fAutoMode;
      }
      // Single button press = 2 state changes (on / off)
      else if(buttonStateChangeCount > 1)
      {
        // Found single click
        Serial.println("SINGLE");

        // Update the current state
        if(currButtonState.fSpeed == 3)
        {
          currButtonState.fSpeed = 0;
        }
        else
        {
          currButtonState.fSpeed++;
        }
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
    }

    buttonInputMetaData.GetExecutionTimer().Stop();

    myDelayMs(gButtonInTaskRateInMs);   // execute task at 20Hz
  }

  Serial.println("Task Monitor: Deleting");
  vTaskDelete( NULL );
}

static void ProcessOutputsTask( void *pvParameters ) 
{

  bool connectingFlag = false;

  bool autoMode = false;
  uint32_t speed = 0;

  StatusMsg_t kayakStatusMsg;
  StateMsg_t stateMsg;
  LedMap_t ledMsg;

  float kayakBatteryVolt = 26;
  float oarbatteryVolt = 4.2;
  uint32_t batteryPercentageReport = 100;
  uint32_t prevBatteryPercentage = 99;    // Set prev percentage 99 to force an animation update

  uint32_t downSampleBatteryReadCount = 0;

  // Define colors for Startup animation
  uint32_t rightColorsStartupAni[6]  = {
    batteryColorIndex[0],   // bottom-most (5)
    batteryColorIndex[1],
    batteryColorIndex[2],
    batteryColorIndex[3],
    batteryColorIndex[4],
    batteryColorIndex[5]    // top-most (0)
    };

  uint32_t leftColorsStartupAni[6] = {
    strip.Color(120, 120, 255),   // bottom-most (6)
    strip.Color(120, 120, 255), 
    strip.Color(120, 120, 255),  
    strip.Color(120, 120, 255), 
    strip.Color(120, 120, 255),  
    strip.Color(120, 120, 255)    // top-most (11)
    };

  while(1)
  {

    processOutputsMetaData.GetMetaData().UpdateTimestamp();   // Meta data tracks rate of task call
    processOutputsMetaData.GetExecutionTimer().Start();       // Execution timer tracks task execution time

    // First check if we have a coms fault on the oar side - check connectingFlag to avoid continous resending
    if(gComsTimeoutFlag && !connectingFlag)
    {
      connectingFlag = true;
      // Set the connecting animation
      ledMsg = 
      {
        .fFrameGenerator = CircularFrameGeneratorGreen,      // When commenting this out, after two cycles I see no animations
        .fDelayInMs = 50,
        .fNumCyclesBlock = 1,
        .fNumFrames = LED_COUNT
      };
      xQueueSend(ledPixelMapQueue, (void *)&ledMsg, 1);
    }
    else
    {
      // First check kayak status
      if(xQueueReceive(kayakStatusQueue, (void *)&kayakStatusMsg, 0) == pdTRUE)
      {
        // 
        if(connectingFlag)
        {
          // Was previously trying to connect - load connected animation
          ledMsg = 
          {
            .fFrameGenerator = PulseFrameGeneratorGreen,
            .fDelayInMs = 75,
            .fNumCyclesBlock = 1,
            .fNumFrames = LED_COUNT
          };
          xQueueSend(ledPixelMapQueue, (void *)&ledMsg, 1);

          // Then immediatly load startup animation after
          ledMsg = 
          {
            .fFrameGenerator = StartupFrameGenerator(leftColorsStartupAni, rightColorsStartupAni),
            .fDelayInMs = 80,
            .fNumCyclesBlock = 1,
            .fNumFrames = LED_COUNT
          };
          xQueueSend(ledPixelMapQueue, (void *)&ledMsg, 1);

          connectingFlag = false;
        }

        // If received new status
        switch(kayakStatusMsg.fStatusType)
        {
          // first check for errors
          case eLowBattery : 
          case eHighCurrent :
          case eComsLoss :
          case eUnknownFault :                                // faults take priority
            Serial.println("FAULT");
            gMotorFaultFlag = true;

            // Set error animation
            ledMsg = 
            {
              .fFrameGenerator = PulseFrameGeneratorRed,
              .fDelayInMs = 75,
              .fNumCyclesBlock = 0,
              .fNumFrames = LED_COUNT
            };
            xQueueSend(ledPixelMapQueue, (void *)&ledMsg, 1);
            break;

          case eFaultCleared :                                // Then check for fault cleared
            gMotorFaultFlag = false;

            // Load fault cleared animations
            ledMsg = 
            {
              .fFrameGenerator = PulseFrameGeneratorGreen,
              .fDelayInMs = 75,
              .fNumCyclesBlock = 1,
              .fNumFrames = LED_COUNT
            };
            xQueueSend(ledPixelMapQueue, (void *)&ledMsg, 1);

            ledMsg = 
            {
              .fFrameGenerator = StartupFrameGenerator(leftColorsStartupAni, rightColorsStartupAni),
              .fDelayInMs = 80,
              .fNumCyclesBlock = 1,
              .fNumFrames = LED_COUNT
            };
            xQueueSend(ledPixelMapQueue, (void *)&ledMsg, 1);
            break;

          case eBatteryVolt :                                 // Then check for battery status
            // Package kayak battery voltage for output processor
            kayakBatteryVolt = kayakStatusMsg.fStatusData;
            break;
          
          default :
            Serial.println("Did not recieve valid RF message");
            break;
        }
      }
    }

    //  Read the batteries every 5 samples (~once per second)
    downSampleBatteryReadCount++;
    if(downSampleBatteryReadCount % 5 == 0)
    {
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
    }

    if(!gMotorFaultFlag && !connectingFlag && !gComsTimeoutFlag && batteryPercentageReport != prevBatteryPercentage)
    {
      // Only send new battery status if battery level has changed
      ledMsg = 
      {
        .fFrameGenerator = GaugeFrameGenerator(speed, batteryPercentageReport),
        .fDelayInMs = 200,
        .fNumCyclesBlock = 0,
        .fNumFrames = LED_COUNT
      };
      xQueueSend(ledPixelMapQueue, (void *)&ledMsg, 1);
      Serial.println("Sending new battery status");
      Serial.println(speed);
      Serial.println(batteryPercentageReport);
      prevBatteryPercentage = batteryPercentageReport;
    }

    // Next check current state and report to LED driver / RF out any changes
    if(xQueueReceive(currentStateQueue, (void *)&stateMsg, 0) == pdTRUE)
    {
      if((stateMsg.fAutoMode && !autoMode) || (!stateMsg.fAutoMode && autoMode))
      {
        // Toggle autoMode
        autoMode = stateMsg.fAutoMode;

        if(!connectingFlag && !gMotorFaultFlag && !gComsTimeoutFlag)
        {
          // Only report to LED driver / RF output if we arn't currently faulted or trying to re-connect
          ledMsg = 
          {
            .fFrameGenerator = PulseFrameGeneratorBlue,
            .fDelayInMs = 75,
            .fNumCyclesBlock = 2,
            .fNumFrames = LED_COUNT
          };
          Serial.print("Mode: ");
          Serial.println(stateMsg.fAutoMode);
          xQueueSend(ledPixelMapQueue, (void *)&ledMsg, 1);
          xQueueSend(rfOutMsgQueue, (void *)&stateMsg, 1);
        }

      }
      else if(stateMsg.fSpeed != speed)
      {
        speed = stateMsg.fSpeed;                            // Update local speed checker variable

        if(!connectingFlag && !gMotorFaultFlag && !gComsTimeoutFlag)
        {
          // Only report to LED driver / RF output if we arn't currently faulted or trying to re-connect
          ledMsg = 
          {
            .fFrameGenerator = GaugeFrameGenerator(speed, batteryPercentageReport),
            .fDelayInMs = 200,
            .fNumCyclesBlock = 0,
            .fNumFrames = LED_COUNT
          };
          Serial.print("Speed: ");
          Serial.println(stateMsg.fSpeed);
          xQueueSend(ledPixelMapQueue, (void *)&ledMsg, 1);
          xQueueSend(rfOutMsgQueue, (void *)&stateMsg, 1);
        }
      }
    }

    processOutputsMetaData.GetExecutionTimer().Stop();

    myDelayMs(gProcessOutTaskRateInMs);    // execute task at 20Hz

  }

  Serial.println("Task Monitor: Deleting");
  vTaskDelete( NULL );
}

static void LedPixelUpdaterTask( void *pvParameters )
{
  // Default annimation to connecting
  LedMap_t pixelMap;
  //  = 
  //         {
  //           .fFrameGenerator = CircularFrameGeneratorGreen,
  //           .fDelayInMs = 50,
  //           .fNumCyclesBlock = 1,
  //           .fNumFrames = LED_COUNT
  //         };

  // Mechanism to track when next annimation update should be  
  volatile TickType_t currPixelTimeout = xTaskGetTickCount();
  volatile TickType_t nextPixelTimeout = xTaskGetTickCount() + (pixelMap.fDelayInMs / portTICK_PERIOD_MS);  

  volatile int cycleCount = 0;    // Counter to track frame cycles
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
      }
    }

    // Update Cycle counters
    if(cycleCount >= pixelMap.fNumFrames)
    {
      // Serial.println(cycleCount);
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
    uint32_t frameBuffer[LED_COUNT];
    if(currPixelTimeout = xTaskGetTickCount() >= nextPixelTimeout)
    {
      // If frame generator loaded, generate next color buffer frame
      if (pixelMap.fFrameGenerator)
      {
        pixelMap.fFrameGenerator(cycleCount, frameBuffer);
      }

      for (int i = 0; i < strip.numPixels(); i++) 
      {
        strip.setPixelColor(i, frameBuffer[i]);
      }
      strip.show();

      nextPixelTimeout = xTaskGetTickCount() + (pixelMap.fDelayInMs / portTICK_PERIOD_MS);  // Reset next annimation timeout
      cycleCount++;
    }

    ledDriverMetaData.GetExecutionTimer().Stop();

    myDelayMs(gLedDriverTaskRateInMs);    // Execute task at 100Hz

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
    myDelayMs(gLedTesterTaskRateInMs);   // execute task at 10 Hz
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
      // Serial.println(outputMsgFirstHalf.fOutputState.fAutoMode);
      // Serial.println(outputMsgFirstHalf.fOutputState.fSpeed);
    }

    if( radioSemaphore != NULL )
    {
      if( xSemaphoreTake( radioSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {
        // radio.setPayloadSize(sizeof(outputMsg)); 
        radio.setPayloadSize(sizeof(outputMsgFirstHalf)); 
        radio.stopListening();    // put radio in TX mode

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
    
    writeRfMetaData.GetExecutionTimer().Stop(); 

    myDelayMs(gWriteRfTaskRateInMs);   // execute task at 20Hz

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

    // // Print average timings
    // // Print read rf diag
    // Serial.print("ReadRF average call rate: ");
    // Serial.print(readRfMetaData.GetMetaData().GetUpdateRateStats().GetAverageTimeInMs());
    // Serial.println("ms");
    // Serial.print("ReadRF average execution rate: ");
    // Serial.print(readRfMetaData.GetTaskExecutionTimeStats().GetAverageTimeInMs());
    // Serial.println("ms");
    // Serial.print("ReadRF stack use: ");
    // Serial.println(readRfMetaData.GetStackUsageHighWaterMark());
    // Serial.println();

    // // Print read IMU diag
    // Serial.print("ReadImu average call rate: ");
    // Serial.print(readImuMetaData.GetMetaData().GetUpdateRateStats().GetAverageTimeInMs());
    // Serial.println("ms");
    // Serial.print("ReadImu average execution rate: ");
    // Serial.print(readImuMetaData.GetTaskExecutionTimeStats().GetAverageTimeInMs());
    // Serial.println("ms");
    // Serial.print("ReadImu stack use: ");
    // Serial.println(readImuMetaData.GetStackUsageHighWaterMark());
    // Serial.println();

    // // Print output processor diag
    // Serial.print("outProcessor average call rate: ");
    // Serial.print(processOutputsMetaData.GetMetaData().GetUpdateRateStats().GetAverageTimeInMs());
    // Serial.println("ms");
    // Serial.print("outProcessor average execution rate: ");
    // Serial.print(processOutputsMetaData.GetTaskExecutionTimeStats().GetAverageTimeInMs());
    // Serial.println("ms");
    // Serial.print("outProcessor stack use: ");
    // Serial.println(processOutputsMetaData.GetStackUsageHighWaterMark());
    // Serial.println();

    // // Print button input diag
    // Serial.print("buttonIn average call rate: ");
    // Serial.print(buttonInputMetaData.GetMetaData().GetUpdateRateStats().GetAverageTimeInMs());
    // Serial.println("ms");
    // Serial.print("buttonIn average execution rate: ");
    // Serial.print(buttonInputMetaData.GetTaskExecutionTimeStats().GetAverageTimeInMs());
    // Serial.println("ms");
    // Serial.print("buttonIn stack use: ");
    // Serial.println(buttonInputMetaData.GetStackUsageHighWaterMark());
    // Serial.println();

    // // Print write RF diag
    // Serial.print("WriteRf average call rate: ");
    // Serial.print(writeRfMetaData.GetMetaData().GetUpdateRateStats().GetAverageTimeInMs());
    // Serial.println("ms");
    // Serial.print("WriteRf average execution rate: ");
    // Serial.print(writeRfMetaData.GetTaskExecutionTimeStats().GetAverageTimeInMs());
    // Serial.println("ms");
    // Serial.print("WriteRf stack use: ");
    // Serial.println(writeRfMetaData.GetStackUsageHighWaterMark());
    // Serial.println();

    // // Print Led driver diag
    // Serial.print("LedDriver average call rate: ");
    // Serial.println(ledDriverMetaData.GetMetaData().GetUpdateRateStats().GetAverageTimeInMs());
    // Serial.print("LedDriver average execution rate: ");
    // Serial.println(ledDriverMetaData.GetTaskExecutionTimeStats().GetAverageTimeInMs());
    // Serial.print("LedDriver stack use: ");
    // Serial.println(ledDriverMetaData.GetStackUsageHighWaterMark());
    // Serial.println();

    // // Print task diag dump diag
    // Serial.print("DiagDump average call rate: ");
    // Serial.print(diagTaskMetaData.GetMetaData().GetUpdateRateStats().GetAverageTimeInMs());
    // Serial.println("ms");
    // Serial.print("DiagDump average execution rate: ");
    // Serial.print(diagTaskMetaData.GetTaskExecutionTimeStats().GetAverageTimeInMs());
    // Serial.println("ms");
    // Serial.print("DiagDump stack use: ");
    // Serial.println(diagTaskMetaData.GetStackUsageHighWaterMark());
    // Serial.println();

    // Serial.println();


    // Print out worst case timings
    // Print read rf diag
    Serial.print("ReadRF max call rate: ");
    Serial.print(readRfMetaData.GetMetaData().GetUpdateRateStats().GetMaxTimeInTicks());
    Serial.println("ms");
    Serial.print("ReadRF max execution rate: ");
    Serial.print(readRfMetaData.GetTaskExecutionTimeStats().GetMaxTimeInMs());
    Serial.println("ms");
    Serial.print("ReadRF stack use: ");
    Serial.println(readRfMetaData.GetStackUsageHighWaterMark());
    Serial.println();

    // Print read IMU diag
    Serial.print("ReadImu max call rate: ");
    Serial.print(readImuMetaData.GetMetaData().GetUpdateRateStats().GetMaxTimeInMs());
    Serial.println("ms");
    Serial.print("ReadImu max execution rate: ");
    Serial.print(readImuMetaData.GetTaskExecutionTimeStats().GetMaxTimeInMs());
    Serial.println("ms");
    Serial.print("ReadImu stack use: ");
    Serial.println(readImuMetaData.GetStackUsageHighWaterMark());
    Serial.println();

    // Print output processor diag
    Serial.print("outProcessor max call rate: ");
    Serial.print(processOutputsMetaData.GetMetaData().GetUpdateRateStats().GetMaxTimeInMs());
    Serial.println("ms");
    Serial.print("outProcessor max execution rate: ");
    Serial.print(processOutputsMetaData.GetTaskExecutionTimeStats().GetMaxTimeInMs());
    Serial.println("ms");
    Serial.print("outProcessor stack use: ");
    Serial.println(processOutputsMetaData.GetStackUsageHighWaterMark());
    Serial.println();

    // Print button input diag
    Serial.print("buttonIn max call rate: ");
    Serial.print(buttonInputMetaData.GetMetaData().GetUpdateRateStats().GetMaxTimeInMs());
    Serial.println("ms");
    Serial.print("buttonIn max execution rate: ");
    Serial.print(buttonInputMetaData.GetTaskExecutionTimeStats().GetMaxTimeInMs());
    Serial.println("ms");
    Serial.print("buttonIn stack use: ");
    Serial.println(buttonInputMetaData.GetStackUsageHighWaterMark());
    Serial.println();

    // Print write RF diag
    Serial.print("WriteRf max call rate: ");
    Serial.print(writeRfMetaData.GetMetaData().GetUpdateRateStats().GetMaxTimeInMs());
    Serial.println("ms");
    Serial.print("WriteRf max execution rate: ");
    Serial.print(writeRfMetaData.GetTaskExecutionTimeStats().GetMaxTimeInMs());
    Serial.println("ms");
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
    Serial.print(diagTaskMetaData.GetMetaData().GetUpdateRateStats().GetMaxTimeInMs());
    Serial.println("ms");
    Serial.print("DiagDump max execution rate: ");
    Serial.print(diagTaskMetaData.GetTaskExecutionTimeStats().GetMaxTimeInMs());
    Serial.println("ms");
    Serial.print("DiagDump stack use: ");
    Serial.println(diagTaskMetaData.GetStackUsageHighWaterMark());
    Serial.println();

    Serial.println();

    diagTaskMetaData.GetExecutionTimer().Stop(); 

    myDelayMs(gDiagDumpTaskRateInMs);   // execute task at .5Hz
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
  radio.setRetries(3, 3);       // Need to test with Rx and Tx running on motor and oar

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
  imuDataQueue = xQueueCreate(1, sizeof(FullImuDataSet_t));                 // Only care about most up to date IMU data
  kayakStatusQueue = xQueueCreate(msgQueueLength, sizeof(StatusMsg_t));
  currentStateQueue = xQueueCreate(msgQueueLength, sizeof(StateMsg_t));
  rfOutMsgQueue = xQueueCreate(msgQueueLength, sizeof(StateMsg_t));
  ledPixelMapQueue = xQueueCreate(msgQueueLength, sizeof(LedMap_t));

  // Set all task call rates
  gReadRfTaskRateInMs = 200; 
  gReadImuTaskRateInMs = 50; 
  gProcessOutTaskRateInMs = 200; 
  gButtonInTaskRateInMs = 20; 
  gWriteRfTaskRateInMs = 50; 
  gLedDriverTaskRateInMs = 50; 
  gLedTesterTaskRateInMs = 50; 
  gDiagDumpTaskRateInMs = 5000; 

  // Create tasks                                                                                                         // Total RAM usage: ~4KB RAM
  xTaskCreate(ReadRfTask, "Read RF", 120, NULL, tskIDLE_PRIORITY + 5, &Handle_ReadRfTask);                                 // 352 bytes RAM
  xTaskCreate(ReadImuTask, "Read IMU", 210, NULL, tskIDLE_PRIORITY + 7, &Handle_ReadImuTask);                              // 736 bytes
  xTaskCreate(ButtonInputTask, "Button In",  80, NULL, tskIDLE_PRIORITY + 8, &Handle_ButtonInputTask);                    // 336 bytes
  xTaskCreate(ProcessOutputsTask, "Process Outputs", 130, NULL, tskIDLE_PRIORITY + 4, &Handle_ProcessOutputsTask);        // 936 bytes
  xTaskCreate(RfOutputTask, "RF Out", 140, NULL, tskIDLE_PRIORITY + 7, &Handle_RfOutputTask);                             // 432 bytes
  xTaskCreate(LedPixelUpdaterTask, "Pixel updater", 130, NULL, tskIDLE_PRIORITY + 6, &Handle_LedPixelUpdaterTask);        // 928 bytes

  // Test tasks
  // xTaskCreate(DumpTaskMetaDataTask, "Diagnostics Dump", 100, NULL, tskIDLE_PRIORITY + 1, &Handle_LedPixelUpdaterTester);
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

// Support Annimation Functions
void PulseFrameGeneratorRed(int index, uint32_t* outputColorBuffer)
{
  // Set default color to red
  uint8_t red = 255;
  uint8_t green = 0;
  uint8_t blue = 0; 

  // Dim down to off first half of annimation
  int half = LED_COUNT / 2;
  float scale = (index < half)
                  ? 1.0f - (static_cast<float>(index) / half)
                  : static_cast<float>(index - half) / half;

  uint8_t rOut = static_cast<uint8_t>((red * scale) + 0.5f);
  uint8_t gOut = static_cast<uint8_t>((green * scale) + 0.5f);
  uint8_t bOut = static_cast<uint8_t>((blue * scale) + 0.5f);

  // Build output buffer
  for (int i = 0; i < LED_COUNT; i++) 
  {
    outputColorBuffer[i] = strip.Color(rOut, gOut, bOut);
  }
}

void PulseFrameGeneratorGreen(int index, uint32_t* outputColorBuffer)
{
  // Set default color to green
  uint8_t red = 0;
  uint8_t green = 255;
  uint8_t blue = 0; 

  // Dim down to off first half of annimation
  int half = LED_COUNT / 2;
  float scale = (index < half)
                  ? 1.0f - (static_cast<float>(index) / half)
                  : static_cast<float>(index - half) / half;

  uint8_t rOut = static_cast<uint8_t>((red * scale) + 0.5f);
  uint8_t gOut = static_cast<uint8_t>((green * scale) + 0.5f);
  uint8_t bOut = static_cast<uint8_t>((blue * scale) + 0.5f);

  // Build output buffer
  for (int i = 0; i < LED_COUNT; i++) 
  {
    outputColorBuffer[i] = strip.Color(rOut, gOut, bOut);
  }
}

void PulseFrameGeneratorBlue(int index, uint32_t* outputColorBuffer)
{
  // Set default color to green
  uint8_t red = 120;
  uint8_t green = 120;
  uint8_t blue = 255; 

  // Dim down to off first half of annimation
  int half = LED_COUNT / 2;
  float scale = (index < half)
                  ? 1.0f - (static_cast<float>(index) / half)
                  : static_cast<float>(index - half) / half;

  uint8_t rOut = static_cast<uint8_t>((red * scale) + 0.5f);
  uint8_t gOut = static_cast<uint8_t>((green * scale) + 0.5f);
  uint8_t bOut = static_cast<uint8_t>((blue * scale) + 0.5f);

  // Build output buffer
  for (int i = 0; i < LED_COUNT; i++) 
  {
    outputColorBuffer[i] = strip.Color(rOut, gOut, bOut);
  }
}

void CircularFrameGeneratorGreen(int frameIndex, uint32_t* outputColorBuffer)
{
    uint8_t red = 0;
    uint8_t green = 255;
    uint8_t blue = 0; 

  // Define the circular path (0-indexed)
  static const uint8_t path[] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0
  };
  static const int pathLength = sizeof(path) / sizeof(path[0]);

  const int trailLength = 3;  // Change this to light more LEDs (e.g., 1 = single point, 4 = 4 LEDs lit)

  // Clear all LEDs first
  for (int i = 0; i < LED_COUNT; ++i) {
    outputColorBuffer[i] = strip.Color(0, 0, 0); // Off
  }

  // Light up `trailLength` LEDs in the path
  for (int t = 0; t < trailLength; ++t)
  {
    int step = (frameIndex - t + pathLength) % pathLength;
    int ledIndex = path[step];

    if (ledIndex >= 0 && ledIndex < LED_COUNT)
    {
      // Optional: Fade tail by intensity
      float fade = 1.0f - (float)t / trailLength;
      uint8_t r = static_cast<uint8_t>(0 * fade);
      uint8_t g = static_cast<uint8_t>(255 * fade);
      uint8_t b = static_cast<uint8_t>(0 * fade * fade);

      outputColorBuffer[ledIndex] = strip.Color(r, g, b);
    }
  }
}

std::function<void(int, uint32_t*)> GaugeFrameGenerator(uint8_t speed, uint32_t batteryPercentage)
{
  return [=](int frameIndex, uint32_t* ledOutputColorBuffer)
  {
    const uint32_t speedColor = strip.Color(120, 120, 255);
    const int half = LED_COUNT / 2;

    // Calculate # of pixels to turn on
    int ledMultiplier = round(half / 3.0f);
    int speedPixelsOn = speed * ledMultiplier;
    int batteryPixelsOn = round(batteryPercentage / (100.0 / (float)half));

    // Set speed LEDs (first half)
    for (int i = half - 1, count = 0; i >= 0; --i, ++count)
    {
      ledOutputColorBuffer[i] = (count < speedPixelsOn) ? speedColor : 0;
    }

    // Set battery LEDs (second half)
    for (int i = half, count = 0; i < LED_COUNT; ++i, ++count)
    {
      ledOutputColorBuffer[i] = (count < batteryPixelsOn) ? batteryColorIndex[count] : 0;
    }
  };
}

std::function<void(int, uint32_t*)> StartupFrameGenerator(
    const uint32_t* leftHalfColors,
    const uint32_t* rightHalfColors)
{
  return [=](int frameIndex, uint32_t* ledBuffer)
  {
    const int halfCount = 6;
    int pixelsToLight = frameIndex % (halfCount + 1); // 0 to 6

    // Clear all LEDs
    for (int i = 0; i < LED_COUNT; ++i)
      ledBuffer[i] = 0;

    // Light up left half from 5 → 0
    for (int i = 0; i < pixelsToLight; ++i)
    {
      int ledIndex = 5 - i;
      ledBuffer[ledIndex] = leftHalfColors[i];
    }

    // Light up right half from 6 → 11
    for (int i = 0; i < pixelsToLight; ++i)
    {
      int ledIndex = 6 + i;
      ledBuffer[ledIndex] = rightHalfColors[i];
    }
  };
}



