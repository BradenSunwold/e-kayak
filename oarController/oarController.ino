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
#define BNO08X_INT 18
#define BNO08X_RESET -1 //15
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
SemaphoreHandle_t imuSem;

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
  eBatteryPercentage,
  eSpeedPercentage,
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
} KayakFeedbackMsg_t;

typedef struct
{
  bool fAutoMode;
  uint8_t fSpeed;     // 0 - 3 (off, low, medium, high)
} PaddleCmdMsg_t;

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
  uint8_t fMessageIndex_1;
  PaddleCmdMsg_t fOutputCmd;
  ImuDataType_t fOutputImuEuler;
  float fPad[3];
} RfOutputMsgFirstHalf_t;

typedef struct 
{
  uint8_t fMessageIndex_2;
  ImuAddReportsVector_t fOutputImuAddReports;
} RfOutputMsgSecondHalf_t;

// Forward decs
std::function<void(int, uint32_t*)> GaugeFrameGenerator(float speedPercentage, float batteryPercentage);
std::function<void(int, uint32_t*)> StartupFrameGenerator(const uint32_t* leftHalfColors, const uint32_t* rightHalfColors);



//######################** Support functions ****************************//
void SetImuReports()
{
  long reportIntervalUs = 20000;    // Setting to 39 creates average output rate of 50Hz when sampling all three reports at ~66Hz for some reason

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
TaskHandle_t Handle_ReadImuTask;
TaskHandle_t Handle_ProcessOutputsTask;
TaskHandle_t Handle_ButtonInputTask;
TaskHandle_t Handle_RfRadioTask;
TaskHandle_t Handle_LedPixelUpdaterTask;

// Test tasks
TaskHandle_t Handle_DumpTaskMetaData;
TaskHandle_t Handle_LedPixelUpdaterTester;

// Global variables for setting task rates
int gReadImuTaskRateInMs = 0xFFFFFFFF; 
int gProcessOutTaskRateInMs = 0xFFFFFFFF; 
int gButtonInTaskRateInMs = 0xFFFFFFFF; 
int gRfRadioTaskRateInMs = 0xFFFFFFFF; 
int gLedDriverTaskRateInMs = 0xFFFFFFFF; 

// Test task rates
int gLedTesterTaskRateInMs = 0xFFFFFFFF; 
int gDiagDumpTaskRateInMs = 0xFFFFFFFF; 

// Global vars for tracking task meta data
TaskMetaData readImuMetaData(Handle_ReadImuTask);
TaskMetaData processOutputsMetaData(Handle_ProcessOutputsTask);
TaskMetaData buttonInputMetaData(Handle_ButtonInputTask);
TaskMetaData rfRadioMetaData(Handle_RfRadioTask);
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

void imuISR() 
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  // Wake up the task
  xSemaphoreGiveFromISR(imuSem, &xHigherPriorityTaskWoken);

  // Request context switch if needed
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


static void ReadImuTask( void *pvParameters ) 
{
  sh2_SensorValue_t sensorData;   // local variables to cpature reported data
  FullImuDataSet_t fullImuDataSet;

  // Flags to track if we have a full set of reports to send out yet
  bool fusedDataFreshFlag = false;
  bool accelDataFreshFlag = false;
  bool gyroDataFreshFlag = false;  

  uint32_t imuReadTimeBegin = 0;
  uint32_t imuReadTimeEnd = 0;

  while(1)
  {
    if (xSemaphoreTake(imuSem, portMAX_DELAY) == pdTRUE)
    {
      readImuMetaData.GetMetaData().UpdateTimestamp();   // Meta data tracks rate of task call
      readImuMetaData.GetExecutionTimer().Start();       // Execution timer tracks task execution time

      // Serial.println("Tik");
      // imuReadTimeBegin = xTaskGetTickCount();

      // Read incoming sensor data
      while(bno08x.getSensorEvent(&sensorData))
      {
        // Serial.println("Tok");
        // Serial.println("AHRS Read");
        // Check for IMU reset
        if (bno08x.wasReset()) 
        {
          Serial.print("sensor was reset ");
          SetImuReports();
        }
        
        switch (sensorData.sensorId) 
        {
          case SH2_ARVR_STABILIZED_RV :
            quaternionToEulerRV(&sensorData.un.arvrStabilizedRV, &fullImuDataSet.fEulerData, true);
            fusedDataFreshFlag = true;

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
            accelDataFreshFlag = true;
            break;
          case SH2_GYROSCOPE_CALIBRATED :
            fullImuDataSet.fAddReportsVect.fX.fGyro = sensorData.un.gyroscope.x;
            fullImuDataSet.fAddReportsVect.fY.fGyro = sensorData.un.gyroscope.y;
            fullImuDataSet.fAddReportsVect.fZ.fGyro = sensorData.un.gyroscope.z;
            gyroDataFreshFlag = true;
            break;
          default : 
            Serial.println("Nothing recieved from IMU");
        }

        // imuReadTimeEnd = xTaskGetTickCount();
        // uint32_t imuReadTime = imuReadTimeEnd - imuReadTimeBegin;

        // Serial.println(imuReadTime);
      }

      if(fusedDataFreshFlag && accelDataFreshFlag && gyroDataFreshFlag)
      {
        xQueueOverwrite(imuDataQueue, (void*)&fullImuDataSet);

        // Reset flags
        fusedDataFreshFlag = false;
        accelDataFreshFlag = false;
        gyroDataFreshFlag = false;
      }

      readImuMetaData.GetExecutionTimer().Stop();

      // myDelayMs(gReadImuTaskRateInMs); 
    }
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
  PaddleCmdMsg_t currButtonState;
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

    // ===== Non-blocking Serial Mock Input =====
    if (Serial.available() > 0) {
      char c = Serial.read(); // Non-blocking read of one character

      if (c == 's') {
        Serial.println("MOCK: SINGLE");
        if (currButtonState.fSpeed == 3) {
          currButtonState.fSpeed = 0;
        } else {
          currButtonState.fSpeed++;
        }
        xQueueSend(currentStateQueue, (void *)&currButtonState, 0);
      }
      else if (c == 'd') {
        Serial.println("MOCK: DOUBLE");
        currButtonState.fAutoMode = !currButtonState.fAutoMode;
        xQueueSend(currentStateQueue, (void *)&currButtonState, 0);
      }
    }

    // ===== Physical button read =====
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
  float speedPercentageReported = 0;
  float previousSpeedPercentageReported = 0;
  float speedPercentageCommanded = 0; 

  KayakFeedbackMsg_t kayakStatusMsg;
  PaddleCmdMsg_t paddleCmdMsg;
  LedMap_t ledMsg;

  float kayakBatteryPercentage = 26;
  float oarbatteryVolt = 4.2;
  float totalBatteryPercentageReport = 100;
  float prevTotalBatteryPercentage = 99;    // Set prev percentage 99 to force an animation update

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
        .fDelayInMs = 75,
        .fNumCyclesBlock = 1,
        .fNumFrames = LED_COUNT
      };
      Serial.println("Comms Loss");
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
            .fDelayInMs = 100,
            .fNumCyclesBlock = 1,
            .fNumFrames = LED_COUNT
          };
          xQueueSend(ledPixelMapQueue, (void *)&ledMsg, 1);

          // Then immediatly load startup animation after
          ledMsg = 
          {
            .fFrameGenerator = StartupFrameGenerator(leftColorsStartupAni, rightColorsStartupAni),
            .fDelayInMs = 100,
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
              .fDelayInMs = 100,
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
              .fDelayInMs = 100,
              .fNumCyclesBlock = 1,
              .fNumFrames = LED_COUNT
            };
            xQueueSend(ledPixelMapQueue, (void *)&ledMsg, 1);

            ledMsg = 
            {
              .fFrameGenerator = StartupFrameGenerator(leftColorsStartupAni, rightColorsStartupAni),
              .fDelayInMs = 100,
              .fNumCyclesBlock = 1,
              .fNumFrames = LED_COUNT
            };
            xQueueSend(ledPixelMapQueue, (void *)&ledMsg, 1);
            break;

          case eSpeedPercentage :                                   // Check for reported speed percentage
            speedPercentageReported = kayakStatusMsg.fStatusData;
            
            // Only send update to display if speed actually changed
            if(speedPercentageReported != previousSpeedPercentageReported)
            {
              ledMsg = 
              {
                .fFrameGenerator = GaugeFrameGenerator(speedPercentageReported, totalBatteryPercentageReport), 
                .fDelayInMs = 50,
                .fNumCyclesBlock = 0,
                .fNumFrames = LED_COUNT
              };
              xQueueSend(ledPixelMapQueue, (void *)&ledMsg, 1);

              previousSpeedPercentageReported = speedPercentageReported;  // Updated previous speed
            }
            break;

          case eBatteryPercentage :                                 // Then check for battery status
            // Package kayak battery voltage for output processor
            kayakBatteryPercentage = kayakStatusMsg.fStatusData;
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
        totalBatteryPercentageReport = oarBatteryPercentage;
      }
      else
      {
        totalBatteryPercentageReport = kayakBatteryPercentage;
      }
    }

    if(!gMotorFaultFlag && !connectingFlag && !gComsTimeoutFlag && totalBatteryPercentageReport != prevTotalBatteryPercentage)
    {
      // Only send new battery status if battery level has changed
      ledMsg = 
      {
        .fFrameGenerator = GaugeFrameGenerator(speedPercentageReported, totalBatteryPercentageReport),
        .fDelayInMs = 50,
        .fNumCyclesBlock = 0,
        .fNumFrames = LED_COUNT
      };
      xQueueSend(ledPixelMapQueue, (void *)&ledMsg, 1);
      Serial.println("Sending new battery status");
      Serial.println(speedPercentageReported);
      Serial.println(totalBatteryPercentageReport);

      prevTotalBatteryPercentage = totalBatteryPercentageReport;  // Update previous battery percentate
    }

    // Next check current state and report to LED driver / RF out any changes
    if(xQueueReceive(currentStateQueue, (void *)&paddleCmdMsg, 0) == pdTRUE)
    {
      if((paddleCmdMsg.fAutoMode && !autoMode) || (!paddleCmdMsg.fAutoMode && autoMode))
      {
        // Toggle autoMode
        autoMode = paddleCmdMsg.fAutoMode;

        if(!connectingFlag && !gMotorFaultFlag && !gComsTimeoutFlag)
        {
          // Only report to LED driver / RF output if we arn't currently faulted or trying to re-connect
          ledMsg = 
          {
            .fFrameGenerator = PulseFrameGeneratorBlue,
            .fDelayInMs = 100,
            .fNumCyclesBlock = 2,
            .fNumFrames = LED_COUNT
          };
          Serial.print("Mode: ");
          Serial.println(paddleCmdMsg.fAutoMode);
          xQueueSend(ledPixelMapQueue, (void *)&ledMsg, 1);
          xQueueSend(rfOutMsgQueue, (void *)&paddleCmdMsg, 1);
        }

      }
      else if(paddleCmdMsg.fSpeed != speedPercentageCommanded)
      {
        speedPercentageCommanded = paddleCmdMsg.fSpeed;                            // Update local speed checker variable

        if(!connectingFlag && !gMotorFaultFlag && !gComsTimeoutFlag)
        {
          // Only report to RF output if we arn't currently faulted or trying to re-connect
          xQueueSend(rfOutMsgQueue, (void *)&paddleCmdMsg, 1);
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

  // Mechanism to track when next annimation update should be  
  volatile TickType_t currPixelTimeout = xTaskGetTickCount();
  volatile TickType_t nextPixelTimeout = xTaskGetTickCount() + (pixelMap.fDelayInMs / portTICK_PERIOD_MS);  

  volatile int cycleCount = 0;    // Counter to track frame cycles
  
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
  uint32_t queueInsertDelayInMs = 500;    // Delay between pushing each pixel map to queue
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

static void RfRadioTask( void *pvParameters )
{
  PaddleCmdMsg_t PaddleCmdOut;

  // Initialize state data
  PaddleCmdOut.fAutoMode = false;
  PaddleCmdOut.fSpeed = 0;

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
  outputMsgFirstHalf.fOutputCmd = PaddleCmdOut;
  outputMsgFirstHalf.fOutputImuEuler = imuDataOut.fEulerData;

  outputMsgSecondHalf.fMessageIndex_2 = 2;
  outputMsgSecondHalf.fOutputImuAddReports = imuDataOut.fAddReportsVect;

  bool newTxData = false;

  // Struct to locally store incoming RF data
  KayakFeedbackMsg_t incomingData;

  // Vars to track coms timeout to motor
  volatile TickType_t lastReceivedMsgTimeInTicks = xTaskGetTickCount();
  double comsDroppedTimeInMs = 3000.0;     // If lose signal for 3 seconds, signal re connecting annimation
  bool firstMessageReceived = false;

  uint8_t rxDownSample = 2;   // Only try to read radio every x iterations
  uint8_t rxDownSampleCounter = 0;

  while(1)
  {
    // rfRadioMetaData.GetMetaData().UpdateTimestamp();   // Meta data tracks rate of task call
    // rfRadioMetaData.GetExecutionTimer().Start();       // Execution timer tracks task execution time

    /************************************** TX *****************************************/
    // Read in IMU / state data
    if (xQueueReceive(imuDataQueue, (void *)&imuDataOut, 0) == pdTRUE)
    {
      rfRadioMetaData.GetMetaData().UpdateTimestamp();   // Meta data tracks rate of task call

      // Package IMU data into RF output struct
      memcpy(&outputMsgFirstHalf.fOutputImuEuler, &imuDataOut.fEulerData, sizeof(imuDataOut.fEulerData));
      memcpy(&outputMsgSecondHalf.fOutputImuAddReports, &imuDataOut.fAddReportsVect, sizeof(imuDataOut.fAddReportsVect));
      newTxData = true;
    }
    if(xQueueReceive(rfOutMsgQueue, (void *)&PaddleCmdOut, 0) == pdTRUE)
    {
      // Package state into RF output struct
      memcpy(&outputMsgFirstHalf.fOutputCmd, &PaddleCmdOut, sizeof(PaddleCmdOut));
      newTxData = true;
      // Serial.println(outputMsgFirstHalf.fOutputState.fAutoMode);
      // Serial.println(outputMsgFirstHalf.fOutputState.fSpeed);
    }

    if(newTxData)
    {
      rfRadioMetaData.GetExecutionTimer().Start();       // Execution timer tracks task execution time
      // New data is available to send
      radio.stopListening();    // put radio in TX mode

      // Send RF data out
      bool report = radio.write(&outputMsgFirstHalf, sizeof(outputMsgFirstHalf));  // transmit & save the report
      report = radio.write(&outputMsgSecondHalf, sizeof(outputMsgSecondHalf));  // transmit & save the report

      radio.startListening();    // put radio in RX mode
      newTxData = false;    // Reset fresh data flag

      rfRadioMetaData.GetExecutionTimer().Stop(); 
    }

    /************************************** RX *****************************************/
    // Rx data twice as slow as outgoing data - down sample for Rx messages
    if(rxDownSampleCounter >= rxDownSample - 1)
    {
      if (radio.available()) 
      {
        uint8_t readLength = radio.getDynamicPayloadSize();
        // Store incoming data to local buffer
        radio.read(&incomingData, readLength);

        // Update coms watchdog
        lastReceivedMsgTimeInTicks = xTaskGetTickCount();
        firstMessageReceived = true;

        // Store relevant data to queue for output processing
        xQueueSend(kayakStatusQueue, (void *)&incomingData, 1);
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
    
    rxDownSampleCounter++;
    // rfRadioMetaData.GetExecutionTimer().Stop(); 

    myDelayMs(gRfRadioTaskRateInMs);   // execute task at 20Hz

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

    // Print average timings

    // Print read IMU diag
    Serial.print("ReadImu average call rate: ");
    Serial.print(readImuMetaData.GetMetaData().GetUpdateRateStats().GetAverageTimeInMs());
    Serial.println("ms");
    Serial.print("ReadImu std dev: ");
    Serial.print(readImuMetaData.GetMetaData().GetUpdateRateStats().GetStdDeviationInMs());
    Serial.println("ms");
    Serial.print("ReadImu average execution rate: ");
    Serial.print(readImuMetaData.GetTaskExecutionTimeStats().GetAverageTimeInMs());
    Serial.println("ms");
    Serial.print("ReadImu stack use: ");
    Serial.println(readImuMetaData.GetStackUsageHighWaterMark());
    Serial.println();

    // Print output processor diag
    Serial.print("outProcessor average call rate: ");
    Serial.print(processOutputsMetaData.GetMetaData().GetUpdateRateStats().GetAverageTimeInMs());
    Serial.println("ms");
    Serial.print("outProcessor std dev: ");
    Serial.print(processOutputsMetaData.GetMetaData().GetUpdateRateStats().GetStdDeviationInMs());
    Serial.println("ms");
    Serial.print("outProcessor average execution rate: ");
    Serial.print(processOutputsMetaData.GetTaskExecutionTimeStats().GetAverageTimeInMs());
    Serial.println("ms");
    Serial.print("outProcessor stack use: ");
    Serial.println(processOutputsMetaData.GetStackUsageHighWaterMark());
    Serial.println();

    // Print button input diag
    Serial.print("buttonIn average call rate: ");
    Serial.print(buttonInputMetaData.GetMetaData().GetUpdateRateStats().GetAverageTimeInMs());
    Serial.println("ms");
    Serial.print("buttonIn std dev: ");
    Serial.print(buttonInputMetaData.GetMetaData().GetUpdateRateStats().GetStdDeviationInMs());
    Serial.println("ms");
    Serial.print("buttonIn average execution rate: ");
    Serial.print(buttonInputMetaData.GetTaskExecutionTimeStats().GetAverageTimeInMs());
    Serial.println("ms");
    Serial.print("buttonIn stack use: ");
    Serial.println(buttonInputMetaData.GetStackUsageHighWaterMark());
    Serial.println();

    // Print write RF diag
    Serial.print("Rf average call rate: ");
    Serial.print(rfRadioMetaData.GetMetaData().GetUpdateRateStats().GetAverageTimeInMs());
    Serial.println("ms");
    Serial.print("Rf std dev: ");
    Serial.print(rfRadioMetaData.GetMetaData().GetUpdateRateStats().GetStdDeviationInMs());
    Serial.println("ms");
    Serial.print("Rf average execution rate: ");
    Serial.print(rfRadioMetaData.GetTaskExecutionTimeStats().GetAverageTimeInMs());
    Serial.println("ms");
    Serial.print("Rf stack use: ");
    Serial.println(rfRadioMetaData.GetStackUsageHighWaterMark());
    Serial.println();

    // Print Led driver diag
    Serial.print("LedDriver average call rate: ");
    Serial.print(ledDriverMetaData.GetMetaData().GetUpdateRateStats().GetAverageTimeInMs());
    Serial.println("ms");
    Serial.print("LedDriver std dev: ");
    Serial.print(ledDriverMetaData.GetMetaData().GetUpdateRateStats().GetStdDeviationInMs());
    Serial.println("ms");
    Serial.print("LedDriver average execution rate: ");
    Serial.print(ledDriverMetaData.GetTaskExecutionTimeStats().GetAverageTimeInMs());
    Serial.println("ms");
    Serial.print("LedDriver stack use: ");
    Serial.println(ledDriverMetaData.GetStackUsageHighWaterMark());
    Serial.println();

    // Print task diag dump diag
    Serial.print("DiagDump average call rate: ");
    Serial.print(diagTaskMetaData.GetMetaData().GetUpdateRateStats().GetAverageTimeInMs());
    Serial.println("ms");
    Serial.print("DiagDump std dev: ");
    Serial.print(diagTaskMetaData.GetMetaData().GetUpdateRateStats().GetStdDeviationInMs());
    Serial.println("ms");
    Serial.print("DiagDump average execution rate: ");
    Serial.print(diagTaskMetaData.GetTaskExecutionTimeStats().GetAverageTimeInMs());
    Serial.println("ms");
    Serial.print("DiagDump stack use: ");
    Serial.println(diagTaskMetaData.GetStackUsageHighWaterMark());
    Serial.println();

    // Serial.println();


    // // Print out worst case timing

    // // Print read IMU diag
    // Serial.print("ReadImu max call rate: ");
    // Serial.print(readImuMetaData.GetMetaData().GetUpdateRateStats().GetMaxTimeInMs());
    // Serial.println("ms");
    // Serial.print("ReadImu max execution rate: ");
    // Serial.print(readImuMetaData.GetTaskExecutionTimeStats().GetMaxTimeInMs());
    // Serial.println("ms");
    // Serial.print("ReadImu stack use: ");
    // Serial.println(readImuMetaData.GetStackUsageHighWaterMark());
    // Serial.println();

    // // Print output processor diag
    // Serial.print("outProcessor max call rate: ");
    // Serial.print(processOutputsMetaData.GetMetaData().GetUpdateRateStats().GetMaxTimeInMs());
    // Serial.println("ms");
    // Serial.print("outProcessor max execution rate: ");
    // Serial.print(processOutputsMetaData.GetTaskExecutionTimeStats().GetMaxTimeInMs());
    // Serial.println("ms");
    // Serial.print("outProcessor stack use: ");
    // Serial.println(processOutputsMetaData.GetStackUsageHighWaterMark());
    // Serial.println();

    // // Print button input diag
    // Serial.print("buttonIn max call rate: ");
    // Serial.print(buttonInputMetaData.GetMetaData().GetUpdateRateStats().GetMaxTimeInMs());
    // Serial.println("ms");
    // Serial.print("buttonIn max execution rate: ");
    // Serial.print(buttonInputMetaData.GetTaskExecutionTimeStats().GetMaxTimeInMs());
    // Serial.println("ms");
    // Serial.print("buttonIn stack use: ");
    // Serial.println(buttonInputMetaData.GetStackUsageHighWaterMark());
    // Serial.println();

    // // Print write RF diag
    // Serial.print("Rf max call rate: ");
    // Serial.print(rfRadioMetaData.GetMetaData().GetUpdateRateStats().GetMaxTimeInMs());
    // Serial.println("ms");
    // Serial.print("Rf max execution rate: ");
    // Serial.print(rfRadioMetaData.GetTaskExecutionTimeStats().GetMaxTimeInMs());
    // Serial.println("ms");
    // Serial.print("Rf stack use: ");
    // Serial.println(rfRadioMetaData.GetStackUsageHighWaterMark());
    // Serial.println();

    // // Print Led driver diag
    // Serial.print("LedDriver max call rate: ");
    // Serial.println(ledDriverMetaData.GetMetaData().GetUpdateRateStats().GetMaxTimeInMs());
    // Serial.print("LedDriver max execution rate: ");
    // Serial.println(ledDriverMetaData.GetTaskExecutionTimeStats().GetMaxTimeInMs());
    // Serial.print("LedDriver stack use: ");
    // Serial.println(ledDriverMetaData.GetStackUsageHighWaterMark());
    // Serial.println();

    // // Print task diag dump diag
    // Serial.print("DiagDump max call rate: ");
    // Serial.print(diagTaskMetaData.GetMetaData().GetUpdateRateStats().GetMaxTimeInMs());
    // Serial.println("ms");
    // Serial.print("DiagDump max execution rate: ");
    // Serial.print(diagTaskMetaData.GetTaskExecutionTimeStats().GetMaxTimeInMs());
    // Serial.println("ms");
    // Serial.print("DiagDump stack use: ");
    // Serial.println(diagTaskMetaData.GetStackUsageHighWaterMark());
    // Serial.println();

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
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);      // default - RF24_1MBPS
  radio.setRetries(3, 2);             // Need to test with Rx and Tx running on motor and oar
  radio.enableDynamicPayloads();
  // radio.setChannel(90);
  // radio.setAutoAck(false);

  // SETUP Button
  pinMode(BUTTON_PIN, INPUT);
  
  // SETUP BNO
  imuSem = xSemaphoreCreateBinary();
  if (imuSem == NULL) {
    Serial.println("Failed to create semaphore!");
    while (1);
  }

  pinMode(BNO08X_INT, INPUT_PULLUP);
  // Attach ISR to falling edge (BNO08x pulls INT low when data ready)
  attachInterrupt(digitalPinToInterrupt(BNO08X_INT), imuISR, FALLING);
  // attachInterrupt(digitalPinToInterrupt(BNO08X_INT), imuISR, LOW);

  Wire.setClock(400000);
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
  rfInMsgQueue = xQueueCreate(msgQueueLength, sizeof(PaddleCmdMsg_t));
  imuDataQueue = xQueueCreate(1, sizeof(FullImuDataSet_t));                 // Only care about most up to date IMU data
  kayakStatusQueue = xQueueCreate(msgQueueLength, sizeof(KayakFeedbackMsg_t));
  currentStateQueue = xQueueCreate(msgQueueLength, sizeof(PaddleCmdMsg_t));
  rfOutMsgQueue = xQueueCreate(msgQueueLength, sizeof(PaddleCmdMsg_t));
  ledPixelMapQueue = xQueueCreate(msgQueueLength, sizeof(LedMap_t));

  // Set all task call rates
  // gReadImuTaskRateInMs = 10; 
  gProcessOutTaskRateInMs = 50; 
  gButtonInTaskRateInMs = 20; 
  gRfRadioTaskRateInMs = 40; 
  gLedDriverTaskRateInMs = 25; 
  gLedTesterTaskRateInMs = 50; 
  gDiagDumpTaskRateInMs = 5000; 

  // Create tasks                                                                                                         // Total RAM usage: ~4KB RAM
  xTaskCreate(ReadImuTask, "Read IMU", 210, NULL, tskIDLE_PRIORITY + 11, &Handle_ReadImuTask);                              // 736 bytes
  xTaskCreate(ButtonInputTask, "Button In",  80, NULL, tskIDLE_PRIORITY + 9, &Handle_ButtonInputTask);                    // 336 bytes
  xTaskCreate(ProcessOutputsTask, "Process Outputs", 130, NULL, tskIDLE_PRIORITY + 9, &Handle_ProcessOutputsTask);        // 936 bytes
  xTaskCreate(RfRadioTask, "RF Out", 160, NULL, tskIDLE_PRIORITY + 10, &Handle_RfRadioTask);                             // 432 bytes
  xTaskCreate(LedPixelUpdaterTask, "Pixel updater", 130, NULL, tskIDLE_PRIORITY + 8, &Handle_LedPixelUpdaterTask);        // 928 bytes

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
                ? static_cast<float>(index) / half        // 0 → 1
                : 1.0f - (static_cast<float>(index - half) / half); // 1 → 0

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
                ? static_cast<float>(index) / half        // 0 → 1
                : 1.0f - (static_cast<float>(index - half) / half); // 1 → 0

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
                ? static_cast<float>(index) / half        // 0 → 1
                : 1.0f - (static_cast<float>(index - half) / half); // 1 → 0

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

std::function<void(int, uint32_t*)> GaugeFrameGenerator(float speedPercentage, float batteryPercentage)
{
  return [=](int frameIndex, uint32_t* ledOutputColorBuffer)
  {
    const uint32_t speedColor = strip.Color(120, 120, 255);
    const int half = LED_COUNT / 2;

    // Calculate # of pixels to turn on
    // int ledMultiplier = round(half / 3.0f);
    // int speedPixelsOn = speed * ledMultiplier;
    int batteryPixelsOn = round((float)batteryPercentage / (100.0 / (float)half));
    int speedPixelsOn = round((float)speedPercentage / (100.0 / (float)half));

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



