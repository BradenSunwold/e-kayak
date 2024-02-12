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

// Global flags
bool fault = false;
bool startup = true;

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

#define VBATPIN A7

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
TaskHandle_t Handle_StateManagerTask;
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

volatile UBaseType_t uxHighWaterMark;

// RF input task
static void ReadRfTask( void *pvParameters ) 
{
  // Struct to locally store incoming RF data
  StatusMsg_t incomingData;

  radioSemaphore = xSemaphoreCreateMutex();     // Create mutex
  
  while(1)
  {
    // taskTime = millis() - beginTime;
    // Serial.println(taskTime);
    // beginTime = millis();

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
          // Serial.println(incomingData.statusData);

          // Store relevant data to queue
          xQueueSend(rfInMsgQueue, (void *)&incomingData, 1);
        }

          xSemaphoreGive( radioSemaphore );
      }
      else
      {
          // Serial.println("Could not take mutex");
      }
    }
    
    // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    // Serial.println(uxHighWaterMark);

    // beginTime = millis();
    // count = 0;
    myDelayMs(100);    // execute task at 20Hz
    // taskTime = millis() - beginTime;
    // Serial.println(taskTime);
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
    // taskTime = millis() - beginTime;
    // Serial.println(taskTime);
    // beginTime = millis();

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
    
    // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    // Serial.println(uxHighWaterMark);

    // beginTime = millis();
    // count = 0;
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

    // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    // Serial.println(uxHighWaterMark);

    // beginTime = millis();
    myDelayMs(50);   // execute task at 20Hz
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


  StatusMsg_t rfReceiverMsg;
  rfReceiverMsg.fStatusType = eStartup;    // Initialize status type to connecting on startup
  rfReceiverMsg.fStatusData = 0.0;
  xQueueSend(kayakStatusQueue, (void *)&rfReceiverMsg, 1);

  unsigned long lastReceivedMsg = 0;
  unsigned long comsReconnectTime = 3000;     // If lose signal for 3 seconds, signal re connecting annimation
  unsigned long comsErrorTime = 8000;         // 8 seconds till hard fault unless re-connects

  bool startupFlag = true;
  bool comsTimeoutFlag = false;

  while(1)
  {
    //////////////////////// Check status from pi /////////////////////////////////
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
        // Keep sending startup message (triggers connecting annimation) until faulted
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


    //////////////////////// Check state ///////////////////////////////
    if(xQueueReceive(buttonQueue, (void *)&buttonPress, 0) == pdTRUE)
    {
      if (!fault && !startup)
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
    }

    // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    // Serial.println(uxHighWaterMark);

    // beginTime = millis();
    myDelayMs(100);    // execute task at 20Hz
    // taskTime = millis() - beginTime;
    // Serial.println(taskTime);
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
          Serial.println("FAULT");
          LoadErrorAnnimation(ledMsg);  // Set pixel map

          // Add to led queue
          xQueueSend(ledPixelMapQueue, (void *)&ledMsg, 1);
          fault = true;
          break;

        // Then check for fault cleared
        case eFaultCleared :
          LoadConnectedAnnimation(ledMsg);  // Set pixel map

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
              LoadConnectedAnnimation(ledMsg);

              xQueueSend(ledPixelMapQueue, (void *)&ledMsg, 1);
              startupConnected = true;
            }
            else
            {
              // Now load startup annimation after connected annimation
              LoadStartupAnnimation(ledMsg);

              xQueueSend(ledPixelMapQueue, (void *)&ledMsg, 1);
              startup = false;    // Signal output processor that startup is over

            }

          }
          break;
        
        case eStartup :
          startup = true;
          startupConnected = false;
          LoadConnectingAnnimation(ledMsg);

          // Add to led queue
          xQueueSend(ledPixelMapQueue, (void *)&ledMsg, 1);
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
        // Toggle autoMode
        autoMode = stateMsg.fAutoMode;

        if(!fault && !startup)
        {
          // Only report to LED driver / RF output if we arn't currently faulted
          LoadModeChangeAnnimation(ledMsg);
          xQueueSend(ledPixelMapQueue, (void *)&ledMsg, 1);
          xQueueSend(rfOutMsgQueue, (void *)&stateMsg, 1);
        }

      }
      else if(stateMsg.fSpeed != speed)
      {
        speed = stateMsg.fSpeed;                            // Update local speed checker variable
        xQueueSend(rfOutMsgQueue, (void *)&stateMsg, 1);
      }
    }

    // Finally read the batteries
    float oarBatteryVoltage = analogRead(VBATPIN);
    oarBatteryVoltage *= 2;        // Account for voltage divider
    oarBatteryVoltage *= 3.3;      // Multiply by 3.3V - reference voltage
    oarBatteryVoltage /= 1024;     // convert to voltage

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

    if(!fault && !startup)
    {
      LoadGaugeUpdateAnnimation(ledMsg, speed, batteryPercentageReport);

      // xQueueSend(ledPixelMapQueue, (void *)&ledMsg, 1);
    }

    // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    // Serial.println(uxHighWaterMark);

    // Serial.println(currTime - prevTime);
    // beginTime = millis();
    myDelayMs(100);    // execute task at 20Hz
    // taskTime = millis() - beginTime;
    // Serial.println(taskTime);
  }

  Serial.println("Task Monitor: Deleting");
  vTaskDelete( NULL );
}

static void LedPixelUpdaterTask( void *pvParameters )
{
  // Pixel map
  LedMap_t pixelMap;

  // Set default pixel map to connecting annimation sequence
  LoadConnectingAnnimation(pixelMap);

  volatile int delayCount = 0;    // Counter to track annimation delay times
  volatile int cycleCount = 0;    // Counter to track 12 cycle counter
  volatile int taskDelay = 25;
  
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

    // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    // Serial.println(uxHighWaterMark);

    delayCount++;
    count = 0;

    // beginTime = millis();
    myDelayMs(taskDelay);    // Execute task at 100Hz
    // taskTime = millis() - beginTime;
    // Serial.println(taskTime);
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
    // Read in IMU / state data
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

    if( radioSemaphore != NULL )
    {
      if( xSemaphoreTake( radioSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {
        radio.setPayloadSize(sizeof(outputMsg)); 
        radio.stopListening();  // put radio in TX mode

        // Send RF data out
        unsigned long start_timer = millis();                // start the timer
        bool report = radio.write(&outputMsg, sizeof(outputMsg));  // transmit & save the report
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

    // beginTime = millis();
    myDelayMs(50);   // execute task at 20Hz
    // taskTime = millis() - beginTime;
    // Serial.println(taskTime);
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
  rfOutMsgQueue = xQueueCreate(msgQueueLength, sizeof(StateMsg_t));
  ledPixelMapQueue = xQueueCreate(msgQueueLength, sizeof(LedMap_t));

  // Create tasks
  xTaskCreate(ReadRfTask, "Read in", 84, NULL, tskIDLE_PRIORITY + 5, &Handle_ReadRfTask);
  xTaskCreate(ReadImuTask, "Read in", 178, NULL, tskIDLE_PRIORITY + 7, &Handle_ReadImuTask);
  xTaskCreate(ButtonInputTask, "Button In",  84, NULL, tskIDLE_PRIORITY + 6, &Handle_ButtonInputTask);
  xTaskCreate(StateManagerTask, "Kayak State", 80, NULL, tskIDLE_PRIORITY + 4, &Handle_StateManagerTask);
  xTaskCreate(ProcessOutputsTask, "Process Outputs", 234, NULL, tskIDLE_PRIORITY + 3, &Handle_ProcessOutputsTask);
  xTaskCreate(RfOutputTask, "RF Out", 108, NULL, tskIDLE_PRIORITY + 7, &Handle_RfOutputTask);
  xTaskCreate(LedPixelUpdaterTask, "Pixel updater", 232, NULL, tskIDLE_PRIORITY + 5, &Handle_LedPixelUpdaterTask);

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
  // Serial.println(count);
  count++;
}

// Support annimation functions
void LoadConnectingAnnimation(LedMap_t &pixelMap)
{
  // for(int i = 0; i < strip.numPixel(); i++)
  // {
  //   for()
  //   {
  //     pixelMap.fPixelColor[i][j] = strip.Color(0, 255, 0);
  //   }
  // }

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

  pixelMap.fDelay = 50;
  pixelMap.fNumCyclesBlock = 1;
}

void LoadConnectedAnnimation(LedMap_t &pixelMap)
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

  pixelMap.fDelay = 75;
  pixelMap.fNumCyclesBlock = 1;
}

void LoadErrorAnnimation(LedMap_t &pixelMap)
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

  pixelMap.fDelay = 75;
  pixelMap.fNumCyclesBlock = 0;
}

void LoadModeChangeAnnimation(LedMap_t &pixelMap)
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

  pixelMap.fDelay = 75;
  pixelMap.fNumCyclesBlock = 2;
}

void LoadStartupAnnimation(LedMap_t &pixelMap)
{

}

void LoadGaugeUpdateAnnimation(LedMap_t &pixelMap, float speed, float batteryPercentage)
{
//   // Startup annimation will be ramp up on each half circle of pixel ring
//   for(int i = 0; i < (strip.numPixels() / 2); i++)
//   {
//     // 6 pixels on each side - Speed side want all green / Battery side want red, orange, yellow, yellow, green, green ?
    
//     // Set speed side
//     pixelMap.fPixelColor[i][j]
//     strip.setPixelColor(i, strip.Color(120, 120, 255)); 

//     // Set battery side
//     strip.setPixelColor(strip.numPixels() - i - 1, batteryColorIndex[i]);

//     strip.show();
//     myDelayMs(wait);
//   }

  // // Ramp speed half back down since not yet moving
  // for(int i = (strip.numPixels() / 2) - 1; i >= 0; i--)
  // {
  //   strip.setPixelColor(i, strip.Color(0, 0, 0)); 
  // }
}





// Finish annimations
// Get video of the RF timeout issues
// Update pixel task to use millis, not rely on task time
// Test all Rf coms
// Dump IMU Rf data from pi into file





