/* Includes für Beschleunigungssensor */
#include "SPI.h"
#include <FIR.h>

#include <MovingAverage.h>
#include "KickFiltersRT.h"
#include <IIRFilter.h>
#include <SOSFilter.h>

/* Includes für Kraftsensor */
#include "ESP32TimerInterrupt.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
#include <SimpleKalmanFilter.h>


#include <ArduinoJson.h>      // Necessary to create JSON document in order to export data in a readable format

// ------------------------------------------------------------------------------------------------- //
// -------------------------------------------- Header Core 1 -------------------------------------- //
// ------------------------------------------------------------------------------------------------- //


/* General defines */
#define SLIP_TRUE true
#define SLIP_FALSE false

#define SLIP_TH 204             // 204 -> 20% slip in array -> global slip true

/* Defines für Beschleunigungssensor */
#define CHIP_SELECT 26
#define MISO_SDO 19
#define MOSI_SDI  23
#define CLK_SPC 18
#define INTERRUPT_INT1 25

/* Defines für Kraftsensor */
#define ANALOG_INPUT 14   // A0
#define numberOfSamples 1024


/* Grenzwerte */
#define GRIP_TH 200
#define FORCE_TH 1400
#define BP_ACC_TH1 2482
#define BP_ACC_TH2 400
#define COMP_FACT 2
#define TANG_F_TH 32

/* Allgemeine Variablen */
uint16_t SlipX[numberOfSamples], SlipY[numberOfSamples];
bool Kriterium1, Kriterium2 = false;

uint16_t numSlipX, numSlipY;
float pctSlipX, pctSlipY;
bool isSlipping = false;
/* Variablen für Beschleunigungssensor */
/* instanciate filter variables */


MovingAverage <uint16_t> BPAve(10, 0);
MovingAverage <uint16_t> BSAve(10, 0);
MovingAverage <uint16_t> BPAveX(10, 0);
MovingAverage <uint16_t> BSAveX(10, 0);
MovingAverage <uint16_t> ForceAve(25, 0);

/* Konstanten Matrix für BP 150-1000Hz */
const float sosMatrix[3][6] = {
  {1,  0,  -1,  1,  -1.796235071228434510004490221035666763783,  0.842630884823877712719308874511625617743},
  {1,  0,  -1,  1,  -1.97067408723792980573819022538373246789,   0.972027057619610390659659060474950820208},
  {1,  0,  -1,  1,  -1.80981310938436656954309000866487622261,   0.817400224667203301187612396461190655828}
};

const float gainArray[3] = {
  0.095340429119196615226350388638820732012,                                                         
  0.095340429119196615226350388638820732012,                                                        
  0.091299887666398349406193801769404672086                                             
};

/* Konstanten Matrix für HP 1000Hz for slip-algorithm */
const float sosMatrix_hp[3][6] = {
  {1,  -1.990471204912039437928683582867961376905,  1,  1,  -1.800447030418101501680894216406159102917,  0.885173554422183261891632355400361120701},
  {1,  -1.99861332831245674945819246204337105155,   1,  1,  -1.215272197834000644789398393186274915934,  0.46717282884497846051630176589242182672}, 
  {1,  -1.983571615931344567229643871542066335678,  1,  1,  -1.921119579817682110345344881352502852678,  0.976066610612674412372768983914284035563}
};

const float gainArray_hp[3] = {
   2.317984596054522672403663818840868771076,                                                                                                
  18.869511143032617894732538843527436256409,                                                                                                
  0.012351371826092600453295133888786949683                                   
};

/* Konstanten Matrix für LP 5Hz */
const float sosMatrix_lp5Hz[3][6] = {
  {1,  2,  1,  1,  -1.999388978184262199988552310969680547714,  0.999390365639474143044651555101154372096},
  {1,  2,  1,  1,  -1.998333940108844686278644076082855463028,  0.998335326831923830503967565164202824235},
  {1,  2,  1,  1,  -1.997725320435868034962823003297671675682,  0.997726706736601909319972492085071280599}
};

const float gainArray_lp5Hz[3] = {
  0.000000346863803017615110605754730893624,                                                        
  0.000000346680769839398760121201934630841,                                                        
  0.000000346575183514444792400347686994744                               
};

//float filteredBuffer[2048];
//float filteredBuffer_hp[2048];
//float filteredBuffer_lp5Hz[2048];
//SOSFilter<10> bp(sosMatrix, gainArray);
//SOSFilter<2> bp(sosMatrix, gainArray);
SOSFilter<3> bp(sosMatrix, gainArray);                // BP 150-1000
SOSFilter<3> hp(sosMatrix_hp, gainArray_hp);          // HP 1000
SOSFilter<3> lp5Hz(sosMatrix_lp5Hz, gainArray_lp5Hz); // LP 5
/* End filter matrixes calculated by matlab filterDesigner for slip-algorithm*/

uint8_t chipID = 0x7B;
bool dataReadyFlag = false;
//const int numberOfSamples = 2048; //2048
int counter = 0;
int16_t rawValuesX[numberOfSamples];
int16_t rawValuesX2[numberOfSamples];
int16_t rawValuesY[numberOfSamples];
int16_t rawValuesY2[numberOfSamples];
int16_t filteredBP[numberOfSamples];
int16_t filteredBPX[numberOfSamples];
int16_t filteredBS[numberOfSamples];
int16_t filteredBSX[numberOfSamples];
uint16_t filteredBPAve[numberOfSamples];
uint16_t filteredBPAveX[numberOfSamples];
uint16_t filteredBSAve[numberOfSamples];
uint16_t filteredBSAveX[numberOfSamples];
int16_t filteredY5Hz[numberOfSamples];
int16_t filteredX5Hz[numberOfSamples];
long double meanSum = 0;
long double meanSumX = 0;
uint32_t absBP, absBS, absBPX, absBSX;
unsigned long t0, t1, deltaT;

/* Constants arrays for graphic visualisation of thresholds*/
uint16_t BP_ACC_1[numberOfSamples];
uint16_t BP_ACC_2[numberOfSamples];
uint16_t T_F_TH[numberOfSamples];
uint16_t BS_ACC[numberOfSamples];
uint16_t BS_ACC_X[numberOfSamples];
/* End Constants for graphic visualisation of thresholds*/

/* Variablen für Kraftsensor */
uint16_t adcBuf[numberOfSamples];
uint16_t adcValue;
int adcCounter;
int timer0Counter = 0;
ESP32Timer ITimer0(0);
bool isGripping = false;
uint16_t algo_force;
int timer0Counter_algo = 0;
double adcMeanValue;
uint16_t mean;
double forceNewton;
/* End Variable für Kraftsensor */

/* Variablen für JSON */
StaticJsonDocument<200> doc;
/* End Variablen für JSON */

/* Funktionsprototypen für Beschleunigungssensor */
int16_t readRegister16(uint8_t adr);
void readSensorData(int16_t dataArray[], int index);
void readSensorDataXY(int16_t dataArrayX[numberOfSamples], int16_t dataArrayY[numberOfSamples], int index);
void displayMeasurement(int16_t messwerte[numberOfSamples], int index);
void readSensorDataY(int16_t dataArray[numberOfSamples], int index);

/* Funktionsprototypen für Kraftsensor */
uint16_t getForce();
//uint16_t adc_mean();

/* ---ISRs--- */
void isr() {
  dataReadyFlag = true;
  //Serial.println("data");
  //digitalWrite(DEBUG_PIN, HIGH);
}

void TimerHandler0()
{
    if(isGripping){
      adcValue = getForce();
      adcBuf[timer0Counter] = adcValue;
      timer0Counter++;
    }
    //digitalWrite(DEBUG_PIN, HIGH);
    else{
      adcValue = getForce();
    }
    if(timer0Counter == numberOfSamples){
      timer0Counter = 0;
      memset(adcBuf, 0, sizeof(adcBuf));
    }
    
    //digitalWrite(DEBUG_PIN, LOW);
    
}
// ------------------------------------------------------------------------------------------------- //
// ------------------------------------------ END Header Core 1 ------------------------------------ //
// ------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------- //
// -------------------------------------------- Header Core 0 -------------------------------------- //
// ------------------------------------------------------------------------------------------------- //

/*
 SimpleKalmanFilter(e_mea, e_est, q);
 e_mea: Measurement Uncertainty 
 e_est: Estimation Uncertainty 
 q: Process Noise
 */
SimpleKalmanFilter simpleKalmanFilter(10, 10, 0.001);

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high


// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL


#define INTERRUPT_PIN 4  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define numberOfSamples 1024
#define dt 0.001

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

int i = 0;

int16_t x_acceleration;
double x[numberOfSamples];
double x_filtered[numberOfSamples];
double x_alt[numberOfSamples];
//x_alt[0] = 0;
int16_t x_dd[numberOfSamples];
double x_d_alt[numberOfSamples];
double x_d_neu[numberOfSamples];
//x_d_alt[0] = 0;
double x_d_filtered[numberOfSamples];
double x_dd_filtered[numberOfSamples];

float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int16_t x_Acc, y_Acc, z_Acc;
//double x_neu, y_neu, z_neu, x_alt = 0, y_alt = 0, z_alt = 0;
double x_acc_filtered, y_acc_filtered, z_acc_filtered;
//double x_d_neu, y_d_neu, z_d_neu, x_d_alt = 0, y_d_alt = 0, z_d_alt = 0;
float roll, pitch, yaw;
bool mFirstRun = false;

// -- Kalman variables -- //
float x_measured_value, x_estimated_value;

/* Filter matrixes calculated by matlab filterDesigner for MPU6050*/
const float sosMatrix_hp_MPU6050[3][6] = {
  {1,  -2,  1,  1,  -1.982892791905700002885737376345787197351,  0.983871712838118361865724637027597054839},
  {1,  -2,  1,  1,  -1.955578240315035243312991042330395430326,  0.956543676511203200263366852595936506987},
  {1,  -2,  1,  1,  -1.94014812744726672377737486385740339756,   0.94110594605536468382211978678242303431}
};
const float gainArray_hp_MPU6050[3] = {
  0.991691126185954563432289887714432552457,                                                         
  0.978030479206559610894089473731582984328,                                                         
  0.970313518375657824144298047031043097377
};


const float sosMatrix_lp50_MPU6050[3][6] = {
  {1,  2,  1,  1,  -1.761249229100169788608809540164656937122,  0.851887031867598065737468004954280331731},
  {1,  2,  1,  1,  -1.561018075800718163392843962355982512236,  0.641351538057563175243558362126350402832},
  {1,  2,  1,  1,  -1.464868193951245123329840680526103824377,  0.540253569427869617669557555927895009518}
};
const float gainArray_lp50_MPU6050[3] = {
  0.022659450691857044996035952522106526885,                                                        
  0.02008336556421123561544384017452102853,                                                        
  0.018846343869156116646035314943219418637
};

SOSFilter<3> hp_MPU6050(sosMatrix_hp_MPU6050, gainArray_hp_MPU6050);
SOSFilter<3> lp50_MPU6050(sosMatrix_lp50_MPU6050, gainArray_lp50_MPU6050);
/* End filter matrixes calculated by matlab filterDesigner for MPU6050 */

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ------------------------------------------------------------------------------------------------- //
// ------------------------------------------ END Header Core 1 ------------------------------------ //
// ------------------------------------------------------------------------------------------------- //
#define LED1 32
#define LED2 33

TaskHandle_t Task1_handle, Task2_handle;
SemaphoreHandle_t sem;




// -------------------------------------------------------------------------------------------------- //
// ---------------------------------------------- GLOBAL SETUP -------------------------------------- //
// -------------------------------------------------------------------------------------------------- //
void setup() {
  // put your setup code here, to run once:
Serial.begin(250000);
pinMode(LED1, OUTPUT);
pinMode(LED2, OUTPUT);



// ------------------------------------------------------------------------------------------------- //
// ------------------------------------------- INIT CORE 0 ---------------------------------------- //
// ------------------------------------------------------------------------------------------------- //
disableCore0WDT();
// join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
   // Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    
//    // verify connection
    Serial.println(F("Testing device connections..."));
   Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
//
//    // wait for ready
//    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
//    while (Serial.available() && Serial.read()); // empty buffer
//    while (!Serial.available());                 // wait for data
//    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setXAccelOffset(200);
    mpu.setYAccelOffset(4000);
    mpu.setZAccelOffset(5914); // 1788 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        //mpu.setIntDataReadyEnabled(true);
        mpu.setDLPFMode(MPU6050_DLPF_BW_188);
        // enable Arduino interrupt detection
        //Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        //Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

   
// ------------------------------------------------------------------------------------------------- //
// ----------------------------------------- END INIT CORE 0 --------------------------------------- //
// ------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------- //
// --------------------------------------------  INIT CORE 1 --------------------------------------- //
// ------------------------------------------------------------------------------------------------- //
  
  counter = 0;
  int16_t messwerte[numberOfSamples];

 
  pinMode(ANALOG_INPUT, INPUT);
  analogReadResolution(12);
  
  /* pin definitions */
  pinMode(CHIP_SELECT, OUTPUT);
  pinMode(INTERRUPT_INT1, INPUT);
  //pinMode(INTERRUPT_INT1, INPUT);


  digitalWrite(CHIP_SELECT, HIGH);
  //digitalWrite(DEBUG_PIN, LOW);

  /* SPI.begin(sck, miso, mosi, ss) */
  SPI.begin();// CLK_SPC, MISO_SDO, MOSI_SDI, CHIP_SELECT
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE3)); // 8000000
  delay(500);
  uint8_t receivedID;
while(receivedID != 0x7B){
   /* TODO: device test -> read who_am_i */
  digitalWrite(CHIP_SELECT, LOW);
  //PORTD &= ~0x01; 
  
  SPI.transfer(0x8F); // read (*|0x80) who_am_i
  
  receivedID = SPI.transfer(0x00);
  digitalWrite(CHIP_SELECT, HIGH);
  
}
 delay(50);
  //PORTD |= 0x01;

  //Serial.print("[main] reading who_am_i (0x0F) register from device: 0x");
  //Serial.println(receivedID, HEX);

  if( receivedID != chipID ) {
    Serial.print("[main] could not read correct device id. Expected 0x");
    Serial.println(chipID, HEX);
    return;
  }


  /* device settings */

  /* accelereometer axis mode */
  digitalWrite(CHIP_SELECT, LOW);
  SPI.transfer(0x15);           // CTRL6_C S.35
  SPI.transfer(0x00);       // select 3axis mode
  digitalWrite(CHIP_SELECT, HIGH);

  delay(1);

  /* enable, range and resolution mode selection */
  digitalWrite(CHIP_SELECT, LOW);
  SPI.transfer(0x10);           // CTRL1_XL S.32
  SPI.transfer(0xA0);       // enable accelerometer (1010XXXX),
                  // accelerometer full-scale selection to +/- 16g (XXXX01XX),
                  // high-resolution selection to "output from first stage digital filtering" (XXXXXX00)
  digitalWrite(CHIP_SELECT, HIGH);

  delay(1);

  /* enable DATA_READY interrupt -> S.25 */
  /* - COUNTER_BDR_REG1 (0Bh) set to 1 (optional) */
  digitalWrite(CHIP_SELECT, LOW);
  SPI.transfer(0x0B);           // COUNTER_BDR_REG1 S.30
  SPI.transfer(0x80);       // enables pulsed data-ready mode (the data ready pulses are 18.75 μs long)
  digitalWrite(CHIP_SELECT, HIGH);

  delay(1);

  /* - INT1_CTRL (0Dh) / INT2_CTRL (0Eh) set to 1 */
  digitalWrite(CHIP_SELECT, LOW);
  SPI.transfer(0x0D);           // INT1_CTRL S.31
  SPI.transfer(0x01);       // enables accelerometer data-ready interrupt on INT1 pin
  digitalWrite(CHIP_SELECT, HIGH);

  delay(1);

  /* FIFO in Bypass disable -> S.17 */
  /* - FIFO_CTRL4 (0Ah) set to 0, FIFO_ MODE[2:0] */
  digitalWrite(CHIP_SELECT, LOW);
  SPI.transfer(0x0A);           // FIFO_CTRL4 S.29
  SPI.transfer(0x00);       // enter Bypass mode
  digitalWrite(CHIP_SELECT, HIGH);
  //SPI.endTransaction();
  /* route external sensor interrupt to internal PIN for handling */
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_INT1), isr, RISING);
  /* Interrupt Timer timing in microseconds 10 000 us -> 100Hz */
  ITimer0.attachInterruptInterval(1000, TimerHandler0);

  /* measurement variables and arrays */


 sem = xSemaphoreCreateMutex();
xTaskCreatePinnedToCore(
  codeForTask1,           // Task function
  "Task_1",               // name of task
  2000,                   // Stack size of task
  NULL,                   // parameter of the task
  1,                      // priority of the task
  &Task1_handle,                 // task handle to keep track of created task
  0);                     // core
   delay(500);
   xTaskCreatePinnedToCore(
  codeForTask2,           // Task function
  "Task_2",               // name of task
  2000,                   // Stack size of task
  NULL,                   // parameter of the task
  1,                      // priority of the task
  &Task2_handle,                 // task handle to keep track of created task
  1);                      // core

  //Serial.println("[main] starting measurement ...");
  
// ------------------------------------------------------------------------------------------------- //
// ----------------------------------------- END INIT CORE 1 --------------------------------------- //
// ------------------------------------------------------------------------------------------------- //
  
}

// ------------------------------------------------------------------------------------------------- //
// ---------------------------------------------- LOOP Core 0 -------------------------------------- //
// ------------------------------------------------------------------------------------------------- //
void codeForTask1(void * parameter)
{
  for(;;){
  
 // -- CODE -- //
  // if programming failed, don't try to do anything
   //yield();
    while(!dmpReady){
      xSemaphoreTake(sem, portMAX_DELAY);
      Serial.println("dmp not ready");
      xSemaphoreGive(sem);
      };
    //Serial.println("Sample Frequency");
   // Serial.println(mpu.dmpGetSampleFrequency());
    // read a packet from FIFO

   
    if(mpuInterrupt){
      //Serial.println("here");
      
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        mpuInterrupt = false;
       
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            xSemaphoreTake(sem, portMAX_DELAY);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
            Serial.println("                    Bewegungsprofil TASK"); 
            xSemaphoreGive(sem); 
                 
    }
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//            Serial.print("areal\t");
//            Serial.print(aaReal.x);
//            Serial.print("\t");
//            Serial.print(aaReal.y);
//            Serial.print("\t");
//            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
//            Serial.print("aworld\t");
//            Serial.print(aaWorld.x);
//            Serial.print("\t");
//            Serial.print(aaWorld.y);
//            Serial.print("\t");
//            Serial.println(aaWorld.z);
            //x_dd[i] = aaReal.x;
            x_acceleration = aaWorld.x;
        #endif
      //delay(10);
      x_dd[i] = x_acceleration;
      i++;
    } 
//Funktionsabsicherung mpuInterrupt Statement
  else if(!mpuInterrupt)
  {
    println('mpuInterrupt is false!')
  }
    
//Serial.println(x_acceleration);
        

   // }
   
   if(i == numberOfSamples){
    for(int j = 0; j < numberOfSamples; j++){
      //x_dd_filtered[j] = (double)lp50.filter(x_dd[j]) / 16384 * 9.81 * 1000; // value in mm/s^2
        /*x_d_neu[j] = x_dd[j] / 16384 * 9.81 * 1000 * dt + x_d_alt[j];
        x_d_alt[j] = x_d_neu[j];
        x_d_filtered[j] = hp.filter(x_d_neu[j]);
        x[j] = 0.5 * x_dd_filtered[j] * dt * dt + x_d_neu[j] * dt + x_alt[j];
        x_alt[j] = x[j];
        x_filtered[j] = hp.filter(x[j]);*/
        
    }
    /*for(int k  = 0; k < numberOfSamples; k++){
      Serial.print("x:");
      Serial.println(x_dd[k]);
      */
      
     // printed here
   }
   
   
   
   }
  }
}
// ----------------------------------------------------------------------------------------------------- //
// ---------------------------------------------- END LOOP Core 0 -------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------- //
// ---------------------------------------------- LOOP Core 1 -------------------------------------- //
// ------------------------------------------------------------------------------------------------- //
void codeForTask2(void * parameter)
{
  for(;;){
  //xSemaphoreTake(sem, portMAX_DELAY);
  // xSemaphoreGive(sem);
  // -- CODE -- //
  t0 = micros();
  //yield();
  if(adcValue > GRIP_TH){
    
    isGripping = true;
      
      if(dataReadyFlag){
       dataReadyFlag = false;
       
       readSensorDataXY(rawValuesX,rawValuesY, counter);
      // readSensorDataY(rawValuesY, counter);
       counter++;
       
      }
  
    if(counter == numberOfSamples){   
      for(int v = 0; v<numberOfSamples; v++){
        //Serial.println(rawValuesY[v], DEC);
        meanSum += rawValuesY[v];
        meanSumX += rawValuesX[v];
      }
      meanSum = meanSum / numberOfSamples;
      meanSumX = meanSumX / numberOfSamples;
      for(int z = 0; z < numberOfSamples; z++){
        rawValuesY2[z] = rawValuesY[z]-meanSum;
        rawValuesX2[z] = rawValuesX[z]-meanSumX;
      }
       timer0Counter_algo = timer0Counter;
        algo_force = adcBuf[timer0Counter-1];
      if(adcBuf[timer0Counter-1] > FORCE_TH){                      
        Kriterium1 = true;
        
        for(int w = 0; w < numberOfSamples; w++){
          BP_ACC_1[w] = BP_ACC_TH1;
          T_F_TH[w] = TANG_F_TH;
          
          filteredBP[w] = bp.filter(rawValuesY2[w]);    // BP 150-1000Hz Filter Y
          filteredBPX[w] = bp.filter(rawValuesX2[w]);  // BP 150-1000Hz Filter X
          
          filteredBS[w] = hp.filter(rawValuesY2[w]);    // HP 1000Hz Filter Y
          filteredBSX[w] = hp.filter(rawValuesX2[w]);    // HP 1000Hz Filter Y
          
          filteredY5Hz[w] = abs(lp5Hz.filter(rawValuesY2[w])); // LP 5Hz Filter Y
          filteredX5Hz[w] = abs(lp5Hz.filter(rawValuesX2[w])); // LP 5Hz Filter Y
          
          absBP = abs(filteredBP[w]);
          absBPX = abs(filteredBPX[w]);
          
          BPAve.push(absBP);                                            // Average BP Filter
          BPAveX.push(absBPX);
          
          filteredBPAve[w] = BPAve.get();                               //    
          filteredBPAveX[w] = BPAveX.get();
          
          absBS = abs(filteredBS[w]);                                   // Average HP Filter
          absBSX = abs(filteredBSX[w]);
          
          BSAve.push(absBS);
          BSAveX.push(absBSX);
          
          filteredBSAve[w] = BSAve.get();
          filteredBSAveX[w] = BSAveX.get();
          
          if(filteredBPAve[w] > BP_ACC_TH1){                // Y
            if(filteredY5Hz[w] > TANG_F_TH){
              SlipY[w] = SLIP_TRUE;
            }
            else{
              SlipY[w] = SLIP_FALSE;
            }
          }
          else if(filteredBPAveX[w] > BP_ACC_TH1){           // X
            if(filteredX5Hz[w] > TANG_F_TH){
              SlipX[w] = SLIP_TRUE;
            }
            else{
              SlipX[w] = SLIP_FALSE;
            }
          }
          else{
            SlipX[w] = SLIP_FALSE;
            SlipY[w] = SLIP_FALSE;
          }
          
        }
      }
      else{
        Kriterium2 = true;
        for(int x = 0; x < numberOfSamples; x++){
          BP_ACC_2[x] = BP_ACC_TH2;
          
          filteredBP[x] = bp.filter(rawValuesY2[x]);
          filteredBPX[x] = bp.filter(rawValuesX2[x]);
          
          filteredBS[x] = hp.filter(rawValuesY2[x]);
          filteredBSX[x] = hp.filter(rawValuesX2[x]);
          
          absBP = abs(filteredBP[x]);
          absBPX = abs(filteredBPX[x]);
          
          BPAve.push(absBP);
          BPAveX.push(absBPX);
          
          filteredBPAve[x] = BPAve.get();
          filteredBPAveX[x] = BPAveX.get();
          
          absBS = abs(filteredBS[x]);
          absBSX = abs(filteredBSX[x]);
          
          BSAve.push(absBS);
          BSAveX.push(absBSX);
          
          filteredBSAve[x] = BSAve.get();
          filteredBSAveX[x] = BSAveX.get();
          
          BS_ACC[x] = COMP_FACT*filteredBSAve[x];
          BS_ACC_X[x] = COMP_FACT*filteredBSAveX[x];
          
          if(filteredBPAve[x] > BP_ACC_TH2){
            if(filteredBPAve[x] > BS_ACC[x]){
              SlipY[x] = SLIP_TRUE;
            }
            else{
              SlipY[x] = SLIP_FALSE;
            }
          }
          else if(filteredBPAveX[x] > BP_ACC_TH2){
            if(filteredBPAveX[x] > BS_ACC_X[x]){
              SlipX[x] = SLIP_TRUE;
            }
            else{
              SlipX[x] = SLIP_FALSE;
            }
          }
          else{
            SlipX[x] = SLIP_FALSE;
            SlipY[x] = SLIP_FALSE;
          }
        }
      }
      
      ForceAve.push(adcValue);
      adcMeanValue = ForceAve.get();
      forceNewton = 0.0095*exp(0.0029*adcMeanValue);
      
      counter = 0;                    // !!! reset sample counter

    for(int k = 0; k < numberOfSamples; k++){
      if(SlipX[k] == true){
        numSlipX++;
      }
      if(SlipY[k] == true){
        numSlipY++;
      }
      pctSlipX = (float)numSlipX / (float)numberOfSamples *100;
      pctSlipY = (float)numSlipY / (float)numberOfSamples *100;
      
    }
    if(numSlipX >= SLIP_TH || numSlipY >= SLIP_TH){
      isSlipping = true;
      // Set slip pin high
    }
    else{isSlipping = false;}
    
    numSlipX = 0;
    numSlipY = 0;
    
    xSemaphoreTake(sem, portMAX_DELAY);
   /* for(int g = 0; g < numberOfSamples; g++){
      Serial.println(filteredBPAve[g]);
    }*/
    
    Serial.print("Slip ?   ");
    Serial.print(isSlipping);
    Serial.print("  Slip percentage X  Y  ");
    Serial.print(pctSlipX);
    Serial.print("\t");
    Serial.print(pctSlipY); 
    Serial.print("\t");
    Serial.print("Gripping force  ");
    Serial.print("\t");
    Serial.print(forceNewton);
    
    t1 = micros();
    deltaT = t1-t0;
    Serial.print("Stick-slip   ");
    Serial.println(deltaT);
    //Serial.print(uxTaskGetStackHighWaterMark(nullptr));
    xSemaphoreGive(sem);
      
    }
    
  }
  else{
    isGripping = false;
    timer0Counter = 0;
    memset(adcBuf, 0, sizeof(adcBuf));
  }
    // print here
  }
}
// ----------------------------------------------------------------------------------------------------- //
// ---------------------------------------------- END LOOP Core 1 -------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //

void loop() {
  // put your main code here, to run repeatedly:
  vTaskDelete(nullptr);
  //vTaskDelay(100 / portTICK_PERIOD_MS);
}

// ------------------------------------------------------------------------------------------------------------ //
// ----------------------------------------- FUNCTION DEFINITION CORE 1 --------------------------------------- //
// ------------------------------------------------------------------------------------------------------------ //
/* Funktion Implementierung */

uint16_t getForce(){
  uint16_t Force;
  return Force = analogRead(ANALOG_INPUT); 
}

int16_t readRegister16(uint8_t adr){
  uint8_t val[2]={0};
  digitalWrite(CHIP_SELECT, LOW);
  SPI.transfer(0x80 | adr);
  val[0]=SPI.transfer(0x00);
  val[1]=SPI.transfer(0x00);
  digitalWrite(CHIP_SELECT, HIGH);
  return (int16_t)((val[1]<<8)|val[0]);
}

int32_t readRegisters32(uint8_t adr){
  uint8_t val[4]={0};
  digitalWrite(CHIP_SELECT, LOW);
  SPI.transfer(0x80 | adr);
  val[0]=SPI.transfer(0x00);
  val[1]=SPI.transfer(0x00);
  val[2]=SPI.transfer(0x00);
  val[3]=SPI.transfer(0x00);
  digitalWrite(CHIP_SELECT, HIGH);
  return (int32_t)((val[1]<<8)|val[0]|((val[3]<<8|val[2])<<16));
}

void readSensorDataY(int16_t dataArray[numberOfSamples], int index) {
    //Serial.println("Enterd");
    /* read axis value */
    /* x: 0x28 | 0x80 -> 0xA8 */
    /* y: 0x2A | 0x80 -> 0xAA */
    /* z: 0x2C | 0x80 -> 0xAC */
    uint8_t val1, val2;
    digitalWrite(CHIP_SELECT, LOW);
   
    SPI.transfer(0xAA);
    
    /* SPI read values in address-incrementing mode */

    val1 = SPI.transfer(0x00);     // read y-axis Low
    val2 = SPI.transfer(0x00);     // read y-axis High
    
    digitalWrite(CHIP_SELECT, HIGH);
    dataArray[index] = int16_t((val2 << 8) | val1);
    
}
void readSensorData(int16_t dataArray[numberOfSamples], int index) {
    //Serial.println("Enterd");
    /* read axis value */
    /* x: 0x28 | 0x80 -> 0xA8 */
    /* y: 0x2A | 0x80 -> 0xAA */
    /* z: 0x2C | 0x80 -> 0xAC */
    uint8_t val1, val2;
    digitalWrite(CHIP_SELECT, LOW);
   
    SPI.transfer(0xA8);
    
    /* SPI read values in address-incrementing mode */

    val1 = SPI.transfer(0x00);     // read x-axis Low
    val2 = SPI.transfer(0x00);     // read x-axis High
    digitalWrite(CHIP_SELECT, HIGH);
    dataArray[index] = int16_t((val2 << 8) | val1);
    
}

void readSensorDataXY(int16_t dataArrayX[numberOfSamples], int16_t dataArrayY[numberOfSamples], int index) {
    //Serial.println("Enterd");
    /* read axis value */
    /* x: 0x28 | 0x80 -> 0xA8 */
    /* y: 0x2A | 0x80 -> 0xAA */
    /* z: 0x2C | 0x80 -> 0xAC */
    uint8_t val1, val2, val3, val4;
    digitalWrite(CHIP_SELECT, LOW);
   
    SPI.transfer(0xA8);
    
    /* SPI read values in address-incrementing mode */

    val1 = SPI.transfer(0x00);     // read x-axis Low
    val2 = SPI.transfer(0x00);     // read x-axis High
    val3 = SPI.transfer(0x00);     // read y-axis Low
    val4 = SPI.transfer(0x00);     // read y-axis High 
    digitalWrite(CHIP_SELECT, HIGH);
    dataArrayX[index] = int16_t((val2 << 8) | val1);
    dataArrayY[index] = int16_t((val4 << 8) | val3);
    
}


void displayMeasurement(int16_t messwerte[], int index) {
  
      Serial.println(messwerte[index], DEC); // ,8

}

//uint16_t adc_mean(){
//  
//  for(int o = 0; o < timer0Counter; o++){
//    mean += adcBuf[o];
//  }
//  
//  if(timer0Counter != 0){
//    return mean = (mean / timer0Counter);
//  }
//  else{
//    return mean = 0;
//  }
// 
//}

// ---------------------------------------------------------------------------------------------------------------- //
// ----------------------------------------- END FUNCTION DEFINITION CORE 1 --------------------------------------- //
// ---------------------------------------------------------------------------------------------------------------- //
