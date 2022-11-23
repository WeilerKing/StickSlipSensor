# Stickslick Sensor - HKA Research Project Version 2.0

Ardunio Code welcher auf ein ESP32 basiertes Entwicklungsboard wie bspw. den tinypico nano geflashed werden kann. Dieser Code kommuniziert mit der Sensorplatine der Fingerkuppe als auch der Auswerteplatine auf welcher der ESP32 Chip montiert werden kann. Außerdem ist eine Schnittstellenkommunikation mit einem Greifer über eine Steckbare Stiftleiste möglich.
#

## Section 0: Installation Ardiono and Enviroment
https://github.com/arduino/Arduino/blob/master/README.md

---

## Section 1: Enabling ESP32 Enviroment
https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html

---

## Section 2: Download libraries

- #include "SPI.h"
    - This library allows you to communicate with SPI devices, with the Arduino as the controller device. This library is bundled with every Arduino platform (avr, megaavr, mbed, samd, sam, arc32), so you do not need to install the library separately.
- #include <FIR.h>
    - https://github.com/sebnil/FIR-filter-Arduino-Library/blob/master/src/FIR.h
- #include <MovingAverage.h> | Version 1.0.3 (Also tested Version 1.0.0 until the newest - nothing works for the statement: MovingAverage <uint16_t> ForceAve(25, 0);)
    - https://www.arduino.cc/reference/en/libraries/movingaverage/
- #include "KickFiltersRT.h" | Version 2.0.0
    - https://github.com/LinnesLab/KickFiltersRT/blob/master/KickFiltersRT.h
- #include <IIRFilter.h> | Version 0.1.0
     - https://tttapa.github.io/Arduino-Filters/Doxygen/index.html  
     - https://github.com/tttapa/Arduino-Filters
- #include <SOSFilter.h> | Version 0.1.0
    - https://github.com/tttapa/Filters/blob/master/src/SOSFilter.h
    - https://github.com/tttapa/Filters
- #include "ESP32TimerInterrupt.h" | Version 2.1.0
    - https://github.com/khoih-prog/ESP32TimerInterrupt/blob/master/src/ESP32TimerInterrupt.h
- #include "I2Cdev.h"
    - https://github.com/ericbarch/arduino-libraries/blob/master/I2Cdev/I2Cdev.h
- #include "MPU6050_6Axis_MotionApps20.h"
    - https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/MPU6050_6Axis_MotionApps20.h
- #include <SimpleKalmanFilter.h> | Version 0.1.0
    - https://github.com/denyssene/SimpleKalmanFilter/blob/master/src/SimpleKalmanFilter.h
- #include <ArduinoJson.h> | Version 6.19.4 
    - https://arduinojson.org/?utm_source=meta&utm_medium=library.properties

---

## Section 3: Commons Issues and Status Quo:
Version of each lib is not documented. Problems with Moving Average and SOSFilter

---

## Section 4: FAQ
1. To Start a measurment you have to press the finger sensor for a couple of seconds.
2. MPU: return devStatus after each device operation (0 = success, !0 = error)
3. DMP Initialization failed Code (devStatus):
    - 1: initial memory load failed (if it's going to break, usually the code will be 1)   
    - 2: DMP configuration updates failed

4. class default I2C address is 0x68 hex

5. I2C frequency is 400kHz (I2C clock)

6. Wrong Chip ID when you get the msg: "could not read correct device id. Expected 0x" + I2C device adress (7B normally)

7. Serial Baudrate is equal to 250.000 (Serial.begin(250000))

8. If MPU initial connection failed you will get a msg: "MPU6050 connection failed" otherwise "MPU6050 connection successful"

9. SPI Speed 10MHz: SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE3));

10. Loop 0: If MPU interrupt is false (0) then you will get the error msg: println('mpuInterrupt is false!')

---

## Section 5: Arduino Linter
- https://github.com/arduino/arduino-lint
- https://arduino.github.io/arduino-lint/1.2/