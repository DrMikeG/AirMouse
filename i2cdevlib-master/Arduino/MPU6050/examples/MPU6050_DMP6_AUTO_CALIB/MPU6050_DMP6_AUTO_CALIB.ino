// MG modded to only output and light LED when button is pressed...
// Setup to auto calibrate

// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/
#define DEBUG
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
MPU6050 mpu;

#define INTERRUPT_PIN 12  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define BUTTON_PIN 8

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
unsigned long time;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Vars for calibration
int16_t ax, ay, az; // Read into locally
int16_t gx, gy, gz;

long axVal, ayVal, azVal; // Track sums
long gxVal, gyVal, gzVal;

int16_t axCal, ayCal, azCal; // Calibration values
int16_t gxCal, gyCal, gzCal;

int nVals;
int roundsOfCalibrationComplete;
int nRoundsForCalibration;
bool unCalibrated = true;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // configure push button
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    mpu.setFullScaleGyroRange(0); //0 = +/- 250 degrees/sec | 1 = +/- 500 degrees/sec | 2 = +/- 1000 degrees/sec | 3 =  +/- 2000 degrees/sec
    mpu.setFullScaleAccelRange(0);  //0 = +/- 2g | 1 = +/- 4g | 2 = +/- 8g | 3 =  +/- 16g 
    
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Calibrating..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        Serial.println(packetSize);
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    mpu.setXAccelOffset(-626);
    mpu.setYAccelOffset(-448);
    mpu.setZAccelOffset(206);
    
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);

    // Setup for calibration
    axCal= mpu.getXAccelOffset();
    ayCal= mpu.getYAccelOffset();
    azCal= mpu.getZAccelOffset();
    gxCal= mpu.getXGyroOffset();
    gyCal= mpu.getYGyroOffset();
    gzCal= mpu.getZGyroOffset();
    
    axVal = 0;
    ayVal = 0;
    azVal = 0;
    gxVal = 0;
    gyVal = 0;
    gzVal = 0;
    nVals = 0;
    roundsOfCalibrationComplete =0;
    nRoundsForCalibration = 10;
    unCalibrated = true;
    digitalWrite(LED_PIN, false);
} // end of setup


// ================================================================
// ===                      FUNCTIONS USED IN LOOP              ===
// ================================================================

void printSerialData()
{
    time = millis(); //time since program started
    Serial.print(time);
    Serial.print("\t\t");
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    // Because of the orientation of my board, I actually read Yaw / Roll / Pitch
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr[2] * 180/M_PI);
    Serial.print("\t");
    mpu.dmpGetAccel(&aa, fifoBuffer);
    //mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    aaReal.x = aa.x - gravity.x*4096;
    aaReal.y = aa.y - gravity.y*4096;
    aaReal.z = aa.z - gravity.z*4096;
    //mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    // The assumption is that acceleration due to gravity (when device is still)
    // should be distributed according to the orientation from the quaternion...
    if (true)
    {
      Serial.print(aaReal.x);
      Serial.print("\t");
      Serial.print(aaReal.y);
      Serial.print("\t");
      Serial.println(aaReal.z);            
    }
    //Serial.println(aa.x + aa.y + aa.z);            
}

void addCalibrationData()
{
  mpu.dmpGetAccel(&aa, fifoBuffer);
  ax = aa.x;
  ay = aa.y;
  az = aa.z;
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  gz = (int16_t)ypr[0];
  gy = (int16_t)ypr[1];
  gx = (int16_t)ypr[2];
  int maxVals = 500;
  double dmaxVals = (double)maxVals;
  if (nVals < maxVals)
  {
    axVal += ax;
    ayVal += ay;
    azVal += az;
    gxVal += gx;
    gyVal += gy;
    gzVal += gz;    
    nVals++;
  }  else if (nVals == maxVals){
    // Number of values for calibration attempt reached!
    Serial.print(axVal / dmaxVals);Serial.print("\t");
    Serial.print(ayVal / dmaxVals);Serial.print("\t");
    Serial.print(azVal / dmaxVals);Serial.print("\t");
    Serial.print(gxVal / dmaxVals);Serial.print("\t");
    Serial.print(gyVal / dmaxVals);Serial.print("\t");
    Serial.print(gzVal / dmaxVals);Serial.print("\n");

    axCal+= -((axVal / dmaxVals))/7.8;         
    ayCal+= -((ayVal / dmaxVals))/7.8;         
    azCal+= (4096 - (azVal / dmaxVals))/7.8;
    //azCal+= (8192 - (azVal / dmaxVals))/7.8;
    //azCal+= (16184 - (azVal / dmaxVals))/7.8;
    //azCal+= -((azVal / dmaxVals))/7.8;
             
    mpu.setXAccelOffset(axCal);
    mpu.setYAccelOffset(ayCal);
    mpu.setZAccelOffset(azCal);
    
    gxCal+= -((gxVal / dmaxVals))/7.8;         
    gyCal+= -((gyVal / dmaxVals))/7.8;         
    gzCal+= -((gzVal / dmaxVals))/7.8;         
    
    mpu.setXGyroOffset(gxCal);
    mpu.setYGyroOffset(gyCal);
    mpu.setZGyroOffset(gzCal);
    if (false)
    {
      Serial.print(axCal); Serial.print("\t"); // -76
      Serial.print(ayCal); Serial.print("\t"); // -2359
      Serial.print(azCal); Serial.print("\t"); // 1688
      Serial.print(gxCal); Serial.print("\t"); // 0
      Serial.print(gyCal); Serial.print("\t"); // 0
      Serial.print(gzCal); Serial.print("\n"); // 0
    }
    axVal =0;
    ayVal =0;
    azVal =0;
    gxVal =0;
    gyVal =0;
    gzVal =0;
    nVals =0;
    // Check if we are done calibrating....
    roundsOfCalibrationComplete++;
    if (roundsOfCalibrationComplete == nRoundsForCalibration)
    {
     Serial.println(F("Calibration complete!"));
     unCalibrated = false; // finished calibrating...
      if (true)
      {
        Serial.print(axCal); Serial.print("\t"); // -76
        Serial.print(ayCal); Serial.print("\t"); // -2359
        Serial.print(azCal); Serial.print("\t"); // 1688
        Serial.print(gxCal); Serial.print("\t"); // 0
        Serial.print(gyCal); Serial.print("\t"); // 0
        Serial.print(gzCal); Serial.print("\n"); // 0
      }
    }
  }// end of else-if
}// end of addCalibrationData

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
      
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        if (unCalibrated)
        {
            addCalibrationData();
            digitalWrite(LED_PIN, true);
        }
        else
        {
          // Process data once calibration is complete
          bool buttonPressed = (digitalRead(BUTTON_PIN) == LOW);
          digitalWrite(LED_PIN, buttonPressed);
          if (buttonPressed)                       
            printSerialData();       
        }// end if unCalibrated
        
    }// end else if data not avail...
}
