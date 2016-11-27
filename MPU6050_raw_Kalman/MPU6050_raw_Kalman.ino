// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

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

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;



// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
// #define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO


#define LED_PIN 13
bool blinkState = false;

double m_SDofXNoise = 5.0;
double m_Xxk1;
double m_Xpk1;
double m_Yxk1;
double m_Ypk1;
double m_Zxk1;
double m_Zpk1;

long axVal;
long ayVal;
long azVal;
long gxVal;
long gyVal;
long gzVal;

int nVals;

void setup() {
  
    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(9600);

    delay(3000);

    // initialize device
    Serial.println("Initializing I2C devices...");

    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // use the code below to change accel/gyro offset values
    
    Serial.println("Updating internal sensor offsets...");
    // -76	-2359	1688	0	0	0
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    //accelgyro.setXGyroOffset(220);
    //accelgyro.setYGyroOffset(76);
    //accelgyro.setZGyroOffset(-85);    

    //- Repeat this process until your program is returning 0 for every gyro, 
    // 0 for X and Y accel, and +16384 for Z accel.

    accelgyro.setXAccelOffset(-612);
    accelgyro.setYAccelOffset(-397);
    accelgyro.setZAccelOffset(2088);
    
    accelgyro.setXGyroOffset(58);
    accelgyro.setYGyroOffset(-29);
    accelgyro.setZGyroOffset(-147);
    
    
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);

    m_Xxk1 = 0.0;
    m_Xpk1 = 1.0;
    m_Yxk1 = 0.0;
    m_Ypk1 = 1.0;
    m_Zxk1 = 0.0;
    m_Zpk1 = 1.0;

 axVal = 0;
 ayVal = 0;
 azVal = 0;
 gxVal = 0;
 gyVal = 0;
 gzVal = 0;
 nVals = 0;
}

double ProcessX(double xD)
{
        // Kalman produce x value...
        //m_Xxk1 = 0.0;
        //m_Xpk1 = 1.0;
        double xZ = xD; // Current measured value
        double xXkMinus1 = m_Xxk1; // xk-1
        double pkMinus1 = m_Xpk1; // p-1
        double xKk = pkMinus1 / (pkMinus1 + m_SDofXNoise);
        double newNewXValue = xXkMinus1 + (xKk*(xZ - xXkMinus1));
        double newErrorEstimate = (1 - xKk)*pkMinus1;
        m_Xxk1 = newNewXValue;
        m_Xpk1 = newErrorEstimate;
        return newNewXValue;     
}
double ProcessY(double yD)
{
        // Kalman produce x value...
        double xZ = yD; // Current measured value
        double yXkMinus1 = m_Yxk1; // xk-1
        double pkMinus1 = m_Ypk1; // p-1
        double yKk = pkMinus1 / (pkMinus1 + m_SDofXNoise);
        double newNewXValue = yXkMinus1 + (yKk*(xZ - yXkMinus1));
        double newErrorEstimate = (1 - yKk)*pkMinus1;
        m_Yxk1 = newNewXValue;
        m_Ypk1 = newErrorEstimate;
        return newNewXValue;     
}
double ProcessZ(double zD)
{
        // Kalman produce x value...
        double xZ = zD; // Current measured value
        double zXkMinus1 = m_Zxk1; // xk-1
        double pkMinus1 = m_Zpk1; // p-1
        double zKk = pkMinus1 / (pkMinus1 + m_SDofXNoise);
        double newNewXValue = zXkMinus1 + (zKk*(xZ - zXkMinus1));
        double newErrorEstimate = (1 - zKk)*pkMinus1;
        m_Zxk1 = newNewXValue;
        m_Zpk1 = newErrorEstimate;
        return newNewXValue;     
}
void loop() {

// 269 / 4 = -67.75
// 100 / 4 = -25
// 590 / 4 = - 147.5


  // Accell
// 5221 = -660
// 3526 = -446
// -1969 250
  
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    if (nVals < 5000)
    {
      axVal += ax;
      ayVal += ay;
      azVal += az;
      gxVal += gx;
      gyVal += gy;
      gzVal += gz;
      
      nVals++;
    }
    else 
    {
      if (nVals == 5000)
      {
        Serial.print(axVal / 5000.0);Serial.print("\t");
        Serial.print(ayVal / 5000.0);Serial.print("\t");
        Serial.print(azVal / 5000.0);Serial.print("\t");
        
        Serial.print(gxVal / 5000.0);Serial.print("\t");
        Serial.print(gyVal / 5000.0);Serial.print("\t");
        Serial.print(gzVal / 5000.0);Serial.print("\n");
        
        //delay(10000);
        axVal =0;
        ayVal =0;
        azVal =0;
        gxVal =0;
        gyVal =0;
        gzVal =0;
        nVals =0;
      }
      
    }
    
    //ax = ProcessX(ax);        
    //ay = ProcessY(ay);        
    //az = ProcessZ(az);        
    
/*
double m_XD = 0.0;
        m_XD = ProcessX(0.39);        
        Serial.println(m_XD);
        m_XD = ProcessX( 0.5);
        Serial.println(m_XD);
        m_XD = ProcessX( 0.48);
        Serial.println(m_XD);
        m_XD = ProcessX( 0.29);
        Serial.println(m_XD);
        m_XD = ProcessX( 0.25);
        Serial.println(m_XD);
        m_XD = ProcessX( 0.32);
        Serial.println(m_XD);
        m_XD = ProcessX( 0.34);
        Serial.println(m_XD);
        m_XD = ProcessX( 0.48);
        Serial.println(m_XD);
        m_XD = ProcessX( 0.41);
        Serial.println(m_XD);
        m_XD = ProcessX( 0.45);
        Serial.println(m_XD);
  */

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
        Serial.print("a/g:\t");
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.print("\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");                
        Serial.println(gz);
    #endif

    #ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    #endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    //delay(25);
}
