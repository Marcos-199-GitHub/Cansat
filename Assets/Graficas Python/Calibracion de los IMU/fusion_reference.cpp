#include <stdio.h>
#include "Kalman.h"
// ----- Debugging Definitions -----
#define KX122    // KX122
#define KXG03    // KXG03
#define MagField // BM1422
// ----- Demo Mode Definitions -----
// #define CSVOutput
#define SensorSamplePeriod 500 // in ms
// ----- Included Files -----
// #include <Wire.h> //Default I2C Library
#define SCL_PIN 5          // A5 //Note that if you are using the I2C based sensors, you will need to download and
#define SCL_PORT PORTC     // install the "SoftI2CMaster" as "Wire" does not support repeated start...
#define SDA_PIN 4          // A4 //References:
#define SDA_PORT PORTC     // http://playground.arduino.cc/Main/SoftwareI2CLibrary
#include "SoftI2CMaster.h" // https://github.com/felias-fogg/SoftI2CMaster
#define I2C_TIMEOUT 1000   // Sets Clock Stretching up to 1sec
#define I2C_FASTMODE 1     // Sets 400kHz operating speed
// ----- Globals -----
int sensorValue = 0;
float sensorConvert = 0;
unsigned int j;
unsigned int lineClear = 0;
double intervalTime = 0;
// Digital Input Globals - Hall Sensor
#ifdef HallSen
int Hall_Out0 = 0;
int Hall_Out1 = 1;
#endif
// I2C globals (using SoftI2CMaster libary) - MEMs Kionix Sensor
int I2C_check = 0;
int matlabData = -1;
#define ACCEL 1
#define MAGN 2
#define GYRO 3
#ifdef KX122
int KX122_DeviceAddress = 0x3C; // this is the 8bit address, 7bit address = 0x1E
int KX122_Accel_X_LB = 0;
int KX122_Accel_X_HB = 0;
int KX122_Accel_Y_LB = 0;
int KX122_Accel_Y_HB = 0;
int KX122_Accel_Z_LB = 0;
int KX122_Accel_Z_HB = 0;
int KX122_Accel_X_RawOUT = 0;
int KX122_Accel_Y_RawOUT = 0;
int KX122_Accel_Z_RawOUT = 0;
float KX122_Accel_X_OUT = 0;
float KX122_Accel_Y_OUT = 0;
float KX122_Accel_Z_OUT = 0;
#endif
#ifdef KXG03
int i = 11;
int t = 1;
short int aveX = 0;
short int aveX2 = 0;
short int aveX3 = 0;
short int aveY = 0;
short int aveY2 = 0;
short int aveY3 = 0;
short int aveZ = 0;
short int aveZ2 = 0;
short int aveZ3 = 0;
int KXG03_DeviceAddress = 0x9E; // this is the 8bit address, 7bit address = 0x4F
int KXG03_Gyro_X_LB = 0;
int KXG03_Gyro_X_HB = 0;
int KXG03_Gyro_Y_LB = 0;
int KXG03_Gyro_Y_HB = 0;
int KXG03_Gyro_Z_LB = 0;
int KXG03_Gyro_Z_HB = 0;
float KXG03_Gyro_X = 0;
float KXG03_Gyro_Y = 0;
float KXG03_Gyro_Z = 0;
short int KXG03_Gyro_X_RawOUT = 0;
short int KXG03_Gyro_Y_RawOUT = 0;
short int KXG03_Gyro_Z_RawOUT = 0;
short int KXG03_Gyro_X_RawOUT2 = 0;
short int KXG03_Gyro_Y_RawOUT2 = 0;
short int KXG03_Gyro_Z_RawOUT2 = 0;
int KXG03_Accel_X_LB = 0;
int KXG03_Accel_X_HB = 0;
int KXG03_Accel_Y_LB = 0;
int KXG03_Accel_Y_HB = 0;
int KXG03_Accel_Z_LB = 0;
int KXG03_Accel_Z_HB = 0;
float KXG03_Accel_X = 0;
float KXG03_Accel_Y = 0;
float KXG03_Accel_Z = 0;
short int KXG03_Accel_X_RawOUT = 0;
short int KXG03_Accel_Y_RawOUT = 0;
short int KXG03_Accel_Z_RawOUT = 0;
#endif
#ifdef MagField
unsigned int BM1422_DeviceAddress = 0x1E;
short int BM1422_Mag_X_LB = 0;
short int BM1422_Mag_X_HB = 0;
short int BM1422_Mag_Y_LB = 0;
short int BM1422_Mag_Y_HB = 0;
short int BM1422_Mag_Z_LB = 0;
short int BM1422_Mag_Z_HB = 0;
short int BM1422_Mag_X_RawOUT = 0;
short int BM1422_Mag_Y_RawOUT = 0;
short int BM1422_Mag_Z_RawOUT = 0;
float BM1422_Mag_X = 0;
float BM1422_Mag_Y = 0;
float BM1422_Mag_Z = 0;
#endif
#define RAD_TO_DEG 57.29578
#define M_PI 3.14159265358979323846
Kalman KFilter[3];
float KFAngle[3];
float CFAngle[3];
#define RawDataOutput 1
float mag_off[3];
unsigned long tStart, tStop, tElapse;
unsigned long eTime;
float yaw_gyro;
void setup()
{
    // Wire.begin(); // start I2C functionality
    Serial.begin(115200); // start serial port at 9600 bps
    while (!Serial)
    {
        ; // wait for serial port to connect. Needed for Leonardo only
    }
    I2C_check = i2c_init();
    if (I2C_check == false)
    {
        while (1)
        {
            Serial.write("I2C Init Failed (SDA or SCL may not be pulled up!");
            Serial.write(0x0A); // Print Line Feed
            Serial.write(0x0D); // Print Carrage Return
            delay(500);
        }
    }
    pinMode(13, OUTPUT); // Setup for the LED on Board
    pinMode(2, INPUT_PULLUP);
    pinMode(3, INPUT_PULLUP);
// pinMode(13, INPUT_PULLUP);
//----- Start Initialization for KX122 Accel Sensor -----
#ifdef KX122
    // 1. CNTL1 (0x18) loaded with 0x40 (Set high resolution bit to 1)
    // 2. CNTL1 (0x18) loaded with 0xC0 (Enable bit on)
    i2c_start(KX122_DeviceAddress); // This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
    i2c_write(0x18);
    i2c_write(0x40);
    i2c_stop();
    i2c_start(KX122_DeviceAddress); // This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
    i2c_write(0x18);
    i2c_write(0xC0);
    i2c_stop();
#endif
//----- END Initialization for KX122 Accel Sensor -----
//----- Start Initialization for KXG03 Gyro Sensor -----
#ifdef KXG03
    // 1. STBY REG (0x43) loaded with 0xEF
    i2c_start(KXG03_DeviceAddress); // This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
    i2c_write(0x43);
    i2c_write(0x00);
    i2c_stop();
#endif
//----- END Initialization for KXG03 Gyro Sensor -----
#ifdef MagField
    i2c_start(BM1422_DeviceAddress); // This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
    i2c_write(0x1B);
    i2c_write(0xC0);
    i2c_stop();
    i2c_start(BM1422_DeviceAddress); // This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
    i2c_write(0x5C);
    i2c_write(0x00);
    i2c_stop();
    i2c_start(BM1422_DeviceAddress); // This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
    i2c_write(0x5D);
    i2c_write(0x00);
    i2c_stop();
    i2c_start(BM1422_DeviceAddress); // This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
    i2c_write(0x1D);
    i2c_write(0x40);
    i2c_stop();
#endif
    mag_off[0] = -28.5;
    mag_off[1] = -22.73;
    mag_off[2] = -21.73;
    delay(2000);
    GetAccelData();
    GetMagnData();
    GetGyroData();
    KFilter[0].setAngle((float)(atan2(KX122_Accel_Y_OUT, KX122_Accel_Z_OUT) + PI) * RAD_TO_DEG);
    KFilter[1].setAngle((float)(atan2(-KX122_Accel_X_OUT, sqrt(KX122_Accel_Y_OUT * KX122_Accel_Y_OUT + KX122_Accel_Z_OUT * KX122_Accel_Z_OUT))) * RAD_TO_DEG);
    float norm = Norm(KX122_Accel_X_OUT, KX122_Accel_Y_OUT, KX122_Accel_Z_OUT);
    float pitchA = -asin(KX122_Accel_X_OUT / norm);             // * RAD_TO_DEG;
    float rollA = asin(KX122_Accel_Y_OUT / cos(pitchA) / norm); // * RAD_TO_DEG;
    float magX = BM1422_Mag_X - mag_off[0];
    float magY = BM1422_Mag_Y - mag_off[1];
    float magZ = BM1422_Mag_Z - mag_off[2];
    norm = sqrt(magX * magX + magY * magY + magZ * magZ);
    float mx = magX / norm;
    float my = -1 * magY / norm;
    float mz = magZ / norm;
    float Mx = mx * cos(pitchA) + mz * sin(pitchA);
    float My = mx * sin(rollA) * sin(pitchA) + my * cos(rollA) - mz * sin(rollA) * cos(pitchA);
    float yaw = atan2(-My, Mx) * RAD_TO_DEG;
    if (yaw > 360)
    {
        yaw -= 360;
    }
    else if (yaw < 0)
    {
        yaw += 360;
    }
    KFilter[2].setAngle(yaw);
    yaw_gyro = yaw;
    CFAngle[2] = yaw;
    tStart = millis();
}
void loop()
{
    GetAccelData();
    GetMagnData();
    GetGyroData();
    float roll = (float)(atan2(KX122_Accel_Y_OUT, KX122_Accel_Z_OUT)) * RAD_TO_DEG;
    float pitch = (float)(atan2(-KX122_Accel_X_OUT, sqrt(KX122_Accel_Y_OUT * KX122_Accel_Y_OUT + KX122_Accel_Z_OUT * KX122_Accel_Z_OUT))) * RAD_TO_DEG;
    float norm = Norm(KX122_Accel_X_OUT, KX122_Accel_Y_OUT, KX122_Accel_Z_OUT);
    float pitchA = -asin(KX122_Accel_X_OUT / norm);             // * RAD_TO_DEG;
    float rollA = asin(KX122_Accel_Y_OUT / cos(pitchA) / norm); // * RAD_TO_DEG;
    float magX = BM1422_Mag_X - mag_off[0];
    float magY = BM1422_Mag_Y - mag_off[1];
    float magZ = BM1422_Mag_Z - mag_off[2];
    norm = sqrt(magX * magX + magY * magY + magZ * magZ);
    float mx = magX / norm;
    float my = -1 * magY / norm;
    float mz = magZ / norm;
    float Mx = mx * cos(pitchA) + mz * sin(pitchA);
    float My = mx * sin(rollA) * sin(pitchA) + my * cos(rollA) - mz * sin(rollA) * cos(pitchA);
    float yaw = atan2(-My, Mx) * RAD_TO_DEG;
    if (yaw > 360)
    {
        yaw -= 360;
    }
    else if (yaw < 0)
    {
        yaw += 360;
    }
    tStop = millis();
    tElapse = tStop - tStart;
    tStart = millis();
    float temp = tElapse / 1000.0;
    KFAngle[0] = KFilter[0].getAngle(roll, KXG03_Gyro_X, temp);
    KFAngle[1] = KFilter[1].getAngle(pitch, KXG03_Gyro_Y, temp);
    KFAngle[2] = KFilter[2].getAngle(yaw, KXG03_Gyro_Z, temp);
    float gyro_newangle = KXG03_Gyro_Z * temp;
    yaw_gyro += gyro_newangle;
    CFAngle[2] = 0.98 * (CFAngle[2] + gyro_newangle) + 0.02 * yaw;
    char t;
    if (Serial.available() > 0)
    {
        while (Serial.available() > 0)
        {
            t = Serial.read();
        }
        Serial.flush();
        Serial.print(KFAngle[0], 4);
        Serial.println();
        Serial.print(KFAngle[1], 4);
        Serial.println();
        Serial.print(KFAngle[2], 4);
        Serial.println();
    }
}
void GetAccelData()
{
//----- START Code for Reading KX122 Accel Sensor -----
#ifdef KX122
    i2c_start(KX122_DeviceAddress);
    i2c_write(0x06);
    i2c_rep_start(KX122_DeviceAddress | 1); // Or-ed with "1" for read bit
    KX122_Accel_X_LB = i2c_read(false);
    KX122_Accel_X_HB = i2c_read(false);
    KX122_Accel_Y_LB = i2c_read(false);
    KX122_Accel_Y_HB = i2c_read(false);
    KX122_Accel_Z_LB = i2c_read(false);
    KX122_Accel_Z_HB = i2c_read(true);
    i2c_stop();
    KX122_Accel_X_RawOUT = (KX122_Accel_X_HB << 8) | (KX122_Accel_X_LB);
    KX122_Accel_Y_RawOUT = (KX122_Accel_Y_HB << 8) | (KX122_Accel_Y_LB);
    KX122_Accel_Z_RawOUT = (KX122_Accel_Z_HB << 8) | (KX122_Accel_Z_LB);
    KX122_Accel_X_OUT = (float)KX122_Accel_X_RawOUT / 16384;
    KX122_Accel_Y_OUT = (float)-1 * KX122_Accel_Y_RawOUT / 16384;
    KX122_Accel_Z_OUT = (float)KX122_Accel_Z_RawOUT / 16384;
#endif
#ifdef RawDataOutput1
    Serial.write("KX122 (X) = ");
    Serial.print(KX122_Accel_X_OUT);
    Serial.write(" g");
    Serial.write(0x0A); // Print Line Feed
    Serial.write(0x0D); // Print Carrage Return
    Serial.write("KX122 (Y) = ");
    Serial.print(KX122_Accel_Y_OUT);
    Serial.write(" g");
    Serial.write(0x0A); // Print Line Feed
    Serial.write(0x0D); // Print Carrage Return
    Serial.write("KX122 (Z) = ");
    Serial.print(KX122_Accel_Z_OUT);
    Serial.write(" g");
    Serial.write(0x0A); // Print Line Feed
    Serial.write(0x0D); // Print Carrage Return
#endif
    //----- END Code for Reading KX122 Accel Sensor -----
}
void GetGyroData()
{
//----- START Code for Reading KXG03 Gyro Sensor -----
#ifdef KXG03
    i2c_start(KXG03_DeviceAddress);
    i2c_write(0x02);
    i2c_rep_start(KXG03_DeviceAddress | 1);
    KXG03_Gyro_X_LB = i2c_read(false);
    KXG03_Gyro_X_HB = i2c_read(false);
    KXG03_Gyro_Y_LB = i2c_read(false);
    KXG03_Gyro_Y_HB = i2c_read(false);
    KXG03_Gyro_Z_LB = i2c_read(false);
    KXG03_Gyro_Z_HB = i2c_read(true);
    i2c_stop();
    KXG03_Gyro_X_RawOUT = (KXG03_Gyro_X_HB << 8) | (KXG03_Gyro_X_LB);
    KXG03_Gyro_Y_RawOUT = (KXG03_Gyro_Y_HB << 8) | (KXG03_Gyro_Y_LB);
    KXG03_Gyro_Z_RawOUT = (KXG03_Gyro_Z_HB << 8) | (KXG03_Gyro_Z_LB);
    // Scale Data
    KXG03_Gyro_X = (float)-1 * (KXG03_Gyro_X_RawOUT * 0.007813 + 0.000004);
    KXG03_Gyro_Y = (float)KXG03_Gyro_Y_RawOUT * 0.007813 + 0.000004;
    KXG03_Gyro_Z = (float)-1 * (KXG03_Gyro_Z_RawOUT * 0.007813 + 0.000004);
#endif
//----- END Code for Reading KXG03 Gyro Sensor -----
#ifdef RawDataOutput1
    Serial.write("KXG03 Gyro (X) = ");
    Serial.print(KXG03_Gyro_X);
    Serial.write(" deg/sec");
    Serial.write(0x0A); // Print Line Feed
    Serial.write(0x0D); // Print Carrage Return
    Serial.write("KXG03 Gyro (Y) = ");
    Serial.print(KXG03_Gyro_Y);
    Serial.write(" deg/sec");
    Serial.write(0x0A); // Print Line Feed
    Serial.write(0x0D); // Print Carrage Return
    Serial.write("KXG03 Gyro (Z) = ");
    Serial.print(KXG03_Gyro_Z);
    Serial.write(" deg/sec");
    Serial.write(0x0A); // Print Line Feed
    Serial.write(0x0D); // Print Carrage Return
#endif
}
void GetMagnData()
{
#ifdef MagField
    i2c_start(BM1422_DeviceAddress);
    i2c_write(0x10);
    i2c_rep_start(BM1422_DeviceAddress | 1);
    BM1422_Mag_X_LB = i2c_read(false);
    BM1422_Mag_X_HB = i2c_read(false);
    BM1422_Mag_Y_LB = i2c_read(false);
    BM1422_Mag_Y_HB = i2c_read(false);
    BM1422_Mag_Z_LB = i2c_read(false);
    BM1422_Mag_Z_HB = i2c_read(true);
    i2c_stop();
    BM1422_Mag_X_RawOUT = (BM1422_Mag_X_HB << 8) | (BM1422_Mag_X_LB);
    BM1422_Mag_Y_RawOUT = (BM1422_Mag_Y_HB << 8) | (BM1422_Mag_Y_LB);
    BM1422_Mag_Z_RawOUT = (BM1422_Mag_Z_HB << 8) | (BM1422_Mag_Z_LB);
    BM1422_Mag_X = BM1422_Mag_X_RawOUT * 0.042;
    BM1422_Mag_Y = BM1422_Mag_Y_RawOUT * 0.042;
    BM1422_Mag_Z = BM1422_Mag_Z_RawOUT * 0.042;
#endif
#ifdef RawDataOutput1
    Serial.write("BM1422 Mag (X) = ");
    Serial.print(BM1422_Mag_X);
    Serial.write("uT");
    Serial.write(0x0A); // Print Line Feed
    Serial.write(0x0D); // Print Carrage Return
    Serial.write("BM1422 Mag (Y) = ");
    Serial.print(BM1422_Mag_Y);
    Serial.write("uT");
    Serial.write(0x0A); // Print Line Feed
    Serial.write(0x0D); // Print Carrage Return
    Serial.write("BM1422 Mag (Z) = ");
    Serial.print(BM1422_Mag_Z);
    Serial.write("uT");
    Serial.write(0x0A); // Print Line Feed
    Serial.write(0x0D); // Print Carrage Return
#endif
}
float Norm(float a, float b, float c)
{
    return sqrt(a * a + b * b + c * c);
}