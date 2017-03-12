#include <NewPing.h>
#include <Wire.h>
#include <VL6180X.h>

#define SONAR_NUM 3      // Number of ultrasonic sensors.
#define MAX_DISTANCE 200 // Maximum distance in cm
#define TOF1_pin 10
#define TOF2_pin 11
#define TOF3_pin 12

#define ldrThreshold 700
#define TOFcalibrationAmount 5
#define UltrasonicCalibrationAmount 123
#define numberOfSamples 10 // for ULtrasonics only

VL6180X sensor1;
VL6180X sensor2;
VL6180X sensor3;

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
        NewPing(4, 5, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
        NewPing(6, 7, MAX_DISTANCE),
        NewPing(8, 9, MAX_DISTANCE)
};

int data[10];
int average[3][numberOfSamples];
int arrayIndex = 0;
uint8_t lineFollower = 0;
long lastPress;

void setup() {

        Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
        Wire.begin();

        pinMode(TOF1_pin, OUTPUT);
        pinMode(TOF2_pin, OUTPUT);
        pinMode(TOF3_pin, OUTPUT);
        // Disbale all 3 TOF sensors
        digitalWrite(TOF1_pin, LOW);
        digitalWrite(TOF2_pin, LOW);
        digitalWrite(TOF3_pin, LOW);

        digitalWrite(TOF1_pin, HIGH); // Bring sensor 1 online
        delay(50);
        sensor1.init();
        sensor1.configureDefault();
        sensor1.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
        sensor1.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
        sensor1.setTimeout(500);
        sensor1.setAddress(0x54); // Set sensor 1 new address

        digitalWrite(TOF2_pin, HIGH); // Bring sensor 2 online
        delay(50);
        sensor2.init();
        sensor2.configureDefault();
        sensor2.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
        sensor2.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
        sensor2.setTimeout(500);
        sensor2.setAddress(0x55); // Set sensor 2 new address

        digitalWrite(TOF3_pin, HIGH); // Bring sensor 3 online
        delay(50);
        sensor3.init();
        sensor3.configureDefault();
        sensor3.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
        sensor3.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
        sensor3.setTimeout(500);
        sensor3.setAddress(0x56); // Set sensor 3 new address

        // Start all 3 sensors on interleaved mode continuous
        sensor1.stopContinuous();
        sensor2.stopContinuous();
        sensor3.stopContinuous();
        delay(300);
        sensor1.startInterleavedContinuous(100);
        sensor2.startInterleavedContinuous(100);
        sensor3.startInterleavedContinuous(100);
        lastPress = millis();
}

void loop()
{
        for (int x=0; x<SONAR_NUM; x++)
        {
          delay(29);
          unsigned int distanceReceived = sonar[0].ping_mm();
            average[x][arrayIndex] = distanceReceived-UltrasonicCalibrationAmount;
            if (average[x][arrayIndex]<0) average[x][arrayIndex] = 0;
          if (average[x][arrayIndex]>(MAX_DISTANCE*10)) average[x][arrayIndex] = 0;

        }
        arrayIndex++;
        if (arrayIndex>(numberOfSamples-1)) arrayIndex=0;

        unsigned long averageA = 0;
        unsigned long averageB = 0;
        unsigned long averageC = 0;

        for (int x=0; x<numberOfSamples; x++)
        {
                averageA = averageA + average[0][x];
                averageB = averageB + average[1][x];
                averageC = averageC + average[2][x];
        }

        data[0] = (unsigned long)averageA/numberOfSamples;
        data[1] = (unsigned long)averageB/numberOfSamples;
        data[2] = (unsigned long)averageC/numberOfSamples;

        data[3] = sensor1.readRangeContinuousMillimeters()-TOFcalibrationAmount;
        data[4] = sensor2.readRangeContinuousMillimeters()-TOFcalibrationAmount;
        data[5] = sensor3.readRangeContinuousMillimeters()-TOFcalibrationAmount;

        readLDRs();
        outputJSONString();
}

void outputJSONString()
{
        char buffer [100];
        sprintf (buffer, "{\"UL\":%u,\"UC\":%u,\"UR\":%u,\"TL\":%u,\"TC\":%u,\"TR\":%u,\"LF\":%u}", data[0], data[1], data[2], data[3], data[4], data[5], lineFollower);
        Serial.println(buffer);
}

void readLDRs()
{
        lineFollower=0;
        if(analogRead(A0)>ldrThreshold) lineFollower |= 1 << 2;
        if(analogRead(A1)>ldrThreshold) lineFollower |= 1 << 3;
        if(analogRead(A2)>ldrThreshold) lineFollower |= 1 << 4;
        if(analogRead(A3)>ldrThreshold) lineFollower |= 1 << 5;
        if(analogRead(A6)>ldrThreshold) lineFollower |= 1 << 6;
        if(analogRead(A7)>ldrThreshold) lineFollower |= 1 << 7;
}
