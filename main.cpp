#include <Arduino.h>
#include <Wire.h>
#include <LSM6DS3.h> // IMU sensor library

#define RECORD_BUTTON_PIN 7    // Pin for record button
#define ENTER_BUTTON_PIN 8     // Pin for enter button
#define LED_INDICATOR_PIN 13   // Success indicator LED

// Global variables
LSM6DS3 imu(I2C_MODE, 0x6A);  // IMU object
float storedSequence[50][6];  // Stored [ax,ay,az,gx,gy,gz], 50 is sequence_length
int storedLength = 0;
bool isRecording = false;
bool isChecking = false;

// Structure to hold IMU data
struct ImuData {
    float ax, ay, az;  // Accelerometer data
    float gx, gy, gz;  // Gyroscope data
};

// Function prototypes
void setupIMU();
void setupButtons();
void recordSequence();
void checkSequence();
float compareSequences(float recorded[], float stored[], int length);
ImuData getImuData();

void setup() {
    Serial.begin(9600);
    Wire.begin();
    
    setupIMU();
    setupButtons();
    
    pinMode(LED_INDICATOR_PIN, OUTPUT);
    digitalWrite(LED_INDICATOR_PIN, LOW);
}

void loop() {
    // Check if record button is pressed
    if (digitalRead(RECORD_BUTTON_PIN) == LOW) {
        delay(50);
        if (digitalRead(RECORD_BUTTON_PIN) == LOW) {
            Serial.println("Recording new sequence...");
            recordSequence();
        }
    }
    
    // Check if enter button is pressed
    if (digitalRead(ENTER_BUTTON_PIN) == LOW) {
        delay(50);
        if (digitalRead(ENTER_BUTTON_PIN) == LOW) {
            Serial.println("Checking sequence...");
            checkSequence();
        }
    }
}

void setupIMU() {
    if (!imu.begin()) {
        Serial.println("IMU initialization failed!");
        while (1);
    }
}

void recordSequence() {
    isRecording = true;
    int sampleCount = 0;
    
    // Recording start
    digitalWrite(LED_INDICATOR_PIN, HIGH);
    delay(500);
    digitalWrite(LED_INDICATOR_PIN, LOW);
    
    while (sampleCount < SEQUENCE_LENGTH) {
        ImuData data = getImuData();
        
        storedSequence[sampleCount][0] = data.ax;
        storedSequence[sampleCount][1] = data.ay;
        storedSequence[sampleCount][2] = data.az;
        storedSequence[sampleCount][3] = data.gx;
        storedSequence[sampleCount][4] = data.gy;
        storedSequence[sampleCount][5] = data.gz;
        
        sampleCount++;
        delay(20);
    }
    
    storedLength = sampleCount;
    isRecording = false;
    
    // Record complete
    digitalWrite(LED_INDICATOR_PIN, HIGH);
    delay(200);
    digitalWrite(LED_INDICATOR_PIN, LOW);
    delay(200);
    digitalWrite(LED_INDICATOR_PIN, HIGH);
    delay(200);
    digitalWrite(LED_INDICATOR_PIN, LOW);
}

ImuData getImuData() {
    ImuData data;
    data.ax = imu.readFloatAccelX();
    data.ay = imu.readFloatAccelY();
    data.az = imu.readFloatAccelZ();
    data.gx = imu.readFloatGyroX();
    data.gy = imu.readFloatGyroY();
    data.gz = imu.readFloatGyroZ();
    return data;
}