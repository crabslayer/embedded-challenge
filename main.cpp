#include <Arduino.h>
#include <Wire.h>
#include <LSM6DS3.h> // IMU sensor library

#define RECORD_BUTTON_PIN     // Pin for record button TODO
#define ENTER_BUTTON_PIN      // Pin for enter button TODO
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
void setupIMU(); // TODO
void setupButtons(); // TODO
void recordSequence(); 
void checkSequence(); // TODO
float compareSequences(float recorded[], float stored[], int length); // TODO
ImuData getImuData();

void setup() {
    Serial.begin(9600);
    Wire.begin();
    
    setupIMU(); // TODO
    setupButtons(); // TODO
    
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
void setupTimer() {
    // CTC mode, prescaler 64
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
    
    OCR1A = 249;
    
    sei(); 
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