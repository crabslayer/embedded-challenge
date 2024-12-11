#include <Arduino.h>
#include <Wire.h>
#define LIS3DH_ADDR 0x18

#define RECORD_BUTTON_PIN 7    // Using PD7
#define ENTER_BUTTON_PIN 4     // Using PD4
#define LED_INDICATOR_PIN 13   // Built-in LED
#define SEQUENCE_LENGTH 50

// LIS3DH registers
#define CTRL_REG1 0x20
#define CTRL_REG4 0x23
#define OUT_X_L 0x28
#define OUT_Y_L 0x2A
#define OUT_Z_L 0x2C

// Global variables
float storedSequence[SEQUENCE_LENGTH][3];  // [ax,ay,az]
int storedLength = 0;
bool isRecording = false;
bool isChecking = false;

struct ImuData {
    float ax, ay, az;
};

void setupIMU() {
    Wire.beginTransmission(LIS3DH_ADDR);
    Wire.write(CTRL_REG1);
    Wire.write(0x57);  // 100Hz, enable XYZ
    Wire.endTransmission();
    
    Wire.beginTransmission(LIS3DH_ADDR);
    Wire.write(CTRL_REG4);
    Wire.write(0x08);  // High resolution mode
    Wire.endTransmission();
}

void setupButtons() {
    DDRD &= ~((1 << RECORD_BUTTON_PIN) | (1 << ENTER_BUTTON_PIN));  // Inputs
    PORTD |= ((1 << RECORD_BUTTON_PIN) | (1 << ENTER_BUTTON_PIN));   // Pull-ups
}

ImuData getImuData() {
    ImuData data;
    uint8_t buffer[6];
    
    Wire.beginTransmission(LIS3DH_ADDR);
    Wire.write(OUT_X_L | 0x80);  // It read all x, y, and z, start from x. Simplified version. 
    Wire.endTransmission(false);
    Wire.requestFrom(LIS3DH_ADDR, 6);
    
    for(int i = 0; i < 6; i++) {
        buffer[i] = Wire.read();
    }
    
    // Combine LSB and MSB
    data.ax = (float)((int16_t)((buffer[1] << 8) | buffer[0])) * 2.0 / 32768.0;
    data.ay = (float)((int16_t)((buffer[3] << 8) | buffer[2])) * 2.0 / 32768.0;
    data.az = (float)((int16_t)((buffer[5] << 8) | buffer[4])) * 2.0 / 32768.0;
    
    return data;
}
void recordSequence() {
   isRecording = true;
   int sampleCount = 0;
   
   digitalWrite(LED_INDICATOR_PIN, HIGH);
   delay(500);
   digitalWrite(LED_INDICATOR_PIN, LOW);
   
   while (sampleCount < SEQUENCE_LENGTH) {
       ImuData data = getImuData();
       
       storedSequence[sampleCount][0] = data.ax;
       storedSequence[sampleCount][1] = data.ay;
       storedSequence[sampleCount][2] = data.az;
       
       sampleCount++;
       delay(20);
   }
   
   storedLength = sampleCount;
   isRecording = false;
   
   // Double blink to indicate completion
   for(int i = 0; i < 2; i++) {
       digitalWrite(LED_INDICATOR_PIN, HIGH);
       delay(200);
       digitalWrite(LED_INDICATOR_PIN, LOW);
       delay(200);
   }
}

void checkSequence() {
    isChecking = true;
    float currentSequence[SEQUENCE_LENGTH][3];
    int sampleCount = 0;
    
    digitalWrite(LED_INDICATOR_PIN, HIGH);
    delay(500);
    digitalWrite(LED_INDICATOR_PIN, LOW);
    
    while (sampleCount < storedLength) {
        ImuData data = getImuData();
        
        currentSequence[sampleCount][0] = data.ax;
        currentSequence[sampleCount][1] = data.ay;
        currentSequence[sampleCount][2] = data.az;
        
        sampleCount++;
        delay(20);
    }
    
    float similarity = compareSequences((float*)currentSequence, (float*)storedSequence, storedLength);
    
    if (similarity > 0.85) {  // 85% match threshold
        digitalWrite(LED_INDICATOR_PIN, HIGH);
        delay(1000);
    } else {
        // Failed match - quick blink
        for(int i = 0; i < 3; i++) {
            digitalWrite(LED_INDICATOR_PIN, HIGH);
            delay(100);
            digitalWrite(LED_INDICATOR_PIN, LOW);
            delay(100);
        }
    }
    
    isChecking = false;
}

float compareSequences(float* recorded, float* stored, int length) {
    float matchCount = 0;
    float tolerance = 0.3;  // ±0.3g tolerance
    
    for(int i = 0; i < length * 3; i++) {  // *3 for x,y,z values
        if(fabs(recorded[i] - stored[i]) < tolerance) {
            matchCount++;
        }
    }
    
    return matchCount / (length * 3);
}