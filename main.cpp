#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_CircuitPlayground.h>

// IMU Definitions
#define LIS3DH_ADDR 0x18
#define CTRL_REG1 0x20
#define CTRL_REG4 0x23
#define OUT_X_L 0x28
#define OUT_Y_L 0x2A
#define OUT_Z_L 0x2C

// Button and LED Definitions
#define LEFT_BUTTON_PIN 4     // PD4
#define RIGHT_BUTTON_PIN 6    // PF6
#define RED_LED_PIN 7        // PC7
#define SLIDE_SWITCH_PIN 4    // PF4

// Gesture Recognition
#define SEQUENCE_LENGTH 600
float storedSequence[SEQUENCE_LENGTH][3];  // [ax,ay,az]
int storedLength = 0;
bool isRecording = false;
bool isChecking = false;

// Password System
int count1 = 0;
int count2 = 0;
bool button_pressed = false;
int start_timer = 0;
int retries = 0;

struct ImuData {
    float ax, ay, az;
};

// Function declarations
void setupIMU();
ImuData getImuData();
void recordSequence();
void checkSequence();
float compareSequences(float* recorded, float* stored, int length);
void Locking_sequence(int start_time);
void Unlocking_sequence(int start_time);

void setup() {
    // Initialize Circuit Playground
    CircuitPlayground.begin();
    
    // Button and LED setup
    DDRD &= ~(1 << LEFT_BUTTON_PIN);  // Left Button INPUT
    DDRF &= ~(1 << RIGHT_BUTTON_PIN); // Right Button INPUT
    DDRC |= (1 << RED_LED_PIN);       // Red LED OUTPUT
    DDRF &= ~(1 << SLIDE_SWITCH_PIN); // Slide switch input
    
    // Initial LED state
    PORTC &= ~(1 << RED_LED_PIN);     // Start with LED turned off
    
    Serial.begin(9600);
}

void loop() {
    // Check right button state
    if ((PINF & (1 << RIGHT_BUTTON_PIN)) != 0) {
        button_pressed = true;
    }

    // Check slide switch
    if ((PINF & (1 << SLIDE_SWITCH_PIN)) != 0) {
        PORTC &= ~(1 << RED_LED_PIN);
    }

    start_timer = millis();

    // Password System Logic
    if (button_pressed) {
        if ((PORTC & (1 << RED_LED_PIN)) != 0) {
            if (retries < 3) {
                Unlocking_sequence(start_timer);
                retries++;
                Serial.println(retries);
            } else {
                Serial.println("System locked - too many attempts");
                delay(200);
            }
        } else {
            Locking_sequence(start_timer);
            retries = 0;
        }
        button_pressed = false;
    }

    // Gesture Recognition Logic - Modified left button check
    if ((PIND & (1 << LEFT_BUTTON_PIN)) != 0) {  // Changed to != 0 to match the password system
        if ((PORTC & (1 << RED_LED_PIN)) == 0) {  // Only if system is unlocked
            if (!isRecording && !isChecking) {
                Serial.println("Starting gesture recording...");
                recordSequence();
            }
        }
    }
}

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

ImuData getImuData() {
    ImuData data;
    uint8_t buffer[6];
    
    Wire.beginTransmission(LIS3DH_ADDR);
    Wire.write(OUT_X_L | 0x80);  // Read all x, y, and z, start from x
    Wire.endTransmission(false);
    Wire.requestFrom(LIS3DH_ADDR, 6);
    
    for(int i = 0; i < 6; i++) {
        buffer[i] = Wire.read();
    }
    
    data.ax = (float)((int16_t)((buffer[1] << 8) | buffer[0])) * 2.0 / 32768.0;
    data.ay = (float)((int16_t)((buffer[3] << 8) | buffer[2])) * 2.0 / 32768.0;
    data.az = (float)((int16_t)((buffer[5] << 8) | buffer[4])) * 2.0 / 32768.0;
    
    return data;
}


void recordSequence() {
    Serial.println("Recording started - 10 second gesture");
    isRecording = true;
    int sampleCount = 0;
    unsigned long startTime = millis();
    
    // Initial blink to indicate start
    CircuitPlayground.redLED(true);
    delay(500);
    CircuitPlayground.redLED(false);
    
    // Record for exactly 10 seconds regardless of buffer size
    while ((millis() - startTime) < 10000) {
        // Use CircuitPlayground's built-in accelerometer functions
        float x = CircuitPlayground.motionX();
        float y = CircuitPlayground.motionY();
        float z = CircuitPlayground.motionZ();
        
        if (sampleCount < SEQUENCE_LENGTH) {  // Only store if we have space
            storedSequence[sampleCount][0] = x;
            storedSequence[sampleCount][1] = y;
            storedSequence[sampleCount][2] = z;
        }
        
        // Print the values
        Serial.print("Time: ");
        Serial.print((millis() - startTime) / 1000.0, 1);  // Show time in seconds
        Serial.print("s - Sample "); 
        Serial.print(sampleCount);
        Serial.print(": X=");
        Serial.print(x, 2);
        Serial.print(" Y=");
        Serial.print(y, 2);
        Serial.print(" Z=");
        Serial.println(z, 2);
        
        // Progress indicator - blink every 2 seconds
        if ((millis() - startTime) % 2000 < 50) {
            CircuitPlayground.redLED(true);
        } else {
            CircuitPlayground.redLED(false);
        }
        
        sampleCount++;
        delay(20);  // Sample every 20ms
    }
    
    storedLength = min(sampleCount, SEQUENCE_LENGTH);
    isRecording = false;
    
    Serial.print("Recording complete. Collected ");
    Serial.print(sampleCount);
    Serial.println(" samples");
    
    // Double blink to indicate completion
    for(int i = 0; i < 2; i++) {
        CircuitPlayground.redLED(true);
        delay(200);
        CircuitPlayground.redLED(false);
        delay(200);
    }
}

void Locking_sequence(int start_time) {
    Serial.println("Locking Sequence Start");
    int time_now = millis();
    count1 = 0;
    
    while(time_now - start_time <= 3000) {
        time_now = millis();
        if ((PIND & (1 << LEFT_BUTTON_PIN)) != 0) {
            count1++;
            delay(200);
        }
    }
    
    Serial.print("Password set: ");
    Serial.println(count1);

    if (time_now - start_time >= 3000) {
        PORTC |= (1 << RED_LED_PIN);
        Serial.println("System Locked");
        Serial.println();
    }
}

void Unlocking_sequence(int start_time) {
    Serial.println("Unlocking Sequence Start");
    int time_now = millis();
    count2 = 0;

    while(time_now - start_time <= 3000) {
        time_now = millis();
        if ((PIND & (1 << LEFT_BUTTON_PIN)) != 0) {
            count2++;
            delay(200);
        }
    }

    Serial.print("Attempt: ");
    Serial.println(count2);

    if (time_now - start_time >= 3000) {
        if (count2 == count1) {
            Serial.println("Access Granted");
            PORTC &= ~(1 << RED_LED_PIN);
            Serial.println();
        } else {
            Serial.println("Access Denied");
            Serial.println();
        }
    }
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