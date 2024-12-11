#include <Arduino.h>
#include <Adafruit_CircuitPlayground.h>

#define SEQUENCE_LENGTH 50
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
bool systemLocked = false;  // Track LED state

// Button hold timing
unsigned long buttonPressStart = 0;
const int HOLD_DURATION = 1000;  // Hold for 1 second to check gesture

void checkSequence();
void Locking_sequence(int start_time);
void Unlocking_sequence(int start_time);
void recordSequence();
float compareSequences(float* recorded, float* stored, int length);

void setup() {
    CircuitPlayground.begin();
    Serial.begin(9600);
    CircuitPlayground.redLED(false);
    systemLocked = false;
}

void loop() {
    // Left button for recording gesture
    if (CircuitPlayground.leftButton()) {
        if (!systemLocked && !isRecording && !isChecking) {
            recordSequence();
        }
    }

    // Right button with hold detection
    if (CircuitPlayground.rightButton()) {
        if (!button_pressed) {  // Button just pressed
            button_pressed = true;
            buttonPressStart = millis();
            start_timer = millis();
            
            // Visual feedback that button is being held
            CircuitPlayground.setPixelColor(0, 0, 0, 255);
        } else {
            // Check if button has been held long enough
            if (millis() - buttonPressStart >= HOLD_DURATION) {
                // Clear hold indicator
                CircuitPlayground.setPixelColor(0, 0, 0, 0);
                
                if (!systemLocked && storedLength > 0 && !isRecording && !isChecking) {
                    checkSequence();
                    button_pressed = false;  // Reset after checking
                }
            }
        }
    } else {
        if (button_pressed) {  // Button was just released
            unsigned long holdTime = millis() - buttonPressStart;
            
            // Clear hold indicator
            CircuitPlayground.setPixelColor(0, 0, 0, 0);
            
            // Only trigger lock/unlock on short presses
            if (holdTime < HOLD_DURATION) {
                if (systemLocked) {
                    if (retries < 3) {
                        Unlocking_sequence(start_timer);
                        retries++;
                        Serial.print("Attempt #");
                        Serial.println(retries);
                    } else {
                        Serial.println("System locked - too many attempts");
                        delay(200);
                    }
                } else {
                    Locking_sequence(start_timer);
                    retries = 0;
                }
            }
        }
        button_pressed = false;
    }

    // Check slide switch for override
    if (CircuitPlayground.slideSwitch()) {
        CircuitPlayground.redLED(false);
        systemLocked = false;
        retries = 0;
    }
}

void Locking_sequence(int start_time) {
    Serial.println("Locking Sequence Start");
    int time_now = millis();
    count1 = 0;
    
    // Visual feedback - blink blue LED
    CircuitPlayground.setPixelColor(0, 0, 0, 255);
    
    while(time_now - start_time <= 3000) {
        time_now = millis();
        if (CircuitPlayground.leftButton()) {
            count1++;
            CircuitPlayground.setPixelColor(count1 % 10, 0, 0, 255); // Visual feedback
            Serial.print("Button press: ");
            Serial.println(count1);
            delay(200);  // Debounce
        }
    }
    
    // Clear NeoPixels
    for(int i=0; i<10; i++) {
        CircuitPlayground.setPixelColor(i, 0, 0, 0);
    }
    
    Serial.print("Password set: ");
    Serial.println(count1);

    CircuitPlayground.redLED(true);  // Lock the system
    systemLocked = true;
    Serial.println("System Locked");
    Serial.println();
}

void Unlocking_sequence(int start_time) {
    Serial.println("Unlocking Sequence Start");
    int time_now = millis();
    count2 = 0;
    
    // Visual feedback - blink green LED
    CircuitPlayground.setPixelColor(0, 0, 255, 0);

    while(time_now - start_time <= 3000) {
        time_now = millis();
        if (CircuitPlayground.leftButton()) {
            count2++;
            CircuitPlayground.setPixelColor(count2 % 10, 0, 255, 0); // Visual feedback
            Serial.print("Button press: ");
            Serial.println(count2);
            delay(200);  // Debounce
        }
    }
    
    // Clear NeoPixels
    for(int i=0; i<10; i++) {
        CircuitPlayground.setPixelColor(i, 0, 0, 0);
    }

    Serial.print("Attempt: ");
    Serial.println(count2);

    if (count2 == count1) {
        Serial.println("Access Granted");
        CircuitPlayground.redLED(false);
        systemLocked = false;
        
        // Success animation
        for(int i=0; i<10; i++) {
            CircuitPlayground.setPixelColor(i, 0, 255, 0);
            delay(50);
        }
        delay(500);
        for(int i=0; i<10; i++) {
            CircuitPlayground.setPixelColor(i, 0, 0, 0);
        }
    } else {
        Serial.println("Access Denied");
        
        // Failure animation
        for(int i=0; i<3; i++) {
            CircuitPlayground.setPixelColor(0, 255, 0, 0);
            delay(100);
            CircuitPlayground.setPixelColor(0, 0, 0, 0);
            delay(100);
        }
    }
    Serial.println();
}

void recordSequence() {
    Serial.println("Recording started - 3 second gesture");
    isRecording = true;
    int sampleCount = 0;
    unsigned long startTime = millis();
    
    // Start recording indicator
    CircuitPlayground.setPixelColor(0, 255, 165, 0); // Orange
    
    while ((millis() - startTime) < 3000 && sampleCount < SEQUENCE_LENGTH) {
        storedSequence[sampleCount][0] = CircuitPlayground.motionX();
        storedSequence[sampleCount][1] = CircuitPlayground.motionY();
        storedSequence[sampleCount][2] = CircuitPlayground.motionZ();
        
        // Visual feedback - light up pixels based on motion
        int intensity = abs(storedSequence[sampleCount][0]) * 255;
        CircuitPlayground.setPixelColor(sampleCount % 10, intensity, 0, intensity);
        
        Serial.print("Sample "); 
        Serial.print(sampleCount);
        Serial.print(": X=");
        Serial.print(storedSequence[sampleCount][0], 2);
        Serial.print(" Y=");
        Serial.print(storedSequence[sampleCount][1], 2);
        Serial.print(" Z=");
        Serial.println(storedSequence[sampleCount][2], 2);
        
        sampleCount++;
        delay(20);
    }
    
    storedLength = sampleCount;
    isRecording = false;
    
    // Clear all pixels
    for(int i=0; i<10; i++) {
        CircuitPlayground.setPixelColor(i, 0, 0, 0);
    }
    
    // Completion animation
    for(int i=0; i<2; i++) {
        CircuitPlayground.setPixelColor(0, 0, 255, 0);
        delay(200);
        CircuitPlayground.setPixelColor(0, 0, 0, 0);
        delay(200);
    }
    
    Serial.print("Recording complete. Collected ");
    Serial.print(sampleCount);
    Serial.println(" samples");
}
void checkSequence() {
    Serial.println("Checking gesture - perform the same motion");
    isChecking = true;
    float currentSequence[SEQUENCE_LENGTH][3];
    int sampleCount = 0;
    unsigned long startTime = millis();
    
    // Start checking indicator
    CircuitPlayground.setPixelColor(0, 255, 0, 255); // Purple
    
    while ((millis() - startTime) < 3000 && sampleCount < storedLength) {
        currentSequence[sampleCount][0] = CircuitPlayground.motionX();
        currentSequence[sampleCount][1] = CircuitPlayground.motionY();
        currentSequence[sampleCount][2] = CircuitPlayground.motionZ();
        
        // Visual feedback
        int intensity = abs(currentSequence[sampleCount][0]) * 255;
        CircuitPlayground.setPixelColor(sampleCount % 10, intensity, 0, intensity);
        
        Serial.print("Check Sample "); 
        Serial.print(sampleCount);
        Serial.print(": X=");
        Serial.print(currentSequence[sampleCount][0], 2);
        Serial.print(" Y=");
        Serial.print(currentSequence[sampleCount][1], 2);
        Serial.print(" Z=");
        Serial.println(currentSequence[sampleCount][2], 2);
        
        sampleCount++;
        delay(20);
    }
    
    float similarity = compareSequences((float*)currentSequence, (float*)storedSequence, storedLength);
    Serial.print("Gesture match: ");
    Serial.print(similarity * 100);
    Serial.println("%");
    
    // Clear all pixels
    for(int i=0; i<10; i++) {
        CircuitPlayground.setPixelColor(i, 0, 0, 0);
    }
    
    // Show result
    if (similarity > 0.85) {  // 85% match threshold
        Serial.println("Gesture Matched!");
        // Success animation - green spiral
        for(int i=0; i<10; i++) {
            CircuitPlayground.setPixelColor(i, 0, 255, 0);
            delay(50);
        }
        delay(500);
    } else {
        Serial.println("Gesture Did Not Match");
        // Failure animation - red flash
        for(int i=0; i<3; i++) {
            for(int j=0; j<10; j++) {
                CircuitPlayground.setPixelColor(j, 255, 0, 0);
            }
            delay(100);
            for(int j=0; j<10; j++) {
                CircuitPlayground.setPixelColor(j, 0, 0, 0);
            }
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