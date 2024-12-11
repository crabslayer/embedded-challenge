#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_CPlay_NeoPixel.h>


// ========================== Accelerometer Code ====================================

// #define CS 8

// // ================== LIS3DH Commands ====================================
// #define READ_CMD 0x80  // Read command
// #define MULTI_BYTE 0x40  // Multi-byte command

// // ================== CONTROL REGISTERS FOR ACCELEROMETER ================
// // uint8_t* CTRL_REG0 = (uint8_t *) 0x1E; // Since these addresses are not in my MCU, I can't use pointers
// // uint8_t* CTRL_REG1 = (uint8_t *) 0x20;
// // uint8_t* CTRL_REG4 = (uint8_t *) 0x24;

// #define CTRL_REG0 0x1E
// #define CTRL_REG2 0x21
// #define CTRL_REG1 0x20
// #define CTRL_REG4 0x23

// // ================== X, Y, and Z DIRECTIONAL REGISTERS ==================
// #define X_LOW 0x28
// #define X_HIGH 0x29
// #define Y_LOW 0x2A
// #define Y_HIGH 0x2B
// #define Z_LOW 0x2C
// #define Z_HIGH 0x2D


// // Function to write to one register
// void writeRegister(uint8_t reg, uint8_t value) {
//     digitalWrite(CS, LOW);
//     SPI.transfer(reg);
//     SPI.transfer(value);
//     digitalWrite(CS, HIGH);
// }

// // Function to read a register
// uint8_t readRegister(uint8_t reg) {
//     digitalWrite(CS, LOW);
//     SPI.transfer(reg | READ_CMD);
//     uint8_t value = SPI.transfer(0x00);
//     digitalWrite(CS, HIGH);
//     return value;
// }

// // Function to read multiple registers
// void readRegisters(uint8_t reg, uint8_t* buffer, uint8_t length) {
//     digitalWrite(CS, LOW);
//     SPI.transfer(reg | READ_CMD | MULTI_BYTE);
//     for (uint8_t i = 0; i < length; i++) {
//         buffer[i] = SPI.transfer(0x00);
//     }
//     digitalWrite(CS, HIGH);
// }

// void setup() {
//   SPI.begin();
//   pinMode(CS, OUTPUT);
//   digitalWrite(CS, HIGH);

//   writeRegister(0x1E, 0x20);       // CTRL_REG0: Configure as needed
//   writeRegister(0x20, 0x57);       // CTRL_REG1: Enable axes and set data rate
//   writeRegister(0x21, 0x08);       // CTRL_REG2: Activate High-Pass Filter
//   writeRegister(0x23, 0x00);       // CTRL_REG4: Set resolution and range


//   Serial.begin(9600);
// }


// int16_t calculateMovingAverage(int16_t* buffer, int size) {
//     int32_t sum = 0;
//     for (int i = 0; i < size; i++) {
//         sum += buffer[i];
//     }
//     return sum / size;
// }


// void loop() {
//   uint8_t rawData[6];  // Buffer to store X, Y, Z data
//   int16_t x, y, z;

//   // Read raw data
//   readRegisters(X_LOW, rawData, 6);
  
//   // Combine LSB and MSB
//   x = (int16_t)((rawData[1] << 8) | rawData[0]);
//   y = (int16_t)((rawData[3] << 8) | rawData[2]);
//   z = (int16_t)((rawData[5] << 8) | rawData[4]);

//   x = x>>6;
//   y = y>>6;
//   z = z>>6;
//   // x = x/32;
//   // y = y/32;
//   // z = z/32;

//   // Moving Average Calculations
//   int16_t MAx[7] = {0};
//   int16_t MAy[7] = {0};
//   int16_t MAz[7] = {0};


//   // Update buffers
//     MAx[0] = x;
//     MAy[0] = y;
//     MAz[0] = z;

//     // Calculate moving averages
//     int16_t xAverage = calculateMovingAverage(MAx, 7);
//     int16_t yAverage = calculateMovingAverage(MAy, 7);
//     int16_t zAverage = calculateMovingAverage(MAz, 7);


//   // Print values (convert to g if needed based on resolution and scale)
//   Serial.print("X: "); Serial.print(xAverage);
//   Serial.print("   Y: "); Serial.print(yAverage);
//   Serial.print("   Z: "); Serial.print(zAverage);
//   Serial.println();

//   delay(100);
// }


// =================== Code to read button inputs as password in 3-second interval ==============
Adafruit_CPlay_NeoPixel neopix (10, 8, NEO_RGBW + NEO_KHZ800);

void setup() {
  // Left Button PD4, Right Button PF6 (USB cord on top)
  // Red LED PC7
  // NeoPixels PB0
  // Slide Switch PF4

  DDRD &= ~(1<<4); // Left Button INPUT
  DDRF &= ~(1<<6); // Right Button INPUT
  DDRC |= (1<<7); // Red LED OUTPUT
  DDRF &= ~(1<<4); // Slide switch input

  PORTC &= ~(1<<7); // Start with LED turned off

  neopix.begin();
  neopix.show();

  Serial.begin(9600);
}

int count1 = 0;
int count2 = 0;
bool button_pressed = false;
int start_timer = 0;

void Locking_sequence(int start_time) {
  Serial.println("Locking Sequence Start");
  int time_now = millis();
  count1 = 0;
  
  // Find the time difference between time_now and the parameter start_time
  // while the time difference is not 3 seconds, keep updating time_now
  // During the 3 second wait, track number of times other button is pressed
  while(time_now - start_time <= 3000) {
    time_now = millis();
    if ((PIND & (1<<4)) != 0) {
      count1++;
      delay(200);
    }
  }
  Serial.print("times button pressed: ");
  Serial.println(count1);

  // After 3 seconds have passed, Red LED turns on letting user know that the board is "locked"
  if (time_now - start_time >= 3000) {
    PORTC |= (1<<7);
    Serial.println("Locking Sequence End");
    Serial.println();
  }
  return;
}

void Unlocking_sequence(int start_time) {
  Serial.println("Unlocking Sequence Start");
  int time_now = millis();
  count2 = 0;

  // Find the time difference between time_now and the parameter start_time
  // while the time difference is not 3 seconds, keep updating time_now
  // During the 3 second wait, track number of times other button is pressed
  while(time_now - start_time <= 3000) {
    time_now = millis();
    if ((PIND & (1<<4)) != 0) {
      count2++;
      delay(200);
    }
  }

  Serial.print("times button pressed: ");
  Serial.println(count2);

  // After 3 seconds have passed, compare button presses with the locking number of presses
  // If number of button presses matches, LED turns off, "unlocking" the board
  // If button presses don't match, press first button again to try password again
  if (time_now - start_time >= 3000) {
    Serial.println("Unlocking Sequence End");
    if (count2 == count1) {
      Serial.println("Correct Password, good job");
      PORTC &= ~(1<<7);
      // Serial.println(count2);
      // Serial.println(count1);
      Serial.println();
    }
    else {
      Serial.println("Wrong password, try again");
      // Serial.println(count2);
      // Serial.println(count1);
      Serial.println();
    }
  }
  return;
}

int retries = 0;

void loop() {

  if ((PINF & (1<<6)) != 0) {
    button_pressed = true;
  }

  if ((PINF & (1<<4)) != 0) {
    PORTC &= ~(1<<7);
  }

  start_timer = millis(); // Used as a base time_start for the locking and unlocking mechanisms

// When LED is off and button is pressed, this starts locking mechanism
  if (button_pressed && ((PORTC & (1<<7)) != 0) && (retries < 3)) { 
    Unlocking_sequence(start_timer);
    retries++;
    Serial.println(retries);
    button_pressed = !button_pressed;
  } 

// When LED is off and button is pressed, this starts unlocking mechanism
  else if (button_pressed && ((PORTC & (1<<7)) == 0)) {
    Locking_sequence(start_timer);
    retries = 0;
    button_pressed = !button_pressed;
  }

// If unlock attempts passes 3, no more attempts given, the board is locked forever.
// Press the reset button to restart the board, or implement slide switch to force shut off LED
  else if (button_pressed && ((PORTC & (1<<7)) != 0) && (retries >= 3)) {
    Serial.println("Sorry, too many attempts tried, System locked forever, unfortunate");
    delay(200);
    button_pressed = !button_pressed;
  }

}