#include <Wire.h>
// #include <hd44780.h>
// #include <hd44780ioClass/hd44780_I2Cexp.h>
#include <Servo.h>

// hd44780_I2Cexp lcd;
const int  LCD_COLS = 16;
const int  LCD_ROWS = 2;

const uint8_t  PWM_PIN        = 9;
const uint16_t PULSE_MIN_US   = 1000;
const uint16_t PULSE_MAX_US   = 2000;

Servo esc;

void setup() {
  // LCD
  // int status = lcd.begin(LCD_COLS, LCD_ROWS);
  // if (status) hd44780::fatalError(status);
  // lcd.clear();
  // lcd.print(F("Serial Control"));

  // ESC
  esc.attach(PWM_PIN, PULSE_MIN_US, PULSE_MAX_US);
  delay(3000);  // Give ESC time to arm

  // Serial
  Serial.begin(115200);
}

void loop() {
  if (Serial.available() >= 2) {
    uint8_t highByte = Serial.read();
    uint8_t lowByte  = Serial.read();
    uint16_t value = ((uint16_t)highByte << 8) | lowByte; // 0–65535

    // Map 16-bit value to pulse width in microseconds
    // Using long arithmetic to avoid overflow: (value * 1000) / 65535 + 1000
    uint16_t pulse = (uint16_t)((((uint32_t)value * (PULSE_MAX_US - PULSE_MIN_US)) / 65535UL) + PULSE_MIN_US);
    esc.writeMicroseconds(pulse);
  }
}