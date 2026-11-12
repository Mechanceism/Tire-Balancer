#include <Wire.h>

#define I2C_SDA 33
#define I2C_SCL 32
#define TOUCH_ADDR 0x15

#define SCREEN_WIDTH 240  // Your actual screen resolution
#define SCREEN_HEIGHT 320
#define X_MIN 30  // Set these to 0 and touch near the screen boarder to get these values
#define X_MAX 210
#define Y_MIN 9
#define Y_MAX 300
bool InvertXY = false;  // enable/disable Inxerting the X and Y axis
bool MirrorX = true;   // enable/disable Mirroring the X axis
bool MirrorY = false;   // enable/disable Mirroring the Y axis

// Declare variables to hold x, y, and press state
uint16_t touchX = 0;
uint16_t touchY = 0;
bool touched = false;

void setup() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.begin(115200);
  Serial.println("Starting touch controller...");
}

void loop() {
  // Call function to get touch data and update x, y, touched
  processTouchData(touchX, touchY, touched);

  // Example usage of updated x, y, touched in loop
  if (touched) {
    Serial.print("Touch detected - X: ");
    Serial.print(touchX);
    Serial.print(", Y: ");
    Serial.println(touchY);
  }

  delay(20);  // Faster updates if you want
}

// Function to process touch data
void processTouchData(uint16_t &touchX, uint16_t &touchY, bool &touched) {
  Wire.beginTransmission(TOUCH_ADDR);
  Wire.write(0x00); // Basically wakes up the chip to ask it for stuff
  Wire.endTransmission(false);

  Wire.requestFrom(TOUCH_ADDR, 9);

  if (Wire.available() == 9) {
    uint8_t buf[9];
    for (int i = 0; i < 9; i++) {
      buf[i] = Wire.read();
    }

    uint8_t fingerNum = buf[2];
    if (fingerNum == 1) {
      uint16_t rawX = (buf[3] << 8) | buf[4];
      uint16_t rawY = (buf[5] << 8) | buf[6];

      uint16_t tempX = (rawX & 0x8000) ? (rawX - 0x8000) : rawX;
      uint16_t tempY = rawY;

      if (MirrorX) {
        tempX = map(tempX, X_MAX, X_MIN, 0, SCREEN_WIDTH);
      } else {
        tempX = map(tempX, X_MIN, X_MAX, 0, SCREEN_WIDTH);
      }

      if (MirrorY) {
        tempY = map(tempY, Y_MAX, Y_MIN, 0, SCREEN_HEIGHT);
      } else {
        tempY = map(tempY, Y_MIN, Y_MAX, 0, SCREEN_HEIGHT);
      }

      uint16_t SCREEN_X = SCREEN_WIDTH;
      uint16_t SCREEN_Y = SCREEN_HEIGHT;

      if (InvertXY) {
        uint16_t tempXMirror = tempY;
        tempY = tempX;
        tempX = tempXMirror;
        uint16_t SCREEN_HEIGHTMirror = SCREEN_Y;
        SCREEN_Y = SCREEN_WIDTH;
        SCREEN_X = SCREEN_HEIGHTMirror;
      }

      // Validate X/Y before updating
      if (tempX <= SCREEN_X && tempY <= SCREEN_Y) {
        touchX = tempX;
        touchY = tempY;
        touched = true; // A touch is detected
      }
      else {
        touched = false; // No valid touch data
      }
    } else {
      touched = false; // No touch or multi-touch, so set to false
    }
  }
}
