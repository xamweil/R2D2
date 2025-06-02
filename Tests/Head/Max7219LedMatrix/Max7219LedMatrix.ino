#include <LedControl.h>

int DIN = 10;
int CS = 7;
int CLK = 8;

LedControl lc=LedControl(DIN, CLK, CS, 0);

// Global variable to keep track of the current command.
String currentCommand = "default";

void setup() {
  Serial.begin(9600);
  while (!Serial)
  // Wake the MAX7219 from power-saving mode.
  lc.shutdown(0, false);
  // Set brightness level (range is 0 to 15).
  lc.setIntensity(0, 8);
  // Clear the display to ensure all LEDs are off.
  lc.clearDisplay(0);

  Serial.println("Enter command: blink, chase, wave, alternate, sparkle, pwm, on, off, animation, pixelart, random");
  currentCommand = "none"; // Default state.
}


void clearDisplay() {
  lc.clearDisplay(0);
}

void displayOn() {
  for (int row=0; row<8; row++){
    for (int col=0; col<8; col++){
      lc.setLed(0, row, col, true);
    }
  }
}

bool checkForNewCommand() {
  if (Serial.available() > 0) {
    currentCommand = Serial.readStringUntil('\n');
    currentCommand.trim(); // Remove any leading/trailing whitespace.
    Serial.print("New command: ");
    Serial.println(currentCommand);
    return true;
  }
  return false;
}

void blinkAll() {
  while (currentCommand == "blink") {
    // Turn all LEDs ON.
    for (int row = 0; row < 8; row++) {
      for (int col = 0; col < 8; col++) {
        lc.setLed(0, row, col, true);
      }
    }
    delay(500);
    if(checkForNewCommand()) break;
    
    // Turn all LEDs OFF.
    for (int row = 0; row < 8; row++) {
      for (int col = 0; col < 8; col++) {
        lc.setLed(0, row, col, false);
      }
    }
    delay(500);
    if(checkForNewCommand()) break;
  }
  clearDisplay();
}
// Chase pattern: light one LED at a time moving sequentially through the matrix.
void chasePattern() {
  int pos = 0;
  const int totalLeds = 8 * 8;
  while (currentCommand == "chase") {
    clearDisplay();
    int row = pos / 8;
    int col = pos % 8;
    lc.setLed(0, row, col, true);
    delay(150);
    pos = (pos + 1) % totalLeds;
    if(checkForNewCommand()) break;
  }
  clearDisplay();
}

void wavePattern() {
  while (currentCommand == "wave") {
    for (int offset = 0; offset < 8; offset++) {
      clearDisplay();
      for (int row = 0; row < 8; row++) {
        // Calculate column position for this row to create a diagonal wave.
        int col = (row + offset) % 8;
        lc.setLed(0, row, col, true);
      }
      delay(150);
      if(checkForNewCommand()) return;
    }
  }
  clearDisplay();
}

void alternatePattern() {
  while (currentCommand == "alternate") {
    // Left segment ON, right segment OFF.
    for (int row = 0; row < 8; row++) {
      for (int col = 0; col < 4; col++) {
        lc.setLed(0, row, col, true);
      }
      for (int col = 4; col < 8; col++) {
        lc.setLed(0, row, col, false);
      }
    }
    delay(500);
    if(checkForNewCommand()) break;
    
    // Left segment OFF, right segment ON.
    for (int row = 0; row < 8; row++) {
      for (int col = 0; col < 4; col++) {
        lc.setLed(0, row, col, false);
      }
      for (int col = 4; col < 8; col++) {
        lc.setLed(0, row, col, true);
      }
    }
    delay(500);
    if(checkForNewCommand()) break;
  }
  clearDisplay();
}

void randomSparkle() {
  // Seed the random number generator using an unconnected analog pin.
  randomSeed(analogRead(A0));
  while (currentCommand == "sparkle") {
    clearDisplay();
    // Light a random set of 10 LEDs.
    for (int i = 0; i < 10; i++) {
      int r = random(8);
      int c = random(8);
      lc.setLed(0, r, c, true);
    }
    delay(100);
    if (checkForNewCommand()) break;
  }
  clearDisplay();
}

void pwmFade() {
  while (currentCommand == "pwm") {
    // Fade in: increase brightness from 0 to 15.
    for (int intensity = 0; intensity < 15; intensity++) {
      lc.setIntensity(0, intensity);
      delay(100);
      if (checkForNewCommand()) return;
    }
    // Fade out: decrease brightness from 15 to 0.
    for (int intensity = 15; intensity >= 0; intensity--) {
      lc.setIntensity(0, intensity);
      delay(100);
      if (checkForNewCommand()) return;
    }
  }
  clearDisplay();
}

void bouncingBallAnimation() {
  // Initialize positions and velocities (in LED-units per second)
  float x1 = 1.5, y1 = 3.5;   // Left ball starting position (in left segment)
  float vx1 = 2.0, vy1 = 1.5;   // Left ball velocity
  
  float x2 = 5.5, y2 = 4.5;     // Right ball starting position (in right segment)
  float vx2 = 2.0, vy2 = 1.7;   // Right ball velocity

  unsigned long previousTime = millis();

  while (currentCommand == "animation") {
    unsigned long currentTime = millis();
    // Calculate the elapsed time in seconds (dt)
    float dt = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;
    
    // --- Update positions for left ball ---
    x1 += vx1 * dt;
    y1 += vy1 * dt;
    
    // Bounce within left segment boundaries: x in [0, 3], y in [0, 7]
    if (x1 < 0)   { x1 = 0;  vx1 = -vx1; }
    if (x1 > 3)   { x1 = 3;  vx1 = -vx1; }
    if (y1 < 0)   { y1 = 0;  vy1 = -vy1; }
    if (y1 > 7)   { y1 = 7;  vy1 = -vy1; }
    
    // --- Update positions for right ball ---
    x2 += vx2 * dt;
    y2 += vy2 * dt;
    
    // Bounce within right segment boundaries: x in [4, 7], y in [0, 7]
    if (x2 < 4)   { x2 = 4;  vx2 = -vx2; }
    if (x2 > 7)   { x2 = 7;  vx2 = -vx2; }
    if (y2 < 0)   { y2 = 0;  vy2 = -vy2; }
    if (y2 > 7)   { y2 = 7;  vy2 = -vy2; }
    
    // Clear the display before drawing new positions
    clearDisplay();
    
    // Convert float positions to nearest integer LED indices
    int col1 = round(x1);
    int row1 = round(y1);
    int col2 = round(x2);
    int row2 = round(y2);
    
    // Draw each ball (one LED lit per ball)
    lc.setLed(0, row1, col1, true);
    lc.setLed(0, row2, col2, true);
    
    // Short delay to control animation speed and smooth out motion
    delay(20);
    if (checkForNewCommand()) break;
  }
  clearDisplay();
}

void pixelArtAnimation() {
  // Define two sprite frames for the walking animation.
  // Each frame is a 4x4 pattern.
  static const int sprite0[4][4] = {
    {0, 1, 1, 0},
    {1, 0, 0, 1},
    {1, 1, 1, 1},
    {1, 0, 0, 1}
  };
  static const int sprite1[4][4] = {
    {0, 1, 1, 0},
    {1, 0, 0, 1},
    {1, 1, 1, 1},
    {0, 1, 1, 0}
  };

  const int spriteWidth  = 4;
  const int spriteHeight = 4;
  // The sprite will slide horizontally from off-screen left to off-screen right.
  int startX = -spriteWidth;
  int endX   = 8; // beyond the rightmost column
  // Fixed vertical placement so that the sprite occupies rows 2 to 5.
  int fixedY = 2;

  // Variables to control frame switching.
  unsigned long lastFrameSwitchTime = millis();
  const int frameSwitchDelay = 300; // milliseconds between sprite frame changes
  bool useSecondFrame = false;       // false uses sprite0, true uses sprite1

  // Move the sprite horizontally.
  for (int x = startX; x <= endX; x++) {
    clearDisplay();

    // Check if it's time to toggle the sprite frame for the walking effect.
    if (millis() - lastFrameSwitchTime >= frameSwitchDelay) {
      useSecondFrame = !useSecondFrame;
      lastFrameSwitchTime = millis();
    }

    // Select current sprite based on walking frame.
    const int (*sprite)[4] = useSecondFrame ? sprite1 : sprite0;

    // Draw the sprite pixel by pixel.
    for (int sy = 0; sy < spriteHeight; sy++) {
      for (int sx = 0; sx < spriteWidth; sx++) {
        int screenX = x + sx;      // horizontal position on the display
        int screenY = fixedY + sy; // vertical position on the display

        // Only draw if the LED is within the display boundaries.
        if (screenX >= 0 && screenX < 8 && screenY >= 0 && screenY < 8) {
          if (sprite[sy][sx] == 1) {
            lc.setLed(0, screenY, screenX, true);
          } else {
            lc.setLed(0, screenY, screenX, false);
          }
        }
      }
    }
    delay(100);  // adjust delay for desired animation speed
    if (checkForNewCommand()) return;
  }
  clearDisplay();
}
void randomPattern() {
  randomSeed(micros());  // seed based on current time
  while (currentCommand == "random") {
    for (int row = 0; row < 8; row++) {
      for (int col = 0; col < 8; col++) {
        // random(2) returns 0 or 1
        lc.setLed(0, row, col, random(2));
      }
    }
    delay(300);               // adjust speed of random updates
    if (checkForNewCommand()) break;
  }
  clearDisplay();
}

void weightedRandom() {
  // 8×8 state array
  bool leds[8][8];
  // Seed RNG
  randomSeed(micros());
  // Initialize: 70% ON, 30% OFF
  for (int row = 0; row < 8; row++) {
    for (int col = 0; col < 8; col++) {
      leds[row][col] = (random(100) < 70);
    }
  }
  // Run until a new command arrives
  while (currentCommand == "default") {
    // For each LED, with 10% chance toggle its state
    for (int row = 0; row < 8; row++) {
      for (int col = 0; col < 8; col++) {
        if (random(100) < 10) {
          leds[row][col] = !leds[row][col];
        }
      }
    }
    // Update display from our state array
    for (int row = 0; row < 8; row++) {
      for (int col = 0; col < 8; col++) {
        lc.setLed(0, row, col, leds[row][col]);
      }
    }
    delay(100);                     // Control update speed
    if (checkForNewCommand()) break;
  }
  clearDisplay();
}

void loop() {
  // Check if a new command has been received.
  if (Serial.available() > 0) {
    currentCommand = Serial.readStringUntil('\n');
    currentCommand.trim(); // remove any extra spaces/newline characters
    Serial.print("Command received: ");
    Serial.println(currentCommand);
  }

  // Decide which animation function to run based on the current command.
  if (currentCommand == "blink") {
    blinkAll();
  } else if (currentCommand == "chase") {
    chasePattern();
  } else if (currentCommand == "wave") {
    wavePattern();
  } else if (currentCommand == "alternate") {
    alternatePattern();
  } else if (currentCommand == "sparkle") {
    randomSparkle();
  } else if (currentCommand == "pwm") {
    pwmFade();
  } else if (currentCommand == "on") {
    displayOn();
  } else if (currentCommand == "off") {
    clearDisplay();
  } else if (currentCommand == "animation") {
    bouncingBallAnimation();
  } else if (currentCommand == "pixelart") {
    pixelArtAnimation();
  } else if (currentCommand == "random") {
    randomPattern();
  } else if (currentCommand == "default") {
    weightedRandom();
  } else {
    // If no valid command is received, just idle.
    delay(10);
  }
}
