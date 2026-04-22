#include "ui.h"
#include "config.h"
#include <Wire.h>
#include <stdio.h>
#include <string.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_WIDTH 128
#define OLED_HEIGHT 64

static Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire);
static bool oledReady = false;

static void copyEllipsized(const char *text, char *buffer, size_t bufferSize, size_t maxChars) {
  if (bufferSize == 0) {
    return;
  }

  if (maxChars + 1 >= bufferSize || strlen(text) <= maxChars) {
    snprintf(buffer, bufferSize, "%s", text);
    return;
  }

  if (maxChars < 4) {
    snprintf(buffer, bufferSize, "%.*s", (int)maxChars, text);
    return;
  }

  size_t keepChars = maxChars - 3;
  memcpy(buffer, text, keepChars);
  buffer[keepChars] = '\0';
  strcat(buffer, "...");
}

static void drawSelectedRow(int y, const char *label, bool selected) {
  if (selected) {
    display.fillRect(0, y, OLED_WIDTH, 9, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  } else {
    display.setTextColor(SSD1306_WHITE);
  }
  display.setCursor(2, y + 1);
  display.print(label);
  display.setTextColor(SSD1306_WHITE);
}

static void drawTuneMenuRow(int y, const char *text, bool selected) {
  if (selected) {
    display.fillRect(0, y, OLED_WIDTH, 10, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  } else {
    display.setTextColor(SSD1306_WHITE);
  }
  display.setCursor(2, y + 1);
  display.print(text);
  display.setTextColor(SSD1306_WHITE);
}

static int mapAnalogBar(int value, int maxHeight) {
  value = constrain(value, 0, 4095);
  return map(value, 0, 4095, 0, maxHeight);
}

static int mapAccelBar(int value, int maxWidth) {
  value = constrain(value, -1024, 1024);
  return map(abs(value), 0, 1024, 0, maxWidth);
}

static const char *wallMenuLabel(int index) {
  switch (index) {
    case WALL_MENU_SIDE: return "Wall";
    case WALL_MENU_DISTANCE: return "Dist";
    case WALL_MENU_KP: return "Kp";
    case WALL_MENU_MOTOR_SPEED: return "Motor";
    default: return "Value";
  }
}

static void drawWallMenuRow(int y, const char *text, bool selected) {
  if (selected) {
    display.fillRect(0, y, OLED_WIDTH, 8, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  } else {
    display.setTextColor(SSD1306_WHITE);
  }
  display.setCursor(2, y);
  display.print(text);
  display.setTextColor(SSD1306_WHITE);
}

void initUI() {
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_YELLOW, OUTPUT);
  pinMode(PIN_LEFT_TURN, OUTPUT);
  pinMode(PIN_RIGHT_TURN, OUTPUT);
  pinMode(PIN_BRAKE_LIGHT, OUTPUT);
  pinMode(PIN_SPEAKER, OUTPUT);
  digitalWrite(PIN_SPEAKER, LOW);

  Wire.setSDA(OLED_SDA);
  Wire.setSCL(OLED_SCL);
  oledReady = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (!oledReady) {
    Serial.println("OLED init failed");
    return;
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.display();
}

void drawHomeScreen(int selectedIndex) {
  if (!oledReady) {
    return;
  }
  static const char *const items[] = {
    "1. Display Sensors",
    "2. Tune Motors/Servo",
    "3. Outputs Test",
    "4. Wall Follow",
    "5. Run The Race",
    "6. Steps"
  };

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Home");
  for (int i = 0; i < (int)(sizeof(items) / sizeof(items[0])); ++i) {
    drawSelectedRow(10 + i * 9, items[i], i == selectedIndex);
  }
  display.display();
}

void drawSensorsScreen(int irL, int irC, int irR, int ldr, int ax, int ay, int az, bool accelReady) {
  if (!oledReady) {
    return;
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Sensors");

  int irHeightL = mapAnalogBar(irL, 18);
  int irHeightC = mapAnalogBar(irC, 18);
  int irHeightR = mapAnalogBar(irR, 18);
  display.drawRect(2, 10, 18, 20, SSD1306_WHITE);
  display.drawRect(24, 10, 18, 20, SSD1306_WHITE);
  display.drawRect(46, 10, 18, 20, SSD1306_WHITE);
  display.fillRect(4, 28 - irHeightL, 14, irHeightL, SSD1306_WHITE);
  display.fillRect(26, 28 - irHeightC, 14, irHeightC, SSD1306_WHITE);
  display.fillRect(48, 28 - irHeightR, 14, irHeightR, SSD1306_WHITE);

  display.setCursor(68, 10);
  display.print("L ");
  display.println(irL);
  display.setCursor(68, 18);
  display.print("C ");
  display.println(irC);
  display.setCursor(68, 26);
  display.print("R ");
  display.println(irR);

  display.setCursor(0, 34);
  display.print("Light ");
  display.println(ldr);

  if (accelReady) {
    int xw = mapAccelBar(ax, 36);
    int yw = mapAccelBar(ay, 36);
    int zw = mapAccelBar(az, 36);
    display.setCursor(0, 44);
    display.print("X");
    display.drawRect(10, 44, 38, 6, SSD1306_WHITE);
    display.fillRect(11, 45, xw, 4, SSD1306_WHITE);
    display.setCursor(52, 44);
    display.print("Y");
    display.drawRect(62, 44, 38, 6, SSD1306_WHITE);
    display.fillRect(63, 45, yw, 4, SSD1306_WHITE);
    display.setCursor(0, 54);
    display.print("Z");
    display.drawRect(10, 54, 38, 6, SSD1306_WHITE);
    display.fillRect(11, 55, zw, 4, SSD1306_WHITE);
    display.setCursor(52, 54);
    display.print(ax);
    display.print("/");
    display.print(ay);
    display.print("/");
    display.print(az);
  } else {
    display.setCursor(0, 48);
    display.println("Accel not found");
  }

  display.display();
}

void drawTuneScreen(int servoAngle, int motorPercent, bool motorRunning, int selectedItem, bool editing) {
  if (!oledReady) {
    return;
  }
  char line[24];

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Servo/Motor");
  display.setCursor(86, 0);
  display.println(motorRunning ? "RUN" : "STOP");
  display.setCursor(0, 10);
  display.print(editing ? "EDIT" : "NAV");
  display.setCursor(46, 10);
  display.print("Start=run");

  snprintf(line, sizeof(line), "Servo %3d deg", servoAngle);
  drawTuneMenuRow(24, line, selectedItem == 0);

  snprintf(line, sizeof(line), "Motor %3d %%", motorPercent);
  drawTuneMenuRow(36, line, selectedItem == 1);

  snprintf(line, sizeof(line), "Drive %s", motorRunning ? "ON" : "OFF");
  drawTuneMenuRow(48, line, selectedItem == 2);

  display.setCursor(0, 60);
  if (selectedItem == 2) {
    display.print("Click toggle");
  } else {
    display.print(editing ? "Turn to change" : "Click to edit");
  }

  display.display();
}

void drawOutputsScreen(const char *label, bool outputOn, int index, int count) {
  if (!oledReady) {
    return;
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Outputs Test");
  display.setTextSize(2);
  display.setCursor(0, 18);
  display.print(outputOn ? "ON " : "OFF");
  display.print(index + 1);
  display.print("/");
  display.println(count);
  display.setTextSize(1);
  display.setCursor(0, 44);
  display.println(label);
  display.setCursor(0, 54);
  display.println("Short toggle");
  display.display();
}

void drawWallFollowScreen(const WallFollowScreenData &screenData) {
  if (!oledReady) {
    return;
  }

  const WallFollowStatus &status = screenData.status;
  const WallFollowTuning &tuning = screenData.tuning;
  char line[24];

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Wall Follow");
  display.setCursor(86, 0);
  display.print(screenData.motorsRunning ? "RUN" : "STOP");

  display.setCursor(0, 10);
  display.print(wallFollowSideName(status.selectedWall));
  display.print(" ");
  display.print(wallFollowStateName(status.state));
  display.setCursor(88, 10);
  display.print(screenData.editing ? "EDIT" : "NAV");
  display.setCursor(0, 20);
  display.print("W ");
  display.print(status.activeWallDistanceInches, 1);
  display.print(" T ");
  display.print(tuning.targetWallDistanceInches, 1);
  display.print(" S ");
  display.println(status.steeringAngle);
  display.setCursor(0, 28);
  display.print("Err ");
  display.print(status.wallErrorInches, 1);
  display.print(" C ");
  display.print(status.centerRawAdc);
  display.print(" O ");
  display.println(status.controlOutputDegrees, 0);

  display.drawLine(0, 36, OLED_WIDTH - 1, 36, SSD1306_WHITE);

  snprintf(line, sizeof(line), "Wall  %s", wallFollowSideName(status.selectedWall));
  drawWallMenuRow(38, line, screenData.selectedItem == WALL_MENU_SIDE);

  snprintf(line, sizeof(line), "Dist  %.1fin", tuning.targetWallDistanceInches);
  drawWallMenuRow(46, line, screenData.selectedItem == WALL_MENU_DISTANCE);

  snprintf(line, sizeof(line), "Kp    %.1f", tuning.kp);
  drawWallMenuRow(54, line, screenData.selectedItem == WALL_MENU_KP);

  snprintf(line, sizeof(line), "Motor %d%%", tuning.motorSpeedPercent);
  drawWallMenuRow(62 - 6, line, screenData.selectedItem == WALL_MENU_MOTOR_SPEED);

  display.display();
}

void drawRunRaceScreen(bool running, const WallFollowStatus &status) {
  if (!oledReady) {
    return;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Run The Race");
  display.setCursor(98, 0);
  display.print(running ? "RUN" : "WAIT");

  display.setCursor(0, 12);
  display.println("Mode RIGHT WALL");
  display.setCursor(0, 24);
  display.println(running ? "STOP to brake" : "Press START");
  display.setCursor(0, 36);
  display.print("State ");
  display.println(wallFollowStateName(status.state));
  display.setCursor(0, 48);
  display.print("Wall ");
  display.print(status.activeWallDistanceInches, 1);
  display.print("in");
  display.setCursor(0, 56);
  display.print("Steer ");
  display.print(status.steeringAngle);
  display.display();
}

void drawStepsScreen(const char *const *stepNames, int stepCount, int selectedIndex, bool running) {
  if (!oledReady || stepCount <= 0) {
    return;
  }

  char line[32];
  char trimmed[24];
  int firstVisible = constrain(selectedIndex - 1, 0, max(0, stepCount - 4));

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Steps");
  display.setCursor(92, 0);
  display.print(running ? "RUN" : "READY");

  copyEllipsized(stepNames[selectedIndex], trimmed, sizeof(trimmed), 20);
  snprintf(line, sizeof(line), "%d/%d %s", selectedIndex + 1, stepCount, trimmed);
  display.setCursor(0, 10);
  display.print(line);

  for (int row = 0; row < 4; ++row) {
    int itemIndex = firstVisible + row;
    if (itemIndex >= stepCount) {
      break;
    }

    copyEllipsized(stepNames[itemIndex], trimmed, sizeof(trimmed), 16);
    snprintf(line, sizeof(line), "%d. %s", itemIndex + 1, trimmed);
    drawTuneMenuRow(20 + row * 11, line, itemIndex == selectedIndex);
  }

  display.display();
}
