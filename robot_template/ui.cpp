#include "ui.h"
#include "config.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_WIDTH 128
#define OLED_HEIGHT 64

static Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire);
static bool oledReady = false;

static void drawSelectedRow(int y, const char *label, bool selected) {
  if (selected) {
    display.fillRect(0, y, 128, 12, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  } else {
    display.setTextColor(SSD1306_WHITE);
  }
  display.setCursor(2, y + 2);
  display.print(label);
  display.setTextColor(SSD1306_WHITE);
}

static void drawTuneRow(
  int y,
  const char *label,
  int value,
  int minValue,
  int maxValue,
  bool selected,
  bool fillBar
) {
  const int labelW = 56;
  const int rowH = 22;
  const int gaugeX = 62;
  const int gaugeY = y + 8;
  const int gaugeW = 60;
  const int gaugeH = 6;

  if (selected) {
    display.fillRect(0, y, labelW, rowH, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  } else {
    display.drawRect(0, y, labelW, rowH, SSD1306_WHITE);
    display.setTextColor(SSD1306_WHITE);
  }

  display.setTextSize(1);
  display.setCursor(4, y + 2);
  display.print(label);
  display.setTextSize(2);
  display.setCursor(4, y + 6);
  if (value < 100) {
    display.print('0');
  }
  if (value < 10) {
    display.print('0');
  }
  display.print(value);

  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.drawRect(gaugeX, gaugeY, gaugeW, gaugeH, SSD1306_WHITE);
  int markerX = map(constrain(value, minValue, maxValue), minValue, maxValue, gaugeX + 1, gaugeX + gaugeW - 2);

  if (fillBar) {
    int fillW = map(constrain(value, minValue, maxValue), minValue, maxValue, 0, gaugeW - 2);
    if (fillW > 0) {
      display.fillRect(gaugeX + 1, gaugeY + 1, fillW, gaugeH - 2, SSD1306_WHITE);
    }
  } else {
    int centerX = map(90, minValue, maxValue, gaugeX + 1, gaugeX + gaugeW - 2);
    display.drawLine(centerX, gaugeY - 2, centerX, gaugeY + gaugeH + 1, SSD1306_WHITE);
    display.fillRect(markerX - 1, gaugeY - 2, 3, gaugeH + 4, SSD1306_WHITE);
  }
}

static int mapAnalogBar(int value, int maxHeight) {
  value = constrain(value, 0, 4095);
  return map(value, 0, 4095, 0, maxHeight);
}

static int mapAccelBar(int value, int maxWidth) {
  value = constrain(value, -1024, 1024);
  return map(abs(value), 0, 1024, 0, maxWidth);
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
  static const char *items[] = {
    "1. Display Sensors",
    "2. Tune Motors/Servo",
    "3. Outputs Test",
    "4. Wall Follow"
  };

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Home");
  for (int i = 0; i < 4; ++i) {
    drawSelectedRow(14 + i * 12, items[i], i == selectedIndex);
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

void drawTuneScreen(int servoAngle, int motorPercent, bool servoSelected, bool motorRunning) {
  if (!oledReady) {
    return;
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Servo/Motor");
  display.setCursor(88, 0);
  display.println(motorRunning ? "RUN" : "STOP");

  drawTuneRow(12, "Servo", servoAngle, 0, 180, servoSelected, false);
  drawTuneRow(38, "Motor", motorPercent, 0, 100, !servoSelected, true);

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

// Draw the Wall Follow setup / tuning screen.
// Layout (128×64, text size 1 = 6×8 px per glyph):
//   y= 0  "WallFollow" title + current FSM state name
//   y= 9  param row 0: Wall side
//   y=18  param row 1: Front obstacle threshold
//   y=27  param row 2: Front clear threshold
//   y=36  param row 3: Target wall distance
//   y=45  param row 4: Proportional gain Kp
//   y=54  param row 5: Motor speed
// The selected row is highlighted (white fill, black text).
// Short-press encoder cycles through rows; encoder rotation adjusts the value.
void drawWallFollowScreen(const char *wallName, const char *stateName,
                          int paramSelect,
                          float frontDet, float frontClr, float target,
                          float kp, int speed) {
  if (!oledReady) {
    return;
  }
  display.clearDisplay();
  display.setTextSize(1);

  // ── Title row ─────────────────────────────────────────────────────────────
  display.setCursor(0, 0);
  display.print("WallFollow");
  display.setCursor(70, 0);
  display.print(stateName);   // "IDLE  " / "FOLLOW" / "AVOID "

  // ── Parameter rows (9 px tall each) ──────────────────────────────────────
  char buf[22];

  // Helper lambda-equivalent: draw one highlighted row
  for (int row = 0; row < 6; ++row) {
    int y = 9 + row * 9;
    bool sel = (row == paramSelect);

    if (sel) {
      display.fillRect(0, y, 128, 9, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }

    display.setCursor(2, y + 1);

    switch (row) {
      case 0:
        snprintf(buf, sizeof(buf), "Wall:  %s", wallName);
        break;
      case 1:
        snprintf(buf, sizeof(buf), "FrntDt:%4.1fin", frontDet);
        break;
      case 2:
        snprintf(buf, sizeof(buf), "FrntCl:%4.1fin", frontClr);
        break;
      case 3:
        snprintf(buf, sizeof(buf), "Target:%4.1fin", target);
        break;
      case 4:
        snprintf(buf, sizeof(buf), "Kp:   %5.1f", kp);
        break;
      case 5:
        snprintf(buf, sizeof(buf), "Speed:    %3d%%", speed);
        break;
      default:
        buf[0] = '\0';
        break;
    }

    display.print(buf);
    display.setTextColor(SSD1306_WHITE);
  }

  display.display();
}
