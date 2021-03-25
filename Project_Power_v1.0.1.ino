// Created by Steven Marshall
// Created 23/03/2020
// v1.0.1
// https://github.com/smarshall-rightside/Project_Power/
// Licensed for non-commercial use ONLY
// Do not remove this header



#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>

U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R2);

unsigned long startMillis;
unsigned long currentMillis;
unsigned long time_passed;
unsigned long lastReset;

const unsigned long period = 50;

const int sensorHistoryLength = 128;
int sensorHistory[sensorHistoryLength];
int sensorHistoryPos = sensorHistoryLength - 1;
int boostPressure;
int boostMax = 0;
int BoostMaxTop = 0;
int boostMin = 0;
float inbar;
float maxinbar;

void setup(void) {
  u8g2.begin();
  startMillis = millis();
  time_passed = millis();
}


void loop(void) {
  // Only read from the sensors every 50 ms
  currentMillis = millis();
  if (currentMillis - startMillis >= period) {
    readSensorData();
    startMillis = currentMillis;
    // Set Max Boost Value
    if (boostPressure >= BoostMaxTop)
    {
      BoostMaxTop = boostPressure;
    }
    else if (boostPressure <  BoostMaxTop)
    {
      if (millis() - lastReset > 5000) {
        lastReset += 5000;
        time_passed = 0;
        BoostMaxTop = boostPressure;
      }
    }
  }

  u8g2.firstPage();
  do {
    // Draw current pressure
    u8g2.setFont(u8g2_font_fub20_tf);
    // Change PSI to BAR
    inbar = boostPressure / 100;
    char cstd[6];
    dtostrf((float)inbar / 14.504, 1, 1, cstd);
    u8g2.drawStr(0, 20, cstd);

    // Draw boost text
    u8g2.setFont(u8g2_font_helvR08_tf);
    u8g2.drawStr(45, 20, "BAR");

    // Draw boost text
    if (boostPressure >= 3500)
    {
      u8g2.setFont(u8g2_font_fub11_tf);
      u8g2.drawStr(45, 10, "MAX");
    }
    else if (boostPressure < 3500)
    {
      u8g2.setFont(u8g2_font_helvR08_tf);
      u8g2.drawStr(45, 10, "");
    }

    // Draw max pressure
    u8g2.setFont(u8g2_font_fub11_tf);
    // Change PSI to BAR
    maxinbar = BoostMaxTop / 100;
    char csta[6];
    dtostrf((float)maxinbar / 14.504, 1, 1, csta);

    int yPos = u8g2.getStrWidth(csta);
    u8g2.drawStr(128 - yPos, 11, csta);

    drawBarGraph(0, 22, 128, 8);
    drawGraph(0, 32, 128, 31);

  } while ( u8g2.nextPage() );
}

float normaliseSensorData(int m) {
  /*
    Scale the sensor reading into range
    m = measurement to be scaled
    rmin = minimum of the range of the measurement
    rmax = maximum of the range of the measurement
    tmin = minimum of the range of the desired target scaling
    tmax = maximum of the range of the desired target scaling
    normalisedValue = ((m − rmin) / (rmax − rmin)) * (tmax − tmin) + tmin
    https://stats.stackexchange.com/a/281164
  */

  /*
    Sensor voltage ranges from 0.5v to 4.5v, converted to analogRead values (0 min, 1023 max) that's 102 to 921. Times the voltage by 204.66
    rmin = 102
    rmax = 921
    Sensor reads from 0 to 50psi
    tmin = 0
    tmax = 5000
    normalisedValue = ((m − 102) / (921 − 102)) * (5000 − 0) + 0
    normalisedValue = ((m − 102) / 819) * 5000
    normalisedValue = (m − 102) / 0.1638
  */

  return (m - 102) / 0.1638;
}

void readSensorData(void) {
  float absolutePressure = normaliseSensorData(analogRead(A0));

  // Subtract 14.7 psi == pressure at sea level
  // Additional 2.57psi subtracted as boost was showing 2.57 with engine off
  boostPressure = absolutePressure - 1727;

  // Update max and min
  if (boostPressure > boostMax) boostMax = boostPressure;
  if (boostPressure < boostMin) boostMin = boostPressure;

  // Log the history
  addSensorHistory(boostPressure);
}

void addSensorHistory(int val) {
  sensorHistory[sensorHistoryPos] = val;
  sensorHistoryPos--;
  if (sensorHistoryPos < 0) sensorHistoryPos = sensorHistoryLength - 1;
}

int getSensorHistory(int index) {
  index += sensorHistoryPos;
  if (index >= sensorHistoryLength) index = index - sensorHistoryLength;
  return sensorHistory[index];
}

// Display functions
void drawGraph(int x, int y, int len, int height) {
  // Draw the lines
  drawHorizontalDottedLine(x, y, len);
  drawHorizontalDottedLine(x, y + height, len);

  //var absMin = Math.abs(boostMin);
  int absMin = abs(boostMin);
  int range = absMin + boostMax;

  // Draw 0 line
  int zeroYPos = mapValueToYPos(absMin, range, y, height);
  drawHorizontalDottedLine(x, zeroYPos, len);

  // Draw the graph line
  for (int i = 0; i < 128; i++) {
    // Scale the values so that the min is always 0
    int valueY = getSensorHistory(i) + absMin;

    // Calculate the coordinants
    int yPos = mapValueToYPos(valueY, range, y, height);
    int xPos = len - i;
    if (yPos < zeroYPos) {
      // Point is above zero line, fill in space under graph
      u8g2.drawVLine(xPos, yPos, zeroYPos + 1 - yPos);
    } else {
      // Point is below zero line, draw graph line without filling in
      u8g2.drawPixel(xPos, yPos);
    }
  }
}

void drawBarGraph(int x, int y, int len, int height) {
  if (boostPressure > 0) {
    // Draw the pressure bar behind the graph
    int barLength = ((float)boostPressure / boostMax) * len;
    u8g2.setDrawColor(2);
    u8g2.drawBox(x, y, barLength, height);
    u8g2.setDrawColor(1);
  }
}

// Maps a value to a y height
int mapValueToYPos(int val, int range, int y, int height) {
  float valueY = ((float)val / range) * height;
  return y + height - (int)valueY;
}

void drawHorizontalDottedLine(int x, int y, int len) {
  for (int i = 0; i < len; i++) {
    if (!(i % 4)) u8g2.drawPixel(x + i, y);
  }
}
