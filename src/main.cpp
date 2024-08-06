#include <Arduino.h>
#include <M5unified.h>
#include <Wire.h>

#include "MAX30100.h"

// Sampling is tightly related to the dynamic range of the ADC.
// refer to the datasheet for further info
#define SAMPLING_RATE MAX30100_SAMPRATE_100HZ
//#define SAMPLING_RATE MAX30100_SAMPRATE_50HZ

// The LEDs currents must be set to a level that avoids clipping and maximises the
// dynamic range
#define IR_LED_CURRENT MAX30100_LED_CURR_50MA
#define RED_LED_CURRENT MAX30100_LED_CURR_27_1MA

// The pulse width of the LEDs driving determines the resolution of
// the ADC (which is a Sigma-Delta).
// set HIGHRES_MODE to true only when setting PULSE_WIDTH to MAX30100_SPC_PW_1600US_16BITS
#define PULSE_WIDTH MAX30100_SPC_PW_1600US_16BITS
#define HIGHRES_MODE true

// Instantiate a MAX30100 sensor class
MAX30100 sensor;

// display: 240 x 135
#define X 240
#define Y 135
#define N 256
#define Nbit 8

uint16_t val_red[N], val_ir[N];
uint8_t px = 0;

#define Navg_base 200
#define Navg_data 10
#define Navg_slope 10
int data0 = 0;
uint8_t fRamp = 0, fRamp0 = 0;
int slope[Navg_slope];
uint8_t pSlope = 0;
uint16_t tNP = 0, tPN = 0, tPN0 = 0, periodPN = 0, periodNP = 0;
uint8_t fValid = 0;
uint32_t sum_base = 0, sum_data = 0;
long sum_slope = 0;

void setup()
{
  M5.begin();

  if (!sensor.begin())
  {
    printf("fail\n");
    for (;;)
      ;
  }
  else
  {
    printf("ok\n");
  }

  // Set up the wanted parameters
  sensor.setMode(MAX30100_MODE_SPO2_HR);
  sensor.setLedsCurrent(IR_LED_CURRENT, RED_LED_CURRENT);
  sensor.setLedsPulseWidth(PULSE_WIDTH);
  sensor.setSamplingRate(SAMPLING_RATE);
  sensor.setHighresModeEnabled(HIGHRES_MODE);
  M5.Lcd.clear();
  for (uint16_t x = 0; x < N; x++)
  {
    val_red[x] = 0;
    val_ir[x] = 0;
  }
}

uint8_t f = 0;

void loop()
{
  M5.update();
  uint16_t ir, red;
  sensor.update();

  while (sensor.getRawValues(&ir, &red))
  {
    val_red[px] = red;
    val_ir[px] = ir;
    uint16_t px0;
    sum_base = 0;
    px0 = (X + px - Navg_base - 1) % X;
    for (uint8_t i = 0; i < Navg_base; i++)
      sum_base += val_red[(px0 + i) % X];
    sum_data = 0;
    px0 = (X + px - Navg_data - 1) % X;
    for (uint8_t i = 0; i < Navg_data; i++)
      sum_data += val_red[(px0 + i) % X];
    uint16_t base = sum_base / Navg_base;
    int data = sum_data / Navg_data - base;
    slope[pSlope] = data - data0;
    pSlope = (pSlope + 1) % Navg_slope;
    sum_slope = 0;
    for (uint8_t i = 0; i < Navg_slope; i++)
      sum_slope += slope[i];
    int avg_slope = sum_slope / Navg_slope;
    if (avg_slope > 0) fRamp = 1; else fRamp = 0;

    if (fRamp0 == 0 && fRamp == 1)
      tNP = millis();
    else if (fRamp0 == 1 && fRamp == 0){
        tPN = millis();
        periodPN = tPN - tNP;
        periodNP = tNP - tPN0;
        tPN0 = tPN;
    }
    fRamp0 = fRamp;
    data0 = data;
    if (periodPN > 400 && periodPN < 800 && periodNP > 100 && periodNP < 300)
      fValid = 1;
    else
      fValid = 0;
    printf("%d %d %d %d %d\n", red, base, data, periodPN, periodNP);
    //    printf(">red:%d\n", val_red[px]);
    //    printf(">max:%d\n", max_red);
    //    printf(">min:%d\n", min_red);
    f++;
    if (f == 4) // draw every 4 samples
    {
      f = 0;
      uint16_t min_red = 65535, min_ir = 65535, max_red = 0, max_ir = 0;
      for (uint8_t x = 0; x < X; x++)
      {
        if (val_red[x] > max_red)
          max_red = val_red[x];
        if (val_red[x] < min_red)
          min_red = val_red[x];
        if (val_ir[x] > max_ir)
          max_ir = val_ir[x];
        if (val_ir[x] < min_ir)
          min_ir = val_ir[x];
      }
      uint16_t mag_red, mag_ir;
      if ((max_red - min_red) < Y)
        mag_red = 1;
      else
        mag_red = (max_red - min_red) / Y;
      if ((max_ir - min_ir) < Y)
        mag_ir = 1;
      else
        mag_ir = (max_ir - min_ir) / Y;
      uint8_t dx = px;
      int y_red, y_ir;
      for (uint8_t x = 0; x < X; x++)
      {
        y_red = (val_red[dx] - min_red) / mag_red;
        if (y_red > Y - 1)
          y_red = Y - 1;
        if (y_red < 0)
          y_red = 0;
        y_ir = (val_ir[dx] - min_ir) / mag_ir;
        if (y_ir > Y - 1)
          y_ir = Y - 1;
        if (y_ir < 0)
          y_ir = 0;
        M5.Lcd.drawFastHLine(0, x, Y, BLACK);
        if (fValid == 1)
          M5.Lcd.drawPixel(Y - y_red, x, WHITE);
        else
          M5.Lcd.drawPixel(Y - y_red, x, RED);
        //      M5.Lcd.drawPixel(Y - y_ir, x, GREEN);
        dx = (dx + 1) % X;
      }
    }
    px = (px + 1) % X;
  }
}
