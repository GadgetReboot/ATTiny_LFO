/*
     Dual MCP4725 12-bit DACs
     Generating low frequency waveforms
     Uses ATTiny 1604
     Uses Adafruit MCP4725 library
     Uses Bounce2 library https://github.com/thomasfredericks/Bounce2

youtube.com/@GadgetReboot
*/

#include <Wire.h>
#include <Bounce2.h>
#include <Adafruit_MCP4725.h>

// create objects to access the LFO dacs
Adafruit_MCP4725 LFO1;
Adafruit_MCP4725 LFO2;

// button debouncers
Bounce button1 = Bounce();
Bounce button2 = Bounce();

// button and led pins assigned to Arduino digital pin numbers by mapping chip port pins from
// https://github.com/SpenceKonde/megaTinyCore/blob/master/megaavr/extras/ATtiny_x04.md
const byte btn1 = 8;
const byte btn2 = 9;
const byte led1 = 0;
const byte led2 = 1;
const byte pot1 = 2;
const byte pot2 = 3;

// sine waveform data table
const PROGMEM uint16_t sine_data[256] = {
  2048, 2098, 2148, 2198, 2248, 2298, 2348, 2398,
  2447, 2496, 2545, 2594, 2642, 2690, 2737, 2784,
  2831, 2877, 2923, 2968, 3013, 3057, 3100, 3143,
  3185, 3226, 3267, 3307, 3346, 3385, 3423, 3459,
  3495, 3530, 3565, 3598, 3630, 3662, 3692, 3722,
  3750, 3777, 3804, 3829, 3853, 3876, 3898, 3919,
  3939, 3958, 3975, 3992, 4007, 4021, 4034, 4045,
  4056, 4065, 4073, 4080, 4085, 4089, 4093, 4094,
  4095, 4094, 4093, 4089, 4085, 4080, 4073, 4065,
  4056, 4045, 4034, 4021, 4007, 3992, 3975, 3958,
  3939, 3919, 3898, 3876, 3853, 3829, 3804, 3777,
  3750, 3722, 3692, 3662, 3630, 3598, 3565, 3530,
  3495, 3459, 3423, 3385, 3346, 3307, 3267, 3226,
  3185, 3143, 3100, 3057, 3013, 2968, 2923, 2877,
  2831, 2784, 2737, 2690, 2642, 2594, 2545, 2496,
  2447, 2398, 2348, 2298, 2248, 2198, 2148, 2098,
  2048, 1997, 1947, 1897, 1847, 1797, 1747, 1697,
  1648, 1599, 1550, 1501, 1453, 1405, 1358, 1311,
  1264, 1218, 1172, 1127, 1082, 1038, 995, 952,
  910, 869, 828, 788, 749, 710, 672, 636,
  600, 565, 530, 497, 465, 433, 403, 373,
  345, 318, 291, 266, 242, 219, 197, 176,
  156, 137, 120, 103, 88, 74, 61, 50,
  39, 30, 22, 15, 10, 6, 2, 1,
  0, 1, 2, 6, 10, 15, 22, 30,
  39, 50, 61, 74, 88, 103, 120, 137,
  156, 176, 197, 219, 242, 266, 291, 318,
  345, 373, 403, 433, 465, 497, 530, 565,
  600, 636, 672, 710, 749, 788, 828, 869,
  910, 952, 995, 1038, 1082, 1127, 1172, 1218,
  1264, 1311, 1358, 1405, 1453, 1501, 1550, 1599,
  1648, 1697, 1747, 1797, 1847, 1897, 1947, 1997
};

// triangle waveform data table
// generated using https://www.daycounter.com/Calculators/Triangle-Wave-Generator-Calculator.phtml
const PROGMEM int16_t triangle_data[256] = {
  32, 64, 96, 128, 160, 192, 224, 256,
  288, 320, 352, 384, 416, 448, 480, 512,
  544, 576, 608, 640, 672, 704, 736, 768,
  800, 832, 864, 896, 928, 960, 992, 1024,
  1056, 1088, 1120, 1152, 1184, 1216, 1248, 1280,
  1312, 1344, 1376, 1408, 1440, 1472, 1504, 1536,
  1568, 1600, 1632, 1664, 1696, 1728, 1760, 1792,
  1824, 1856, 1888, 1920, 1952, 1984, 2016, 2048,
  2079, 2111, 2143, 2175, 2207, 2239, 2271, 2303,
  2335, 2367, 2399, 2431, 2463, 2495, 2527, 2559,
  2591, 2623, 2655, 2687, 2719, 2751, 2783, 2815,
  2847, 2879, 2911, 2943, 2975, 3007, 3039, 3071,
  3103, 3135, 3167, 3199, 3231, 3263, 3295, 3327,
  3359, 3391, 3423, 3455, 3487, 3519, 3551, 3583,
  3615, 3647, 3679, 3711, 3743, 3775, 3807, 3839,
  3871, 3903, 3935, 3967, 3999, 4031, 4063, 4095,
  4063, 4031, 3999, 3967, 3935, 3903, 3871, 3839,
  3807, 3775, 3743, 3711, 3679, 3647, 3615, 3583,
  3551, 3519, 3487, 3455, 3423, 3391, 3359, 3327,
  3295, 3263, 3231, 3199, 3167, 3135, 3103, 3071,
  3039, 3007, 2975, 2943, 2911, 2879, 2847, 2815,
  2783, 2751, 2719, 2687, 2655, 2623, 2591, 2559,
  2527, 2495, 2463, 2431, 2399, 2367, 2335, 2303,
  2271, 2239, 2207, 2175, 2143, 2111, 2079, 2048,
  2016, 1984, 1952, 1920, 1888, 1856, 1824, 1792,
  1760, 1728, 1696, 1664, 1632, 1600, 1568, 1536,
  1504, 1472, 1440, 1408, 1376, 1344, 1312, 1280,
  1248, 1216, 1184, 1152, 1120, 1088, 1056, 1024,
  992, 960, 928, 896, 864, 832, 800, 768,
  736, 704, 672, 640, 608, 576, 544, 512,
  480, 448, 416, 384, 352, 320, 288, 256,
  224, 192, 160, 128, 96, 64, 32, 0
};


void setup(void) {

  // For MCP4725A0 the address is 0x60 or 0x61
  LFO1.begin(0x60);
  LFO2.begin(0x61);

  pinMode(btn1, INPUT_PULLUP);  // button input pin
  pinMode(btn2, INPUT_PULLUP);  // button input pin
  pinMode(led1, OUTPUT);        // LED1
  pinMode(led2, OUTPUT);        // LED2

  button1.attach(btn1);  // button debouncer
  button1.interval(30);  // interval in ms
  button2.attach(btn2);  // button debouncer
  button2.interval(30);  // interval in ms

  // test blink LEDs on power up
  digitalWrite(led1, HIGH);
  delay(500);
  digitalWrite(led1, LOW);

  digitalWrite(led2, HIGH);
  delay(500);
  digitalWrite(led2, LOW);
}

void loop(void) {

  for (int i = 0; i < 256; i++) {
    // write next data sample to each DAC to generate waveforms
    LFO1.setVoltage(pgm_read_word(&(sine_data[i])), false);
    LFO2.setVoltage(pgm_read_word(&(triangle_data[i])), false);
    /*
    // process button presses
    button1.update();
    if (button1.fell()) {  // if button was pressed
      digitalWrite(led1, HIGH);
      delay(10);
      digitalWrite(led1, LOW);
    }

    button2.update();
    if (button2.fell()) {  // if button was pressed
      digitalWrite(led2, HIGH);
      delay(10);
      digitalWrite(led2, LOW);
    }
*/

    // control the rate of sample writing on the DACs to set the waveform frequency
    int delayTime = analogRead(pot1);
    delayTime = map(delayTime, 0, 1023, 20000, 0);  // map pot voltage on ADC to a uS delay so that ccw to cw is a low to high frequency waveform
    delayMicroseconds(delayTime);

    // make LED brightness follow the waveform frequency by using pwm brightness control
    if (i < 128)
      analogWrite(led1, map(i, 0, 256, 0, 64));  // ramp brightness up for first half of wave cycle
    else
      analogWrite(led1, map(i, 0, 256, 64, 0));  // ramp brightness down for second half of wave cycle
  }
}
