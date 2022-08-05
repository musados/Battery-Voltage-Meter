
#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define PIN        2
#define NUMPIXELS 5
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

#define DELAYVAL 500

int r1 = 10000;
int r2 = 1000;
float max_pulse = 1023.0;
float max_voltage = 5.02;
int batteryPin = A0;
int chargingPin = A1;
bool isCharging = false;
float max_battery_voltage = 8.4;
float min_battery_voltage = 5.6;
float min_safe_battery_voltage = 6.4;

int tx_pin = 0;
int rx_pin = 1;
SoftwareSerial mySerial(rx_pin, tx_pin); // RX, TX

void setup() {
  //  Serial.begin(115200);
  mySerial.begin(115200);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  //Serial.println(analogRead(A1));
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  pixels.begin();
}

bool isShowTime = false;

void loop() {
  float batt_voltage = calculateVoltage();
  //Serial.println("batt_volt: " + String(batt_voltage));
  float batt_percents = getSafeBatteryPercents(batt_voltage);
  //  Serial.print("Battery percents: ");
  //  Serial.println(batt_percents);
  int chargeVal = analogRead(chargingPin);
  isCharging = chargeVal > 10 ? true : false;
//  Serial.println(String(isCharging ? "Charging... :)" : "Not charging!"));

  // The first NeoPixel in a strand is #0, second is 1, all the way up
  // to the count of pixels minus one.
  if (batt_percents < 100) {
    //pixels.clear(); // Set all pixel colors to 'off'
    //pixels.show();   // Send the updated pixel colors to the hardware.
    //delay(DELAYVAL); // Pause before next pass through loop
    int sector = (int)((batt_percents / (100 / NUMPIXELS)) + 1);
    //Serial.println("Sector: " + String(sector));
    for (int i = 0; i < NUMPIXELS; i++) { // For each pixel...

      if (i < sector && (isShowTime || i < (sector - 1) )) {
        // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
        // Here we're using a moderately bright green color:
        pixels.setPixelColor(i, pixels.Color(30, 30, 30));
      }
      else {

        pixels.setPixelColor(i, pixels.Color(0, 0, 0));
      }

      pixels.show();   // Send the updated pixel colors to the hardware.
      isShowTime = !isShowTime;
    }


    //    delay(DELAYVAL); // Pause before next pass through loop
    //    pixels.setPixelColor(sector - 1, pixels.Color(0, 0, 0));
    //    pixels.show();   // Send the updated pixel colors to the hardware.



  }
  else {
    for (int i = 0; i < NUMPIXELS; i++) { // For each pixel...

      // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
      // Here we're using a moderately bright green color:
      pixels.setPixelColor(i, pixels.Color(30, 30, 30));
    }
    pixels.show();   // Send the updated pixel colors to the hardware.
  }

  // Check for serial data - receive and send
  checkSerialData();
  delay(DELAYVAL); // Pause before next pass through loop
}

void checkSerialData() {
  if (mySerial.available()) {
    // Check for the data here and send data back
  }
}

float calculateVoltage() {
  float result = 0.0;
  result = (float)analogRead(batteryPin);
  //Serial.println("analog: " + String(result));
  float divider_sum = float (float(r1 + r2) / 1000) * max_voltage;
  //Serial.println(divider_sum);
  float batt_div = max_pulse / divider_sum;
  //Serial.print("batt_div:");
  //Serial.println(batt_div);
  result = result / batt_div;

  return result;
}

int getBatteryPercents(float voltage) {
  int result = 0;
  float dammy_voltage = voltage - min_battery_voltage;
  if (voltage >= 0 && dammy_voltage >= 0) {
    float dammy_max = max_battery_voltage - min_battery_voltage;
    result = (dammy_voltage * 100) / dammy_max;
  }
  //else{
  //  Serial.println("Invalid battery Voltage!! " + String(voltage));
  //}

  return result;
}

int getSafeBatteryPercents(float voltage) {
  int result = 0;
  float dammy_voltage = voltage - min_safe_battery_voltage;
  if (voltage >= 0 && dammy_voltage >= 0) {
    float dammy_max = max_battery_voltage - min_safe_battery_voltage;
    result = (dammy_voltage * 100) / dammy_max;
  }
  //else{
  //  Serial.println("Invalid battery Voltage!! " + String(voltage));
  //}

  return result;
}

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

// Function created to obtain chip's actual Vcc voltage value, using internal bandgap reference
// This demonstrates ability to read processors Vcc voltage and the ability to maintain A/D calibration with changing Vcc
// Now works for 168/328 and mega boards.
// Thanks to "Coding Badly" for direct register control for A/D mux
// 1/9/10 "retrolefty"


int getBandgap(void) // Returns actual value of Vcc (x 100)
{

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  // For mega boards
  const long InternalReferenceVoltage = 1115L;  // Adjust this value to your boards specific internal BG voltage x1000
  // REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc reference
  // MUX4 MUX3 MUX2 MUX1 MUX0  --> 11110 1.1V (VBG)         -Selects channel 30, bandgap voltage, to measure
  ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (0 << MUX5) | (1 << MUX4) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0);

#else
  // For 168/328 boards
  const long InternalReferenceVoltage = 1056L;  // Adjust this value to your boards specific internal BG voltage x1000
  // REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc external reference
  // MUX3 MUX2 MUX1 MUX0  --> 1110 1.1V (VBG)         -Selects channel 14, bandgap voltage, to measure
  ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0);

#endif
  delay(50);  // Let mux settle a little to get a more stable A/D conversion
  // Start a conversion
  ADCSRA |= _BV( ADSC );
  // Wait for it to complete
  while ( ( (ADCSRA & (1 << ADSC)) != 0 ) );
  // Scale the value
  int results = (((InternalReferenceVoltage * 1024L) / ADC) + 5L) / 10L; // calculates for straight line value
  return results;

}
