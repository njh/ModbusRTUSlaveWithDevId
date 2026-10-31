// Fake temperature sensor for Arduino for testing ModbusRTUSlaveWithDevId
//
// Answers Read Device Identification (FC 0x2B / MEI 0x0E).
//
// Input register 0 holds the temperature in tenths of a degree C (signed).
// A new reading is taken every READ_INTERVAL ms, independently of Modbus
// requests, and the built-in LED on D13 flashes each time.

#include <ModbusRTUSlaveWithDevId.h>

const uint8_t UNIT_ID = 1;
const unsigned long BAUD = 9600;
const unsigned long READ_INTERVAL = 2000;   // ms between readings
const unsigned long LED_FLASH = 50;         // ms the LED stays on per reading

ModbusRTUSlaveWithDevId modbus(Serial);

uint16_t inputRegisters[1];   // 0 = temperature x10 (int16)

int16_t temperature = 210;    // 21.0 C
unsigned long lastReading;

// Random walk of +/-0.1 C per reading, kept between 15.0 and 30.0 C
int16_t readFakeTemperature() {
  temperature += random(-1, 2);
  temperature = constrain(temperature, 150, 300);
  return temperature;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  randomSeed(analogRead(A0));   // floating pin gives a different seed each boot
  inputRegisters[0] = temperature;
  lastReading = millis();

  // Device identification strings, kept in flash. No more than 244 characters each.
  modbus.configureDeviceIdentification(F("aelius labs"),    // VendorName
                                       F("MBRTU-S-WDI-FT"), // ProductCode
                                       F("0.1.0"));         // MajorMinorRevision
  modbus.setVendorUrl(F("https://www.aelius.com/"));
  modbus.setProductName(F("Fake Temperature Sketch"));
  modbus.setModelName(F("Arduino"));

  modbus.configureInputRegisters(inputRegisters, 1);
  Serial.begin(BAUD);
  modbus.begin(UNIT_ID, BAUD);
}

void loop() {
  const unsigned long now = millis();
  if (now - lastReading >= READ_INTERVAL) {
    lastReading = now;
    digitalWrite(LED_BUILTIN, HIGH);
    inputRegisters[0] = readFakeTemperature();
  }
  if (now - lastReading >= LED_FLASH) digitalWrite(LED_BUILTIN, LOW);

  modbus.poll();
}
