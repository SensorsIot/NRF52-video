/**
 * BLEThermo
 * 
 * Allen Snook
 * December 2020
 * 
 * Reads temperature from a DS18B20 one wire thermometer connected
 * to pin 5 on a Adafruit Feather nRF52840.
 * 
 * It then acts as a BLE Peripheral, implementing the Environmental
 * Sensing Service and Temperature Characteristic, notifying a connected
 * Central with the current temperature.
 */

 #include <bluefruit.h>

 #define DALLAS_PIN 12

// Most recent temperature measurement in degrees F
double tempF = 0;

// Most recent temperature measurement ready for BLE transport
uint8_t tempBLEData[2] = {0};

// GATT Service - 0x181A - Environmental Sensing
BLEService        es_svc = BLEService(UUID16_SVC_ENVIRONMENTAL_SENSING);

// GATT Characteristic and Object Type - 0x2A6E - Temperature
BLECharacteristic tchar = BLECharacteristic(UUID16_CHR_TEMPERATURE);

// DIS (Device Information Service) helper class instance
BLEDis bledis;

/**
 * Reset the one wire bus
 * 
 * To reset the bus, hold the bus low for 480 us, release for 70 us,
 * and then "sample" for 410 us
 */
void oneWire_reset(uint8_t pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
  delayMicroseconds(480);

  pinMode(pin, INPUT);
  delayMicroseconds(70);

  delayMicroseconds(410);
}

/**
 * Write one byte to the one wire bus
 * 
 * Send the LSb first
 * To write a 0 on the bus, hold low for 60 us then release for 10 us
 * To write a 1 on the bus, hold low for 5 us then release for 65 us
 */
void oneWire_writeByte(uint8_t pin, uint8_t value) {
  for (uint8_t i = 0; i < 8; i++) {
    if (value & 0x01) {
      digitalWrite(pin, LOW);
      pinMode(pin, OUTPUT);
      delayMicroseconds(5);

      pinMode(pin, INPUT);
      delayMicroseconds(65);
    } else {
      digitalWrite(pin, LOW);
      pinMode(pin, OUTPUT);
      delayMicroseconds(60);

      pinMode(pin, INPUT);
      delayMicroseconds(10);
    }
    value = value >> 1;
  }
  delayMicroseconds(200);
}

/**
 * Read one byte from the one wire bus
 * 
 * The LSb is the first bit to arrive
 */
uint8_t oneWire_readByte(uint8_t pin) {
  uint8_t value = 0;
  
  for (uint8_t i = 0; i < 8; i++) {
    value = value >> 1;
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
    delayMicroseconds(5);

    pinMode(pin, INPUT);
    delayMicroseconds(5);

    if (digitalRead(pin)) {
      value |= 0x80;
    }

    delayMicroseconds(50);
  }
  return value;
}

/**
 * Ask the thermometer to start a measurement
 */
void dallas_startMeasurement(uint8_t pin) {
  oneWire_reset(pin);
  oneWire_writeByte(pin, 0xCC);
  oneWire_writeByte(pin, 0x44);

  // TODO - wait for high instead of this hardcoded delay
  delayMicroseconds(410);
}

/**
 * Read the last measurement from the thermometer
 * 
 * Reads the temperature and stores it globally
 * in tempF (degrees fahrenheit) and also
 * in tempBLE (two bytes, LSB.MSB, with 0.01 degC resolution)
 */
void dallas_readTemperature(uint8_t pin) {
  oneWire_reset(pin);
  oneWire_writeByte(pin, 0xCC);
  oneWire_writeByte(pin, 0xBE);

  uint8_t scratchpad[9];
  for (uint8_t i=0; i < 9; i++) {
    scratchpad[i] = oneWire_readByte(pin);
    Serial.print(scratchpad[i], HEX);
    Serial.print(" ");
  }
  Serial.println("");

  // TODO - check the scratchpad CRC
  // The DS18B20 scratchpad holds a CRC in the last byte, byte 8
  
  // The DS18B20 scratchpad holds the temperature in the first two
  // elements of the scratchpad. Byte 1 holds the MSB and byte 0 the LSB.
  // https://datasheets.maximintegrated.com/en/ds/DS18B20.pdf

  int16_t rawTemp = 0;
  rawTemp |= scratchpad[1] << 8;
  rawTemp |= scratchpad[0];

  double tempC = 1.0 * rawTemp / 16.0;

  // Sanity check - ignore impossible temperatures
  if (tempC > 70) {
    return;
  }
  if (tempC < -50) {
    return;
  }
  
  tempF = (tempC * 9.0 / 5.0) + 32.0;

  Serial.print("Temperature: ");
  Serial.print(tempC, 3);
  Serial.println("");

  int16_t tempCScaled = 100 * tempC;
  tempBLEData[0] = tempCScaled & 0xFF; // LSB
  tempBLEData[1] = (tempCScaled >> 8) & 0xFF; // MSB

  Serial.print("BLE:");
  Serial.print(tempBLEData[0], HEX);
  Serial.print(" ");
  Serial.print(tempBLEData[1], HEX);
  Serial.println("");
}

/**
 * Set up the Environmental Sensing Service (BLE)
 */
void setupESS() {
  es_svc.begin();

  // The temperature characteristic is two bytes, LSB then MSB
  // e.g. 2E09 -> 092E
  // then convert to decimal 092E -> 2350
  // then divide by 100 to get degC 2350 -> 23.5

  tchar.setProperties(CHR_PROPS_NOTIFY);
  tchar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  tchar.setFixedLen(2);
  tchar.begin();

  // Optionally set an initial value for the temperature characteristic
  // uint8_t tdata[2] = { 0b00000110, 0x40 };
  // tchar.notify(tdata, 2);
}

/**
 * Start BLE Advertising
 */
void startAdvertising() {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(es_svc);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(160, 1600);   // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

/**
 * On connect
 */
void connectCallback(uint16_t conn_handle) {
}

/**
 * On disconnect
 */
void disconnectCallback(uint16_t conn_handle, uint8_t reason) {
}

/**
 * Setup the Feather
 */
void setup() {
  dallas_startMeasurement(DALLAS_PIN);
  delayMicroseconds(400);
  dallas_readTemperature(DALLAS_PIN);
  delay(1000);

  Bluefruit.begin();
  Bluefruit.setName("BlueThermDevice");
  Bluefruit.Periph.setConnectCallback(connectCallback);
  Bluefruit.Periph.setDisconnectCallback(disconnectCallback);

  bledis.setManufacturer("ALLENDAV ENGINEERING");
  bledis.setModel("NRF52840-DS18B20-001");
  bledis.begin();

  setupESS();
  startAdvertising();
}

/**
 * The Feather's cyclic executive
 */
void loop() {
  if ( Bluefruit.connected() ) {
    Serial.println("Connected, sending notification");
    tchar.notify(tempBLEData, 2); // TODO - validate this is the correct structure
  } else {
    Serial.println("Not connected");
  }

  dallas_startMeasurement(DALLAS_PIN);
  delayMicroseconds(400);
  dallas_readTemperature(DALLAS_PIN);
  delay(1000);
}
