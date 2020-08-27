/**
 * 
 */
#include <Arduino.h>

#include <avr/eeprom.h>
#define USE_EEPROM

#include <Streaming.h>

#include <LowPower.h>

#include <ADS1115_WE.h>

#include <ArduinoUniqueID.h>

#include <Wire.h>

#define LoRaPlaNet_Protocol_Version 2

#define SERIAL_BAUDRATE 38400

#define SLEEP_SECONDS 600 // must be a multiple of 8

#define SENSOR_POWER_PIN 5
#define SENSOR_POWER_ON LOW
#define SENSOR_POWER_OFF HIGH

#define I2C_ADC_ADDRESS 0x4A

ADS1115_WE adc(I2C_ADC_ADDRESS);
#define V_SENSOR_MAX 3000

#define I2C_BME_ADDRESS 0x76
#define TINY_BME280_I2C
#include <TinyBME280.h>
tiny::BME280 bme;

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <lora_config.hpp>

/**
 * These should go to lora_config.hpp - it's not included in the repo ..

#define _NWSKEY { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }

#define _APPSKEY { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }

#define _DEVADDR 0x00000000 // <-- Change this address for every node!

#define _ee_sensor_min { 500,  500,  500,  500}
#define _ee_sensor_max {2500, 2500, 2500, 2500}
*/

// LoRaWAN NwkSKey, network session key
// This should be in big-endian (aka msb).
#if defined(USE_EEPROM)
uint8_t EEMEM NWKSKEY[16] = _NWSKEY;
#else
static const PROGMEM u1_t NWKSKEY[16] = _NWSKEY;
#endif

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
#ifdef USE_EEPROM
uint8_t EEMEM APPSKEY[16] = _APPSKEY;
#else
static const u1_t PROGMEM APPSKEY[16] = _APPSKEY;
#endif

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
#ifdef USE_EEPROM
uint32_t EEMEM DEVADDR = _DEVADDR;
#else
static const u4_t DEVADDR = _DEVADDR; // <-- Change this address for every node!
#endif

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui(u1_t* buf) { }
void os_getDevEui(u1_t* buf) { }
void os_getDevKey(u1_t* buf) { }

#include <CayenneLPP.h>
CayenneLPP lpp(51);

#ifdef USE_EEPROM
uint16_t EEMEM ee_sensor_min[4] = _ee_sensor_min;
uint16_t EEMEM ee_sensor_max[4] = _ee_sensor_max;
#else
static const uint16_t PROGMEM ee_sensor_min[4] = _ee_sensor_min;
static const uint16_t PROGMEM ee_sensor_max[4] = _ee_sensor_max;
#endif

static osjob_t sendjob;
static int wakeup_counter = 0;

// Pin mapping
const lmic_pinmap lmic_pins = {
		.nss = 10,
		.rxtx = LMIC_UNUSED_PIN,
		.rst = 9,
		.dio = {2, 3, LMIC_UNUSED_PIN},
};

/**
 * 
 */
void adc_init(void) {
  if (!adc.init()) {
    Serial << F("ADS1115 not connected!") << endl;
  }

  /* Set the voltage range of the ADC to adjust the gain
   * Please note that you must not apply more than VDD + 0.3V to the input pins!
   * 
   * ADS1115_RANGE_6144  ->  +/- 6144 mV
   * ADS1115_RANGE_4096  ->  +/- 4096 mV
   * ADS1115_RANGE_2048  ->  +/- 2048 mV (default)
   * ADS1115_RANGE_1024  ->  +/- 1024 mV
   * ADS1115_RANGE_0512  ->  +/- 512 mV
   * ADS1115_RANGE_0256  ->  +/- 256 mV
   */
  adc.setVoltageRange_mV(ADS1115_RANGE_4096); //comment line/change parameter to change range

  /* Set the inputs to be compared
   *  
   *  ADS1115_COMP_0_1    ->  compares 0 with 1 (default)
   *  ADS1115_COMP_0_3    ->  compares 0 with 3
   *  ADS1115_COMP_1_3    ->  compares 1 with 3
   *  ADS1115_COMP_2_3    ->  compares 2 with 3
   *  ADS1115_COMP_0_GND  ->  compares 0 with GND
   *  ADS1115_COMP_1_GND  ->  compares 1 with GND
   *  ADS1115_COMP_2_GND  ->  compares 2 with GND
   *  ADS1115_COMP_3_GND  ->  compares 3 with GND
   */
  //adc.setCompareChannels(ADS1115_COMP_0_GND); //uncomment if you want to change the default

  /* Set number of conversions after which the alert pin will be active
   * - or you can disable the alert 
   *  
   *  ADS1115_ASSERT_AFTER_1  -> after 1 conversion
   *  ADS1115_ASSERT_AFTER_2  -> after 2 conversions
   *  ADS1115_ASSERT_AFTER_4  -> after 4 conversions
   *  ADS1115_DISABLE_ALERT   -> disable comparator / alert pin (default) 
   */
  //adc.setAlertPinMode(ADS1115_ASSERT_AFTER_1); //uncomment if you want to change the default

  /* Set the conversion rate in SPS (samples per second)
   * Options should be self-explaining: 
   * 
   *  ADS1115_8_SPS 
   *  ADS1115_16_SPS  
   *  ADS1115_32_SPS 
   *  ADS1115_64_SPS  
   *  ADS1115_128_SPS (default)
   *  ADS1115_250_SPS 
   *  ADS1115_475_SPS 
   *  ADS1115_860_SPS 
   */
  //adc.setConvRate(ADS1115_128_SPS); //uncomment if you want to change the default

  /* Set continuous or single shot mode:
   * 
   *  ADS1115_CONTINUOUS  ->  continuous mode
   *  ADS1115_SINGLE     ->  single shot mode (default)
   */
  //adc.setMeasureMode(ADS1115_CONTINUOUS); //uncomment if you want to change the default

   /* Choose maximum limit or maximum and minimum alert limit (window)in Volt - alert pin will 
   *  be active when measured values are beyond the maximum limit or outside the window 
   *  Upper limit first: setAlertLimit_V(MODE, maximum, minimum)
   *  In max limit mode the minimum value is the limit where the alert pin will be deactivated (if 
   *  not latched)  
   * 
   *  ADS1115_MAX_LIMIT
   *  ADS1115_WINDOW
   * 
   */
  //adc.setAlertModeAndLimit_V(ADS1115_MAX_LIMIT, 3.0, 1.5); //uncomment if you want to change the default
  
  /* Enable or disable latch. If latch is enabled the alarm pin will be active until the
   * conversion register is read (getResult functions). If disabled the alarm pin will be
   * deactivated with next value within limits. 
   *  
   *  ADS1115_LATCH_DISABLED (default)
   *  ADS1115_LATCH_ENABLED
   */
  //adc.setAlertLatch(ADS1115_LATCH_ENABLED); //uncomment if you want to change the default

  /* Sets the alert pin polarity if active:
   *  
   * Enable or disable latch. If latch is enabled the alarm pin will be active until the
   * conversion register is read (getResult functions). If disabled the alarm pin will be
   * deactivated with next value within limits. 
   *  
   * ADS1115_ACT_LOW  ->  active low (default)   
   * ADS1115_ACT_HIGH ->  active high
   */
  //adc.setAlertPol(ADS1115_ACT_LOW); //uncomment if you want to change the default
 
  /* With this function the alert pin will be active, when a conversion is ready.
   * In order to deactivate, use the setAlertLimit_V function  
   */
  //adc.setAlertPinToConversionReady(); //uncomment if you want to change the default
}

/**
 * 
 */
void bme_init(void) {
  if (!bme.beginI2C(I2C_BME_ADDRESS)) {
    Serial << F("bme280 not responding ..") << endl;
  }
}

/**
 * 
 */
float readChannel(ADS1115_MUX channel) {
  adc.setCompareChannels(channel);
  adc.startSingleMeasurement();
  
  while (adc.isBusy()) {}
  
  return adc.getResult_mV(); // alternative: getResult_mV for Millivolt
}

/**
 *
 */
long readVcc(void) {
  long result;

  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  
  ADCSRA |= _BV(ADSC); // Convert
  
  while (bit_is_set(ADCSRA, ADSC));
  
  result = ADCL;
  result |= ADCH << 8;
  result = 1126400L / result; // Back-calculate AVcc in mV

  return result;
}

/**
 * 
 */
void do_send(osjob_t* j) {
  Serial << (os_getTime()) << F(": ");

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial << (F("OP_TXRXPEND, not sending")) << endl;
  } else {
    float voltage[4];// = {0.0};

    uint16_t sensor_min[4];// = {0};
    uint16_t sensor_max[4];// = {0};

#ifdef USE_EEPROM
    eeprom_read_block((void*) sensor_min, (const void*) ee_sensor_min, sizeof(sensor_min));
    eeprom_read_block((void*) sensor_max, (const void*) ee_sensor_max, sizeof(sensor_max));
#else
    memcpy_P(sensor_min, ee_sensor_min, sizeof(ee_sensor_min);
    memcpy_P(sensor_max, ee_sensor_max, sizeof(ee_sensor_max));
#endif

    Serial << F("min: ");

    for (int i = 0; i < 4; i++) {
      Serial << i << F(": ") << sensor_min[i] << (i < 3 ? F(", ") : F("\n"));
    }

    Serial << F("max: ");

    for (int i = 0; i < 4; i++) {
      Serial << i << F(": ") << sensor_max[i] << (i < 3 ? F(", ") : F("\n"));
    }

    long vcc = readVcc();

    pinMode(SENSOR_POWER_PIN, OUTPUT);
    digitalWrite(SENSOR_POWER_PIN, SENSOR_POWER_ON);

    Wire.begin();

    bme_init();

    adc_init();

    delay(200); // ensure Sensors stabilized level

    int32_t temperature = bme.readFixedTempC();
    uint32_t humidity = bme.readFixedHumidity();
    uint32_t barometric_pressure = bme.readFixedPressure();

    voltage[0] = readChannel(ADS1115_COMP_0_GND);
    voltage[1] = readChannel(ADS1115_COMP_1_GND);
    voltage[2] = readChannel(ADS1115_COMP_2_GND);
    voltage[3] = readChannel(ADS1115_COMP_3_GND);

    for (int i = 0; i < 4; i++) {
      voltage[i] = (voltage[i] < V_SENSOR_MAX ? voltage[i] : 0);
    }

    digitalWrite(SENSOR_POWER_PIN, SENSOR_POWER_OFF);
    pinMode(SENSOR_POWER_PIN, INPUT_PULLUP);

    Wire.end();

    for (int i = 0; i < 4; i++) {
      Serial << i << F(": ") << voltage[i] << F(",  ");
      Serial.flush();
    }

    Serial << F("Vcc: ") << vcc;
    Serial << F(",   Deg C: ") << (temperature / 100.0) << F(",   rH%: ") << (humidity / 1000.0) << F(",   Pa: ") << (barometric_pressure / 100.0) << endl;

  	int channel = 1;

  	lpp.reset();

  	lpp.addDigitalInput      (channel++, LoRaPlaNet_Protocol_Version);
      
    lpp.addVoltage           (channel++, vcc / 1000.0);

    lpp.addTemperature       (channel++, temperature / 100.0);

    lpp.addRelativeHumidity  (channel++, humidity / 1000.0);

    lpp.addBarometricPressure(channel++, barometric_pressure / 100.0);

    for (int i = 0; i < 4; i++) {
      lpp.addVoltage         (channel++, voltage[i] / 1000.0);
    }

    for (int i = 0; i < 4; i++) {
      uint32_t mapped = map(voltage[i], sensor_min[i], sensor_max[i], 100, 0);
      Serial << i << F(": ") << mapped << F(",  ");
      lpp.addPercentage      (channel++, mapped);
    }

    Serial << endl << F("packet-size: ") << lpp.getSize() << endl;

    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), false);
      
    Serial << (F("Packet queued")) << endl;
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

/**
 * 
 */
void onEvent(ev_t ev) {
  Serial << (os_getTime()) << F(": ");

  switch(ev) {
/*
    case EV_SCAN_TIMEOUT:
      Serial << (F("EV_SCAN_TIMEOUT")) << endl;
      break;
*/        
    case EV_TXCOMPLETE:
      Serial << (F("EV_TXCOMPLETE (includes waiting for RX windows)")) << endl;
            
      if (LMIC.txrxFlags & TXRX_ACK) {
        Serial << (F("Received ack")) << endl;
      }
           
      if (LMIC.dataLen) {
        // TODO: implement setting of sensor_min/sensor_max in eeprom
        Serial << (F("Received ")) << endl;
        Serial << (LMIC.dataLen) << endl;
        Serial << (F(" bytes of payload")) << endl;
      }
  
      wakeup_counter = SLEEP_SECONDS / 8;

      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(1), do_send);

      Serial.flush();

      Serial.end();

      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_ON);

      break;
/*
    case EV_RESET:
      Serial << (F("EV_RESET")) << endl;
      break;
*/

    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial << (F("EV_RXCOMPLETE")) << endl;
      break;


    case EV_LINK_DEAD:
      Serial << (F("EV_LINK_DEAD")) << endl;
      break;

    
    case EV_LINK_ALIVE:
      Serial << (F("EV_LINK_ALIVE")) << endl;
      break;

    default:
      Serial << (F("Unknown event: ")) << endl;
      Serial << ((unsigned) ev) << endl;
      break;
  }
}

/**
 * 
 */
void setup(void) {
  Serial.begin(SERIAL_BAUDRATE);

  analogReference(INTERNAL1V1);

  Serial << F("LoRa_PlantGuard") << endl;

  Serial << F("UniqueID: ");

	for (size_t i = 0; i < 8; i++) {
		Serial << F("0x") << _HEX(UniqueID8[i]) << (i < 7 ? F(":") : F("\n"));
	}

  Serial << endl;

  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];// = {0};
  uint8_t nwkskey[sizeof(NWKSKEY)];// = {0};

#ifdef USE_EEPROM
  eeprom_read_block((void*) appskey, (const void*) APPSKEY, sizeof(APPSKEY));
  eeprom_read_block((void*) nwkskey, (const void*) NWKSKEY, sizeof(NWKSKEY));
  uint32_t devaddr = eeprom_read_dword(&DEVADDR);
#else
	memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
	memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  uint32_t devaddr = DEVADDR;
#endif

  Serial << F("appskey: ");

	for (size_t i = 0; i < 16; i++) {
		Serial << F("0x") <<_HEX(appskey[i]) << (i < 15 ? F(":") : F("\n"));
	}

  Serial << F("nwkskey: ");

	for (size_t i = 0; i < 16; i++) {
		Serial << F("0x") <<_HEX(nwkskey[i]) << (i < 15 ? F(":") : F("\n"));
	}

  Serial << F("devaddr: 0x") << _HEX(devaddr) << endl;

	LMIC_setSession(0x13, devaddr, nwkskey, appskey);

  // Set up the channels used by the Things Network, which corresponds
	// to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  // NA-US channels 0-71 are configured automatically
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band

	// TTN defines an additional channel at 869.525Mhz using SF9 for class B
	// devices' ping slots. LMIC does not have an easy way to define set this
	// frequency and support for class B is spotty and untested, so this
	// frequency is not configured here.

  // Disable link check validation
  LMIC_setLinkCheckMode(false);

  LMIC_setAdrMode(false);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
	LMIC_setDrTxpow(DR_SF7, 14); // should only be used if data rate adaption is disabled

  // Use with Arduino Pro Mini ATmega328P 3.3V 8 MHz
  // Let LMIC compensate for +/- 1% clock error
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

 	// Start job (sending automatically starts OTAA too)
 	do_send(&sendjob);

  Serial.flush();
  Serial.end();
}

/**
  read voltage of the rail (Vcc)
  output mV (2 bytes)
*/
/*
uint16_t vccVoltage(void) {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  // default ADMUX REFS1 and REFS0 = 0

  // #define _BV(bit) (1 << (bit))

  // 1.1V (I Ref)(2) 100001
  ADMUX = _BV(MUX5) | _BV(MUX0);

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  uint16_t result = (high<<8) | low;

  // result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  // number of steps = 1023??
  result = (1125300L / result) ; // Calculate Vcc (in mV);

  return result;
}
*/

/**
 * 
 */
void loop(void) {
  Serial.begin(SERIAL_BAUDRATE);

  if (wakeup_counter > 0) { // continue sleeping
    wakeup_counter--;

    Serial << F("continue sleping another 8 s .. counter: ") << wakeup_counter << endl;

    Serial.flush();
    Serial.end();

    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_ON);
  } else { // do real stuff
    //Serial << F("going to do stuff .. counter: ") << wakeup_counter << endl;
  
    os_runloop_once();
  }
}
