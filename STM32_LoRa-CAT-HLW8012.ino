#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <Arduino.h>

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include "TSL2561.h"

TSL2561 tsl(TSL2561_ADDR_FLOAT); 

inline void lDelay(uint32_t delay) { 
  uint32_t i, j;

  j = delay * 7300;
  for (i = 0; i < j; i++) {
    asm volatile(
      "nop \n\t"
     ::);
  } 
}

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C #define BME280_ADDRESS                (0x76)

#define I2C_ADDRESS 0x3C
#define RST_PIN -1
SSD1306AsciiWire oled;

float x,V,A,W,Wh,F,PF = 0; 
uint16_t V_int=0,A_int=0,W_int=0,Wh_int=0;

float temperature = 0;
float humidity = 0;
uint16_t tempC_int = 0;
uint8_t hum_int = 0;
uint16_t lux_int = 0;

#define DEBUG
#define SLEEP

#define voltage   PA0

#include "HLW8012.h"

#define SEL_PIN                         PB10
#define CF1_PIN                         PA12
#define CF_PIN                          PA15

#define UPDATE_TIME                     5000

#define CURRENT_MODE                    HIGH

// These are the nominal values for the resistors in the circuit
#define CURRENT_RESISTOR                0.001
#define VOLTAGE_RESISTOR_UPSTREAM       ( 5 * 470000 ) // Real: 2280k
#define VOLTAGE_RESISTOR_DOWNSTREAM     ( 1000 ) // Real 1.009k

HLW8012 hlw8012;

#define USE_SPI   2

// Lorawan CAT 
// LoRaWAN NwkSKey, your network session key, 16 bytes (from staging.thethingsnetwork.org)
static unsigned char NWKSKEY[16] = { 0x00, 0x00, 0xD2, 0x2B, 0x7E, 0x15, 0x16, 0xA6, 0x09, 0xCF, 0xAB, 0xF7, 0x15, 0x88, 0x4F, 0x3C };

// LoRaWAN AppSKey, application session key, 16 bytes  (from staging.thethingsnetwork.org)
static unsigned char APPSKEY[16] = { 0x00, 0x00, 0xAE, 0x2B, 0x7E, 0x15, 0xD2, 0xA6, 0xAB, 0xF7, 0xCF, 0x4F, 0x3C, 0x15, 0x88, 0x09 };

// LoRaWAN end-device address (DevAddr), ie 0x91B375AC  (from staging.thethingsnetwork.org)
static const u4_t DEVADDR = 0x0000167A;

static const u1_t PROGMEM DEVEUI[8]={ 0x00, 0x00, 0x5B, 0x05, 0x6D, 0x60, 0x97, 0x75 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// STM32 Unique Chip IDs
#define STM32_ID  ((u1_t *) 0x1FFFF7E8)

SPIClass mySPI(USE_SPI);

extern SPIClass *SPIp;
int channel = 0;
int txInterval = 60;

unsigned long sampletime_ms = 30000;  // 30 Seconds

#define RATE        DR_SF10

struct {
  uint8_t Header1[2] = {0x01,0x67};
  char temp[2];
  uint8_t Header2[2] = {0x02,0x68};
  char humid[1];
  uint8_t Header3[2] = {0x03,0x02};
  char vbat[2];  
  uint8_t Header4[2] = {0x04,0x65};
  char lux[2];   
  uint8_t Header5[2] = {0x05,0x02};
  char volt[2];
  uint8_t Header6[2] = {0x06,0x02};
  char amp[2];    
  uint8_t Header7[2] = {0x07,0x02};
  char watt[2];  
}mydata;

  byte power;
  byte rate2;

#ifdef SLEEP

// Defined for power and sleep functions pwr.h and scb.h
#include <libmaple/pwr.h>
#include <libmaple/scb.h>

#include <RTClock.h>
#include <TimeLib.h>

uint32_t tt = 0;
RTClock rt(RTCSEL_LSI, 399); // 10 milli second alarm

// Define the Base address of the RTC registers (battery backed up CMOS Ram), so we can use them for config of touch screen or whatever.
// See http://stm32duino.com/viewtopic.php?f=15&t=132&hilit=rtc&start=40 for a more details about the RTC NVRam
// 10x 16 bit registers are available on the STM32F103CXXX more on the higher density device.
#define BKP_REG_BASE   ((uint32_t *)(0x40006C00 +0x04))

void storeBR(int i, uint32_t v) {
  BKP_REG_BASE[2 * i] = (v << 16);
  BKP_REG_BASE[2 * i + 1] = (v & 0xFFFF);
}

uint32_t readBR(int i) {
  return ((BKP_REG_BASE[2 * i] & 0xFFFF) >> 16) | (BKP_REG_BASE[2 * i + 1] & 0xFFFF);
}

bool next = false;
static void int_fun() {};

void setPLL(rcc_pll_multiplier mult) 
{
  rcc_switch_sysclk(RCC_CLKSRC_HSI);
  rcc_turn_off_clk(RCC_CLK_PLL);
  rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE , mult);
}

void sleepMode(bool deepSleepFlag)
{
  // Clear PDDS and LPDS bits
  PWR_BASE->CR &= PWR_CR_LPDS | PWR_CR_PDDS | PWR_CR_CWUF;

  // Set PDDS and LPDS bits for standby mode, and set Clear WUF flag (required per datasheet):
  PWR_BASE->CR |= PWR_CR_CWUF;
  // Enable wakeup pin bit.
  PWR_BASE->CR |=  PWR_CSR_EWUP;

  SCB_BASE->SCR |= SCB_SCR_SLEEPDEEP;

  // System Control Register Bits. See...
  // http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0497a/Cihhjgdh.html
  if (deepSleepFlag) {
    // Set Power down deepsleep bit.
    PWR_BASE->CR |= PWR_CR_PDDS;
    // Unset Low-power deepsleep.
    PWR_BASE->CR &= ~PWR_CR_LPDS;
  } else {
    adc_disable(ADC1);
    adc_disable(ADC2);
#if STM32_HAVE_DAC
    dac_disable_channel(DAC, 1);
    dac_disable_channel(DAC, 2);
#endif
    //  Unset Power down deepsleep bit.
    PWR_BASE->CR &= ~PWR_CR_PDDS;
    // set Low-power deepsleep.
    PWR_BASE->CR |= PWR_CR_LPDS;
  }

  // Now go into stop mode, wake up on interrupt
  asm("    wfi");

  // Clear SLEEPDEEP bit so we can use SLEEP mode
  SCB_BASE->SCR &= ~SCB_SCR_SLEEPDEEP;
}

uint32 sleepTime;

long int alarmDelay = 15;

void AlarmFunction () {
  // We always wake up with the 8Mhz HSI clock!
  // So adjust the clock if needed...

#if F_CPU == 8000000UL
  // nothing to do, using about 8 mA
#elif F_CPU == 16000000UL
  rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE , RCC_PLLMUL_2);
#elif F_CPU == 48000000UL
  rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE , RCC_PLLMUL_6);
#elif F_CPU == 72000000UL
  rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE , RCC_PLLMUL_9);
#else
#error "Unknown F_CPU!?"
#endif

  extern volatile uint32 systick_uptime_millis;
  systick_uptime_millis += sleepTime;
}

void mdelay(int n, bool mode = false)
{
  sleepTime = n;
  time_t nextAlarm = (rt.getTime() + n / 10); // Calculate from time now.
  rt.createAlarm(&AlarmFunction, nextAlarm);
  sleepMode(mode);
}

void msleep(uint32_t ms)
{
  uint32_t start = rt.getTime();

  while (rt.getTime() - start < ms) {
    asm("    wfi");
  }
}

#endif

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
//void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = PB12,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = PA8,
  .dio = {PB1, PB0, LMIC_UNUSED_PIN}
};


bool TX_done = false;

bool joined = false;

void onEvent (ev_t ev) {
#ifdef DEBUG
  Serial.println(F("Enter onEvent"));
#endif

  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      joined = true;
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      TX_done = true;
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      //mdelay(txInterval *1000, 0);
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(txInterval), do_send);
      //sleepMode_1(false,txInterval/1000);  // stop
      //setPLL(RCC_PLLMUL_9);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
#ifdef DEBUG
  Serial.println(F("Leave onEvent"));
#endif
#ifdef SLEEP
  next = true; // Always send after any event, to recover from a dead link
#endif
}

void do_send(osjob_t* j) {

#ifdef DEBUG
  Serial.println(F("Enter do_send"));
#endif
      readData();
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else { 
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, (unsigned char *)&mydata, sizeof(mydata), 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
#ifdef DEBUG
  Serial.println(F("Leave do_send"));
#endif
  TX_done = false;
}

int vbat_int;

void readData(){
    uint32_t lum = tsl.getFullLuminosity();
    uint16_t ir, full;
    ir = lum >> 16;
    full = lum & 0xFFFF;
    lux_int = tsl.calculateLux(full, ir);
  
    temperature = bme.readTemperature();
    humidity = bme.readHumidity();
    tempC_int = temperature*10;
    hum_int = humidity*2;
    adc_enable(ADC1);
    vbat_int = 120 * 4096 / adc_read(ADC1, 17);
    adc_disable(ADC1);
      
    mydata.temp[0] = tempC_int >> 8;
    mydata.temp[1] = tempC_int;
    mydata.humid[0] =  hum_int ;
    mydata.vbat[0] = vbat_int >> 8;
    mydata.vbat[1] = vbat_int;    
    mydata.lux[0] = lux_int >> 8;
    mydata.lux[1] = lux_int;    

    W = hlw8012.getActivePower(); 
    V = hlw8012.getVoltage();
    A = hlw8012.getCurrent(); 

    V_int = V*100;
    mydata.volt[0] = V_int >> 8;
    mydata.volt[1] = V_int;

    A_int = A*100;
    mydata.amp[0] = A_int >> 8;
    mydata.amp[1] = A_int;   

    W_int = W*100;
    mydata.watt[0] = W_int >> 8;
    mydata.watt[1] = W_int;
 
    oled.clear();
    oled.println("CAT Lorawan");
    oled.println("AS923 SF10");
    oled.setCursor(0,2);oled.print(temperature,1);oled.print("C ");oled.print(humidity,1);oled.print("%");
    oled.setCursor(0,4);oled.print("lux:");oled.print(lux_int);oled.print(" Batt:");oled.print(vbat_int*0.01);    
    oled.setCursor(0,6);oled.print(V,2);oled.print("V ");oled.print(A,2);oled.print("A ");oled.print(W,2);oled.print("W ");  
             


#ifdef DEBUG
    Serial.print(temperature);Serial.print(" ");Serial.print(humidity);Serial.print(" ");Serial.print(vbat_int*0.01);Serial.print(" ");Serial.println(tsl.calculateLux(full, ir));
    Serial.println();
    Serial.println();  
        Serial.print("[HLW] Active Power (W)    : "); Serial.println(W);
        Serial.print("[HLW] Voltage (V)         : "); Serial.println(V);
        Serial.print("[HLW] Current (A)         : "); Serial.println(A);
        Serial.print("[HLW] Apparent Power (VA) : "); Serial.println(hlw8012.getApparentPower());
        Serial.print("[HLW] Power Factor (%)    : "); Serial.println((int) (100 * hlw8012.getPowerFactor()));        
             
#endif   
}

void setup_vdd_sensor() {
    adc_reg_map *regs = ADC1->regs;
    regs->CR2 |= ADC_CR2_TSVREFE; // enable VREFINT and temp sensor
    regs->SMPR1 = (ADC_SMPR1_SMP17 /* | ADC_SMPR1_SMP16 */); // sample rate for VREFINT ADC channel
}

void forceTxSingleChannelDr() {
    for(int i=0; i<9; i++) { // For EU; for US use i<71
        if(i != channel) {
            LMIC_disableChannel(i);
        }
    }
    // Set data rate (SF) and transmit power for uplink
    LMIC_setDrTxpow(RATE, 14);
}

// When using interrupts we have to call the library entry point
// whenever an interrupt is triggered
void hlw8012_cf1_interrupt() {
    hlw8012.cf1_interrupt();
}
void hlw8012_cf_interrupt() {
    hlw8012.cf_interrupt();
}

void setInterrupts() {
    attachInterrupt(digitalPinToInterrupt(CF1_PIN), hlw8012_cf1_interrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CF_PIN), hlw8012_cf_interrupt, CHANGE);
}

void unblockingDelay(unsigned long mseconds) {
    unsigned long timeout = millis();
    while ((millis() - timeout) < mseconds) delay(1);
}

void calibrate() {

    // Let some time to register values
    unsigned long timeout = millis();
    while ((millis() - timeout) < 10000) {
        delay(1);
    }

    // Calibrate using a 60W bulb (pure resistive) on a 230V line
    hlw8012.expectedActivePower(60);
    hlw8012.expectedVoltage(230);
    hlw8012.expectedCurrent(60/230);

    // Show corrected factors
    Serial.print("[HLW] New current multiplier : "); Serial.println(hlw8012.getCurrentMultiplier());
    Serial.print("[HLW] New voltage multiplier : "); Serial.println(hlw8012.getVoltageMultiplier());
    Serial.print("[HLW] New power multiplier   : "); Serial.println(hlw8012.getPowerMultiplier());
    Serial.println();
}

void setup() {
  setup_vdd_sensor();
  SPIp = &mySPI;

  pinMode(SEL_PIN, OUTPUT);
  pinMode(CF1_PIN, INPUT_PULLUP);
  pinMode(CF_PIN, INPUT_PULLUP);

  Serial.begin(9600);
  delay(2000);
  
  if (tsl.begin()) {
    Serial.println("Found TLS2561 sensor");
  } else {
    Serial.println("No TLS2561 sensor?");
    while (1);
  }  

  tsl.setGain(TSL2561_GAIN_16X);      // set 16x gain (for dim situations)
  tsl.setTiming(TSL2561_INTEGRATIONTIME_13MS);  // shortest integration time (bright light)  
   
  bool status = bme.begin();  
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }
 
  #if RST_PIN >= 0
    oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
  #else // RST_PIN >= 0
    oled.begin(&Adafruit128x64, I2C_ADDRESS);
  #endif // RST_PIN >= 0

  oled.setFont(Adafruit5x7);
    
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
  LMIC_setupChannel(0, 923200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 923400000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 923600000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 923800000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band    
  LMIC_setupChannel(4, 924000000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band    
  LMIC_setupChannel(5, 924200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band 
  LMIC_setupChannel(6, 924400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band          
  LMIC_setupChannel(7, 924600000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band    

#if F_CPU == 8000000UL
  // HSI is less accurate
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
#endif

  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  //LMIC.dn2Dr = DR_SF9;

  forceTxSingleChannelDr(); 
  // Set data rate and transmit power (note: txpow seems to be ignored by the library)
  //LMIC_setDrTxpow(RATE, 14);

    hlw8012.begin(CF_PIN, CF1_PIN, SEL_PIN, CURRENT_MODE, true);
  //hlw8012.begin(CF_PIN, CF1_PIN, SEL_PIN, CURRENT_MODE, false, 500000);
    hlw8012.setResistors(CURRENT_RESISTOR, VOLTAGE_RESISTOR_UPSTREAM, VOLTAGE_RESISTOR_DOWNSTREAM);
    
    setInterrupts();
      delay(100);
    //calibrate();
    hlw8012.setCurrentMultiplier(0.00);
    hlw8012.setVoltageMultiplier(7832199.83);
    hlw8012.setPowerMultiplier(6332823.51);
  
#ifdef DEBUG
  Serial.println(F("Leave setup"));
  Serial.println();
#endif   

  // Start job
  do_send(&sendjob);
}

void loop() {
  bool newdata = false;
  unsigned long start = millis();

if (next == false) {  
    LMIC.skipRX = 1; // Do NOT wait for downstream data!
    os_runloop_once();       
  }else {
        delay(1000);    
        next = false;
        do_send(&sendjob);
  }
}


