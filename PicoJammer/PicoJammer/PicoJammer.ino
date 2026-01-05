#include <Arduino.h>
#include <SPI.h> 
#include <map>

#define BridgeSerial Serial1

#define VDEBUG //Verbose DEBUG -> more output

#define EXTSERIALTX_PIN          16
#define EXTSERIALRX_PIN          17
#define EXTSERIALBAUDRATE        57600

#define BUTTON_PIN               15
#define LED_PIN                  25

#define CC1101_SCK_PIN           10  // Valid SPI1 SCK
#define CC1101_MOSI_PIN          11  // Valid SPI1 TX
#define CC1101_MISO_PIN          12  // Valid SPI1 RX
#define CC1101_CSN_PIN           13  // Chip Select
#define CC1101_GDO0_PIN          9   // Interrupt

#define SIZE_ADDRESS       (6+1)    // address has 6 chars
#define SIZE_SERIAL        (10+1)   // serial has 10 chars
#define SIZE_TYPE            32
#define SIZE_FLAGS           32
#define SIZE_MSG             256

#define DEBOUNCE_DELAY 50

template <uint8_t CS, uint32_t CLOCK = 2000000, BitOrder BITORDER = MSBFIRST, uint8_t MODE = SPI_MODE0>
class PicoSPI {
  public:
    void init() {
      pinMode(CS, OUTPUT);
      digitalWrite(CS, HIGH);
      
      // Map SPI1 to the valid hardware pins
      SPI1.setSCK(CC1101_SCK_PIN); // GP10
      SPI1.setTX(CC1101_MOSI_PIN); // GP11
      SPI1.setRX(CC1101_MISO_PIN); // GP12
      
      SPI1.begin(); 
    }

    void shutdown() {
      SPI1.end();
      pinMode(CS, INPUT);
    }

    void select() {
      digitalWrite(CS, LOW);
    }

    void deselect() {
      digitalWrite(CS, HIGH);
    }

    // Ping is required to wake the radio
    void ping() {
      SPI1.beginTransaction(SPISettings(CLOCK, BITORDER, MODE));
      select();
      SPI1.transfer(0); 
      deselect();
      SPI1.endTransaction();
    }

    void waitMiso() {
      // Safety timeout to prevent crashing if wiring is wrong
      unsigned long start = millis();
      while(digitalRead(CC1101_MISO_PIN) && (millis() - start < 1000)); 
    }

    uint8_t send(uint8_t data) {
      SPI1.beginTransaction(SPISettings(CLOCK, BITORDER, MODE));
      uint8_t ret = SPI1.transfer(data);
      SPI1.endTransaction();
      return ret;
    }
    
    uint8_t strobe(uint8_t cmd) {
      SPI1.beginTransaction(SPISettings(CLOCK, BITORDER, MODE));
      select();
      uint8_t ret = SPI1.transfer(cmd);
      deselect();
      SPI1.endTransaction();
      return ret;
    }

    void readBurst(uint8_t * buf, uint8_t regAddr, uint8_t len) {
      SPI1.beginTransaction(SPISettings(CLOCK, BITORDER, MODE));
      select();
      SPI1.transfer(regAddr);
      for (uint8_t i = 0; i < len; i++) {
        buf[i] = SPI1.transfer(0x00);
      }
      deselect();
      SPI1.endTransaction();
    }

    void writeBurst(uint8_t regAddr, const uint8_t* buf, uint8_t len) {
      SPI1.beginTransaction(SPISettings(CLOCK, BITORDER, MODE));
      select();
      SPI1.transfer(regAddr);
      for (uint8_t i = 0; i < len; i++) {
        SPI1.transfer(buf[i]);
      }
      deselect();
      SPI1.endTransaction();
    }

    uint8_t readReg(uint8_t regAddr, uint8_t regType = 0) {
      SPI1.beginTransaction(SPISettings(CLOCK, BITORDER, MODE));
      select();
      SPI1.transfer(regAddr | regType);
      uint8_t val = SPI1.transfer(0x00);
      deselect();
      SPI1.endTransaction();
      return val;
    }

    void writeReg(uint8_t regAddr, uint8_t val) {
      SPI1.beginTransaction(SPISettings(CLOCK, BITORDER, MODE));
      select();
      SPI1.transfer(regAddr);
      SPI1.transfer(val);
      deselect();
      SPI1.endTransaction();
    }
};

#define EI_NOTEXTERNAL
#include <AskSinPP.h>
#include <Device.h>
#include <Register.h>
#include <Message.h>
#include <Radio.h>

// substring: ending index is exclusive, so it is fine to use the next starting index as end
#define STRPOS_RSSI_BEGIN     1
#define STRPOS_LENGTH_BEGIN   3
#define STRPOS_COUNT_BEGIN    5
#define STRPOS_FLAGS_BEGIN    7
#define STRPOS_TYPE_BEGIN     9
#define STRPOS_FROM_BEGIN     11
#define STRPOS_TO_BEGIN       17
#define STRPOS_PAYLOAD_BEGIN  23

struct _SerialBuffer {
  String   Msg            = "";
  time_t   t              = 0;
} SerialBuffer[255];
uint8_t  msgBufferCount = 0;
uint32_t allCount = 0;


void receiveMessages() {
  static String inStr = "";
  while ( (BridgeSerial.available() > 0) ) {
    char inChar = (char)BridgeSerial.read();
    if (inChar != ';') {
      if (inChar != '\n' && inChar != '\r') {
        inStr += inChar;
      }
    } else {
      bool messageFound = false;
      //DPRINTLN("MESSAGE #"+String(msgBufferCount)+" ADDED: "+inStr);
      /* each telegramm shall start with ':' and end with ';' */
      //DPRINTLN("inStr: " + inStr);
      if (inStr[0] == ':') {
        messageFound = true;
      } else {
        if (inStr.startsWith("Packet too big")) {
          DPRINT(F("INVALID MESSAGE (too big) DISCARDED: ")); DPRINTLN(inStr);
        } else {
          int startPos = inStr.lastIndexOf(':');
          if (startPos == -1) {
            DPRINT(F("INVALID MESSAGE (no ':' found) DISCARDED: ")); DPRINTLN(inStr);
          } else {
            DPRINT(F("MESSAGE DOES NOT START WITH ':' ")); DPRINTLN(inStr);
            inStr = inStr.substring(startPos);
            messageFound = true;
            DPRINT(F("CORRECTED MESSAGE: ")); DPRINTLN(inStr);
          }
        }
      }
      if (messageFound) {
        if (inStr.length() > 3){
          //normal telegram
          if (msgBufferCount > sizeof(SerialBuffer)){
            msgBufferCount = 0;
          }
          digitalWrite(LED_BUILTIN, HIGH);
          SerialBuffer[msgBufferCount].Msg = inStr;
          SerialBuffer[msgBufferCount].t = 0;
          msgBufferCount++;
          delay(10);
          digitalWrite(LED_BUILTIN, LOW);
        }else{
          //RSSI message
          String rssiIn = inStr.substring(STRPOS_RSSI_BEGIN, STRPOS_LENGTH_BEGIN);
          int rssi = -1 * (strtol(&rssiIn[0], NULL, 16) & 0xFF);
          if (rssi > -30){ 
            DPRINTLN("RSSI:" + String(rssi));
          }
        }
      }
      inStr = "";
    }
  }
}



bool fillLogTable(const _SerialBuffer &sb, uint8_t b) {
  // bool dataIsRSSIOnly = ((sb.Msg).length() == 3);

  // #ifdef VDEBUG
  // if (!dataIsRSSIOnly) {
    DPRINTLN(F("# PROCESSING SERIAL DATA #"));
    DPRINT("IP");
    DPRINT(" #");
    DPRINT(String(b));
    DPRINT(": ");
    DPRINTLN(sb.Msg);
  // }
  // #endif

  String rssiIn = (sb.Msg).substring(STRPOS_RSSI_BEGIN, STRPOS_LENGTH_BEGIN);
  int rssi = -1 * (strtol(&rssiIn[0], NULL, 16) & 0xFF);


  // if (dataIsRSSIOnly) {
  //   if (rssi > -30){ 
  //     DPRINTLN("RSSI:" + String(rssi));
  //   }
  //   return false;
  // }

  String lengthIn = (sb.Msg).substring(STRPOS_LENGTH_BEGIN, STRPOS_COUNT_BEGIN);
  uint8_t len = strtol(&lengthIn[0], NULL, 16) & 0xFF;

  String countIn = (sb.Msg).substring(STRPOS_COUNT_BEGIN, STRPOS_FLAGS_BEGIN);
  uint8_t cnt = strtol(&countIn[0], NULL, 16) & 0xFF;
  
  String flagsStr =  (sb.Msg).substring(STRPOS_FLAGS_BEGIN, STRPOS_TYPE_BEGIN);
  uint8_t flags = strtol(&flagsStr[0], NULL, 16) & 0xFF;

  String typStr =  (sb.Msg).substring(STRPOS_TYPE_BEGIN, STRPOS_FROM_BEGIN);
  uint8_t typ = strtol(&typStr[0], NULL, 16) & 0xFF;
  
  String fromStr = "";
  String toStr = "";

  if (fromStr == "")  fromStr = "  " + (sb.Msg).substring(STRPOS_FROM_BEGIN, STRPOS_TO_BEGIN) + "  ";
  if (toStr == "")    toStr = "  " + (sb.Msg).substring(STRPOS_TO_BEGIN, STRPOS_PAYLOAD_BEGIN) + "  ";

  
  char msg[SIZE_MSG];
  // pomijanie nagłówka
  (sb.Msg).substring(1+10+12).toCharArray(msg, SIZE_MSG);
  String msgStr = "";
  for (uint8_t i = 0; i< SIZE_MSG; i++) {
    if (msg[i] == 0) break;
    msgStr += msg[i];
    if (i % 2) msgStr += " ";
  }

  DPRINTLN("RSSI:" + String(rssi));
  DPRINTLN("LENGTH:" + String(len));
  DPRINTLN("COUNT:" + String(cnt));
  DPRINTLN("FLAGS:" + String(flags));
  DPRINTLN("TYPE:" + String(typ));
  DPRINTLN("FROM:" + String(fromStr));
  DPRINTLN("TO:" + String(toStr));
  DPRINTLN("MSG:" + String(msgStr));
  allCount++;

  #ifdef VDEBUG
  //DPRINTLN(F("\nAdded to LogTable: "));
  DPRINT(F("######## PROCESSING ")); DDEC(allCount); DPRINTLN(F(" END ########\n"));
  #endif
  return true;
}


using namespace as;

const struct DeviceInfo PROGMEM devinfo = {
  {0xFF, 0xFF, 0xFF},     // Device ID
  "..........",           // Device Serial
  {0x00, 0x00},           // Device Model
  0x10,                   // Firmware Version
  as::DeviceType::Remote, // Device Type
  {0x00, 0x00}            // Info Bytes
};

typedef AskSin<StatusLed<4>, NoBattery, Radio<PicoSPI<CC1101_CSN_PIN, 2000000, MSBFIRST, SPI_MODE0>, CC1101_GDO0_PIN>> HalType;
class JammerDevice : public Device<HalType, DefList0>, Alarm {
    DefList0 l0;
  public:
    typedef Device<HalType, DefList0> BaseDevice;
    JammerDevice (const DeviceInfo& i, uint16_t addr) : BaseDevice(i, addr, l0, 0), Alarm(0), l0(addr)  {}
    virtual ~JammerDevice () {}

    virtual void trigger (__attribute__ ((unused)) AlarmClock& clock) {
      // set(millis2ticks(RSSI_POLL_INTERVAL));
      // clock.add(*this);
      // this->radio().pollRSSI();
      // DPRINT(":"); DHEX(this->radio().rssi());DPRINTLN(";");
    }

    

    bool init (HalType& hal) {
      HMID id;
      this->setHal(hal);
      this->getDeviceID(id);
      hal.init(id);
      hal.config(this->getConfigArea());
      sysclock.add(*this);
      return false;
    }
};

HalType hal;
JammerDevice sdev(devinfo, 0x20);


Message deg5;
uint8_t fullData[] = {0x17, 0x10, 0x00, 0x8E, 0xBE, 0xBD, 0x0D, 0x64, 0x4A, 0xB1, 0x00, 0x00, 0x24, 0xEF, 0xE5, 0x22, 0x06, 0x85, 0xCB, 0x88, 0xCD, 0x27, 0x79, 0xE5};


bool sendRaw(uint8_t* data, uint8_t len) {
  
 // 1. Force IDLE
  // 0x36 = SIDLE (Exit RX/TX, turn off freq synthesizer)
  SPI1.beginTransaction(SPISettings(6000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CC1101_CSN_PIN, LOW);
  SPI1.transfer(0x36); 
  digitalWrite(CC1101_CSN_PIN, HIGH);
  SPI1.endTransaction();

  // 2. FLUSH TX FIFO (The Fix!)
  // 0x3B = SFTX (Flush the TX buffer)
  // This deletes any leftover data from a previous failed/cut-off transmission.
  SPI1.beginTransaction(SPISettings(6000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CC1101_CSN_PIN, LOW);
  SPI1.transfer(0x3B); 
  digitalWrite(CC1101_CSN_PIN, HIGH);
  SPI1.endTransaction();
  
  // 3. Write Data to TX FIFO
  SPI1.beginTransaction(SPISettings(6000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CC1101_CSN_PIN, LOW);
  SPI1.transfer(0x7F | 0x40); // Burst write to FIFO
  SPI1.transfer(len);         // Send length
  for (uint8_t i = 0; i < len; i++) {
    SPI1.transfer(data[i]);
  }
  digitalWrite(CC1101_CSN_PIN, HIGH);
  SPI1.endTransaction();

  // 4. Strobe TX
  SPI1.beginTransaction(SPISettings(6000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CC1101_CSN_PIN, LOW);
  SPI1.transfer(0x35); // STX
  digitalWrite(CC1101_CSN_PIN, HIGH);
  SPI1.endTransaction();
  delay(5);
  //digitalWrite(LED_BUILTIN, LOW);
  return true;
}

void setup() {

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  // 1. PC Connection
  Serial.begin(115200); 

  // 2. Arduino Connection via GP16/GP17
  // Speed: 57600 for 8MHz Pro Mini
  BridgeSerial.setTX(EXTSERIALTX_PIN);
  BridgeSerial.setRX(EXTSERIALRX_PIN);
  BridgeSerial.begin(EXTSERIALBAUDRATE);

  delay(2000);
  DPRINTLN(F("- INIT START"));
  

  sdev.init(hal);
  
  hal.radio.initReg(CC1101_FREQ2, 0x21);
  hal.radio.initReg(CC1101_FREQ1, 0x65);
  hal.radio.initReg(CC1101_FREQ0, 0xCA);
  hal.runready();

  DPRINTLN(F("- INIT COMPLETE."));
  HMID stacja(0xBE, 0xBD, 0x0D);
  HMID termo(0x64,0x4A, 0xB1);
  uint8_t data[] = {0x00, 0x00, 0x24, 0xEF, 0xE5, 0x22, 0x06, 0x85, 0xCB, 0x88, 0xCD, 0x27, 0x79, 0xE5}; 
  // len powinno być 0x17, ale musimy odjąć 14 bajtów payloadu
  deg5.init(0x17-sizeof(data), 0x10, 0x8E, 0x00, 0x00, 0x00);
  deg5.to(termo);
  deg5.from(stacja);
  deg5.append(&data, sizeof(data));

  msgBufferCount = 0;
}
unsigned long packetCounter = 0;
unsigned long lastTime = 0;
unsigned long lastReleaseTime = 0;

void loop() {
  // PC -> Arduino (BridgeSerial)
  if (Serial.available()) {
    BridgeSerial.write(Serial.read());
  }

  // // Arduino (BridgeSerial) -> PC
  // if (BridgeSerial.available()) {
  //   Serial.write(BridgeSerial.read());
  // }


  receiveMessages();
    

  bool isPressed = (digitalRead(BUTTON_PIN) == LOW);
  unsigned long currentMillis = millis();

  if (isPressed && currentMillis - lastReleaseTime > DEBOUNCE_DELAY) {
      if (lastTime == 0){
       lastTime = millis(); 
      }
      // sdev.send(deg5);
      sendRaw(fullData, sizeof(fullData));
      packetCounter++;

  }else{
    
    if (lastTime != 0){
      lastReleaseTime = currentMillis;
      SPI1.beginTransaction(SPISettings(6000000, MSBFIRST, SPI_MODE0));
         digitalWrite(CC1101_CSN_PIN, LOW);
         SPI1.transfer(0x36); 
         digitalWrite(CC1101_CSN_PIN, HIGH);
         SPI1.endTransaction();

      DPRINTLN("Transmitted "+String(packetCounter)+" in "+String((millis() - lastTime))+"ms");
      lastTime = 0;
      packetCounter = 0;
    }
    if (msgBufferCount > 0) {
      for (uint8_t b = 0; b < msgBufferCount; b++) {
        bool isTelegram = fillLogTable(SerialBuffer[b], b);
      }
      msgBufferCount = 0;
    }
  }

}
