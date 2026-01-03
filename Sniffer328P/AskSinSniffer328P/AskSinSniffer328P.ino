//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2016-10-31 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
// 2019-05-24 jp112sdl Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//- -----------------------------------------------------------------------------------------------------------------------
// ci-test=yes board=328p aes=no

#define MaxDataLen   128 //to receive hmip messages that can have a bigger payload than bidcos messages 

#include <SPI.h>
#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#include <AskSinPP.h>
#include <Device.h>
#include <Register.h>
#include <Message.h>
#define RSSI_POLL_INTERVAL 750 //milliseconds

// all library classes are placed in the namespace 'as'
using namespace as;

// define all device properties
const struct DeviceInfo PROGMEM devinfo = {
  {0xFF, 0xFF, 0xFF},     // Device ID
  "..........",           // Device Serial
  {0x00, 0x00},           // Device Model
  0x10,                   // Firmware Version
  as::DeviceType::Remote, // Device Type
  {0x00, 0x00}            // Info Bytes
};

typedef AskSin<StatusLed<4>, NoBattery, Radio<LibSPI<10>, 2>> HalType;

class SnifferDevice : public Device<HalType, DefList0>, Alarm {
    DefList0 l0;
  public:
    typedef Device<HalType, DefList0> BaseDevice;
    SnifferDevice (const DeviceInfo& i, uint16_t addr) : BaseDevice(i, addr, l0, 0), Alarm(0), l0(addr)  {}
    virtual ~SnifferDevice () {}

    virtual void trigger (__attribute__ ((unused)) AlarmClock& clock) {
      set(millis2ticks(RSSI_POLL_INTERVAL));
      clock.add(*this);
      this->radio().pollRSSI();
      DPRINT(":"); DHEX(this->radio().rssi());DPRINTLN(";");
    }

    virtual bool process(Message& msg) {
      DPRINT(F(":"));
      DHEX(radio().rssi());
      DHEX(msg.length());
      DHEX(msg.count());
      DHEX(msg.flags());
      DHEX(msg.type());
      msg.from().dump();
      msg.to().dump();
      for (uint8_t l = 0; l < msg.length() - 9; l++) DHEX(msg.buffer()[l + 9]);
      DPRINTLN(";");
      this->led().ledOn(millis2ticks(100));
      return true;
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
SnifferDevice sdev(devinfo, 0x20);

void sendRaw(uint8_t* data, uint8_t len) {
  // 1. Ensure the radio is in Idle state before touching FIFO
  //hal.radio.setIdle();

  // 2. Write the raw bytes directly to the CC1101 Transmit FIFO
  // Note: The first byte of 'data' must be the Length byte
  hal.radio.sndData(data, len, 1);




    // hal->prepareSend(msg);
    bool result = false;
    uint8_t maxsend = 10;
    // bool ledmode = list0.ledMode();
    // if( ledmode == 1 ) {
    //   led().set(LedStates::send);
    // }
    while( result == false && maxsend > 0 ) {
      result = hal.radio.sndData(data, len, 0);
      DPRINT(F("<- "));
      //msg.dump();
      maxsend--;
        DPRINT(F("waitAck: ")); DHEX((uint8_t)result); DPRINTLN(F(""));
      }
    
    return result;
}

void setup () {
  DINIT(57600, ASKSIN_PLUS_PLUS_IDENTIFIER);
  sdev.init(hal);
  hal.radio.initReg(CC1101_FREQ2, 0x21);
  hal.radio.initReg(CC1101_FREQ1, 0x65);
  hal.radio.initReg(CC1101_FREQ0, 0xCA);
  hal.runready();
  HMID stacja(0xBE, 0xBD, 0x0D);
  HMID termo(0x64,0x4A, 0xB1);
  uint8_t data[] = {0x00, 0x00, 0x24, 0xEF, 0xE5, 0x22, 0x06, 0x85, 0xCB, 0x88, 0xCD, 0x27, 0x79, 0xE5}; 

  // uint8_t data[] = {0x17, 0x10, 0x00, 0x8E, 0xBE, 0xBD, 0x0D, 0x64, 0x4A, 0xB1, 0x00, 0x00, 0x24, 0xEF, 0xE5, 0x22, 0x06, 0x85, 0xCB, 0x88, 0xCD, 0x27, 0x79, 0xE5}; 


  Message deg5;
  deg5.init(0x17, 0x10, 0x8E, 0x00, 0x00, 0x00);
  deg5.to(termo);
  deg5.from(stacja);
  deg5.append(&data, sizeof(data));
  // memcpy(deg5.buffer(), data, sizeof(data));
  // memcpy(deg5.buffer() + 12, data, sizeof(data));
  DPRINTLN("Sending SET Message...");
  sdev.send(deg5);
  // sendRaw(data, sizeof(data));
}

void loop() {

  hal.runready();
  sdev.pollRadio();
}
