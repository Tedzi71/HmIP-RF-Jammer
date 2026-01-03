#include <Arduino.h>
#include <AskSinPP.h>

// UART BridgeSerial(16, 17, -1, -1); 
#define BridgeSerial Serial1
template<class TYPE,int SIZE>
class RingStack {
  TYPE  _buffer[SIZE];
  TYPE* _current;
  int   _count;

  TYPE& first () {
    return _buffer[0];
  }

  TYPE& last () {
    return _buffer[SIZE-1];
  }

public:
  RingStack () {
    _current = &first();
    _count = 0;
  }

  int size () const { return SIZE; }

  int count () const { return _count; }

  bool shift () {
    if( _count < SIZE ) _count++;
    _current--;
    if( _current < _buffer ) _current = &last();
    return _count == SIZE;
  }

  bool shift (const TYPE& data) {
    bool result = shift();
    *_current = data;
    return result;
  }

  TYPE& operator [] (int index) {
    index = std::min(index,_count-1);
    return *(_buffer + ((_current - _buffer + index) % SIZE));
  }

  const TYPE& operator [] (int index) const {
    index = std::min(index,_count-1);
    return *(_buffer + ((_current - _buffer + index) % SIZE));
  }
};


#define VDEBUG //Verbose DEBUG -> more output


#include <map>

#define HAS_DISPLAY 0


const String CCU_SV_DEVLIST = "AskSinAnalyzerDevList";  //name of the used system variable on the CCU containing the device list
const String CCU_SV_ALARM   = "AskSinAnalyzerAlarm";  //name of the used system variable on the CCU for alarms

#define VERSION_UPPER "3"
#define VERSION_LOWER "7"

//Pin definitions for external switches
#define START_WIFIMANAGER_PIN    15 //LOW = on boot: start wifimanager, on run: switch between tft screens
#define SHOW_DISPLAY_LINES_PIN   13 //LOW = show lines between rows
#define SHOW_DISPLAY_DETAILS_PIN 12 //LOW = show detailed information on display, HIGH = show only main infos
#define RSSI_PEAK_HOLD_MODE_PIN   4 //LOW = show peak line only for noisefloor, HIGH = show also for hm(ip) messages
#define ONLINE_MODE_PIN          14 //LOW = enable WIFI

//Pin definition for LED
#define AP_MODE_LED_PIN          32

#define SD_CS                    27

//Pin definitions for serial connection to AskSinSniffer
#define EXTSERIALTX_PIN          17
#define EXTSERIALRX_PIN          16
#define EXTSERIALBAUDRATE        57600

#ifdef USE_DISPLAY
#define TFT_LED                  33
#define TFT_CS                    5
#define TFT_RST                  26
#define TFT_DC                   25
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
U8G2_FOR_ADAFRUIT_GFX u8g;

#define DISPLAY_LOG_LINE_HEIGHT  15
#define DISPLAY_LOG_OFFSET_TOP   27
enum Screens { TELEGRAM_LIST, RSSI_TEXT, RSSI_GRAPHIC, INFO };
uint8_t currentScreen = TELEGRAM_LIST;
uint16_t currentCircleColor = ILI9341_RED;
#endif

#define CSV_HEADER                  "num;time;rssi;fromaddress;from;toaddress;to;len;cnt;typ;flags;msg;"

#define IPSIZE                16
#define VARIABLESIZE          255
#define DEFAULT_NTP_SERVER    "pool.ntp.org"
#define DEFAULT_HOSTNAME      "AskSinAnalyzer"

#define RSSI_PEAK_HOLD_MILLIS 30000 //30 seconds peak hold on rssi text screen

struct _NetConfig {
  char ip[IPSIZE]             = "0.0.0.0";
  char netmask[IPSIZE]        = "0.0.0.0";
  char gw[IPSIZE]             = "0.0.0.0";
  char hostname[VARIABLESIZE] = DEFAULT_HOSTNAME;
  char ntp[VARIABLESIZE]      = DEFAULT_NTP_SERVER;
} NetConfig;

enum BackendTypes { BT_CCU, BT_OTHER };
struct _HomeMaticConfig {
  char ccuIP[IPSIZE]            = "";
  uint8_t backendType           = BT_CCU;
  bool CCUuseHTTPS              = false;
  char backendUrl[VARIABLESIZE] = "";
} HomeMaticConfig;

struct _RSSIConfig {
  uint8_t histogramBarWidth = 5;
  int8_t  alarmThreshold    = 0;
  uint8_t alarmCount        = 0;
} RSSIConfig;

#define MAX_LOG_ENTRIES      51
#define MAX_RSSILOG_ENTRIES 255
#define SIZE_ADDRESS       (6+1)    // address has 6 chars
#define SIZE_SERIAL        (10+1)   // serial has 10 chars
#define SIZE_TYPE            32
#define SIZE_FLAGS           32
#define SIZE_MSG             256

struct _LogTable {
  uint32_t lognumber                  = 0;
  uint8_t  len                        = 0;
  uint8_t  cnt                        = 0;
  time_t   time                       = 0;
  int      rssi                       = -255;
  char     fromSerial [SIZE_SERIAL];
  char     toSerial   [SIZE_SERIAL];
  char     fromAddress[SIZE_ADDRESS];
  char     toAddress  [SIZE_ADDRESS];
  uint8_t  typ                        = 0x00;
  uint8_t  flags                      = 0x00;
  char     msg        [SIZE_MSG];
};
RingStack<_LogTable,MAX_LOG_ENTRIES> LogTable;

uint16_t   logLengthDisplay           = 0;

enum RssiTypes { RSSITYPE_NONE, RSSITYPE_HMRF, RSSITYPE_HMIP };
struct _RSSILogTable {
  time_t   time                       = 0;
  int      rssi                       = -255;
  uint8_t  type                       = RSSITYPE_NONE;
  char     fromSerial [SIZE_SERIAL];
};
RingStack<_RSSILogTable,MAX_RSSILOG_ENTRIES> RSSILogTable;
bool       rssiValueAdded                 = false;
bool       rssiAlarmTriggered             = false;

struct _SerialBuffer {
  String   Msg            = "";
  time_t   t              = 0;
} SerialBuffer[255];
uint8_t  msgBufferCount = 0;

// JsonArray devices;

uint32_t allCount              = 0;
unsigned long lastDebugMillis  = 0;
bool     updating              = false;
bool     formatfs              = false;
bool     isOnline              = false;
bool     timeOK                = false;
bool     SPIFFSAvailable       = false;
bool     sdAvailable           = false;
bool     startWifiManager      = false;
bool     ONLINE_MODE           = false;
bool     RESOLVE_ADDRESS       = true;
bool     msgBufferProcessing   = true;
uint8_t  DISPLAY_LOG_LINES     = 15;
time_t   bootTime              = 0;
String   updateUrl             = "https://raw.githubusercontent.com/jp112sdl/AskSinAnalyzer/master/ota/AskSinAnalyzerESP32.bin";


// template <class T>
//   inline void DPRINT(T str) { Serial.print(str); }
//   template <class T>
//   inline void DPRINTLN(T str) { DPRINT(str); DPRINT(F("\n")); }
//   inline void DHEX(uint8_t b) {
//     if( b<0x10 ) Serial.print('0');
//     Serial.print(b,HEX);
//   }
//   inline void DHEX(uint16_t b) { 
//     if( b<0x10 ) Serial.print(F("000")); 
//     else if( b<0x100 ) Serial.print(F("00"));
//     else if( b<0x1000 ) Serial.print(F("0"));
//     Serial.print(b,HEX);
//   }
//   inline void DHEX(uint32_t b) {
//     if( b<0x10 ) Serial.print(F("0000000"));
//     else if( b<0x100 ) Serial.print(F("000000"));
//     else if( b<0x1000 ) Serial.print(F("00000"));
//     else if( b<0x10000 ) Serial.print(F("0000"));
//     else if( b<0x100000 ) Serial.print(F("000"));
//     else if( b<0x1000000 ) Serial.print(F("00"));
//     else if( b<0x10000000 ) Serial.print(F("0"));
//     Serial.print(b,HEX);
//   }

//   template<typename TYPE>
//   inline void DDEC(TYPE b) {
//     Serial.print(b,DEC);
//   }

//   #define DINIT(baudrate,msg) \
//     Serial.begin(baudrate); \
//     DPRINTLN(msg);

//   #define DDEVINFO(dev) \
//     HMID devid; \
//     dev.getDeviceID(devid); \
//     DPRINT(F("ID: "));devid.dump(); \
//     uint8_t serial[11]; \
//     dev.getDeviceSerial(serial); \
//     serial[10]=0; \
//     DPRINT(F("  Serial: "));DPRINTLN((char*)serial);

//   inline void DHEX(const uint8_t* b,uint8_t l) {
//     for( int i=0; i<l; i++, b++) {
//       DHEX(*b); DPRINT(F(" "));
//     }
//   }
//   inline void DHEXLN(uint8_t b) { DHEX(b); DPRINT(F("\n")); }
//   inline void DHEXLN(uint16_t b) { DHEX(b); DPRINT(F("\n")); }
//   inline void DHEXLN(uint32_t b) { DHEX(b); DPRINT(F("\n")); }
//   template<typename TYPE>
//   inline void DDECLN(TYPE b) { DDEC(b); DPRINT(F("\n")); }
//   inline void DHEXLN(const uint8_t* b,uint8_t l) { DHEX(b,l); DPRINT(F("\n")); }



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
        SerialBuffer[msgBufferCount].Msg = inStr;
        SerialBuffer[msgBufferCount].t = 0;
        msgBufferCount++;
        if (msgBufferCount > 1) {
          DPRINTLN(F("****************"));
          DPRINT(F("!message Buffer = "));
          DDECLN(msgBufferCount);
          DPRINTLN(F("****************"));
        }
      }
      inStr = "";
    }
  }
}

// substring: ending index is exclusive, so it is fine to use the next starting index as end
#define STRPOS_RSSI_BEGIN     1
#define STRPOS_LENGTH_BEGIN   3
#define STRPOS_COUNT_BEGIN    5
#define STRPOS_FLAGS_BEGIN    7
#define STRPOS_TYPE_BEGIN     9
#define STRPOS_FROM_BEGIN     11
#define STRPOS_TO_BEGIN       17
#define STRPOS_PAYLOAD_BEGIN  23

bool fillLogTable(const _SerialBuffer &sb, uint8_t b) {
  bool dataIsRSSIOnly = ((sb.Msg).length() == 3);

#ifdef VDEBUG
  if (!dataIsRSSIOnly) {
    DPRINTLN(F("# PROCESSING SERIAL DATA #"));
    DPRINT("I ");
    DPRINT(dataIsRSSIOnly ? "R" : "P");
    DPRINT(" #");
    DPRINT(String(b));
    DPRINT(": ");
    DPRINTLN(sb.Msg);
  }
#endif

  String rssiIn = (sb.Msg).substring(STRPOS_RSSI_BEGIN, STRPOS_LENGTH_BEGIN);
  int rssi = -1 * (strtol(&rssiIn[0], NULL, 16) & 0xFF);


  if (dataIsRSSIOnly) {
    //addRssiValueToRSSILogTable(rssi, sb.t, RSSITYPE_NONE, "NOISE");
    if (rssi > -50){ 
      DPRINTLN("RSSI:" + String(rssi));
    }
    return false;
  }

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
  // if (ONLINE_MODE && RESOLVE_ADDRESS) {
  //   fromStr = getSerialFromAddress(hexToDec((sb.Msg).substring(STRPOS_FROM_BEGIN, STRPOS_TO_BEGIN)));
  //   toStr = getSerialFromAddress(hexToDec((sb.Msg).substring(STRPOS_TO_BEGIN, STRPOS_PAYLOAD_BEGIN)));
  // }

  if (fromStr == "")  fromStr = "  " + (sb.Msg).substring(STRPOS_FROM_BEGIN, STRPOS_TO_BEGIN) + "  ";
  if (toStr == "")    toStr = "  " + (sb.Msg).substring(STRPOS_TO_BEGIN, STRPOS_PAYLOAD_BEGIN) + "  ";

  
  char msg[SIZE_MSG];
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


void addRssiValueToRSSILogTable(int8_t rssi, time_t ts, uint8_t type, const char * fromSerial) {
  RSSILogTable.shift();
  RSSILogTable[0].time = ts;
  RSSILogTable[0].rssi = rssi;
  RSSILogTable[0].type = type;
  memcpy(RSSILogTable[0].fromSerial, fromSerial, SIZE_SERIAL);
  rssiValueAdded = !rssiValueAdded;
}

String getSerialFromAddress(int intAdr) {

  return "";
}


void setup() {

  // 1. PC Connection
  Serial.begin(115200); 

  // 2. Arduino Connection via GP16/GP17
  // Speed: 57600 for 8MHz Pro Mini (or 115200 for some 16MHz)
  BridgeSerial.setTX(16);
  BridgeSerial.setRX(17);
  BridgeSerial.begin(57600);

  DPRINTLN(F("- INIT COMPLETE.\n--------------------------------"));
}

void loop() {
  // PC -> Arduino (BridgeSerial)
  if (Serial.available()) {
    BridgeSerial.write(Serial.read());
  }

  // // Arduino (BridgeSerial) -> PC
  // if (BridgeSerial.available()) {
  //   Serial.write(BridgeSerial.read());
  // }



  if (msgBufferProcessing == true && updating == false) {
  
    receiveMessages();

    if (msgBufferCount > 0) {
      for (uint8_t b = 0; b < msgBufferCount; b++) {
        bool isTelegram = fillLogTable(SerialBuffer[b], b);
        if (isTelegram && logLengthDisplay < DISPLAY_LOG_LINES) logLengthDisplay++;
      }
      msgBufferCount = 0;
    }

  }
}
