/*
 * Stock Arduino libraries
 */
#include <EEPROM.h>
#include <SPI.h>

/*
 * Custom libraries
 */
#include <U8g2lib.h>
#include <mcp23s08.h>
#include <EventManager.h>

/*
 * Own sources
 */
#include "src/IRremote/IRremoteCAP.h"
#include "src/Biino/BiinoInput.h"
#include "src/Biino/BiinoVolume.h"
#include "src/Biino/BiinoChannel.h"
#include "ir_codes_DNT_RC11.h"

/*
 * Local functions
 */
void pciSetup(byte);
 
/*
 * Definitions of constants
 */
const unsigned long BUTTON_TIMEOUT_MS = 750;

const int PIN_CS_BIINO_VOL = SS; // 10, SS signal (PB2)
const int PIN_CS_BIINO_INP = SS; // 10, SS signal (PB2)

const int PIN_CS_DISP = A0; // 14, CS signal (PC0)
const int PIN_DC_DISP = A1; // 15, DC signal (PC1)
const int PIN_RESET_DISP = A5; // 19, RESET signal (PC5)

const int PIN_AMP_MUTE = 5; // 5, D5 (PD5)
const int PIN_REL1 = 6; // 6, D6 (PD6)
const int PIN_REL2 = 7; // 7, D7 (PD7)
const int PIN_POT = A2; // 16, A2 (PC2)
const int PIN_BUTTON = A3; // 17, A3 (PC3)
const int PIN_IR_RCV = 2; // 2, D2, INT0 (PD2)
const int PIN_ROT_STEP = 3; // 3, D3, INT1 (PD3)
const int PIN_ROT_DIR = 4; // 4, D4 (PD4)
const int PIN_ROT_BUTTON = 8; // 8, D8 (PB0)

const int INPUT_CHANNEL_CNT = 3;
const int INPUT_CHANNEL_INV = -1;
const int EE_BIINO_CHAN = 100;
const int EE_BIINO_VOL = 101;
const int EE_BIINO_VOL_START = 102;
const int EE_BIINO_VOL_END = EE_BIINO_VOL_START + INPUT_CHANNEL_CNT - 1;

const int ADDR_BIINO_VOL = 0x20;
const int ADDR_BIINO_INP = 0x21;


enum class RawEvents : int {
  // An input button was pressed and released within BUTTON_TIMEOUT_MS, param: button id, e.g. PIN_ROT_BUTTON, PIN_BUTTON
  kEventButtonPressedShort,
  // An input button was pressed and held exceeding BUTTON_TIMEOUT_MS, param: button id, e.g. PIN_ROT_BUTTON, PIN_BUTTON
  kEventButtonPressedLong,
  // The rotary button was turned, param: DIRECTION (1 or 0)
  kEventRotaryTurnTriggered,
  // 
  kEventUpdateDisplay,
  kEventIrAmpPower,
  kEventIrAmpMute,
  kEventIrVolumeUp,
  kEventIrVolumeDown,
  kEventVolumeSet,
  kEventIrInputSelectPrev,
  kEventIrInputSelectNext,
  kEventIrInputSelectId
  };
  
enum class SysState : int8_t {
  INVALID = -1,
  INITIAL = 1,
  AMP_MUTE = 2,
  AMP_OFF = 3,
  RUNNING_MODE_VOL = 4,
  RUNNING_MODE_INP = 5,  
};

enum class Rs232State : int8_t {
  WAIT_FOR_SENTENCE_START,
  WAIT_FOR_CMD,
  WAIT_FOR_PARAMETER,
  WAIT_FOR_SENTENCE_DELIMITER,
  SENTENCE_COMPLETE,
  SENTENCE_INVALID  
};

/*
 * Global variables
 */
IRrecv gIrReceiver(PIN_IR_RCV);
decode_results results;
U8G2_SSD1306_128X64_NONAME_1_4W_HW_SPI u8g2(U8G2_R0, PIN_CS_DISP, PIN_DC_DISP, PIN_RESET_DISP);
EventManager gMyEventManager;

static struct {
  bool last_state = LOW;
  bool cur_state = LOW;
  bool active = false;
  unsigned long tstart_ms;
  unsigned long tcur_ms;
  } gButtonState[2];


// May save 16 bytes of RAM when defined within loop!
static struct {
  unsigned long last_cmd = 0;
  unsigned long tfirst_ms = 0;
  unsigned long tlast_ms = 0;
  unsigned long cmd_cnt = 0;
} gIrState;

static struct {
  char last_cmd = 0;
  char last_param[3] = {0,0,0};
  char ptr = 0;
  unsigned long ser_tlast_ms = 0;
} gSerialBuffer;

SysState gState = SysState::AMP_OFF;
Rs232State gRs232State = Rs232State::WAIT_FOR_SENTENCE_START;

BiinoInput gBiinoInput(0,(uint8_t)((1<<INPUT_CHANNEL_CNT) - 1),PIN_CS_BIINO_VOL,ADDR_BIINO_INP,EE_BIINO_CHAN);
BiinoVolume gBiinoVolume(0,PIN_CS_BIINO_VOL,ADDR_BIINO_VOL,EE_BIINO_VOL);

BiinoChannel channels[INPUT_CHANNEL_CNT] = { 
  BiinoChannel(0,"Network Player", "NET", EE_BIINO_VOL_START, &gBiinoInput), 
  BiinoChannel(1,"Auxiliary 1", "AUX 1", EE_BIINO_VOL_START + 1, &gBiinoInput),
  BiinoChannel(2,"Auxiliary 2", "AUX 2", EE_BIINO_VOL_START + 2, &gBiinoInput)
};

/*
BiinoChannel channels[INPUT_CHANNEL_CNT] = { 
  BiinoChannel(0,"Network Player", "NET", EE_BIINO_VOL_START), 
  BiinoChannel(1,"Auxiliary 1", "AUX 1", EE_BIINO_VOL_START + 1),
  BiinoChannel(2,"Auxiliary 2", "AUX 2", EE_BIINO_VOL_START + 2)
};
*/

void setup() {
  // Setup digital I/O pin directions
  pinMode(PIN_CS_BIINO_VOL,OUTPUT);
  pinMode(PIN_CS_BIINO_INP,OUTPUT);
  pinMode(PIN_CS_DISP,OUTPUT);
  pinMode(PIN_DC_DISP,OUTPUT);
  pinMode(PIN_RESET_DISP,OUTPUT);
  pinMode(PIN_AMP_MUTE,OUTPUT);
  pinMode(PIN_REL1,OUTPUT);  
  pinMode(PIN_REL2,OUTPUT);  
  pinMode(PIN_BUTTON,INPUT);
  digitalWrite(PIN_BUTTON,HIGH);
  pinMode(PIN_ROT_STEP,INPUT);  
  pinMode(PIN_ROT_DIR,INPUT);
  pinMode(PIN_ROT_BUTTON,INPUT);
  digitalWrite(PIN_ROT_BUTTON,HIGH);

  // Initial amplifier state
  amp_off();  
  amp_unmute();

  // Power up network player
  rpi_on();

  // Setup serial port
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }  

  // Setup biino
  gBiinoInput.setup();
  gBiinoVolume.setup();

  // Init event manager (16 listeners max)
  gMyEventManager.addListener( (int)RawEvents::kEventButtonPressedShort, button_listener );
  gMyEventManager.addListener( (int)RawEvents::kEventButtonPressedLong, button_listener );
  gMyEventManager.addListener( (int)RawEvents::kEventRotaryTurnTriggered, rotary_listener );
  gMyEventManager.addListener( (int)RawEvents::kEventIrAmpPower, ir_listener );
  gMyEventManager.addListener( (int)RawEvents::kEventIrAmpMute, ir_listener );
  gMyEventManager.addListener( (int)RawEvents::kEventIrVolumeUp, ir_listener );
  gMyEventManager.addListener( (int)RawEvents::kEventIrVolumeDown, ir_listener );
  gMyEventManager.addListener( (int)RawEvents::kEventIrInputSelectPrev, ir_listener );
  gMyEventManager.addListener( (int)RawEvents::kEventIrInputSelectNext, ir_listener );
  gMyEventManager.addListener( (int)RawEvents::kEventIrInputSelectId, ir_listener );
  gMyEventManager.addListener( (int)RawEvents::kEventUpdateDisplay, display_listener );

  // Setup Rotary encoder
  attachInterrupt(digitalPinToInterrupt(PIN_ROT_STEP),read_rotary,FALLING);
  
  // Assign pin change interrupts
  pciSetup(PIN_BUTTON);
  pciSetup(PIN_ROT_BUTTON);
  
  // Setup IR receiver
  gIrReceiver.enableIRIn();
  
  // Init display
  u8g2.begin();
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_inb19_mf); //u8g2_font_ncenB14_tr);
    u8g2.drawStr(0,24,"Welcome!");
    u8g2.drawStr(0,63,">>>><<<<");
  } while ( u8g2.nextPage() );
  delay(2000);
  gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
}

/*
 * Main loop
 */
void loop() {
  unsigned long tcur_ms = 0;
  
  tcur_ms = millis();

  /*  
   * Process IR commands
   */
  if (gIrReceiver.decode(&results)) {
    if(results.decode_type == NEC) {
      if(results.bits == 32 || (results.bits == 0 && gIrState.last_cmd != 0)) {
        if(results.bits == 32) {
          // New command
          gIrState.last_cmd = results.value;
          gIrState.tfirst_ms = tcur_ms;
          gIrState.cmd_cnt = 1;
        }
        else {
          // Repeat
          gIrState.cmd_cnt++;  
        }

        gIrState.tlast_ms = tcur_ms;

        switch(gIrState.last_cmd) {
          case (unsigned long) ir_codes_dnt_rc11::KEY_VOLUMEUP:
            gMyEventManager.queueEvent( (int)RawEvents::kEventIrVolumeUp, gIrState.cmd_cnt);
            break;
          case (unsigned long) ir_codes_dnt_rc11::KEY_VOLUMEDOWN:
            gMyEventManager.queueEvent( (int)RawEvents::kEventIrVolumeDown, gIrState.cmd_cnt );
            break;
          case (unsigned long) ir_codes_dnt_rc11::KEY_POWER:
            gMyEventManager.queueEvent( (int)RawEvents::kEventIrAmpPower, gIrState.cmd_cnt );
            break;
          case (unsigned long) ir_codes_dnt_rc11::KEY_MUTE:
            gMyEventManager.queueEvent( (int)RawEvents::kEventIrAmpMute, gIrState.cmd_cnt );
            break;
          case (unsigned long) ir_codes_dnt_rc11::KEY_DOWN:
            gMyEventManager.queueEvent( (int)RawEvents::kEventIrInputSelectPrev, gIrState.cmd_cnt );
            break;
          case (unsigned long) ir_codes_dnt_rc11::KEY_UP:
            gMyEventManager.queueEvent( (int)RawEvents::kEventIrInputSelectNext, gIrState.cmd_cnt );
            break;                        
          case (unsigned long) ir_codes_dnt_rc11::KEY_1:
            gMyEventManager.queueEvent( (int)RawEvents::kEventIrInputSelectId, 1 );
            break;                        
          case (unsigned long) ir_codes_dnt_rc11::KEY_2:
            gMyEventManager.queueEvent( (int)RawEvents::kEventIrInputSelectId, 2 );
            break;                        
          case (unsigned long) ir_codes_dnt_rc11::KEY_3:
            gMyEventManager.queueEvent( (int)RawEvents::kEventIrInputSelectId, 3 );
            break;                        
        }

      }
    }
    
    gIrReceiver.resume();
  }

  /*  
   * Process button states
   */
  noInterrupts();
    
    // PIN_BUTTON: Input button (toggle between mute and power off)
    if(gButtonState[1].active == true) {
      if((tcur_ms-gButtonState[1].tstart_ms) >= BUTTON_TIMEOUT_MS) {
        // Perform action, if user presses button longer than BUTTON_TIMEOUT_MS
        // Short push: MUTE ON/OFF
        // Long push: AMP ON/OFF
        gMyEventManager.queueEvent( (int)RawEvents::kEventButtonPressedLong, PIN_BUTTON );
        
        gButtonState[1].active = false;
      }
    }

    // PIN_ROT_BUTTON: Rotary encoder's push button
    if(gButtonState[0].active == true) {
      if((tcur_ms-gButtonState[0].tstart_ms) >= BUTTON_TIMEOUT_MS) {
        // Perform action, if user presses button longer than BUTTON_TIMEOUT_MS
        gMyEventManager.queueEvent( (int)RawEvents::kEventButtonPressedLong, PIN_ROT_BUTTON );
        gButtonState[0].active = false;
      }
    }
    
  interrupts();  

  /*
   * Process serial commands
   */

  if(Serial.available() > 0)
  {
    char serbyte = Serial.read();
    
    switch(gRs232State) {
      case Rs232State::WAIT_FOR_SENTENCE_START:
        if(serbyte == '#')
          gRs232State = Rs232State::WAIT_FOR_CMD;
        break;
      case Rs232State::WAIT_FOR_CMD:
        if(serbyte == 'm' || serbyte == 'p' || serbyte == 'i' || serbyte == 'v') {
          gSerialBuffer.last_cmd = serbyte;
          gRs232State = Rs232State::WAIT_FOR_PARAMETER;
        }
        else
          gRs232State = Rs232State::WAIT_FOR_SENTENCE_START;
        break;
      case Rs232State::WAIT_FOR_PARAMETER:
        if((gSerialBuffer.last_cmd == 'm' || gSerialBuffer.last_cmd == 'p') && (serbyte == '1' || serbyte == '0')) {
          gSerialBuffer.last_param[0] = serbyte;
          gRs232State = Rs232State::WAIT_FOR_SENTENCE_DELIMITER;  
        }
        else
        if(gSerialBuffer.last_cmd == 'i' && (serbyte == '1' || serbyte == '2' || serbyte == '3')) {
          gSerialBuffer.last_param[0] = serbyte;
          gRs232State = Rs232State::WAIT_FOR_SENTENCE_DELIMITER;
        }
        else
        if(gSerialBuffer.last_cmd == 'v' && (serbyte == '+' || serbyte == '-')) {
          gSerialBuffer.last_param[0] = serbyte;
          gRs232State = Rs232State::WAIT_FOR_SENTENCE_DELIMITER;
        }
        else
        if(gSerialBuffer.last_cmd == 'v' && (serbyte >= '0' || serbyte <= '9' || serbyte == '$')) {
          if(serbyte != '$') {
            if(gSerialBuffer.ptr < 2) {
              gSerialBuffer.last_param[gSerialBuffer.ptr++] = serbyte;
            } 
            else {
              gRs232State = Rs232State::WAIT_FOR_SENTENCE_DELIMITER;
            }
          }
          else {
            gRs232State = Rs232State::SENTENCE_COMPLETE;
          } 
        }
        break;
      case Rs232State::WAIT_FOR_SENTENCE_DELIMITER:
        if(serbyte == '$')
          gRs232State = Rs232State::SENTENCE_COMPLETE;
        else
          gRs232State = Rs232State::SENTENCE_INVALID;

        if(gRs232State == Rs232State::SENTENCE_COMPLETE) {
          switch(gSerialBuffer.last_cmd) {
            case 'p':
                gMyEventManager.queueEvent( (int)RawEvents::kEventIrAmpPower, 1 );
              break;
            case 'm':
                gMyEventManager.queueEvent( (int)RawEvents::kEventIrAmpMute, 1 );
              break;
          }
        } 

        gSerialBuffer.last_cmd = 0;
        gSerialBuffer.last_param[0] = 0;
        gSerialBuffer.last_param[1] = 0;
        gSerialBuffer.last_param[2] = 0;
        gSerialBuffer.ptr = 0;
        gRs232State = Rs232State::WAIT_FOR_SENTENCE_START; 
        break;
    }
    Serial.println((int8_t)gRs232State,DEC);
    
  } // Serial.Available > 0

  /*
   * Process event queue
   */
  gMyEventManager.processEvent();
  
}

/*
 * AMP Mute, Unmute, On, Off Functionality
 */
void amp_mute()
{
  digitalWrite(PIN_AMP_MUTE,0);
}

void amp_unmute()
{
  digitalWrite(PIN_AMP_MUTE,1);
}

void amp_off()
{
  digitalWrite(PIN_REL1,0);
}

void amp_on()
{
  digitalWrite(PIN_REL1,1);
}

void rpi_on()
{
  digitalWrite(PIN_REL2,1);
}

/*
 * Setup interrupts
 */
void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

/*
 * Event listeners
 */
void button_listener( int event, int param )
{
  if(event == (int)RawEvents::kEventButtonPressedShort) {
    if(param == PIN_BUTTON) {
      switch(gState) {
        case SysState::INITIAL:
          // TODO
          break;
        case SysState::AMP_MUTE:
          amp_unmute();
          gState = SysState::RUNNING_MODE_VOL;    
          gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
          break;
        case SysState::AMP_OFF:
          // Do nothing.
          break;
        case SysState::RUNNING_MODE_VOL:
          amp_mute();
          gState = SysState::AMP_MUTE;
          gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
          break;
        case SysState::RUNNING_MODE_INP:
          amp_mute();
          gState = SysState::AMP_MUTE;
          gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
          break;
        default:
          // Do nothing.
          break;
      }
    }
    else
    if(param == PIN_ROT_BUTTON) {
      switch(gState) {
        case SysState::INITIAL:
          // TODO
          break;
        case SysState::AMP_MUTE:
          // Do nothing.
          break;
        case SysState::AMP_OFF:
          // Do nothing.
          break;
        case SysState::RUNNING_MODE_VOL:
          gState = SysState::RUNNING_MODE_INP;
          gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
          break;
        case SysState::RUNNING_MODE_INP:
          gState = SysState::RUNNING_MODE_VOL;
          gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
          break;
        default:
          // Do nothing.
          break;
      }
    }
  } // if(event == (int)RawEvents::kEventButtonPressedShort)
  else
  if(event == (int)RawEvents::kEventButtonPressedLong) {
    if(param == PIN_BUTTON) {
      switch(gState) {
        case SysState::INITIAL:
          // TODO
          break;
        case SysState::AMP_MUTE:
          amp_off();
          gState = SysState::AMP_OFF;    
          gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
          break;
        case SysState::AMP_OFF:
          amp_on();
          amp_unmute();
          gState = SysState::RUNNING_MODE_VOL;
          gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
          break;
        case SysState::RUNNING_MODE_VOL:
          amp_off();
          gState = SysState::AMP_OFF;
          gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
          break;
        case SysState::RUNNING_MODE_INP:
          amp_off();
          gState = SysState::AMP_OFF;
          gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
          break;
        default:
          // Do nothing.
          break;
      }
    }
  } // if(event == (int)RawEvents::kEventButtonPressedLong)
}
    
void rotary_listener( int event, int param )
{
  switch(gState) {
    case SysState::INITIAL:
      // TODO
      break;
    case SysState::AMP_MUTE:
      // Do nothing.
      break;
    case SysState::AMP_OFF:
      // Do nothing.
      break;
    case SysState::RUNNING_MODE_VOL:
      if(param == 1) {
        gBiinoVolume.incVolume();
      }
      else if(param == 0) {
        gBiinoVolume.decVolume();
      }
      else
        gBiinoVolume.setVolume(0);
      
      gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
      break;
    case SysState::RUNNING_MODE_INP:
      if(param == 1) {
        gBiinoInput.selectNext();
      }
      else if(param == 0) {
        gBiinoInput.selectPrevious();
      }
      else
        gBiinoInput.selectFirst();
        
      gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
      break;
    default:
      // Do nothing.
      break;
  }
}

void ir_listener( int event, int param ) {
  switch(event) {
    case (int)RawEvents::kEventIrAmpPower:
      if(gState != SysState::AMP_OFF && param == 1) {
        amp_off();
        gState = SysState::AMP_OFF;
        gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
      }
      else
      if(gState == SysState::AMP_OFF && param == 1) {
        amp_on();
        amp_unmute();
        gState = SysState::RUNNING_MODE_VOL;
        gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
      }
      break;
    case (int)RawEvents::kEventIrAmpMute:
      if((gState == SysState::RUNNING_MODE_VOL || gState == SysState::RUNNING_MODE_INP) && param == 1) {
        amp_mute();
        gState = SysState::AMP_MUTE;
        gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
      }
      else
      if(gState == SysState::AMP_MUTE && param == 1) {
        amp_unmute();
        gState = SysState::RUNNING_MODE_VOL;
        gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
      }
      break;
    case (int)RawEvents::kEventIrVolumeUp:
      if(gState == SysState::AMP_MUTE && param == 1) {
        amp_unmute();
        gState = SysState::RUNNING_MODE_VOL;
        gBiinoVolume.incVolume();
        gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
      }
      else
      if((gState == SysState::RUNNING_MODE_VOL || gState == SysState::RUNNING_MODE_INP) && param >= 1) {
        gState = SysState::RUNNING_MODE_VOL;
        gBiinoVolume.incVolume();
        gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
      }
      break;
    case (int)RawEvents::kEventIrVolumeDown:
      if((gState == SysState::RUNNING_MODE_VOL || gState == SysState::RUNNING_MODE_INP) && param >= 1) {
        gState = SysState::RUNNING_MODE_VOL;
        gBiinoVolume.decVolume();
        gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
      }
      break;
    case (int)RawEvents::kEventIrInputSelectNext:
      if((gState == SysState::RUNNING_MODE_VOL || gState == SysState::RUNNING_MODE_INP) && param == 1) {
        gState = SysState::RUNNING_MODE_INP;
        gBiinoInput.selectNext();
        gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
      }    
      break;
    case (int)RawEvents::kEventIrInputSelectPrev:
      if((gState == SysState::RUNNING_MODE_VOL || gState == SysState::RUNNING_MODE_INP) && param == 1) {
        gState = SysState::RUNNING_MODE_INP;
        gBiinoInput.selectPrevious();
        gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
      }    
      break;
    case (int)RawEvents::kEventIrInputSelectId:
      if((gState == SysState::RUNNING_MODE_VOL || gState == SysState::RUNNING_MODE_INP)) {
        gState = SysState::RUNNING_MODE_INP;
        gBiinoInput.select((uint8_t)(1 << (param-1)));
        gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
      }    
      break;      
    default:
      break;
  }
}

void display_listener( int event, int param ) {
  switch(gState) {
    case SysState::INITIAL:
      // TODO
      break;
    case SysState::AMP_MUTE:
      u8g2.firstPage();
      do {
        u8g2.setFont(u8g2_font_inb19_mf);
        u8g2.drawStr(0,24,"Mute");
      } while ( u8g2.nextPage() );
      break;
    case SysState::AMP_OFF:
      u8g2.firstPage();
      do {
        u8g2.setFont(u8g2_font_inb19_mf);
        u8g2.drawStr(0,24,"Off");
      } while ( u8g2.nextPage() );
      break;
    case SysState::RUNNING_MODE_VOL:
      u8g2.firstPage();
      do {
        u8g2.setFont(u8g2_font_inb19_mf);
        u8g2.setCursor(0,24);
        u8g2.print("VOL: ");
        u8g2.print(gBiinoVolume.getVolume(),DEC);
//        u8g2.print("ROM: ");
//        u8g2.print(EEPROM[gBiinoVolume.ee_addr_cur_volume],DEC);
      } while ( u8g2.nextPage() );
      break;
    case SysState::RUNNING_MODE_INP:
      u8g2.firstPage();
      do {
        u8g2.setFont(u8g2_font_inb19_mf);
        u8g2.setCursor(0,24);
        u8g2.print("INP: ");
        u8g2.print(gBiinoInput.getCurrentChannelNo(),DEC);
      } while ( u8g2.nextPage() );
      break;
    default:
      // Do nothing.
      break;
  }
}


/*
 * Interrupt routines
 */
void read_rotary(void)
{
  gMyEventManager.queueEvent( (int)RawEvents::kEventRotaryTurnTriggered, digitalRead(PIN_ROT_DIR));
}

/*
 * Port B PCINT (D8 to D13, PCINT0 to PCINT7, PB0 to PB7)
 */
ISR (PCINT0_vect)
{
  gButtonState[0].cur_state = digitalRead(PIN_ROT_BUTTON);
  
  if (gButtonState[0].cur_state == HIGH && gButtonState[0].last_state == LOW) {
    gButtonState[0].tstart_ms = millis(); 
    gButtonState[0].tcur_ms = gButtonState[0].tstart_ms;
    gButtonState[0].active = true;
  }
  else
  if (gButtonState[0].cur_state == LOW && gButtonState[0].last_state == HIGH) {
    gButtonState[0].tcur_ms = millis();
    gButtonState[0].active = false;
    
    if((gButtonState[0].tcur_ms - gButtonState[0].tstart_ms) < BUTTON_TIMEOUT_MS)
      gMyEventManager.queueEvent( (int)RawEvents::kEventButtonPressedShort, PIN_ROT_BUTTON );
  }

  gButtonState[0].last_state = gButtonState[0].cur_state;
} 

/*
 * Port C PCINT (A0 to A5, PCINT8 to PCINT14, PC0 to PC6)
 */
ISR (PCINT1_vect)
{
  gButtonState[1].cur_state = digitalRead(PIN_BUTTON);
  
  if (gButtonState[1].cur_state == HIGH && gButtonState[1].last_state == LOW) {
    // Button pushed.
    gButtonState[1].tstart_ms = millis(); 
    gButtonState[1].tcur_ms = gButtonState[1].tstart_ms;
    gButtonState[1].active = true;
  }
  else
  if (gButtonState[1].cur_state == LOW && gButtonState[1].last_state == HIGH) {
    // Button released.
    gButtonState[1].tcur_ms = millis();
    gButtonState[1].active = false;
    
    if((gButtonState[1].tcur_ms - gButtonState[1].tstart_ms) < BUTTON_TIMEOUT_MS)
      gMyEventManager.queueEvent( (int)RawEvents::kEventButtonPressedShort, PIN_BUTTON );
  }

  gButtonState[1].last_state = gButtonState[1].cur_state;
}  

/*
 * Port D PCINT (D0 to D7, PCINT16 to PCINT23, PD0 to PD7)
 */
ISR (PCINT2_vect)
{
}  

