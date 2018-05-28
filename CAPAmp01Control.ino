/*
 * Stock Arduino libraries
 */
#include <EEPROM.h>
#include <SPI.h>
#include <avr/pgmspace.h>

/*
 * Custom libraries
 */
#include <U8g2lib.h>
#include <mcp23s08.h>
#include <TimerOne.h>
#include <EventManager.h>
#include "includes/ir/IRremoteCAP.h" // load IRremote library with custom protocol support

/*
 * Own sources
 */
#include "src/Biino/BiinoInput.h"
#include "src/Biino/BiinoVolume.h"
#include "src/Biino/BiinoChannel.h"
#include "includes/ir/ir_codes_DNT_RC11.h"
#include "includes/xbm/duck-bw-64x64.h"
//#include "xbm/squirrel-bw-67x64.h"

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
const int PIN_ROT_A = 4; // 4, D4 (PD4)
const int PIN_ROT_B = 3; // 3, D3, INT1 (PD3)
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
  kEventButtonPressedShort = 1,
  // An input button was pressed and held exceeding BUTTON_TIMEOUT_MS, param: button id, e.g. PIN_ROT_BUTTON, PIN_BUTTON
  kEventButtonPressedLong = 2,
  // The rotary button was turned, param: DIRECTION (1 or 0)
  kEventRotaryTurnTriggered = 3,
  // 
  kEventUpdateDisplay = 4,
  //
  kEventAmpControl = 5,
  kEventVolumeControl = 6,
  kEventInputControl = 7,
  //
  kEventAmpPowerOn = -1,
  kEventAmpPowerOff = -2,
  kEventAmpMute = -3,
  kEventAmpUnMute = -4,
  kEventAmpRelaisOn = -5,
  kEventAmpRelaisOff = -6,
  kEventAmpPowerToggle = -7,
  kEventAmpMuteToggle = -8,
  kEventVolumeUp = -9,
  kEventVolumeDown = -10,
  kEventInputSelectPrev = -11,
  kEventInputSelectNext = -12
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
EventManager gMyEventManager;

U8G2_SSD1306_128X64_NONAME_1_4W_HW_SPI u8g2(U8G2_R0, PIN_CS_DISP, PIN_DC_DISP, PIN_RESET_DISP);
const static uint8_t* default_font = u8g2_font_inb19_mr;
const static uint8_t* big_font = u8g2_font_inb27_mn; // 27,33,38

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
  unsigned long tlast_ms = 0;
} gSerialBuffer;

SysState gState = SysState::AMP_OFF;
Rs232State gRs232State = Rs232State::WAIT_FOR_SENTENCE_START;

BiinoInput gBiinoInput(0,(uint8_t)((1<<INPUT_CHANNEL_CNT) - 1),PIN_CS_BIINO_VOL,ADDR_BIINO_INP,EE_BIINO_CHAN);
BiinoVolume gBiinoVolume(0,PIN_CS_BIINO_VOL,ADDR_BIINO_VOL,EE_BIINO_VOL);

BiinoChannel gBiinoChannels[INPUT_CHANNEL_CNT] = { 
  BiinoChannel(0,"CD Player", "CD", EE_BIINO_VOL_START), 
  BiinoChannel(1,"Network Player", "NET", EE_BIINO_VOL_START + 1),
  BiinoChannel(2,"Auxiliary", "AUX", EE_BIINO_VOL_START + 2)
};


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
  pinMode(PIN_ROT_A,INPUT);  
  pinMode(PIN_ROT_B,INPUT);
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

  // Init event manager (8 listeners max)
  gMyEventManager.addListener( (int)RawEvents::kEventButtonPressedShort, button_listener );
  gMyEventManager.addListener( (int)RawEvents::kEventButtonPressedLong, button_listener );
  gMyEventManager.addListener( (int)RawEvents::kEventRotaryTurnTriggered, rotary_listener );
  gMyEventManager.addListener( (int)RawEvents::kEventAmpControl, amp_listener );
  gMyEventManager.addListener( (int)RawEvents::kEventInputControl, biino_listener );
  gMyEventManager.addListener( (int)RawEvents::kEventVolumeControl, biino_listener );
  gMyEventManager.addListener( (int)RawEvents::kEventUpdateDisplay, display_listener );

  // Setup buttons: Assign pin change interrupts
  pciSetup(PIN_BUTTON);
  pciSetup(PIN_ROT_BUTTON);
  
  // Setup IR receiver
  gIrReceiver.enableIRIn();

  // Setup rotary encoder
  encoder_init();
  Timer1.initialize(1000);
  Timer1.attachInterrupt(timer1_isr);

  // Init display
  u8g2.begin();
  
  gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
}

/*
 * Main loop
 */
void loop() {
  unsigned long tcur_ms = 0;
  int delta = 0;

  tcur_ms = millis();

  /*  
   * Process Encoder commands
   */
  delta = encoder1_read4();

  if(delta != 0) {
    gMyEventManager.queueEvent( (int)RawEvents::kEventRotaryTurnTriggered, delta);
  }
  
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
          gIrState.tlast_ms = tcur_ms;
        }
        else {
          if( abs(tcur_ms - gIrState.tlast_ms) >= IR_DNT_RC11_CMD_TIMEOUT_MS && gIrState.cmd_cnt == 1) {
            // We assume to have missed the transmission of a new command, if the first repetition
            //   after receiving the last command took place late.
            gIrState.last_cmd = 0;      
          } 
          else {
            // Repeat command
            gIrState.cmd_cnt++;  
            gIrState.tlast_ms = tcur_ms;
          }
        }

        switch(gIrState.last_cmd) {
          case (unsigned long) ir_codes_dnt_rc11::KEY_VOLUMEUP:
            gMyEventManager.queueEvent( (int)RawEvents::kEventVolumeControl, (int)RawEvents::kEventVolumeUp );
            break;
          case (unsigned long) ir_codes_dnt_rc11::KEY_VOLUMEDOWN:
            gMyEventManager.queueEvent( (int)RawEvents::kEventVolumeControl, (int)RawEvents::kEventVolumeDown );
            break;
          case (unsigned long) ir_codes_dnt_rc11::KEY_POWER:
            if(gIrState.cmd_cnt == 1)
              gMyEventManager.queueEvent( (int)RawEvents::kEventAmpControl, (int)RawEvents::kEventAmpPowerToggle );
            break;
          case (unsigned long) ir_codes_dnt_rc11::KEY_MUTE:
            if(gIrState.cmd_cnt == 1)
              gMyEventManager.queueEvent( (int)RawEvents::kEventAmpControl, (int)RawEvents::kEventAmpMuteToggle );
            break;
          case (unsigned long) ir_codes_dnt_rc11::KEY_DOWN:
            if(gIrState.cmd_cnt == 1)
              gMyEventManager.queueEvent( (int)RawEvents::kEventInputControl, (int)RawEvents::kEventInputSelectPrev );
            break;
          case (unsigned long) ir_codes_dnt_rc11::KEY_UP:
            if(gIrState.cmd_cnt == 1)
              gMyEventManager.queueEvent( (int)RawEvents::kEventInputControl, (int)RawEvents::kEventInputSelectNext );
            break;                        
          case (unsigned long) ir_codes_dnt_rc11::KEY_1:
            if(gIrState.cmd_cnt == 1)
              gMyEventManager.queueEvent( (int)RawEvents::kEventInputControl, 1 );
            break;                        
          case (unsigned long) ir_codes_dnt_rc11::KEY_2:
            if(gIrState.cmd_cnt == 1)
              gMyEventManager.queueEvent( (int)RawEvents::kEventInputControl, 2 );
            break;                        
          case (unsigned long) ir_codes_dnt_rc11::KEY_3:
            if(gIrState.cmd_cnt == 1)
              gMyEventManager.queueEvent( (int)RawEvents::kEventInputControl, 3 );
            break;                        
        } // switch
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
        if(serbyte == 'm' || serbyte == 'p' || serbyte == 'i' || serbyte == 'v' || serbyte == 'r') {
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
        if(gSerialBuffer.last_cmd == 'v' && (serbyte >= '0' && serbyte <= '9')) {
          if(gSerialBuffer.ptr < 2) {
            gSerialBuffer.last_param[gSerialBuffer.ptr] = serbyte;
            gSerialBuffer.ptr++;
          } 
          
          if(gSerialBuffer.ptr >= 2)
            gRs232State = Rs232State::WAIT_FOR_SENTENCE_DELIMITER;
        }
        else
        if(gSerialBuffer.last_cmd == 'r' && (serbyte == '0' || serbyte == '1')) {
          gSerialBuffer.last_param[0] = serbyte;
          gRs232State = Rs232State::WAIT_FOR_SENTENCE_DELIMITER;
        }
        else {
          gRs232State = Rs232State::WAIT_FOR_SENTENCE_START;
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
              if(gSerialBuffer.last_param[0] == '0')
                gMyEventManager.queueEvent( (int)RawEvents::kEventAmpControl, (int)RawEvents::kEventAmpPowerOff );
              else
              if(gSerialBuffer.last_param[0] == '1')
                gMyEventManager.queueEvent( (int)RawEvents::kEventAmpControl, (int)RawEvents::kEventAmpPowerOn );
              break;
            case 'm':
              if(gSerialBuffer.last_param[0] == '0')
                gMyEventManager.queueEvent( (int)RawEvents::kEventAmpControl, (int)RawEvents::kEventAmpUnMute );
              else
              if(gSerialBuffer.last_param[0] == '1')
                gMyEventManager.queueEvent( (int)RawEvents::kEventAmpControl, (int)RawEvents::kEventAmpMute );
              break;
            case 'i':
              gMyEventManager.queueEvent( (int)RawEvents::kEventInputControl, (int)(gSerialBuffer.last_param[0] - '0') );
              break;
            case 'v':
              if(gSerialBuffer.last_param[0] == '+')
                gMyEventManager.queueEvent( (int)RawEvents::kEventVolumeControl, (int)RawEvents::kEventVolumeUp );                
              else
              if(gSerialBuffer.last_param[0] == '-')
                gMyEventManager.queueEvent( (int)RawEvents::kEventVolumeControl, (int)RawEvents::kEventVolumeDown );                
              else
                gMyEventManager.queueEvent( (int)RawEvents::kEventVolumeControl, atoi(gSerialBuffer.last_param) );                
              break;
          }
        } 

        gSerialBuffer.last_cmd = 0;
        gSerialBuffer.last_param[0] = 0;
        gSerialBuffer.last_param[1] = 0;
        gSerialBuffer.last_param[2] = 0;
        gSerialBuffer.ptr = 0;
        gRs232State = Rs232State::WAIT_FOR_SENTENCE_START; 
    }
  } // if(Serial.available() > 0)

  
  /*
   * Process event queue
   */
  gMyEventManager.processEvent();
  
}

/*
 * Sys state transitions
 */
//SysState set_state(SysState newState)
//{
//  if(newState == gState)
//    return gState;
//
//  switch(newState) {
//    case SysState::INITIAL:
//      // TODO
//      break;
//    case SysState::AMP_MUTE:
//      amp_mute();
//      gState = newState;
//      gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
//      break;
//    case SysState::AMP_OFF:
//      amp_off();
//      break;
//    case SysState::RUNNING_MODE_VOL:
//      amp_mute();
//      gState = SysState::AMP_MUTE;
//      gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
//      break;
//    case SysState::RUNNING_MODE_INP:
//      amp_mute();
//      gState = SysState::AMP_MUTE;
//      gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
//      break;
//    default:
//      // Do nothing.
//      break;
//  }
//}

/*
 * Plot Functions
 */
//const char decibels[64][6] PROGMEM = {"-75.6\0", "-74.4\0", "-73.2\0", "-72.0\0", "-70.8\0", "-69.6\0", "-68.4\0", "-67.2\0", "-66.0\0", "-64.8\0", "-63.6\0", "-62.4\0", "-61.2\0", "-60.0\0", "-58.8\0", "-57.6\0", "-56.4\0", "-55.2\0", "-54.0\0", "-52.8\0", "-51.6\0", "-50.4\0", "-49.2\0", "-48.0\0", "-46.8\0", "-45.6\0", "-44.4\0", "-43.2\0", "-42.0\0", "-40.8\0", "-39.6\0", "-38.4\0", "-37.2\0", "-36.0\0", "-34.8\0", "-33.6\0", "-32.4\0", "-31.2\0", "-30.0\0", "-28.8\0", "-27.6\0", "-26.4\0", "-25.2\0", "-24.0\0", "-22.8\0", "-21.6\0", "-20.4\0", "-19.2\0", "-18.0\0", "-16.8\0", "-15.6\0", "-14.4\0", "-13.2\0", "-12.0\0", "-10.8\0", " -9.6\0", " -8.4\0", " -7.2\0", " -6.0\0", " -4.8\0", " -3.6\0", " -2.4\0", " -1.2\0", " -0.0\0"};

void plot_main(int8_t highlight,int8_t volume, int8_t input)
{
  //static const char* mtext[INPUT_CHANNEL_CNT] = {"CD\0","NET\0","AUX\0"};
  
  if(gState == SysState::AMP_OFF || gState == SysState::AMP_MUTE)
  {
    u8g2.firstPage();
    do {
      u8g2.drawXBMP(0,0,64,64,duck_bw_64x64_bits);
      //u8g2.drawXBMP(0,0,67,64,squirrel_bw_67x64_bits);
      u8g2.setFont(default_font);
      u8g2.setFontPosTop();
      u8g2.setCursor(55,40);
      //u8g2.drawTriangle(0,0,8,10,0,20);
      if(gState == SysState::AMP_OFF)
        u8g2.print("Off.");
      else
        u8g2.print("Mute.");
    } while ( u8g2.nextPage() );  
  }
  else {
    u8g2.firstPage();
    do {
      u8g2.setFont(default_font);
      u8g2.setFontPosTop();
      u8g2.setCursor(12,0);
      if(input >= 1 && input <= INPUT_CHANNEL_CNT) {
        if(highlight == 1)
          u8g2.drawTriangle(0,0,8,10,0,20);
        u8g2.print(gBiinoChannels[input-1].title_short);
      }
  
      u8g2.drawStr(127-40,0,"db"); // 63-37
      
      if(highlight == 2)
        u8g2.drawTriangle(0,63-37-1,8,63-37+13,0,63-37+26+1);
  
      u8g2.setFont(big_font);
      u8g2.setCursor(10,63-37);
//      u8g2.print(volume,DEC);
          
      float db = -1*(gBiinoVolume.max_volume-volume)*1.2;
      char val[7];
      dtostrf(db, 5, 1,val);
      
      u8g2.print(val);
  
    } while ( u8g2.nextPage() );  
  }
}
 
/*
 * Rotary encoder 1 functionality
 */
volatile int8_t enc1_delta; // -128 ... 127
static int8_t enc1_last;

// Init encoder functionality
void encoder_init( void )
{
  int8_t cval;

  cval = 0;
  if( digitalRead(PIN_ROT_A) ) cval = 3;
  if( digitalRead(PIN_ROT_B) ) cval ^= 1;     // convert gray to binary
  enc1_last = cval;                           // power on state
  enc1_delta = 0;
}

// Read 4 step encoder status, call this at least at 8 Hz in loop()
int8_t encoder1_read4( void )
{
  int8_t val;

  cli();
  val = enc1_delta;
  enc1_delta = val & 3;
  sei();
  return val >> 2;
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

void rpi_off()
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
  int8_t curvol;
  
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
      curvol = gBiinoVolume.getVolume() + (int8_t) param;
      if(curvol < BiinoVolume::min_volume)
        gBiinoVolume.setVolume(BiinoVolume::min_volume);
      else
      if(curvol > BiinoVolume::max_volume)
        gBiinoVolume.setVolume(BiinoVolume::max_volume);
      else
        gBiinoVolume.setVolume(curvol);
      
      gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
      break;
    case SysState::RUNNING_MODE_INP:
      if(param > 0) {
        gBiinoInput.selectNext();
      }
      else if(param < 0) {
        gBiinoInput.selectPrevious();
      }
        
      gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
      break;
    default:
      // Do nothing.
      break;
  }
}

void amp_listener(int event, int param) {
  if(event != (int)RawEvents::kEventAmpControl)
    return;

  if(param == (int)RawEvents::kEventAmpPowerToggle) {
      if(gState == SysState::AMP_OFF)
        param = (int)RawEvents::kEventAmpPowerOn;
      else
        param = (int)RawEvents::kEventAmpPowerOff;
  }
  else
  if(param == (int)RawEvents::kEventAmpMuteToggle) {
      if((gState == SysState::RUNNING_MODE_VOL || gState == SysState::RUNNING_MODE_INP))
        param = (int)RawEvents::kEventAmpMute;
      else
        param = (int)RawEvents::kEventAmpUnMute;
  }
  
  switch(param) {
    case (int)RawEvents::kEventAmpPowerOff:
      if(gState != SysState::AMP_OFF) {
        amp_off();
        gState = SysState::AMP_OFF;
        gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
      }
      break;
    case (int)RawEvents::kEventAmpPowerOn:
      if(gState == SysState::AMP_OFF) {
        amp_on();
        amp_unmute();
        gState = SysState::RUNNING_MODE_VOL;
        gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
      }
      break;
    case (int)RawEvents::kEventAmpMute:
      if((gState == SysState::RUNNING_MODE_VOL || gState == SysState::RUNNING_MODE_INP)) {
        amp_mute();
        gState = SysState::AMP_MUTE;
        gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
      }
      break;
    case (int)RawEvents::kEventAmpUnMute:
      if(gState == SysState::AMP_MUTE) {
        amp_unmute();
        gState = SysState::RUNNING_MODE_VOL;
        gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
      }
      break;
    case (int)RawEvents::kEventAmpRelaisOn:
        rpi_on();
      break;
    case (int)RawEvents::kEventAmpRelaisOff:
        rpi_off();
      break;
  }
}

void biino_listener( int event, int param ) {
  if(event == (int)RawEvents::kEventVolumeControl) {
    switch(param) {
      case (int)RawEvents::kEventVolumeUp:
        if(gState == SysState::AMP_MUTE) {
          amp_unmute();
          gState = SysState::RUNNING_MODE_VOL;
          gBiinoVolume.incVolume();
          gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
        }
        else
        if((gState == SysState::RUNNING_MODE_VOL || gState == SysState::RUNNING_MODE_INP)) {
          gState = SysState::RUNNING_MODE_VOL;
          gBiinoVolume.incVolume();
          gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
        }
        break;
      case (int)RawEvents::kEventVolumeDown:
        if((gState == SysState::RUNNING_MODE_VOL || gState == SysState::RUNNING_MODE_INP)) {
          gState = SysState::RUNNING_MODE_VOL;
          gBiinoVolume.decVolume();
          gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
        }
        break;
      default:
        if( (param >= BiinoVolume::min_volume && param <=  BiinoVolume::max_volume) &&
            (gState == SysState::RUNNING_MODE_VOL || gState == SysState::RUNNING_MODE_INP)) {
          gState = SysState::RUNNING_MODE_VOL;
          gBiinoVolume.setVolume((uint8_t)param);
          gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
        }
        break;
    } // switch
  }
  else
  if(event == (int)RawEvents::kEventInputControl) {
    switch(param) {
      case (int)RawEvents::kEventInputSelectNext:
        if((gState == SysState::RUNNING_MODE_VOL || gState == SysState::RUNNING_MODE_INP)) {
          gState = SysState::RUNNING_MODE_INP;
          gBiinoInput.selectNext();
          gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
        }    
        break;
      case (int)RawEvents::kEventInputSelectPrev:
        if((gState == SysState::RUNNING_MODE_VOL || gState == SysState::RUNNING_MODE_INP)) {
          gState = SysState::RUNNING_MODE_INP;
          gBiinoInput.selectPrevious();
          gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
        }    
        break;
      default:
        if(param >= 1 && param <= 3 && (gState == SysState::RUNNING_MODE_VOL || gState == SysState::RUNNING_MODE_INP)) {
          gState = SysState::RUNNING_MODE_INP;
          gBiinoInput.select((uint8_t)(1 << (param-1)));
          gMyEventManager.queueEvent( (int)RawEvents::kEventUpdateDisplay, 0 );
        }    
        break;      
    } // switch   
  }
}

void display_listener( int event, int param ) {
  switch(gState) {
    case SysState::INITIAL:
      // TODO
      break;
    case SysState::AMP_MUTE:
      plot_main(0,0,0);
      break;
    case SysState::AMP_OFF:
      plot_main(0,0,0);
      break;
    case SysState::RUNNING_MODE_VOL:
      plot_main(2,gBiinoVolume.getVolume(),gBiinoInput.getCurrentChannelNo());
      break;
    case SysState::RUNNING_MODE_INP:
      plot_main(1,gBiinoVolume.getVolume(),gBiinoInput.getCurrentChannelNo());
      break;
    default:
      // Do nothing.
      break;
  }
}


/*
 * Interrupt routines
 */

/*
 * Timer1 interrupt routine
 */
void timer1_isr(void)
{
  int8_t cval = 0, diff = 0;

  cval = 0;
  if( digitalRead(PIN_ROT_A) ) cval = 3;
  if( digitalRead(PIN_ROT_B) ) cval ^= 1 ;  // convert gray to binary
  
  diff = enc1_last - cval;                  // difference last - new
  if( diff & 1 ) {                          // bit 0 = value (1)
    enc1_last = cval;                       // store new as next last
    enc1_delta += (diff & 2) - 1;           // bit 1 = direction (+/-)
  }  
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

