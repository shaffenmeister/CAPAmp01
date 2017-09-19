
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

const int PIN_AMP_MUTE = 9; // 5, D5 (PD5)
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

enum class user_event {
  INVALID = -1,
  AMP_POWER = 1,
  AMP_MUTE = 2,
  CHOOSE_INPUT_1 = 3,
  CHOOSE_INPUT_2 = 4,
  CHOOSE_INPUT_3 = 5,
  VOL_UP = 6,
  VOL_DOWN = 7
};

enum class sys_state {
  INVALID = -1,
  INITIAL = 1,
  STANDBY = 2,
  AMP_OFF = 3,
  RUNNING = 4
};

/*
 * Global variables
 */
IRrecv g_ir_receiver(PIN_IR_RCV);
decode_results results;
U8G2_SSD1306_128X64_NONAME_1_4W_HW_SPI u8g2(U8G2_R0, PIN_CS_DISP, PIN_DC_DISP, PIN_RESET_DISP);
EventManager gMyEventManager;

static struct {
  bool last_state = LOW;
  bool cur_state = LOW;
  bool active = false;
  unsigned long tstart_ms;
  unsigned long tcur_ms;
  } button_state[2];

BiinoInput g_biino_input(0,(uint8_t)((1<<INPUT_CHANNEL_CNT) - 1),PIN_CS_BIINO_VOL,ADDR_BIINO_INP,EE_BIINO_CHAN);
BiinoVolume g_biino_volume(0,PIN_CS_BIINO_VOL,ADDR_BIINO_VOL,EE_BIINO_VOL);

BiinoChannel channels[INPUT_CHANNEL_CNT] = { 
  BiinoChannel(0,"Network Player", "NET", EE_BIINO_VOL_START, &g_biino_input), 
  BiinoChannel(1,"Auxiliary 1", "AUX 1", EE_BIINO_VOL_START + 1, &g_biino_input),
  BiinoChannel(2,"Auxiliary 2", "AUX 2", EE_BIINO_VOL_START + 2, &g_biino_input)
};

/*
BiinoChannel channels[INPUT_CHANNEL_CNT] = { 
  BiinoChannel(0,"Network Player", "NET", EE_BIINO_VOL_START), 
  BiinoChannel(1,"Auxiliary 1", "AUX 1", EE_BIINO_VOL_START + 1),
  BiinoChannel(2,"Auxiliary 2", "AUX 2", EE_BIINO_VOL_START + 2)
};
*/

void setup() {
  // Setup serial port
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }  

//  Serial.println("Setting up...");  
//  Serial.println("Setting up Biino Input...");  
  g_biino_input.Setup();
//  Serial.print("\tChannel mask: ");
//  Serial.println(g_biino_input.channel_mask,BIN);
//  Serial.print("\tExpected mask: ");
//  Serial.println((uint8_t)((1<<INPUT_CHANNEL_CNT) - 1),BIN);
//  Serial.print("\tCurrent channel: ");
//  Serial.println(g_biino_input.GetCurrentChannel(),BIN);
//  Serial.println("Setting up Biino Input...Done.");  
//
//  Serial.println("Setting up Biino Volume...");
  g_biino_volume.Setup();
//  Serial.println("Setting up Biino Volume...Done.");    

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
  
  // Setup Rotary encoder
  attachInterrupt(digitalPinToInterrupt(PIN_ROT_STEP),read_rotary,FALLING);
  
  // Assign pin change interrupts
  pciSetup(PIN_BUTTON);
  pciSetup(PIN_ROT_BUTTON);
  
  // Setup IR receiver
  g_ir_receiver.enableIRIn();
  
  // Mute amplifier  
  mute_amp();
  
  // Init display
  u8g2.begin();
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_inb19_mf); //u8g2_font_ncenB14_tr);
    u8g2.drawStr(0,24,"Welcome!");
  } while ( u8g2.nextPage() );
  delay(1000);
}

volatile static int g_rs_cmd = 0;
volatile static uint8_t g_cur_channel = 0xFF;
volatile static int rot_count = 0;
volatile static uint16_t pot_count = 0;
static sys_state state = sys_state::INITIAL;

/*
 * Main loop
 */
void loop() {
  int rot = rot_count;
  uint16_t pot = pot_count;
  bool update_screen = false;
  unsigned long tcur_ms = 0;
  unsigned long ir_last_cmd = 0;
  unsigned long ir_tlast_ms = 0;

  tcur_ms = millis();

  /*  
   * Process IR commands
   */
  if (g_ir_receiver.decode(&results)) {
    if(results.decode_type == NEC) {
      if(results.bits == 32) {
        // New command
        ir_tlast_ms = tcur_ms;
        ir_last_cmd = results.value;
      }
      else if(results.bits == 0) {
        // Repetition of last command
        if(ir_last_cmd != 0) {
          ir_tlast_ms = tcur_ms;
        }
      }
    }

    g_ir_receiver.resume();
  }

  /*  
   * Process button states
   */
  noInterrupts();
    
    // PIN_ROT_BUTTON: Rotary encoder's push button
    if(button_state[0].active == true) {
      if((tcur_ms-button_state[0].tstart_ms) >= BUTTON_TIMEOUT_MS) {
        // Perform action, if user presses button longer than BUTTON_TIMEOUT_MS
        rot_count = 0;
        button_state[0].active = false;
      }
    }
  
    // PIN_BUTTON: Input button
    if(button_state[1].active == true) {
      if((tcur_ms-button_state[1].tstart_ms) >= BUTTON_TIMEOUT_MS) {
        // Perform action, if user presses button longer than BUTTON_TIMEOUT_MS
        // Short push: MUTE ON/OFF
        // Long push: AMP ON/OFF
        rot_count = 0;
        button_state[1].active = false;
        
      }
    }

  interrupts();  

  switch(state)
  {
    case sys_state::INITIAL:
      state = sys_state::RUNNING;
      break;
    case sys_state::STANDBY:
      state = sys_state::RUNNING;    
      break;
    case sys_state::AMP_OFF:
      state = sys_state::RUNNING;
      break;
    case sys_state::RUNNING:
      break;
  }
  
  gMyEventManager.processEvent();
  
  if( rot != rot_count)  {
    rot = rot_count;
    update_screen = true;
  }

  pot_count = analogRead(PIN_POT);
  if( abs(pot - pot_count) >= 4) {
    pot = pot_count;
    update_screen = true;
  }      

/*
 * Update display
 */
  if(update_screen == true) {
    u8g2.firstPage();
    u8g2.setFont(u8g2_font_inb19_mf); //u8g2_font_ncenB14_tr);
    do {
      u8g2.setCursor(0, 31);
      u8g2.print(rot);
      u8g2.setCursor(0, 63);
      u8g2.print(pot);
    } while ( u8g2.nextPage() );

    update_screen = false;    
  }
     
//  if(Serial.available() > 0)
//    {
//      g_rs_cmd = Serial.read();
//      g_cur_channel = g_biino_input.GetCurrentChannel();
//      
//      switch(g_rs_cmd) {
//        case '+':
//          if(g_biino_input.SelectNext() == EXIT_FAILURE) 
//            g_biino_input.SelectFirst();
//          Serial.print("Current channel (+): ");
//          Serial.print(g_biino_input.GetCurrentChannel(),BIN);       
//          Serial.print(" / ");
//          Serial.println(g_biino_input.GetCurrentChannelNo(),DEC);       
//          break;
//        case '-':
//          if(g_biino_input.SelectPrevious() == EXIT_FAILURE) 
//            g_biino_input.SelectLast();
//          Serial.print("Current channel (-): ");
//          Serial.print(g_biino_input.GetCurrentChannel(),BIN);       
//          Serial.print(" / ");
//          Serial.println(g_biino_input.GetCurrentChannelNo(),DEC);       
//          break;
//        case 'M':
//          if(digitalRead(PIN_AMP_MUTE) == 1)
//          {
//            mute_amp();
//          }
//          else
//          {
//            unmute_amp();  
//          }
//          
//          Serial.print("AMP status: ");
//          Serial.println(digitalRead(PIN_AMP_MUTE));
//          break;
//        case 'v':
//          n = Serial.readBytesUntil('#', buffer, 3);
//          buffer[n] = 0;
//           
//          Serial.print("Current volume: ");
//          Serial.println(g_biino_volume.GetVolume(),DEC); 
//          
// /*         if(g_biino_volume.IncVolume() == EXIT_FAILURE) {
//            Serial.println("Error setting volume");
//            g_biino_volume.SetVolume(0);
//          }
//          */
//          
//          newvol = atoi(buffer);  
//          g_biino_volume.SetVolume(newvol);
//
//          Serial.print("New volume: ");
//          Serial.println(g_biino_volume.GetVolume(),DEC); 
//          
//          break;  
//      } 
//     
//    }
}

/*
 * AMP Mute and Unmute Functionality
 */
void mute_amp()
{
  digitalWrite(PIN_AMP_MUTE,0);
}

void unmute_amp()
{
  digitalWrite(PIN_AMP_MUTE,1);
}

void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

/*
 * Interrupt routines
 */
void read_rotary(void)
{
  if (digitalRead(PIN_ROT_DIR)) rot_count--;
    else rot_count++;
}

/*
 * Port B PCINT (D8 to D13, PCINT0 to PCINT7, PB0 to PB7)
 */
ISR (PCINT0_vect)
{
  button_state[0].cur_state = digitalRead(PIN_ROT_BUTTON);
  
  if (button_state[0].cur_state == HIGH && button_state[0].last_state == LOW) {
    button_state[0].tstart_ms = millis(); 
    button_state[0].tcur_ms = button_state[0].tstart_ms;
    button_state[0].active = true;
  }
  else
  if (button_state[0].cur_state == LOW && button_state[0].last_state == HIGH) {
    button_state[0].tcur_ms = millis();
    button_state[0].active = false;
    
    if((button_state[0].tcur_ms - button_state[0].tstart_ms) < BUTTON_TIMEOUT_MS)
      rot_count += 10;
//    else
//      rot_count = 0;
  }

  button_state[0].last_state = button_state[0].cur_state;
} 

/*
 * Port C PCINT (A0 to A5, PCINT8 to PCINT14, PC0 to PC6)
 */
ISR (PCINT1_vect)
{
  button_state[1].cur_state = digitalRead(PIN_BUTTON);
  
  if (button_state[1].cur_state == HIGH && button_state[1].last_state == LOW) {
    button_state[1].tstart_ms = millis(); 
    button_state[1].tcur_ms = button_state[1].tstart_ms;
    button_state[1].active = true;
  }
  else
  if (button_state[1].cur_state == LOW && button_state[1].last_state == HIGH) {
    button_state[1].tcur_ms = millis();
    button_state[1].active = false;
    
    if((button_state[1].tcur_ms - button_state[1].tstart_ms) < BUTTON_TIMEOUT_MS)
      rot_count += 10;
    else
      rot_count = 0;
  }

  button_state[1].last_state = button_state[1].cur_state;
}  

/*
   uint8_t state = digitalRead(PIN_BUTTON);
  static uint8_t last_state = LOW;
  static unsigned long tstart_ms = 0;
  unsigned long tend_ms = 0;

  if (state == HIGH && last_state == LOW) {
    tstart_ms = millis(); 
    tend_ms = tstart_ms;
  }
  else if (state == LOW && last_state == HIGH) {
    tend_ms = millis();

    if((tend_ms - tstart_ms) < 1000)
      rot_count += 10;
    else
      rot_count = 0;
  }

  last_state = state;
 */

/*
 * Port D PCINT (D0 to D7, PCINT16 to PCINT23, PD0 to PD7)
 */
ISR (PCINT2_vect)
{
}  

