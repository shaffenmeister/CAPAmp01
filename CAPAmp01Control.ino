/*
 * Stock Arduino libraries
 */
#include <EEPROM.h>
#include <SPI.h>

/*
 * Custom libraries
 */
#include <mcp23s08.h>
#include <IRLibDecodeBase.h>
#include <IRLib_P03_RC5.h>
#include <IRLibCombo.h>
#include <IRLibRecvPCI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "BiinoMK2.h"

/*
 * Definitions of constants
 */
const int PIN_CS_BIINO_VOL = 11;
const int PIN_CS_BIINO_INP = 11;
const int PIN_IR_RCV = 2;

const int PIN_BUTTON = 14;
const int PIN_AMP_MUTE = 15;

const int INPUT_CHANNEL_CNT = 3;
const int INPUT_CHANNEL_INV = -1;
const int EE_BIINO_CHAN = 100;
const int EE_BIINO_VOL_START = 101;
const int EE_BIINO_VOL_END = 102;
const int EE_BIINO_VOL = 103;

const int ADDR_BIINO_VOL = 0x20;
const int ADDR_BIINO_INP = 0x21;

/*
 * Global variables
 */
IRdecode g_ir_decoder;
IRrecvPCI g_ir_receiver(PIN_IR_RCV);

BiinoInput g_biino_input(0,(uint8_t)((1<<INPUT_CHANNEL_CNT) - 1),PIN_CS_BIINO_VOL,ADDR_BIINO_INP,EE_BIINO_CHAN);
//BiinoInput g_biino_volume(0,PIN_CS_BIINO_VOL,ADDR_BIINO_VOL);

void setup() {
  // Setup serial port
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }  

  Serial.println("Setting up...");  
  Serial.println("Setting up Biino Input...");  
  g_biino_input.Setup();
  Serial.print("\tChannel mask: ");
  Serial.println(g_biino_input.channel_mask,BIN);
  Serial.print("\tExpected mask: ");
  Serial.println((uint8_t)((1<<INPUT_CHANNEL_CNT) - 1),BIN);
  Serial.print("\tCurrent channel: ");
  Serial.println(g_biino_input.GetCurrentChannel(),BIN);
  Serial.println("Setting up Biino Input...Done.");  
  
  // Setup amplifier mute pin and mute amp
  setup_amp_mute();
  mute_amp();

  // Setup input button
//  pinMode(PIN_BUTTON,INPUT);


//  g_biino_volume.begin();
//  g_biino_input.begin();
//  g_biino_volume.gpioPinMode(OUTPUT);
//  g_biino_input.gpioPinMode(OUTPUT);
 
  
  // Setup IR receiver
  digitalPinToInterrupt(PIN_IR_RCV);
  g_ir_receiver.enableIRIn();
  
}

volatile int g_rs_cmd = 0;
volatile uint8_t g_cur_channel = 0xFF;

void loop() {
  if(Serial.available() > 0)
    {
      g_rs_cmd = Serial.read();
      g_cur_channel = g_biino_input.GetCurrentChannel();
      
      switch(g_rs_cmd) {
        case '+':
          if(g_biino_input.SelectNext() == EXIT_FAILURE) 
            g_biino_input.SelectFirst();
          Serial.print("Current channel (+): ");
          break;
        case '-':
          if(g_biino_input.SelectPrevious() == EXIT_FAILURE) 
            g_biino_input.SelectLast();
          Serial.print("Current channel (-): ");
          break;
      } 
      
      Serial.print(g_biino_input.GetCurrentChannel(),BIN);       
      Serial.print(" / ");
      Serial.println(g_biino_input.GetCurrentChannelNo(),DEC);       
    }
}

/*
 *  
 */

int set_volume(int vol)
{
}

int get_volume()
{
}

/*
 * Inut channel management
 */
void setup_channels()
{
//  g_input_channels[0] = new input_chan(1,"CD Player","CD",EE_BIINO_VOL_1);
//  g_input_channels[1] = new input_chan(2,"Network","NET",EE_BIINO_VOL_2);
//  g_input_channels[2] = new input_chan(3,"Auxiliary","AUX",EE_BIINO_VOL_3);
}

int set_channel(int no)
{
  
}

/*
 * AMP Mute and Unmute Functionality
 */
void setup_amp_mute()
{
  pinMode(PIN_AMP_MUTE,OUTPUT);
}

int mute_amp()
{
  digitalWrite(PIN_AMP_MUTE,0);
}

int unmute_amp()
{
  digitalWrite(PIN_AMP_MUTE,1);
}

