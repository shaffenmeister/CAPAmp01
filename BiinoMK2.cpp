#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include <Arduino.h>
#include <EEPROM.h>

#include "BiinoMK2.h"

BiinoInput::BiinoInput(uint8_t id, uint8_t channel_mask, const uint8_t pin_cs, const uint8_t spi_addr, uint8_t ee_addr_cur_channel)
{
  this->biino_id = id;
  this->biino_input = new mcp23s08(pin_cs,spi_addr,MAXSPISPEED);
  this->channel_mask = channel_mask;
  this->ee_addr_cur_channel = ee_addr_cur_channel;
}

void BiinoInput::Setup(void)
{
  this->biino_input->begin();
  this->biino_input->gpioPinMode(OUTPUT);
  this->DeselectAll();
  delay(250);
  this->Select(EEPROM[this->ee_addr_cur_channel]);
}

bool BiinoInput::IsChannelValid(uint8_t channel)
{
  if((this->channel_mask & channel) != 0)
    return true;
  return false;
}

void BiinoInput::DeselectAll(void)
{
  // deselect all channels, also modifies output pins not used by biino!
  this->biino_input->gpioPort(0);
}

int BiinoInput::Select(uint8_t channel)
{
  if(this->IsChannelValid(channel))
    {
      this->DeselectAll();
      delay(200);
      this->biino_input->gpioPort(channel);
      EEPROM.update(this->ee_addr_cur_channel,channel);
      return EXIT_SUCCESS;
    }     
  return EXIT_FAILURE;
}

int BiinoInput::Select(bool next)
{
  uint8_t channel;

  channel = this->GetCurrentChannel();
  
  if(this->IsChannelValid(channel))
    {
      if(next == true)
      // Select next channel.
        channel <<= 1;
      else
      // Select previous channel.
        channel >>= 1;
      
       return this->Select(channel);        
    }

  return EXIT_FAILURE;
}

int BiinoInput::SelectNext(void)
{
  return this->Select(true);  
}

int BiinoInput::SelectPrevious(void)
{
  return this->Select(false);  
}

int BiinoInput::SelectFirst(void)
{
  uint8_t channel;

  channel = this->channel_mask;
  channel = this->channel_mask & ~(this->channel_mask - 1);
      
  return this->Select(channel);  
}

int BiinoInput::SelectLast(void)
{
  uint8_t channel;
  int8_t cnt;

  // start with bit 7 (msb)
  cnt = 7;
  
  while(cnt >= 0)
    {
      channel = 1 << cnt;

      if( (channel & this->channel_mask) != 0)
        return this->Select(channel);    

      cnt--;
    }
     
   return EXIT_FAILURE;    
}

uint8_t BiinoInput::GetCurrentChannel(void)
{
  uint8_t channel;

  channel = EEPROM[this->ee_addr_cur_channel];
  
  if(this->IsChannelValid(channel))
    return channel;
  else
    return 0xFF;
}

int BiinoInput::GetCurrentChannelNo(void)
{
  uint8_t channel;

  channel = this->GetCurrentChannel();
  
  if(this->IsChannelValid(channel))
    return (channel >> 1) + 1;
  else
    return EXIT_FAILURE;
}

BiinoVolume::BiinoVolume(uint8_t id, const uint8_t pin_cs, const uint8_t spi_addr, uint8_t ee_addr)
{
  this->biino_id = id;
  this->biino_volume = new mcp23s08(pin_cs,spi_addr,MAXSPISPEED);
}

