#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "BiinoInput.h"

BiinoInput::BiinoInput(uint8_t id, uint8_t channel_mask, uint8_t pin_cs, uint8_t spi_addr, uint8_t ee_addr_cur_channel)
{
  this->biino_id = id;
  this->biino_input = new mcp23s08(pin_cs,spi_addr,MAXSPISPEED);
  this->channel_mask = channel_mask;
  this->ee_addr_cur_channel = ee_addr_cur_channel;
}

void BiinoInput::setup(void)
{
  this->biino_input->begin();
  this->biino_input->gpioPinMode(OUTPUT);
  this->deselectAll();
  this->select(EEPROM[this->ee_addr_cur_channel]);
}

bool BiinoInput::isChannelValid(uint8_t channel)
{
  if((this->channel_mask & channel) != 0)
    return true;

  return false;
}

void BiinoInput::deselectAll(void)
{
  // Deselect all channels immediately, also modifies output pins not used by biino!
  // Inverse logic! 1 = relais off, 0 = relais on, 0xff = all off, 0x00 = all on
  // Inverse logic only used for direct output to / direct input from mcp23s08!
  this->biino_input->gpioPort(0xff);
}

int BiinoInput::select(uint8_t channel)
{
  if(this->isChannelValid(channel))
    {
      // Inverse logic! 1 = relais off, 0 = relais on, 0xff = all off, 0x00 = all on
      // Inverse logic only used for direct output to / direct input from mcp23s08!
      if(this->biino_input->readGpioPort() != 0xff)
        {
          this->deselectAll();
        }

      if(this->channel_switch_delay_ms > 0) {
        delay(this->channel_switch_delay_ms);
      }

      // Enable corresponding relais on channel.
      // Inverse logic! 1 = relais off, 0 = relais on, 0xff = all off, 0x00 = all on
      // Inverse logic only used for direct output to / direct input from mcp23s08!
      this->biino_input->gpioPort(~channel);

      EEPROM.update(this->ee_addr_cur_channel,channel);
      return EXIT_SUCCESS;
    }
  return EXIT_FAILURE;
}

int BiinoInput::select(bool next)
{
  uint8_t channel;

  channel = this->getCurrentChannel();
  
  if(this->isChannelValid(channel))
    {
      if(next == true)
      // Select next channel.
        channel <<= 1;
      else
      // Select previous channel.
        channel >>= 1;
      
       return this->select(channel);
    }

  return EXIT_FAILURE;
}

int BiinoInput::selectNext(void)
{
  return this->select(true);
}

int BiinoInput::selectPrevious(void)
{
  return this->select(false);
}

int BiinoInput::selectFirst(void)
{
  uint8_t channel;

  channel = this->channel_mask;
  channel = this->channel_mask & ~(this->channel_mask - 1);

  return this->select(channel);  
}

int BiinoInput::selectLast(void)
{
  uint8_t channel;
  int8_t cnt;

  // start with bit 7 (msb)
  cnt = 7;

  while(cnt >= 0)
    {
      channel = 1 << cnt;

      if( (channel & this->channel_mask) != 0)
        return this->select(channel);

      cnt--;
    }

   return EXIT_FAILURE;    
}

uint8_t BiinoInput::getCurrentChannel(void)
{
  uint8_t channel;

  channel = EEPROM[this->ee_addr_cur_channel];

  if(this->isChannelValid(channel))
    return channel;
  else
    return 0xFF;
}

int BiinoInput::getCurrentChannelNo(void)
{
  uint8_t channel;

  channel = this->getCurrentChannel();
  
  if(this->isChannelValid(channel))
    return (channel >> 1) + 1;
  else
    return EXIT_FAILURE;
}
