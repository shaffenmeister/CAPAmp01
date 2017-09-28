#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "BiinoVolume.h"

BiinoVolume::BiinoVolume(uint8_t id, const uint8_t pin_cs, const uint8_t spi_addr, const uint8_t ee_addr_cur_volume)
{
  this->biino_id = id;
  this->biino_volume = new mcp23s08(pin_cs,spi_addr,MAXSPISPEED);
  this->ee_addr_cur_volume = ee_addr_cur_volume;
}

void BiinoVolume::setup(void)
{
  this->biino_volume->begin();
  this->biino_volume->gpioPinMode(OUTPUT);
  if(this->setVolume(EEPROM[this->ee_addr_cur_volume]) == EXIT_FAILURE)
    this->setVolume(0);
}

bool BiinoVolume::isValid(uint8_t volume)
{
  if(volume >= 0 && volume <= 63)
    return true;

  return false;
}

int BiinoVolume::setVolume(uint8_t volume)
{
    if(this->isValid(volume)) {
      if(this->volume_switch_delay_ms > 0) {
        delay(this->volume_switch_delay_ms);
      }

      // Enable corresponding relais on channel.
      // Inverse logic! 1 = relais off, 0 = relais on, 0xff = all off, 0x00 = all on
      // Inverse logic only used for direct output to / direct input from mcp23s08!
      this->biino_volume->gpioPort(~volume);

      EEPROM.update(this->ee_addr_cur_volume,volume);
      return EXIT_SUCCESS;
    }

  return EXIT_FAILURE;
}

int BiinoVolume::getVolume(void)
{
  return EEPROM[this->ee_addr_cur_volume];
}

int BiinoVolume::incVolume(void)
{
  return this->setVolume(this->getVolume()+1);
}

int BiinoVolume::decVolume(void)
{
  return this->setVolume(this->getVolume()-1);
}
