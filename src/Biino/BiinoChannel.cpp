#include "BiinoChannel.h"

BiinoChannel::BiinoChannel(uint8_t id, String title, String title_short, uint8_t ee_addr_volume, BiinoInput* biino_input)
{
  this->id = id;
  this->title = title;
  this->title_short = title_short;
  this->ee_addr_volume = ee_addr_volume;
  this->biino_input = biino_input;
}

BiinoChannel::BiinoChannel(uint8_t id, String title, String title_short, uint8_t ee_addr_volume) : BiinoChannel(id, title, title_short, ee_addr_volume, NULL)
{
  
}

