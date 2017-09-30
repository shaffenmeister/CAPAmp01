#ifndef _BIINOCHANNEL_H_
#define _BIINOCHANNEL_H_

/*
 * Stock Arduino libraries
 */
#include <Arduino.h>

/*
 * Custom libraries
 */
#include "BiinoInput.h"

/*
 * Class definitions
 */

/*
 * Class BiinoChannel
 */
class BiinoChannel
{
  public:
    uint8_t id; // Channel number (1 to 6*n), n refers to the corresponding number of BiinoInput modules.
    String title; //
    String title_short; //
    uint8_t ee_addr_volume; // EEPROM address to store and restore the assigned volume.
    BiinoInput* biino_input = NULL;
  public:
    BiinoChannel(uint8_t id, const String title, const String title_short, const uint8_t ee_addr, BiinoInput* biino_input);
    BiinoChannel(uint8_t id, const String title, const String title_short, const uint8_t ee_addr );    
};
#endif
