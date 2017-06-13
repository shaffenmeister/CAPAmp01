#ifndef _BIINOVOLUME_H_
#define _BIINOVOLUME_H_

/*
 * Stock Arduino libraries
 */
#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>

/*
 * Custom libraries
 */
#include <mcp23s08.h>

/*
 * Class definitions
 */

/*
* Class BiinoVolume allows to adjust the volume of a single Biino volume board using the MCP23S08 SPI I/O expander.
* It also stores the last volume selected in the internal EEPROM.
*/
class BiinoVolume
{
  public:
    uint8_t biino_id; // more than one volume boards supported, first = 0, second = 1, ...
    uint8_t ee_addr_cur_volume; // EEPROM address to store and restore the last defined volume.
    uint16_t volume_switch_delay_ms = 0; // delay when switching volume: old volume <deselect> - <delay> - new volume <select>
    
  protected:
    mcp23s08* biino_volume; // reference to MCP23S08

  public:  
    BiinoVolume(uint8_t id, uint8_t pin_cs, uint8_t spi_addr, uint8_t ee_addr_cur_volume);
    void Setup();
    bool IsValid(uint8_t volume);
    int SetVolume(uint8_t volume);
    int GetVolume(void);
    int IncVolume(void);
};

#endif
