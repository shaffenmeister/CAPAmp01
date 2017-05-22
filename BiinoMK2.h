#ifndef _BIINOMK2_H_
#define _BIINOMK2_H_

/*
 * Stock Arduino libraries
 */
#include <SPI.h>

/*
 * Custom libraries
 */
#include <mcp23s08.h>

/*
 * Class definitions
 */

/*
 * Class BiinoInputChannel
 */
class BiinoInputChannel
{
  public:
    uint8_t id;
    String title;
    String title_short;
    uint8_t volume;
    uint8_t ee_addr_volume;
  
  public:  
    BiinoInputChannel(uint8_t id, String title, String title_short, uint8_t ee_addr);
};

/*
 * Class BiinoInput allows to select and deselect input channels of a single Biino input board using the MCP23S08 SPI I/O expander.
 * It also stores the last channel selected in the internal EEPROM. Channels are referred to a bits inside a bit mask.
 * I.e. channel 1 = bit 0 (2^0 = 1), 5 = bit 4 (2^4 = 16).
 */
class BiinoInput
{
  public:
    uint8_t biino_id; // more than one input boards supported, first = 0, second = 1, ...
    uint8_t channel_mask; // bit mask for all channels to be used. 
    uint8_t ee_addr_cur_channel; // EEPROM address to store and restore the last selected channel.
    uint16_t channel_switch_delay_ms = 200; // delay when switching channels: old channel <deselect> - <delay> - new channel <select>
            
  protected:
    mcp23s08* biino_input; // reference to MCP23S08

  public:  
    BiinoInput(uint8_t id, uint8_t channel_mask, uint8_t pin_cs, uint8_t spi_addr, uint8_t ee_addr_cur_channel);
    void Setup();
    void DeselectAll(void);
    int Select(uint8_t channel);

    int SelectNext(void);    
    int SelectPrevious(void);        
    int SelectFirst(void);    
    int SelectLast(void);        
    
    bool IsChannelValid(uint8_t channel);
    uint8_t GetCurrentChannel(void);
    int GetCurrentChannelNo(void);
    
  private:
    int Select(bool next);
};

class BiinoVolume
{
  public:
    uint8_t biino_id; // more than one volume boards supported, first = 0, second = 1, ...
    uint8_t ee_addr_cur_volume; // EEPROM address to store and restore the last defined volume.
  protected:
    mcp23s08* biino_volume; // reference to MCP23S08

  public:  
    BiinoVolume(uint8_t id, uint8_t pin_cs, uint8_t spi_addr, uint8_t ee_addr_cur_volume);
    void Setup();
    bool IsValid(uint8_t volume);
    int SetVolume(uint8_t volume);
    int GetVolume(void);
};

#endif
