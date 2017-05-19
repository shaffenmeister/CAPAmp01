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

class BiinoInput
{
  public:
    uint8_t biino_id; // more than one input boards supported, first = 0, second = 1, ...
    uint8_t channel_mask;
    uint8_t ee_addr_cur_channel;
        
  protected:
    mcp23s08* biino_input; // (PIN_CS_BIINO_VOL,ADDR_BIINO_VOL,MAXSPISPEED);

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
    
  protected:
    mcp23s08* biino_volume; // (PIN_CS_BIINO_VOL,ADDR_BIINO_VOL,MAXSPISPEED);

  public:  
    BiinoVolume(uint8_t id, const uint8_t pin_cs, const uint8_t spi_addr, uint8_t ee_addr);
};

#endif
