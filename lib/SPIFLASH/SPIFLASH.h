#ifndef SPIFLASH_H_
#define SPIFLASH_H_

#include <stdint.h>

#define FLASH_SCLK_PIN GPIO_NUM_4
#define FLASH_MOSI_PIN GPIO_NUM_5
#define FLASH_MISO_PIN GPIO_NUM_6
#define FLASH_DEFAULT_CS_PIN GPIO_NUM_7

class SPIFLASH {
private:
int cs_pin;
    uint8_t send_cmd(const uint8_t cmd);
    void busy_wait();
    void write_enable();
public:
    SPIFLASH(int cs_pin);
    void init();
    void read(uint32_t addr, uint8_t *buffer, uint16_t len);
    void erase();
    void write_page(uint32_t addr, uint8_t *buffer, uint16_t len); //One page = 256 bytes max.
};

#endif
