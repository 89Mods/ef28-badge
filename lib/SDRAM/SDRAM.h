#ifndef SDRAM_H_
#define SDRAM_H_

#define SPI_SCLK_PIN GPIO_NUM_14
#define SPI_MOSI_PIN GPIO_NUM_13
#define SPI_MISO_PIN GPIO_NUM_12
#define QPI_D2_PIN GPIO_NUM_8
#define QPI_D3_PIN GPIO_NUM_18

#include <stdint.h>
#include <driver/spi_master.h>

class SDRAM {
private:
	int cs_pin;
	void send_cmd(const uint8_t cmd);
public:
	SDRAM(int cs_pin);
	void init();
	void reset();
	void read(uint32_t addr, uint8_t *buffer, uint16_t len);
	void write(uint32_t addr, uint8_t *buffer, uint16_t len);
	//Use if you are ABSOLUTELY SURE the memory access does not cross a 1024-byte page boundary
	//i.e. a RISC-V emulator, where the RV spec defines all halfword/word access to be alligned
	//these are slightly faster due to skipping some checks
	void IRAM_ATTR read_alligned(uint32_t addr, uint8_t *buffer, uint16_t len);
	void IRAM_ATTR write_alligned(uint32_t addr, uint8_t *buffer, uint16_t len);
};

#endif
