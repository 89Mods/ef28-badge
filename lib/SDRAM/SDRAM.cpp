#include <Arduino.h>
#include <esp_attr.h>
#include <driver/spi_master.h>
#include <EFLogging.h>

#include "SDRAM.h"

#undef QUESTIONABLE_SPI_FIXES
#define USE_QUAD
#define USE_QUAD_READ
#define PSRAM_SPI_CLOCK 60000000

static spi_device_handle_t handle;

void SDRAM::init() {
	spi_bus_config_t spi_bus_config = {
		.mosi_io_num = SPI_MOSI_PIN,
		.miso_io_num = SPI_MISO_PIN,
		.sclk_io_num = SPI_SCLK_PIN,
#ifdef USE_QUAD
		.data2_io_num = QPI_D2_PIN,
		.data3_io_num = QPI_D3_PIN,
#else
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
#endif
		.max_transfer_sz = 0,
#ifdef USE_QUAD
		.flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_QUAD
#else
		.flags = SPICOMMON_BUSFLAG_MASTER
#endif
	};
#ifndef USE_QUAD
	pinMode(QPI_D2_PIN, INPUT);
	pinMode(QPI_D3_PIN, INPUT);
#endif
	esp_err_t ret = spi_bus_initialize(SPI2_HOST, &spi_bus_config, SPI_DMA_CH_AUTO);
	if(ret != ESP_OK) LOG_ERROR("spi_bus_initialize error");
	spi_device_interface_config_t devcfg;
	memset(&devcfg, 0, sizeof(spi_device_interface_config_t));
	devcfg.clock_speed_hz = PSRAM_SPI_CLOCK;
	devcfg.spics_io_num = -1;
	devcfg.queue_size = 1;
	devcfg.command_bits = 8;
	devcfg.address_bits = 24;
	devcfg.flags = SPI_DEVICE_HALFDUPLEX;
#ifdef QUESTIONABLE_SPI_FIXES
#ifndef QUAD_MODE
	devcfg.mode = 2;
#endif
#endif
	ret = spi_bus_add_device(SPI2_HOST, &devcfg, &handle);
	if(ret != ESP_OK) LOG_ERROR("spi_bus_add_device error");
}

void SDRAM::send_cmd(const uint8_t cmd) {
	spi_transaction_ext_t t = { };
	t.base.flags = SPI_TRANS_VARIABLE_ADDR;
	t.base.cmd = cmd;
	t.base.length = 0;
	t.command_bits = 8U;
	t.address_bits = 0;
	spi_device_polling_transmit(handle, (spi_transaction_t*)&t);
}

SDRAM::SDRAM(int cs_pin) : cs_pin(cs_pin) {
	pinMode(cs_pin, OUTPUT);
	digitalWrite(cs_pin, HIGH);
	delay(1);
}

void SDRAM::reset() {
	send_cmd(0); //Some clocks while CS is high, as required by the datasheet
	digitalWrite(cs_pin, LOW);
	send_cmd(0x66);
	send_cmd(0x99);
	digitalWrite(cs_pin, HIGH);
	delay(1);

	digitalWrite(cs_pin, LOW);
	uint8_t rxdata[6];
	spi_transaction_ext_t t = { };
	t.base.cmd = 0x9F;
	t.base.addr = 0;
	t.base.rx_buffer = rxdata;
	t.base.length = 0;
	t.base.tx_buffer = NULL;
	t.base.rxlength = 6 * 8;
#ifdef QUESTIONABLE_SPI_FIXES
	t.base.flags = SPI_TRANS_VARIABLE_DUMMY;
	t.dummy_bits = 1;
#endif
	esp_err_t ret = spi_device_polling_transmit(handle, (spi_transaction_t*)&t);
	if(ret != ESP_OK) { LOG_ERROR("spi_device_polling_transmit error"); }
	else {
		char strbuff[32];
		sprintf(strbuff, "PSRAM ID: %02x%02x%02x%02x%02x%02x", rxdata[0], rxdata[1], rxdata[2], rxdata[3], rxdata[4], rxdata[5]);
		LOG_DEBUG(strbuff);
		if(rxdata[1] != 0x5D) { LOG_WARNING("KGD is FALSE!"); }
	}
	digitalWrite(cs_pin, HIGH);
	delay(1);
}

void SDRAM::read(uint32_t addr, uint8_t *buffer, uint16_t len) {
	if(len == 0) return;
	addr &= 0x007FFFFF;
	uint32_t final_addr = addr + len - 1;
	if((final_addr & 0xFFFFFC00) != (addr & 0xFFFFFC00)) {
		//This read crosses a page boundary
		uint32_t diff = 0x3FF - (addr & 0x3FF) + 1;
		this->read(addr, buffer, diff);
		this->read(addr + diff, buffer + diff, len - diff);
		return;
	}
	read_alligned(addr, buffer, len);
}

void SDRAM::write(uint32_t addr, uint8_t *buffer, uint16_t len) {
	if(len == 0) return;
	addr &= 0x007FFFFF;
	uint32_t final_addr = addr + len - 1;
	if((final_addr & 0xFFFFFC00) != (addr & 0xFFFFFC00)) {
		//This write crosses a page boundary
		uint32_t diff = 0x3FF - (addr & 0x3FF) + 1;
		this->write(addr, buffer, diff);
		this->write(addr + diff, buffer + diff, len - diff);
		return;
	}
	write_alligned(addr, buffer, len);
}

void IRAM_ATTR SDRAM::read_alligned(uint32_t addr, uint8_t *buffer, uint16_t len) {
	if(len == 0) return;
	addr &= 0x007FFFFF;
	spi_transaction_ext_t t = { };
	memset(&t, 0, sizeof(spi_transaction_t));
	t.base.addr = addr;
	t.base.rx_buffer = buffer;
	t.base.length = 0;
	t.base.tx_buffer = NULL;
	t.base.rxlength = len * 8;
#ifdef USE_QUAD_READ
	t.base.cmd = 0xEB;
	t.base.flags = SPI_TRANS_VARIABLE_DUMMY | SPI_TRANS_MODE_QIO | SPI_TRANS_MULTILINE_ADDR;
	t.dummy_bits = 6;
#else
	t.base.flags = SPI_TRANS_VARIABLE_DUMMY;
	t.base.cmd = 0x0B;
#ifdef QUESTIONABLE_SPI_FIXES
	t.dummy_bits = 9;
#else
	t.dummy_bits = 8;
#endif
#endif
	digitalWrite(cs_pin, LOW);
	spi_device_polling_transmit(handle, (spi_transaction_t*)&t);
	digitalWrite(cs_pin, HIGH);
	asm volatile("nop");
	asm volatile("nop");
}

void IRAM_ATTR SDRAM::write_alligned(uint32_t addr, uint8_t *buffer, uint16_t len) {
	if(len == 0) return;
	addr &= 0x007FFFFF;
	spi_transaction_t t = { };
	memset(&t, 0, sizeof(spi_transaction_t));
	t.addr = addr;
	t.tx_buffer = buffer;
	t.length = len * 8;
	t.rx_buffer = NULL;
	t.rxlength = 0;
#ifdef USE_QUAD
	t.cmd = 0x38;
	t.flags = SPI_TRANS_MODE_QIO | SPI_TRANS_MULTILINE_ADDR;
#else
	t.cmd = 0x02;
#endif
	digitalWrite(cs_pin, LOW);
	spi_device_polling_transmit(handle, &t);
	digitalWrite(cs_pin, HIGH);
	asm volatile("nop");
	asm volatile("nop");
}