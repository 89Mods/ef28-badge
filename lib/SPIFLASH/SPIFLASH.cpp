#include <Arduino.h>
#include <EFLogging.h>

#include "SPIFLASH.h"

void SPIFLASH::init() {
    digitalWrite(cs_pin, LOW);
    send_cmd(0xFF);
    digitalWrite(cs_pin, HIGH);
    digitalWrite(cs_pin, LOW);
    send_cmd(0xAB);
    for(int i = 0; i < 4; i++) send_cmd(0x00);
    digitalWrite(cs_pin, HIGH);
    digitalWrite(cs_pin, LOW);
    send_cmd(0x90);
    for(int i = 0; i < 3; i++) send_cmd(0x00);
    uint8_t mf = send_cmd(0x00);
    uint8_t id = send_cmd(0x00);
    digitalWrite(cs_pin, HIGH);
    char strbuff[32];
    sprintf(strbuff, "SPIFLASH MFR and ID: %02x, %02x", mf, id);
    LOG_DEBUG(strbuff);
}

uint8_t SPIFLASH::send_cmd(uint8_t cmd) {
    uint8_t res = 0;
    for(int i = 0; i < 8; i++) {
        if((cmd & 128) != 0) digitalWrite(FLASH_MOSI_PIN, HIGH);
        else digitalWrite(FLASH_MOSI_PIN, LOW);
        cmd <<= 1;
        digitalWrite(FLASH_SCLK_PIN, HIGH);
        res <<= 1;
        res |= digitalRead(FLASH_MISO_PIN);
        digitalWrite(FLASH_SCLK_PIN, LOW);
    }
    digitalWrite(FLASH_MOSI_PIN, LOW);
    return res;
}

SPIFLASH::SPIFLASH(int cs_pin) : cs_pin(cs_pin) {
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);
    delay(1);
    pinMode(FLASH_MISO_PIN, INPUT);
    pinMode(FLASH_SCLK_PIN, OUTPUT);
    pinMode(FLASH_MOSI_PIN, OUTPUT);
    digitalWrite(FLASH_SCLK_PIN, LOW);
    digitalWrite(FLASH_MISO_PIN, LOW);
    digitalWrite(FLASH_MOSI_PIN, LOW);
}

void SPIFLASH::read(uint32_t addr, uint8_t *buffer, uint16_t len) {
    digitalWrite(cs_pin, LOW);
    send_cmd(0x03);
    send_cmd(addr >> 16);
    send_cmd(addr >> 8);
    send_cmd(addr);
    uint8_t rec;
    for(int i = 0; i < len; i++) {
        digitalWrite(FLASH_SCLK_PIN, HIGH);
        rec = digitalRead(FLASH_MISO_PIN);
        digitalWrite(FLASH_SCLK_PIN, LOW);
        rec <<= 1;
        digitalWrite(FLASH_SCLK_PIN, HIGH);
        rec |= digitalRead(FLASH_MISO_PIN);
        digitalWrite(FLASH_SCLK_PIN, LOW);
        rec <<= 1;
        digitalWrite(FLASH_SCLK_PIN, HIGH);
        rec |= digitalRead(FLASH_MISO_PIN);
        digitalWrite(FLASH_SCLK_PIN, LOW);
        rec <<= 1;
        digitalWrite(FLASH_SCLK_PIN, HIGH);
        rec |= digitalRead(FLASH_MISO_PIN);
        digitalWrite(FLASH_SCLK_PIN, LOW);
        rec <<= 1;
        digitalWrite(FLASH_SCLK_PIN, HIGH);
        rec |= digitalRead(FLASH_MISO_PIN);
        digitalWrite(FLASH_SCLK_PIN, LOW);
        rec <<= 1;
        digitalWrite(FLASH_SCLK_PIN, HIGH);
        rec |= digitalRead(FLASH_MISO_PIN);
        digitalWrite(FLASH_SCLK_PIN, LOW);
        rec <<= 1;
        digitalWrite(FLASH_SCLK_PIN, HIGH);
        rec |= digitalRead(FLASH_MISO_PIN);
        digitalWrite(FLASH_SCLK_PIN, LOW);
        rec <<= 1;
        digitalWrite(FLASH_SCLK_PIN, HIGH);
        rec |= digitalRead(FLASH_MISO_PIN);
        digitalWrite(FLASH_SCLK_PIN, LOW);
        *buffer++ = rec;
    }
    digitalWrite(cs_pin, HIGH);
}

void SPIFLASH::busy_wait() {
    while(1) {
        digitalWrite(cs_pin, LOW);
        send_cmd(0x05);
        uint8_t sr1 = send_cmd(0x00);
        digitalWrite(cs_pin, HIGH);
        if((sr1 & 1) == 0) break;
    }
}

void SPIFLASH::write_enable() {
    digitalWrite(cs_pin, LOW);
    send_cmd(0x06);
    digitalWrite(cs_pin, HIGH);
    digitalWrite(cs_pin, LOW);
    send_cmd(0x05);
    uint8_t sr1 = send_cmd(0x00);
    digitalWrite(cs_pin, HIGH);
    if((sr1 & 2) == 0) LOG_ERROR("SPIFLASH WEL in SR1 did not go high when it was supposed to!");
}

void SPIFLASH::erase() {
    write_enable();
    digitalWrite(cs_pin, LOW);
    send_cmd(0xC7);
    digitalWrite(cs_pin, HIGH);
    busy_wait();
    LOG_DEBUG("SPIFLASH erased");
}

void SPIFLASH::write_page(uint32_t addr, uint8_t *buffer, uint16_t len) {
    if(len > 256) return;
    write_enable();
    digitalWrite(cs_pin, LOW);
    send_cmd(0x02);
    send_cmd(addr >> 16);
    send_cmd(addr >> 8);
    send_cmd(addr);
    for(int i = 0; i < len; i++) {
        send_cmd(buffer[i]);
    }
    digitalWrite(cs_pin, HIGH);
    busy_wait();
}