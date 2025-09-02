#include <Arduino.h>
#include <Preferences.h>
#include <EFLogging.h>
#include <SDRAM.h>
#include <SPIFLASH.h>
#include <WiFiClientSecure.h>
#include <EFTouch.h>
#ifdef TELNET_SHELL
#include "ESPTelnet.h"
#endif

#define SECOND_CORE_IMPL
#include "SecondCore.h"
#include "util.h"

#ifdef TELNET_SHELL
ESPTelnet telnet;
void on_telnet_connect(String ip);
void on_telnet_disconnect(String ip);
void on_telnet_reconnect(String ip);
void on_telnet_connection_attempt(String ip);
void on_telnet_input(String str);
void telnet_buff_flush();

uint8_t telnet_in_buffer[256];
uint16_t telnet_in_wptr = 0;
uint16_t telnet_in_rptr = 0;
uint8_t telnet_out_buffer[256];
uint16_t telnet_out_wptr = 0;
uint64_t telnet_last_w = 0;
#endif

Preferences pref2;

//SecondCore.h thingies
TaskHandle_t Task2;
volatile uint8_t led_overrides[EFLED_TOTAL_NUM];
volatile uint32_t led_override_values[EFLED_TOTAL_NUM];
volatile bool sdram_initialized = false;

// mini-rv32ima variables
int fail_on_all_faults = 0;
uint64_t lastTime = 0;
struct MiniRV32IMAState *core;

// Functions prototype
static uint32_t HandleException(uint32_t ir, uint32_t retval);
static uint32_t HandleControlStore(uint32_t addy, uint32_t val);
static uint32_t HandleControlLoad(uint32_t addy);
static void HandleOtherCSRWrite(uint8_t * image, uint16_t csrno, uint32_t value);
static int32_t HandleOtherCSRRead(uint8_t * image, uint16_t csrno);

// Load / store helper
static uint32_t load1_signed(uint32_t ofs);
static uint32_t load2_signed(uint32_t ofs);
static uint32_t store4(uint32_t ofs, uint32_t val);
static uint16_t store2(uint32_t ofs, uint16_t val);
static uint8_t inline store1(uint32_t ofs, uint8_t val);
static uint32_t load4(uint32_t ofs);
static uint16_t load2(uint32_t ofs);
static uint8_t inline load1(uint32_t ofs);
static uint32_t loadi(uint32_t ofs);

void gpio_write(uint8_t gpio_idx, uint8_t reg, uint32_t val);
uint32_t gpio_read(uint8_t gpio_idx, uint8_t reg);

// Config
const uint32_t RAM_SIZE = 16777216UL; // Minimum RAM amount (in bytes), just tested (may reduce further by custom kernel)
#define DTB_SIZE 4096               // DTB size (in bytes), must recount manually each time DTB changes
#define INSTRS_PER_FLIP 2048        // Number of instructions executed before checking status. See loop()

// Setup mini-rv32ima
// This is the functionality we want to override in the emulator.
// think of this as the way the emulator's processor is connected to the outside world.
#define MINIRV32WARN( x... ) LOG_WARNING(x);
#define MINIRV32_DECORATE  static
#define MINI_RV32_RAM_SIZE RAM_SIZE
#define MINIRV32_IMPLEMENTATION // Minimum rv32 emulator
#define MINIRV32_POSTEXEC( pc, ir, retval ) { if( retval > 0 ) { if( fail_on_all_faults ) { puts("FAULT\r\n"); return 3; } else retval = HandleException( ir, retval ); } }
#define MINIRV32_HANDLE_MEM_STORE_CONTROL( addy, val ) if( HandleControlStore( addy, val ) ) return val;
#define MINIRV32_HANDLE_MEM_LOAD_CONTROL( addy, rval ) rval = HandleControlLoad( addy );
#define MINIRV32_OTHERCSR_WRITE( csrno, value ) HandleOtherCSRWrite( image, csrno, value );
#define MINIRV32_OTHERCSR_READ( csrno, value ) value = HandleOtherCSRRead( image, csrno );
#define MINIRV32_CUSTOM_MEMORY_BUS

// Macro for accessing RAM
#define MINIRV32_STORE4( ofs, val ) store4(ofs, val)
#define MINIRV32_STORE2( ofs, val ) store2(ofs, val)
#define MINIRV32_STORE1( ofs, val ) store1(ofs, val)
#define MINIRV32_LOAD4( ofs ) load4(ofs)
#define MINIRV32_LOAD2_SIGNED( ofs ) (int16_t)load2(ofs)
#define MINIRV32_LOAD2( ofs ) load2(ofs)
#define MINIRV32_LOAD1_SIGNED( ofs ) (int8_t)load1(ofs)
#define MINIRV32_LOAD1( ofs ) load1(ofs)
#define MINIRV32_LOADI( ofs ) loadi(ofs)

#include "mini-rv32ima.h"

SDRAM first_chip(GPIO_NUM_11);
SDRAM second_chip(GPIO_NUM_47);

bool watchdog_enabled = false;
bool watchdog_clr_edge = false;
bool leds_latch_edge = false;
uint64_t last_watchdog_time;

void err_leds() {
    for(uint8_t i = 0; i < EFLED_TOTAL_NUM; i++) led_overrides[i] = 0;
    EFLed.setAllSolid(CRGB::Red);
    EFLed.ledsShow();
}

uint32_t rng_state;

uint32_t xorshift32() {
	uint32_t x = rng_state;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	return rng_state = x;
}

#define IMPLEMENT_CACHE
#ifdef IMPLEMENT_CACHE
 //Must be even
#define CACHE_ENTRIES 200

struct cache_entry {
    uint8_t* data;
    uint32_t ofs;
    //On dirty entries, keep track of both the highest addressed and lowest addressed instances of modified bytes
    //This reduces the amount of data needing to be written back to the PSRAM when the entry is evicted
    uint16_t lowest;
    uint16_t highest;
    bool dirty;
};

cache_entry c_entries[CACHE_ENTRIES];
//Keep track of the two most recently accessed cache entries. These will be checked first when testing for a cache hit.
//The idea is that the CPU will commonly be in a scenario where it is executing code from one same block and accessing data from another same block
//This speeds up hit detection in this case
uint8_t recent_entry = 0;
uint8_t second_recent_entry = 0;
void free_cache() { for(int i = 0; i < CACHE_ENTRIES; i++) free(c_entries[i].data); }
#endif

//Used only during initialization from compressed RAM image
void sdram_write_buffer(uint8_t* buff, uint16_t len, uint32_t addr) {
    if(addr >= 16*1024*1024) return;
    if(addr + len - 1 >= 16*1024*1024) {
        len = 16*1024*1024 - addr;
    }
    if(addr < 8*1024*1024 && (addr + len - 1) >= 8*1024*1024) {
        uint32_t limit = 8*1024*1024 - addr;
        first_chip.write(addr, buff, limit);
        second_chip.write(addr + limit, buff + limit, len - limit);
    }else if(addr < 8*1024*1024) first_chip.write(addr, buff, len);
    else second_chip.write(addr - 8*1024*1024, buff, len);
}

void Task2main(void* pvParameters) {
    sdram_initialized = false;
    sleep(3);
    LOG_INFO("(2nd Core) I’m alive!");
    first_chip.init();
full_emulator_reset:
    first_chip.reset();
    second_chip.reset();
    rng_state = 0xABC89105;
    watchdog_enabled = false;
    watchdog_clr_edge = false;
    leds_latch_edge = false;

    //A few iterations of simple write/readback test for the PSRAM
    bool ram_okay = true;
    uint32_t test_base_addr = 0;
    for(int x = 0; x < 2; x++) {
        const int test_len = 1024;
        uint8_t test_data[test_len];
        for(int i = 0; i < test_len; i++) {
            test_data[i] = xorshift32();
        }
        uint8_t rec_buff[test_len];
        first_chip.write(test_base_addr, test_data, test_len);
        for(uint16_t i = 0; i < test_len; i++) test_data[i] *= 7;
        second_chip.write(test_base_addr, test_data, test_len);

        memset(rec_buff, 0, test_len);
        first_chip.read(test_base_addr, rec_buff, test_len);
        bool passes = true;
        for(uint16_t i = 0; i < test_len; i++) {
            if(((rec_buff[i] * 7) & 0xFF) != test_data[i]) passes = false;
        }
        if(passes) { LOG_INFO("First RAM IC passes"); }
        else { LOG_ERROR("First RAM IC fails!"); }
        ram_okay &= passes;
        memset(rec_buff, 0, test_len);
        second_chip.read(test_base_addr+9, rec_buff, test_len-9);
        passes = true;
        for(uint16_t i = 9; i < test_len; i++) {
            if(rec_buff[i-9] != test_data[i]) passes = false;
            //printf("%u\r\n", rec_buff[i]);
        }
        if(passes) { LOG_INFO("Second RAM IC passes"); }
        else { LOG_ERROR("Second RAM IC fails!"); }
        ram_okay &= passes;

        test_base_addr += 1024;
        test_base_addr %= 16*1024*1024;
    }
    if(!ram_okay) {
        LOG_FATAL("RAM test fails");
        vTaskDelete(NULL);
        while(true);
    }

    //Download RAM image
    #ifdef ONLINE_LINUX_IMAGE
    httpsClient.setCACert(root_ca);
    if(!httpsClient.connect(boot_server, 443)) {
        LOG_FATAL("Download RAM image fail: Connection failed");
        vTaskDelete(NULL);
        while(true);
    }
    httpsClient.print("GET ");
    httpsClient.print(boot_url);
    httpsClient.println(" HTTP/1.0");
    httpsClient.print("Host: ");
    httpsClient.println(boot_server);
    httpsClient.println("Connection: close");
    httpsClient.println();
    //Skip headers
    while(httpsClient.connected()) {
        String line = httpsClient.readStringUntil('\n');
        if(line == "\r") {
            break;
        }
    }
    #endif
    #ifdef ONLINE_LINUX
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    #endif
    SPIFLASH spiflash(GPIO_NUM_7);
    spiflash.init();
    uint32_t stream_position = 0;
    //Loads a RLC image into RAM
    //Its not true RLC - only streams of zeroes are compressed
    {
        #ifdef ONLINE_LINUX_IMAGE
        #ifdef FLASH_UPDATE
        spiflash.erase();
        #endif
        #endif
        uint8_t rbuffer[512];
        uint16_t rptr = 0xFFFF;
        uint8_t wbuffer[512];
        uint32_t ram_ptr = 0;
        uint16_t wptr = 0;
        uint16_t rem;
        bool flag1 = false;
        uint32_t skip_len;
        uint8_t addr_progress = 0;
        putchar('\n');
        int lastProgressI = 0;
        while(1) {
            float progressF = (float)ram_ptr / (float)(RAM_SIZE) * 50;
            int progressI = (int)progressF;
            if(progressI != lastProgressI) {
                putchar('\r');
                for(int i = 0; i < 51; i++) {
                    if(i == 0 || i == 50) {
                        putchar('|');
                        continue;
                    }
                    if((i - 1) < progressI) putchar('#');
                    else putchar(' ');
                }
                fflush(stdout);
            }
            lastProgressI = progressI;
            if(ram_ptr >= RAM_SIZE) break;

            if(rptr >= 512) {
                #ifdef ONLINE_LINUX_IMAGE
                rem = httpsClient.readBytes(rbuffer, 512);
                #ifdef FLASH_UPDATE
                spiflash.write_page(stream_position, rbuffer, 256);
                spiflash.write_page(stream_position + 256, rbuffer + 256, 256);
                #endif
                #else
                spiflash.read(stream_position, rbuffer, 512);
                rem = 512;
                #endif
                stream_position += rem;
                rptr = 0;
            }
            if(rem == 0) break;
            uint8_t nb = rbuffer[rptr++];
            rem--;
            if(addr_progress) {
                skip_len <<= 8;
                skip_len |= nb;
                addr_progress++;
                if(addr_progress != 3) continue;
                addr_progress = 0;
                if(wptr != 0) {
                    uint16_t to_write = wptr;
                    if(ram_ptr + to_write >= RAM_SIZE) to_write = RAM_SIZE - ram_ptr;
                    if(to_write) sdram_write_buffer(wbuffer, to_write, ram_ptr);
                    ram_ptr += wptr;
                    wptr = 0;
                }
                //Write out zeroes
                memset(wbuffer, 0, 512);
                while(skip_len) {
                    uint32_t wlen = skip_len > 512 ? 512 : skip_len;
                    sdram_write_buffer(wbuffer, wlen, ram_ptr);
                    skip_len -= wlen;
                    ram_ptr += wlen;
                }
                continue;
            }
            if(flag1) {
                flag1 = false;
                if(nb != 0xFF) {
                    skip_len = nb;
                    addr_progress = 1;
                    continue;
                }
            }else if(nb == 0xFF) {
                flag1 = true;
                continue;
            }
            wbuffer[wptr++] = nb;
            if(wptr >= 512) {
                uint16_t to_write = wptr;
                if(ram_ptr + to_write >= RAM_SIZE) to_write = RAM_SIZE - ram_ptr;
                if(to_write) sdram_write_buffer(wbuffer, wptr, ram_ptr);
                ram_ptr += wptr;
                wptr = 0;
            }
        }
        if(wptr != 0) sdram_write_buffer(wbuffer, wptr, ram_ptr);
        putchar('\r');
        putchar('\n');
    }
    #ifdef ONLINE_LINUX_IMAGE
    httpsClient.stop();
    #ifdef FLASH_UPDATE
    LOG_DEBUG("SPIFLASH contents updated");
    #endif
    #endif
    sdram_initialized = true;
    while(WiFi.isConnected()) sleep(1);

    core = (struct MiniRV32IMAState *)malloc(sizeof(struct MiniRV32IMAState));
    if(!core) {
        LOG_FATAL("Init fail: malloc failed on core (struct MiniRV32IMAState)");
        vTaskDelete(NULL);
        while(true);
    }
    memset(core, 0, sizeof(struct MiniRV32IMAState));

#ifdef IMPLEMENT_CACHE
    for(int i = 0; i < CACHE_ENTRIES; i++) {
        c_entries[i].ofs = 0xFFFFFF00 + i;
        c_entries[i].dirty = false;
        c_entries[i].lowest = c_entries[i].highest = 0;
        c_entries[i].data = (uint8_t *)malloc(1024);
        if(!c_entries[i].data) {
            LOG_FATAL("Init fail: malloc failed on cache entries");
            vTaskDelete(NULL);
            while(true);
        }
    }
    recent_entry = second_recent_entry = 0;
#endif

    //Setup core
    core->pc = MINIRV32_RAM_IMAGE_OFFSET;
    core->regs[10] = 0x00; //hart ID
    core->regs[11] = RAM_SIZE - sizeof(struct MiniRV32IMAState) - DTB_SIZE + MINIRV32_RAM_IMAGE_OFFSET; // dtb_pa (Must be valid pointer) (Should be pointer to dtb)
    core->extraflags |= 3; // Machine-mode.

    yield();
    yield();
    printf("Free heap after emulator allocations: %u\r\n", esp_get_free_heap_size());
    LOG_INFO("RAM Initialized, starting mini-rv32ima");

    #ifdef TELNET_SHELL
    sleep(5);
    telnet.onConnect(on_telnet_connect);
    telnet.onConnectionAttempt(on_telnet_connection_attempt);
    telnet.onReconnect(on_telnet_reconnect);
    telnet.onDisconnect(on_telnet_disconnect);
    telnet.onInputReceived(on_telnet_input);
    if(telnet.begin(23)) {
        LOG_INFO("Telnet ready");
        sleep(3);
    }else {
        LOG_ERROR("Telnet init fail. Telnet will not be available during this session.");
    }
    #endif

    //Run the emulator in an infinite loop

    //This determines the rate at which a simulated timer increments
    //It cannot increment too fast compared to the IPS of the simulated CPU, as this timer is used to trigger interrupts
    //You do NOT want the CPU to spend most of its time serving one interrupt after the other
    //For now, its safe to make it run at 1/10th real-time
    const uint64_t time_div = 10;
    uint64_t b = (micros() / time_div);
    do {
        yield();
        #ifdef TELNET_SHELL
        telnet.loop();
        if(millis() - telnet_last_w > 3000) {
            telnet_last_w = millis();
            if(telnet_out_wptr) telnet_buff_flush();
        }
        #endif
        uint64_t *this_ccount = ((uint64_t*)&(core->cyclel));
        uint32_t diff = (uint32_t)((micros() / time_div) - b);
        int ret = MiniRV32IMAStep(core, NULL, 0, diff, INSTRS_PER_FLIP); // Execute upto INSTRS_PER_FLIP cycles before breaking out.
        b += diff;
        switch(ret) {
            case 0: break;
            case 0x7777:
            case 0x5555:
                LOG_INFO("mini-rv32ima POWEROFF");
                free(core);
#ifdef IMPLEMENT_CACHE
                free_cache();
#endif
                vTaskDelete(NULL);
                break;
            case 1:
                *this_ccount += INSTRS_PER_FLIP;
                break;
            case 3:
                break;
            default:
                LOG_FATAL("mini-rv32ima UNKNOWN FAILURE");
                free(core);
#ifdef IMPLEMENT_CACHE
                free_cache();
#endif
                err_leds();
                vTaskDelete(NULL);
                break;
        }
        if(watchdog_enabled && (millis() - last_watchdog_time) > 15000) {
            LOG_FATAL("mini-rv32ima watchdog timer expired");
            err_leds();
            free(core);
            free_cache();
            sleep(5);
            EFLed.clear();
            EFLed.ledsShow();
            goto full_emulator_reset;
        }
    }while(true);
}

void StartSecondCore() {
    for(uint8_t i = 0; i < EFLED_TOTAL_NUM; i++) led_overrides[i] = 0;
    xTaskCreatePinnedToCore(Task2main, "Task2", 10240, NULL, 1, &Task2, 1);
    delay(500);
}

//Emulated GPIO ports, based on https://www.gaisler.com/products/grlib/grip.pdf , page 741

uint32_t gpio_data_buffers[17+1];
uint32_t gpio_dir_buffers[17+1]; //Unused in simulation, but registers emulated so reads from 0x08 are consistent

void gpioWrite(uint8_t gpio_idx, uint8_t reg, uint32_t val) {
    if(gpio_idx >= 18) return;
    //No idea why the values Linux writes a rotated left by 8 from what they should be
    //But gotta fix that here
	uint32_t nv = val >> 8;
	nv |= val << 24;
    if(gpio_idx == 17) {
        nv = val >> 24;   
    }
    if(reg == 0x04) gpio_data_buffers[gpio_idx] = nv & 0x01FFFFFF;
    if(reg == 0x08) gpio_dir_buffers[gpio_idx] = nv & 0x01FFFFFF;
    if(reg == 0x54) gpio_data_buffers[gpio_idx] |= nv & 0x01FFFFFF;
    if(reg == 0x58) gpio_dir_buffers[gpio_idx] |= nv & 0x01FFFFFF;
    if(reg == 0x64) gpio_data_buffers[gpio_idx] &= nv;
    if(reg == 0x68) gpio_dir_buffers[gpio_idx] &= nv;
    if(reg == 0x74) gpio_data_buffers[gpio_idx] ^= nv & 0x01FFFFFF;
    if(reg == 0x78) gpio_dir_buffers[gpio_idx] ^= nv & 0x01FFFFFF;

    for(uint8_t i = 0; i < 17; i++) {
        led_overrides[i] = (gpio_data_buffers[i] & 0x01000000) != 0;
        led_override_values[i] = gpio_data_buffers[i] & 0x0000FF00;
        led_override_values[i] |= (gpio_data_buffers[i] & 0x000000FF) << 16;
        led_override_values[i] |= (gpio_data_buffers[i] & 0x00FF0000) >> 16;
    }
    bool leds_latch = (gpio_data_buffers[17] & 8) != 0;
    if(leds_latch && !leds_latch_edge) {
        EFLed.ledsShow();

        pref2.begin(PREF_SPACE, false);
        pref2.putUInt("ledBrightPcent", EFLed.getBrightnessPercent());
        pref2.end();
    }
    leds_latch_edge = leds_latch;
    bool watchdog_en = (gpio_data_buffers[17] & 16) != 0;
    bool watchdog_clr = (gpio_data_buffers[17] & 32) != 0;
    if((watchdog_en && !watchdog_enabled) || (watchdog_clr && !watchdog_clr_edge)) {
        watchdog_clr_edge = false;
        last_watchdog_time = millis();
    }
    watchdog_enabled = watchdog_en;
    watchdog_clr_edge = watchdog_clr;
}

uint32_t gpioRead(uint8_t gpio_idx, uint8_t reg) {
    if(gpio_idx >= 18) return 0;
    if(reg == 0) {
        if(gpio_idx == 17) {
            //Only gpio with inputs
            //Bit 0 is always 1
            return (1 | (EFTouch.isFingerprintTouched() ? 2 : 0) | (EFTouch.isNoseTouched() ? 4 : 0)) << 24;
        }
        return 0;
    }
    uint32_t res = 0;
    if(reg == 0x04) res = gpio_data_buffers[gpio_idx];
    if(reg == 0x08) res = gpio_dir_buffers[gpio_idx];
    if(reg == 0x1C) return 0b0000000000000011000; //Capabilities register - no special capabilities

    //Linux probably expects the read values to be rotated as well
    uint32_t res2 = res << 8;
    res2 |= res >> 24;

    return res2;
}

//Stubs - Implement these if you need an interactive shell
bool uartHasByte() {
    #ifdef TELNET_SHELL
    return telnet_in_rptr != telnet_in_wptr;
    #else
    return LOG_DEV_SERIAL.available() != 0;
    #endif
}

char uartGetNextByte() {
    #ifdef TELNET_SHELL
    if(telnet_in_rptr == telnet_in_wptr) return 0;
    uint8_t res = telnet_in_buffer[telnet_in_rptr];
    telnet_in_rptr = 0xFF & (telnet_in_rptr + 1);
    return res;
    #else
    return LOG_DEV_SERIAL.read();
    #endif
}

static uint32_t HandleException(uint32_t ir, uint32_t code) {
    if(code != 3 && code != 9) LOG_ERROR("mini-rv32ima UNKNOWN EXCEPTION");
    return code;
}

#ifdef TELNET_SHELL
void telnet_buff_flush() {
    telnet_out_buffer[telnet_out_wptr] = 0;
    if(telnet.isConnected()) telnet.write(telnet_out_buffer, telnet_out_wptr);
    telnet_out_wptr = 0;
}
#endif

void serial_putchar(uint8_t c) {
    putchar(c);
    #ifdef TELNET_SHELL
    if(telnet.isConnected()) {
        telnet_out_buffer[telnet_out_wptr++] = c;
        if(c == '\r' || c == '\n' || telnet_out_wptr == 255) telnet_buff_flush();
    }
    #endif
    //Very short delay
    delayMicroseconds(1);
}

static uint32_t HandleControlStore(uint32_t addy, uint32_t val) {
    if(addy == 0x10000000) {
        //UART 8250 / 16550 Data Buffer
        serial_putchar((uint8_t)val);
    }else if(addy == 0x11004004) { //CLNT
		core->timermatchh = val;
    }else if(addy == 0x11004000) { //CLNT
		core->timermatchl = val;
    }else if(addy == 0x11100000) { //SYSCON (reboot, poweroff, etc.)
		core->pc = core->pc + 4;
		return val; // NOTE: PC will be PC of Syscon.
    }else if(addy >= 0x11400000 && addy < 0x11400900) gpioWrite(addy >> 7, addy & 0x7F, val);
	//else printf("WRITE %08x %08x\r\n", addy, val);
    return 0;
}

static uint32_t HandleControlLoad(uint32_t addy) {
    //Emulating a 8250 / 16550 UART
    if(addy == 0x10000005) {
        return 0x60 | (uartHasByte() ? 1 : 0);
    }else if(addy == 0x10000000) {
        //UART receive
        return uartHasByte() ? uartGetNextByte() : 0;
    }else if(addy == 0x1100bffc) { // https://chromitem-soc.readthedocs.io/en/latest/clint.html
		return core->timerh;
    }else if(addy == 0x1100bff8) {
		return core->timerl;
    //GPIO ports - each corresponds to an LED
    //Excepts the last, which is used to get inputs from the touch buttons
    }else if(addy >= 0x11400000 && addy < 0x11400900) return gpioRead(addy >> 7, addy & 0x7F);
    else if(addy >= 0x11500000 && addy < 0x11500800) {
        //RTC Chip emulation (DS1742)
        //Read-only, attempts to write to these addresses will fail
        uint16_t reg = addy & 0x7FF;
        if(reg < 0x7F8) return 0;
        reg -= 0x7F8;

        struct tm ts;
        #ifdef ONLINE_LINUX
        if(!getLocalTime(&ts)) {
            LOG_WARNING("getLocalTime failed, RTC will not work");
            return 0;
        }
        #else
        //Use fixed time if we’re not connected to the internet
        ts.tm_hour = 18;
        ts.tm_min = 43;
        ts.tm_sec = 38;
        ts.tm_mon = 8;
        ts.tm_mday = 2;
        ts.tm_wday = 2;
        ts.tm_year = 2025 - 1900;
        #endif
        uint32_t year = ts.tm_year + 1900;
		switch(reg) {
			default:
				return 0;
			case 0:
				//W R 10Century Century
				year /= 100;
				return ((year/10)<<4)|(year%10)|64;
			case 1:
				//OSCn 10Seconds Seconds
				return ((ts.tm_sec/10)<<4)|(ts.tm_sec%10);
			case 2:
				//X 10Minutes Minutes
				return ((ts.tm_min/10)<<4)|(ts.tm_min%10);
			case 3:
				//XX 10Hour Hour
				return ((ts.tm_hour/10)<<4)|(ts.tm_hour%10);
			case 4:
				//BF FT X X X Day
				return ((ts.tm_wday/10)<<4)|(ts.tm_wday%10)|128;
			case 5:
				//X X 10Date Date
				return ((ts.tm_mday/10)<<4)|(ts.tm_mday%10);
			case 6:
				//X X X 10Month Month
				return (((ts.tm_mon+1)/10)<<4)|((ts.tm_mon+1)%10);
			case 7:
				//10Year Year
				year %= 100;
				return ((year/10)<<4)|(year%10);
		}
    }
    //else printf("READ %08x\r\n", addy);
    return 0;
}

static void HandleOtherCSRWrite(uint8_t * image, uint16_t csrno, uint32_t value) {
    if(csrno == 0x136) {
        printf("%ld", value);
    }
    if(csrno == 0x137) {
        printf("%08lx", value);
    }
    if(csrno == 0x138) {
        //Print "string"
        uint32_t ptrstart = value - MINIRV32_RAM_IMAGE_OFFSET;
        uint32_t ptrend = ptrstart;
        while(ptrend < RAM_SIZE) {
            if(load1(ptrend) == 0) break;
            ptrend++;
        }
        if(ptrend != ptrstart) {
            for(; ptrstart <= ptrend; ptrstart++) {
                serial_putchar(load1(ptrstart));
            }
        }
    }
    if(csrno == 0x139) {
        serial_putchar((uint8_t)value);
    }
}

static int32_t HandleOtherCSRRead(uint8_t * image, uint16_t csrno) {
    if(csrno == 0x140) {
        //UART receive
        return uartHasByte() ? uartGetNextByte() : -1;
    }
    return 0;
}

#ifdef IMPLEMENT_CACHE

//Flush a random cache entry to RAM and replace it with data from a different location in RAM
uint8_t IRAM_ATTR swap_cache_entry(uint32_t ofs) {
    //Evicting a random entry turned out to be the most performant cache replacement policy
    //Trust me, I tested a LOT
    uint8_t random_entry = (uint8_t)(xorshift32() >> 7) % CACHE_ENTRIES;
    cache_entry* e = c_entries + random_entry;
    if(e->dirty) {
        uint16_t lowest = e->lowest;
        uint16_t len = e->highest - lowest + 1;
        if(e->ofs < 8*1024) first_chip.write_alligned((e->ofs << 10) + lowest, e->data + lowest, len);
        else second_chip.write_alligned(((e->ofs - 8*1024) << 10) + lowest, e->data + lowest, len);
    }
    e->ofs = ofs;
    e->dirty = false;
    e->highest = 0;
    e->lowest = 0xFFFF;
    if(ofs < 8*1024) first_chip.read_alligned(ofs << 10, e->data, 1024);
    else second_chip.read_alligned((ofs - 8*1024) << 10, e->data, 1024);
    return random_entry;
}

uint8_t IRAM_ATTR find_cache_entry(uint32_t block_addr) {
    uint8_t c_h = 0;
    if(c_entries[recent_entry].ofs == block_addr) return recent_entry;
    if(c_entries[second_recent_entry].ofs == block_addr) return second_recent_entry;
    for(;c_h < CACHE_ENTRIES; c_h++) {
        //Funni partial loop unroll for better performance
        if(c_entries[c_h].ofs == block_addr) break;
        c_h++;
        if(c_entries[c_h].ofs == block_addr) break;
        c_h++;
        if(c_entries[c_h].ofs == block_addr) break;
        c_h++;
        if(c_entries[c_h].ofs == block_addr) break;
    }
    if(c_h >= CACHE_ENTRIES) c_h = swap_cache_entry(block_addr);
    second_recent_entry = recent_entry;
    recent_entry = c_h;
    return c_h;
}

uint32_t IRAM_ATTR cached_load(uint32_t ofs) {
    uint32_t block_addr = ofs >> 10;
    uint8_t c_h = find_cache_entry(block_addr);
    cache_entry* e = c_entries + c_h;
    uint16_t in_block_addr = ofs & 0x3FF;
    //Don’t actually need to differentiate between access length
    //The calling function will only use the bytes it needs
    //if(len == 0) (uint32_t)e->data[in_block_addr];
    //if(len == 1) (uint32_t)*((uint16_t *)(e->data + in_block_addr));
    return *((uint32_t *)(e->data + in_block_addr));
}

void IRAM_ATTR cached_store(uint32_t ofs, uint32_t value, uint8_t len) {
    uint32_t block_addr = ofs >> 10;
    uint8_t c_h = find_cache_entry(block_addr);
    cache_entry* e = c_entries + c_h;
    uint16_t in_block_addr = ofs & 0x3FF;
    e->dirty = true;
    if(in_block_addr < e->lowest) e->lowest = in_block_addr;
    switch(len) {
        case 0:
            e->data[in_block_addr] = (uint8_t)value;
            break;
        case 1:
            *((uint16_t *)(e->data + in_block_addr)) = (uint16_t)value;
            in_block_addr++;
            break;
        case 2:
            *((uint32_t *)(e->data + in_block_addr)) = value;
            in_block_addr += 3;
            break;
    }
    if(in_block_addr > e->highest) e->highest = in_block_addr;
}
#endif

//Memory access functions

#ifdef IMPLEMENT_CACHE

//This instruction queue thing improves bootup speed by an amazing 2 seconds
//...I’m keeping it
uint32_t instr_queue[256];
uint32_t last_pc = 0xFFFF00FF;
uint32_t queue_start = 0xFFFFFFFF;

static IRAM_ATTR uint32_t loadi(uint32_t ofs) {
    if((ofs & 0xFFFFFC00) == queue_start) {
    }else {
        queue_start = ofs & 0xFFFFFC00;
        uint32_t block_addr = queue_start >> 10;
        uint8_t c_h = find_cache_entry(block_addr);
        uint16_t in_block_addr = queue_start & 0x3FF;
        memcpy(instr_queue, c_entries[c_h].data + in_block_addr, 256 * 4);
    }
    return instr_queue[(ofs & 0x3FF) >> 2];
}
#else
static IRAM_ATTR uint32_t loadi(uint32_t ofs) {
    return load4(ofs);
}
#endif

//This is a not-smart way of handling misalligned accesses, but they rarely happen anyways
static uint32_t load4_misalligned(uint32_t ofs) {
    uint32_t res = load1(ofs);
    res |= ((uint32_t)load1(ofs + 1)) << 8;
    res |= ((uint32_t)load1(ofs + 2)) << 16;
    res |= ((uint32_t)load1(ofs + 3)) << 24;
    return res;
}

static IRAM_ATTR uint32_t load4(uint32_t ofs) {
    if((ofs & 3) != 0) return load4_misalligned(ofs);
#ifdef IMPLEMENT_CACHE
    return cached_load(ofs);
#else
    uint32_t res;
    if(ofs < 8*1024*1024) first_chip.read_alligned(ofs, (uint8_t*)(&res), 4);
    else second_chip.read_alligned(ofs - 8*1024*1024, (uint8_t*)(&res), 4);
    return res;
#endif
}

static uint16_t load2_misalligned(uint32_t ofs) {
    uint16_t res = load1(ofs);
    res |= ((uint16_t)load1(ofs + 1)) << 8;
    return res;
}

static uint16_t load2(uint32_t ofs) {
    if((ofs & 1) != 0) return load2_misalligned(ofs);
#ifdef IMPLEMENT_CACHE
    return cached_load(ofs);
#else
    uint16_t res;
    if(ofs < 8*1024*1024) first_chip.read_alligned(ofs, (uint8_t*)(&res), 2);
    else second_chip.read_alligned(ofs - 8*1024*1024, (uint8_t*)(&res), 2);
    return res;
#endif
}

static inline uint8_t load1(uint32_t ofs) {
#ifdef IMPLEMENT_CACHE
    return (uint8_t)cached_load(ofs);
#else
    uint8_t res;
    if(ofs < 8*1024*1024) first_chip.read_alligned(ofs, &res, 1);
    else second_chip.read_alligned(ofs - 8*1024*1024, &res, 1);
    return res;
#endif
}

static void store4_misalligned(uint32_t ofs, uint32_t val) {
    store1(ofs, val);
    store1(ofs + 1, val >> 8);
    store1(ofs + 2, val >> 16);
    store1(ofs + 3, val >> 24);
}

static IRAM_ATTR uint32_t store4(uint32_t ofs, uint32_t val) {
    if((ofs & 3) != 0) store4_misalligned(ofs, val);
#ifdef IMPLEMENT_CACHE
    else cached_store(ofs, val, 2);
#else
    else {
        if(ofs < 8*1024*1024) first_chip.write_alligned(ofs, (uint8_t*)(&val), 4);
        else second_chip.write_alligned(ofs - 8*1024*1024, (uint8_t*)(&val), 4);
    }
#endif
    return val;
}

static void store2_misalligned(uint32_t ofs, uint16_t val) {
    store1(ofs, val);
    store1(ofs + 1, val >> 8);
}

static uint16_t store2(uint32_t ofs, uint16_t val) {
    if((ofs & 1) != 0) store2_misalligned(ofs, val);
#ifdef IMPLEMENT_CACHE
    else cached_store(ofs, val, 1);
#else
    else {
        if(ofs < 8*1024*1024) first_chip.write_alligned(ofs, (uint8_t*)(&val), 2);
        else second_chip.write_alligned(ofs - 8*1024*1024, (uint8_t*)(&val), 2);
    }
#endif
    return val;
}

static inline uint8_t store1(uint32_t ofs, uint8_t val) {
#ifdef IMPLEMENT_CACHE
    cached_store(ofs, val, 0);
#else
    if(ofs < 8*1024*1024) first_chip.write_alligned(ofs, &val, 1);
    else second_chip.write_alligned(ofs - 8*1024*1024, &val, 1);
#endif
    return val;
}

#ifdef TELNET_SHELL
void on_telnet_connect(String ip) {
    LOG_INFO("Telnet connection opened");
    telnet.println("You've connected to Tholin's Cyber LED Badge running Linux. Please be nice.");
    telnet.println("(Use ^] + q  to disconnect.)");
    telnet.println("");
    if(watchdog_enabled) telnet.print("~ # ");
}

void on_telnet_disconnect(String ip) {
    LOG_INFO("Telnet connection closed");
}

void on_telnet_reconnect(String ip) {

}

void on_telnet_connection_attempt(String ip) {
    
}

void on_telnet_input(String str) {
    //LOG_DEV_SERIAL.println(str);
    uint16_t refptr = 0xFF & (telnet_in_rptr - 1);
    for(int i = 0; i < str.length() + 1; i++) {
        if(telnet_in_wptr == refptr) {
            telnet.println("Error: Buffer overflow!");
            return;
        }
        telnet_in_buffer[telnet_in_wptr] = i == str.length() ? '\n' : str.charAt(i);
        telnet_in_wptr = (telnet_in_wptr + 1) & 0xFF;
    }
}
#endif