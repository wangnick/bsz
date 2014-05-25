// Runtime consumption counter for Attiny84A, nRF24L01+ and HMC5883L, developped using Atmel Studio.
// See http://s.wangnick.de/doku.php?id=betriebsstundenzaehler for target hardware.
// (C) Copyright 2014 Sebastian Wangnick.
// Usage under "CC Attribution-Noncommercial-Share Alike 3.0 Unported" as described in http://creativecommons.org/licenses/by-nc-sa/3.0/ is granted.
//
// ATtiny84A, nRF24L01P, HMC5883L pin mappings
//                                    +-\/-+
//                              VCC  1|o   |14  GND
//                            XTAL1  2|    |13  PA0 - HMC SDA <-> 2.2kOhm VCC
//                nRF XTAL1 - XTAL2  3|    |12  PA1 - HMC SCL <-> 2.2kOhm VCC
//                           !RESET  4|    |11  PA2 AIN1 <A-> DEBUG UART TX 115200, 22kOhm VCC, 27kOhm GND
//                   nRF VCC <- PB2  5|    |10  PA3 -> nRF CE
//                   nRF CSN <- PA7  6|    |9   PA4 USCK -> nRF SCK
//          nRF MISO -> PA6 MOSI, DI 7|    |8   PA5 MISO, DO -> nRF MOSI
//                                    +----+

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/boot.h>
#include <util/crc16.h>
#define F_CPU 8000000 // 8 MHz
#include <util/delay.h>

#include "i2cmaster.h"
#include "nRF24L01.h"
#include "ds.h"

char* p;

//#define DEBUG_UART
//#define DEBUG_IDLE
//#define DEBUG_ACO

#ifdef DEBUG_UART
//#define DEBUG_UART_RX
#define DEBUG_UART_TX
//#define DEBUG_UART_ACO
//#define DEBUG_UART_BSZ
#define DEBUG_UART_FLASH
//#define DEBUG_UART_HMC
//#define DEBUG_UART_HMC_ALL

char dbg[80];

void ser_writes (char* txt) {
    while (*txt) {
        ser_write((uint8_t)(*txt++));
    }
}

#define DEBUG_INIT 
#define DEBUG_IDLE 
#define DEBUG_ACTIVE
#define DEBUG_BLIP
#else
#define DEBUG_INIT DDRA |= 1<<PINA2
#define DEBUG_HIGH PORTA |= 1<<PINA2
#define DEBUG_LOW PORTA &= ~(1<<PINA2)
#ifdef DEBUG_IDLE
#undef DEBUG_IDLE
#define DEBUG_IDLE DEBUG_HIGH
#define DEBUG_ACTIVE DEBUG_LOW
#define DEBUG_BLIP DEBUG_IDLE;DEBUG_ACTIVE
#else
#define DEBUG_IDLE
#define DEBUG_ACTIVE
#define DEBUG_BLIP
#endif
#endif

#define hex(digit) ((digit)+((digit)>9?'A'-10:'0'))
char* hex8 (char* p, uint8_t val) {
    *p++ = hex(val>>4);
    *p++ = hex(val&0xF);
    *p = 0;
    return p;
}
char* hex16 (char* p, uint16_t val) {
    p = hex8(p,val>>8);
    p = hex8(p,val&0xFF);
    return p;
}

char* text (char* p, char* txt) {
    while (*txt) {
        *p++ = *txt++;
    }
    *p = 0;
    return p;
}

char* strend (char* p) {
    while (*p) p++;
    return p;
}

void sleep_us (uint8_t us) {
 	// Activate Timer 0 clock, prescaler 1/8 -> 1us/1MHz @ 8MHz
 	TCNT0 = 0;
    TCCR0A = 1<<WGM01; // CTC
 	TCCR0B = 1<<CS01;
    TIMSK0 = 1<<OCIE0A;
    OCR0A = us-4;
    TCNT0 = 0;
    uint8_t didr0 = DIDR0;
    DIDR0 = 0xFF;
    DEBUG_IDLE;
    sleep_mode(); // Sleep until TIMER0 overflows
    DEBUG_ACTIVE;
    DIDR0 = didr0;
    TIMSK0 = 0;
}

void sleep_ms (uint16_t ms) {
    while (ms--) {
        sleep_us(250);
        sleep_us(250);
        sleep_us(250);
        sleep_us(250);
    }
}


#define HMC5883L 0x3C
enum HMC5883L_REGISTERS {
    HMC_CRA, HMC_CRB, HMC_MODE, HMC_XH, HMC_XL, HMC_YH, HMC_YL, HMC_ZH, HMC_ZL, HMC_STATUS, HMC_ID1, HMC_ID2, HMC_ID3
};
enum HMC5883L_CRA {
    HMC_DO0_75=0, HMC_DO1_5=1<<2, HMC_DO3=2<<2, HMC_DO7_5=3<<2, HMC_DO15=4<<2, HMC_DO30=5<<2, HMC_DO75=6<<2
};
enum HMC5883L_CRB {
    HMC_GN1370=0, HMC_GN1090=1<<5, HMC_GN820=2<<5, HMC_GN660=3<<5, HMC_GN440=4<<5, HMC_GN390=5<<5, HMC_GN330=6<<5, HMC_GN230=7<<5
};
enum HMC5883L_STATUS {
    HMC_RDY=1, HMC_LOCK=2
};
enum HMC5883L_MODE {
    HMC_CONT, HMC_SINGLE, HMC_IDLE1, HMC_IDLE2
};

uint8_t hmc_inited = 0;
uint8_t hmc_startup = 1;
void hmc_init (void) {
    uint8_t id[3], err;
    hmc_inited = 0;
    err = i2c_start(HMC5883L | I2C_WRITE);
    err |= i2c_write(HMC_ID1); // ID: 'H' '4' '3'
    err |= i2c_rep_start(HMC5883L | I2C_READ);
    id[0] = i2c_readAck();
    id[1] = i2c_readAck();
    id[2] = i2c_readNak();
    i2c_stop();
    err = err || id[0]!='H' || id[1]!='4' || id[2]!='3';
    if (err) return;
    err = i2c_start(HMC5883L | I2C_WRITE);
    err |= i2c_write(HMC_CRA);
    err |= i2c_write(HMC_DO75); // 75Hz (maximum), no averaging.
    err |= i2c_rep_start(HMC5883L | I2C_WRITE);
    err |= i2c_write(HMC_CRB);
    err |= i2c_write(HMC_GN440); // 440units/Gauss, 2.27mGauss/unit
    i2c_stop();
    if (err) return;
    hmc_inited = 1;
}

#define NOMEASURE 0xFFFF

uint16_t hmc_measure (void) {
    int16_t valx,valy,valz;
    uint16_t measure;
    uint8_t err, status;
    
    if (!hmc_inited) return NOMEASURE;
    err = i2c_start(HMC5883L | I2C_WRITE); // 3 bytes, 27 bit @ 400kHz: 68us
    err |= i2c_write(HMC_MODE);
    err |= i2c_write(HMC_SINGLE);
    i2c_stop();
    // This time interval seems sufficient to perform one measurement
    err |= i2c_start(HMC5883L | I2C_WRITE); // 13 bytes, 117 bit: 292us
    err |= i2c_write(HMC_STATUS);
    err |= i2c_rep_start(HMC5883L | I2C_READ);
    status = i2c_readNak();
    err |= i2c_rep_start(HMC5883L | I2C_WRITE);
    err |= i2c_write(HMC_XH);
    err |= i2c_rep_start(HMC5883L | I2C_READ);
    valx = i2c_readAck()<<8;
    valx |= i2c_readAck(); // 0xF800–0x07FF
    valy = i2c_readAck()<<8;
    valy |= i2c_readAck();
    valz = i2c_readAck()<<8;
    valz |= i2c_readNak();
    i2c_stop();
    if (err || !(status & HMC_RDY)) return NOMEASURE;
    if (hmc_startup) {hmc_startup=0; return NOMEASURE;} // Have to ignore first measurement after gain change
#ifdef DEBUG_UART_HMC_ALL
    p = hex16(dbg,valx);
    p = hex16(p,valy);
    p = hex16(p,valz);
    p = text(p,"\n");
    ser_writes(dbg);
#endif
    measure = 0;
    if (valx<0) measure-=valx; else measure+=valx;
    if (valy<0) measure-=valy; else measure+=valy;
    if (valz<0) measure-=valz; else measure+=valz;
    return measure;
}

void spi_init(void) {
    USCK_DDR |= (1<<USCK_BIT); // SPI CLK
    DO_DDR |= (1<<DO_BIT); // SPI DO
    DI_DDR &= ~(1<<DI_BIT); // SPI DI
    DI_PORT |= (1<<DI_BIT); // SPI DI pullup
}

uint8_t spi_transfer (uint8_t data) {
    USIDR = data;
    USISR = 1<<USIOIF; // Clear completion flag
    while (!(USISR & 1<<USIOIF)) {
        USICR = 1<<USIWM0 | 1<<USICS1 | 1<<USICLK | 1<<USITC;
    }
    return USIDR;
}

void spi_write (uint8_t* buffer, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        spi_transfer(buffer[i]);
    }
}

void spi_read (uint8_t* buffer, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        buffer[i] = spi_transfer(0xFF);
    }
}

#define MIRFVCC (1<<PINB2)
#define CSN (1<<PINA7)
#define CE (1<<PINA3)
#define CE_HI PORTA |= CE
#define CE_LO PORTA &= ~CE
#define CSN_HI PORTA |= CSN
#define CSN_LO PORTA &= ~CSN

uint8_t payload;
uint8_t config;
enum {PWRDOWN, PWRUPRX, PWRUPTX, UNKNOWN} pmode = UNKNOWN;

uint8_t mirf_exec (uint8_t command) {
    CSN_LO;
    uint8_t status = spi_transfer(command);
    CSN_HI;
    return status;
}

uint8_t mirf_read_register (uint8_t reg) {
    CSN_LO;
    spi_transfer(R_REGISTER | (REGISTER_MASK & reg));
    uint8_t value = spi_transfer(NOP);
    CSN_HI;
    return value;
}

uint8_t mirf_write_register (uint8_t reg, uint8_t data) {
    CSN_LO;
    uint8_t status = spi_transfer(W_REGISTER | (REGISTER_MASK & reg));
    spi_transfer(data);
    CSN_HI;
    return status;
}

void mirf_read_registers (uint8_t reg, uint8_t *data, uint8_t len) {
    CSN_LO;
    spi_transfer(R_REGISTER | (REGISTER_MASK & reg));
    spi_read(data, len);
    CSN_HI;
}

void mirf_write_registers (uint8_t reg, uint8_t *data, uint8_t len) {
    CSN_LO;
    spi_transfer(W_REGISTER | (REGISTER_MASK & reg));
    spi_write(data, len);
    CSN_HI;
}

void mirf_powerup_tx (uint8_t nodelay) {
    if (pmode!=PWRUPTX) {
        mirf_write_register(CONFIG, (config & ~(1<<PRIM_RX)) | 1<<PWR_UP);
        if (pmode==PWRDOWN && !nodelay) {
            sleep_us(150); // Tpd2stby with external clock = 150us. Time required before CE high is allowed.
        }        
        pmode = PWRUPTX;
    }    
}

void mirf_powerup_rx (void) {
    if (pmode!=PWRUPRX) {
        mirf_write_register(CONFIG, config | 1<<PRIM_RX | 1<<PWR_UP);
        if (pmode==PWRDOWN) {
            sleep_us(150); // Tpd2stby with external clock = 150us. Time required before CE high is allowed.
        }
        pmode = PWRUPRX;
    }
}

void mirf_powerdown (void) {
    if (pmode!=PWRDOWN) {
        mirf_write_register(CONFIG, config & ~(1<<PWR_UP));
        pmode = PWRDOWN;
    }    
}
#define ENAA 0x00

// Reads payload bytes
void mirf_read (uint8_t *data) {
    CSN_LO;
    spi_transfer(R_RX_PAYLOAD);
    spi_read(data, payload);
    CSN_HI;
    mirf_write_register(STATUS, 1<<RX_DR); // Reset status register
}

// Emits payload bytes. Returns true if TX_DS is set after sending, which should always be the case if ack is false,
// and when ack is true is set when a proper ACK has been received from the receiver. Set ack to 2 to await reply.
#define AUTO_ACK 1
#define REPLY_ACK 2
uint8_t txcycle, rxcycle;
uint8_t mirf_write (uint8_t *data, uint8_t ack) {
#ifdef DEBUG_UART_TX
    p = text(dbg,"mirf_write ");
    p = text(p,(char*)data);
    p = text(p,"\n");
    ser_writes((char*)dbg);
#endif

    mirf_powerup_tx(1); // We need no 130us delay

    CSN_LO; // The transfer of 32 bytes of payload plus the TX_PAYLOAD write command takes 225us (measured), so no additional delay needed
    spi_transfer(ENAA==0 || (ack&AUTO_ACK)? W_TX_PAYLOAD: W_TX_PAYLOAD_NOACK);
    spi_write(data, payload);
    CSN_HI;

    CE_HI; // Start transmission
    sleep_us(15);
    CE_LO;

    // PLL lock time: 130us from CE high
    // Time on air: 1 byte preamble + 5 byte address + 9 bit pcf + 32 bytes payload + 2 bytes crc @ 1Mbps = 385us
    // Time on air: @ 2Mbps = 192us, @250kbps = 1540us
    // Total TX time: 322/515/1670us
    // If ack, RX time is added, making: 644/1030/3340us
    // Worst case: TX+(ARD+TX)*ARC+ARD+RX
    // ARD (250us-4000us, default 250us), ARC (0-15, default 3, in our case 1)
    // ARD 250us, ARC 3, worst case times @ 1Mbps: 3575 -> 29*125us
    // ARD 250us, ARC 1, worst case times @ 1Mbps: 2045 -> 17*125us
    // Time IRQ = 8.2us @ 1Mbps, 6us @ 2Mbps
    // Measured values ARD 250us, ARC/maxcycle until MAX_RT: 0/5, 1/10, 2/15, 3/20, 4/25,

    uint8_t txstatus, rxstatus = 0, maxcycle = ENAA&&(ack&AUTO_ACK)?32:8; // Wait 32=4ms in the auto-ack case, 1ms in the fire-and-forget case.
    for (txcycle=0; txcycle<maxcycle; txcycle++) {
        if (mirf_read_register(STATUS) & (1<<TX_DS|1<<MAX_RT)) break;
        sleep_us(125);
    }
    mirf_powerdown();
    txstatus = mirf_write_register(STATUS, 1<<TX_DS|1<<MAX_RT|1<<RX_DR);
    rxcycle = 0xFF;
    if (txstatus!=0xFF) {
        if (!(txstatus & (1<<TX_DS))) {
            mirf_exec(FLUSH_TX);
        }
        if (txstatus & (1<<RX_DR)) {
            mirf_read(data);
            mirf_exec(FLUSH_RX);
        }
        if ((ack&REPLY_ACK) && (ENAA==0 || (txstatus & (1<<TX_DS)))) {
            mirf_powerup_rx(); // This will take 150us due to the mirf_powerdown above ...
            CE_HI;
            // Wait 20=2.5ms for a reply so as not to drain our power supply. This allows an Arduino sender 3 tries (6, 12, 18).
            for (rxcycle=0; rxcycle<20; rxcycle++) { 
                if (mirf_read_register(STATUS) & (1<<RX_DR)) break;
                sleep_us(125);
            }
            CE_LO;
            mirf_powerdown();
            rxstatus = mirf_write_register(STATUS, 1<<RX_DR);
            if (rxstatus & (1<<RX_DR)) {
                mirf_read(data);
                mirf_exec(FLUSH_RX);
            } else {
                rxcycle = 0xFF;
            }
        }
    }    
#ifdef DEBUG_UART_TX
    p = text(dbg,"tx status ");
    p = hex8(p,txstatus);
    p = text(p,", cycle ");
    p = hex8(p,txcycle);
    p = text(p,", OBSERVE_TX ");
    p = hex8(p,mirf_read_register(OBSERVE_TX));
    p = text(p,", FIFO_STATUS ");
    p = hex8(p,mirf_read_register(FIFO_STATUS));
    p = text(p,"\n");
    ser_writes((char*)dbg);
#endif
#ifdef DEBUG_UART_RX
    if (rxcycle!=0xFF) {
        p = text(dbg,"rx status ");
        p = hex8(p,rxstatus);
        p = text(p,", cycle ");
        p = hex8(p,rxcycle);
        p = text(p,", FIFO_STATUS ");
        p = hex8(p,mirf_read_register(FIFO_STATUS));
        if (rxstatus & (1<<RX_DR)) {
            p = text(p,", response ");
            p = text(p,(char*)data);
        }
        p = text(p,"\n");
        ser_writes((char*)dbg);
   }    
 #endif
    return txstatus | (rxstatus & (1<<RX_DR));
}

#define CHANNEL			    83
#define PAYLOAD			    32
#define ADDRLEN             5
#define SADDR				(uint8_t *)"bsz1S"
#define CADDR				(uint8_t *)"bsz1C"
uint8_t sendbuf[PAYLOAD];

void mirf_config () {
    payload = PAYLOAD;
    mirf_write_register(RX_PW_P0, payload);
    mirf_write_register(RX_PW_P1, payload);
    config = 1<<EN_CRC|1<<CRCO;
    mirf_write_register(CONFIG, config);
    mirf_write_register(RF_SETUP,1<<RF_PWR_LOW|1<<RF_PWR_HIGH|1<<RF_DR_HIGH); // 0db, 2Mbps
    //mirf_write_register(FEATURE,1<<EN_DYN_ACK);
    mirf_write_register(RF_CH, CHANNEL);
    mirf_write_register(SETUP_AW, ADDRLEN-2);
    mirf_write_registers(TX_ADDR, SADDR, ADDRLEN);
    mirf_write_registers(RX_ADDR_P0, SADDR, ADDRLEN);
    mirf_write_registers(RX_ADDR_P1, CADDR, ADDRLEN);
    mirf_write_register(EN_RXADDR, 1<<ERX_P0|1<<ERX_P1);
    mirf_write_register(SETUP_RETR,0<<ARD|0<<ARC); // 250us ARD, 3 retries. Setting this (to whatever) seems to be mandatory to be able to send at all.
    mirf_write_register(EN_AA,ENAA);
    mirf_write_register(DYNPD,0);
    mirf_write_register(STATUS,1<<RX_DR|1<<TX_DS|1<<MAX_RT);
    mirf_exec(FLUSH_RX);
    mirf_exec(FLUSH_TX);
}

void mirf_startup (uint8_t phase) {
    if (phase==0 || phase==1) {
        DDRA |= CSN|CE;
        DDRB |= MIRFVCC;
        spi_init();
        CE_LO;
        CSN_HI;
        PORTB |= MIRFVCC;
    }
    if (phase==0) {    
        sleep_ms(120);
    }
    if (phase==0 || phase==2) { 
        pmode = PWRDOWN;
        mirf_config();
    }    
}

void mirf_shutdown () {
    DDRA &= ~(CSN|CE|1<<USCK_BIT|1<<DO_BIT|1<<DI_BIT);
    PORTA &= ~(CSN|CE|1<<USCK_BIT|1<<DO_BIT|1<<DI_BIT);
    DDRB &= ~MIRFVCC;
    PORTB &= ~MIRFVCC;
}

void send () {
    static uint8_t lasttxcycle=35, lastrxcycle=35;
    static uint8_t lastbuf[PAYLOAD];
    p = (char*)sendbuf; while (*p) p++;
    // p = text(p,"O");
    // p = hex8(p,mirf_read_register(OBSERVE_TX));
    // p = text(p,"I");
    // p = hex8(p,mirf_read_register(FIFO_STATUS));
    p = text(p,"*");
    *p++ = lasttxcycle>10?lasttxcycle-10+'a':lasttxcycle+'0';
    *p++ = lastrxcycle>10?lastrxcycle-10+'a':lastrxcycle+'0';
    *p = 0;
    lastbuf[6] = 0;
    p = text(p,(char*)lastbuf);
	mirf_write(sendbuf,AUTO_ACK|REPLY_ACK);
    lasttxcycle = txcycle;
    if (rxcycle!=0xFF) {
        for (uint8_t i=0; i<PAYLOAD; i++) {
            lastbuf[i] = sendbuf[i];
        }
        lastrxcycle = rxcycle;
    } else {
        lastbuf[0] = 0;
        lastrxcycle = 35;
    }
}

#define VCCPIN (1<<PINA2)

uint8_t vcc (void) {
    ADMUX = 1<<MUX5 | 1<<MUX0; // AREF=VCC, ADMUX=IREF(1.1V)
    ADCSRB = 1<<ADLAR; // Left-adjust result (highest 8 bits in ADCH)
    ADCSRA = 1<<ADEN|1<<ADSC|1<<ADPS2; // Enable, start conversion, prescaler 16->1MHz - very rough measurement only
    while (ADCSRA & (1<<ADSC)) sleep_us(10);
    uint8_t result = ADCH;
    ADCSRA = 0;
    return result; // As from 147 and above (1.15/1.95*256) the BOD might trigger!
}

#define BSZBUCKETS 10
#define BSZSIZE BSZBUCKETS+1
#define BSZWRITTEN BSZBUCKETS
#define BSZPAGES 16
#define FLASHBSZGOOD 0x5A3C
typedef struct {
    uint32_t data[BSZSIZE];
    uint16_t crc;
    uint16_t good;
    uint8_t pad[SPM_PAGESIZE-4*BSZBUCKETS-4-2-2];
} Flashbszpage;
Flashbszpage flashbsz[BSZPAGES] __attribute__ ((section (".flashbszdata")));
Flashbszpage bsz;
uint8_t bszpage;

uint16_t eemyid;

void checkvcclow () {
    uint8_t ddr = AIN1_DDR;
    uint8_t port = AIN1_PORT;
    AIN1_DDR &= ~(1<<AIN1_BIT);
    AIN1_PORT &= ~(1<<AIN1_BIT);
    asm("NOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP"); // Give the pin 3us to stabilize
    asm("NOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP");
    asm("NOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP\n\tNOP");
    uint8_t aco = ACSR&(1<<ACO);
    AIN1_PORT = port;
    AIN1_DDR = ddr;
    if (!aco) return;

    // We are suffering power loss:
    //      Reduce consumption to the max.
    //      Save utilisation data to eeprom
    //      Wait for brown-out
    // If rather then power recovers, bail out and continue (TODO: recording such low power period in eeprom data later as well)

#ifdef DEBUG_UART_ACO
    p = text(dbg,"aco\n");        
    ser_writes(dbg);
#endif
#ifdef DEBUG_ACO
    DEBUG_HIGH;
#endif

    mirf_shutdown();
    i2c_start(HMC5883L | I2C_WRITE); // 3 bytes, 27 bit @ 100kHz: 272us
    i2c_write(HMC_MODE);
    i2c_write(HMC_IDLE1); // In IDLE mode HMC5883L should consume only 2uA
    i2c_stop();
    PRR |= 1<<PRTIM0|1<<PRTIM1|1<<PRUSI|1<<PRADC; // We still need Timer 0
    ACSR |= 1<<ACD; // Disable analog comparator
    DIDR0 = 0xFF; // Disable all digital input buffers

    uint8_t idx, *byte = (uint8_t*)&bsz.data;
    bsz.crc = 0xFFFF;
    for (idx=0; idx<sizeof(bsz.data); idx++) {
        bsz.crc = _crc16_update(bsz.crc,byte[idx]);
    }
    byte = (uint8_t*)&bsz;
    for (idx=0; idx<sizeof(bsz); idx+=2) {
        boot_page_fill(idx,byte[idx+1]<<8|byte[idx]); // low byte is written first, then high byte as second
    }
    // Store the page
    CLKPR = 1<<CLKPCE;
    //CLKPR = 1<<CLKPS2; // Reduce speed to 1MHz, consumption should drop to 300uA/45uA (active/idle 2.7V)
    CLKPR = 1<<CLKPS2|1<<CLKPS0; // Reduce speed to 500kHz, consumption should drop to 160uA/24uA (active/idle 2.7V)

#ifdef DEBUG_ACO
    DEBUG_LOW;
#endif
    boot_page_write(&flashbsz[bszpage]);   // Store buffer in flash page. This also erases the temporary buffer again.
#ifdef DEBUG_ACO
    DEBUG_HIGH;
#endif
    boot_spm_busy_wait();
#ifdef DEBUG_ACO
    DEBUG_LOW;
#endif

    // Now consume the remaining capacity and go into brown-out
    CLKPR = 1<<CLKPCE;
    CLKPR = 1<<CLKPS1|1<<CLKPS0; // Increase speed back to 4MHz (8MHz is officially safe only at >2.475V)
    ACSR &= ~(1<<ACD); // Re-enable analog comparator

#ifdef DEBUG_UART_ACO
    p = text(dbg,"eep\n");
    ser_writes(dbg);
#endif

    while (1) {
        AIN1_PORT = port;
        AIN1_DDR = ddr;
        port ^= 1<<AIN1_BIT;
        asm("NOP\n\tNOP\n\tNOP\n\tNOP"); // Wait 3us
        asm("NOP\n\tNOP\n\tNOP\n\tNOP");
        asm("NOP\n\tNOP\n\tNOP\n\tNOP");
        AIN1_DDR &= ~(1<<AIN1_BIT);
        AIN1_PORT &= ~(1<<AIN1_BIT);
        asm("NOP\n\tNOP\n\tNOP\n\tNOP"); // Give the pin 3us to stabilize
        asm("NOP\n\tNOP\n\tNOP\n\tNOP");
        asm("NOP\n\tNOP\n\tNOP\n\tNOP");
        aco = ACSR&(1<<ACO);
        if (!aco) break;
    }
        
    // Oops, we got power back. Perform a full WDT reset.
    wdt_enable(WDTO_15MS);
    while (1) continue;
}

EMPTY_INTERRUPT(TIM0_OVF_vect);
EMPTY_INTERRUPT(TIM0_COMPA_vect);

#define WAVEMILLIS 20
#define MILLISPERPOINT 2
#define WAVESPERSLICE 8
#define SLICEMILLIS (WAVEMILLIS*WAVESPERSLICE)
#define SLICESPERSEND 16
volatile uint8_t quartermillis = 0;
volatile uint8_t millis = 0;
volatile uint32_t uptimeslices = 0;

ISR(TIM1_COMPA_vect) {
    checkvcclow();
    quartermillis++;
    if (quartermillis>=4) {
        quartermillis = 0;
        millis++;
        if (millis>=SLICEMILLIS) {
            millis = 0;
            uptimeslices++;
        }        
    }
}

void bszsend (void) {
    static uint8_t bucket = 0;
    static uint16_t seqno = 0;
    p = hex16((char*)sendbuf,eemyid);                // 4c
    p = hex16(p,seqno++);               // 8c
    p = text(p,":");                    // 9c
    p = hex8(p,bucket);                 // 11c
    p = text(p,"=");                    // 12c
    p = hex16(p,bsz.data[bucket]>>16);   // 16c
    p = hex16(p,bsz.data[bucket]&0xFFFF); // 20c
    send();                             //30c
    bucket = (bucket+1)%BSZBUCKETS;
}

int main(void) {
#ifdef DEBUG_UART    
    uint8_t mcusr = MCUSR;
#endif
  	MCUSR = 0; // After watchdog-initiated reset, WDRF must be cleared in order to be able to clear WDE
  	wdt_disable();

    // Go up to 8MHz speed. This is safe down to 2.5V (and probably even down to 1.8V ...)
    CLKPR = 1<<CLKPCE;
    CLKPR = 1<<CLKPS0;

	ACSR = 1<<ACBG; // Analog comparison 1.1V versus AIN1.
	DIDR0 = 1<<ADC2D; // Disable digital input buffer for PA2/AIN1
    PRR = 0<<PRTIM0|0<<PRTIM1|0<<PRUSI|0<<PRADC; // We need them all

  	// Activate Timer 1 clock, prescaler 1, TOP=2000 -> @8MHz, OVF every 250us
  	TCCR1A = 0;
  	TCCR1B = 1<<WGM12|1<<CS10;
    OCR1A = 2000;
    TIMSK1 = 1<<OCIE1A;
    TCNT1 = 0;
    
    DEBUG_INIT;
    
    eemyid = eeprom_read_word((uint16_t*)(E2END+1-2));

#ifdef DEBUG_UART
    ser_init();
    p = text(dbg,"\nMCUSR=");
    p = hex8(p,mcusr);
    p = text(p,", MYID=");
    p = hex16(p,eemyid);
    p = text(p,", ");
    ser_writes(dbg);
    ser_writes(__FILE__ " " __DATE__ " " __TIME__ "\n");
#endif
    
    uint8_t idx, *byte;
    uint16_t crc,crccomp;
    uint32_t written, maxwritten = 0;
#ifdef DEBUG_UART_FLASH
    p = 0;
#endif
    for (uint8_t page=0; page<BSZPAGES; page++) {
#ifdef DEBUG_UART_FLASH
        if (p) {
            p = text(p,"\n");
            ser_writes(dbg);
        }
        p = text(dbg,"p");
        p = hex8(p,page);
#endif
        if (pgm_read_word(&flashbsz[page].good)!=FLASHBSZGOOD) continue;
        crc = 0xFFFF;
        byte = (uint8_t*)&flashbsz[page].data;
        for (idx=0; idx<sizeof(flashbsz[page].data); idx++) {
            crc = _crc16_update(crc,pgm_read_byte(byte+idx));
        }
        crccomp = pgm_read_word(&flashbsz[page].crc);
#ifdef DEBUG_UART_FLASH
        p = text(p,":");
        p = hex16(p,crc);
        p = hex16(p,crccomp);
#endif
        if (crc!=crccomp) continue;
        written = pgm_read_dword(&flashbsz[page].data[BSZWRITTEN]);
#ifdef DEBUG_UART_FLASH
        p = hex16(p,written);
#endif
        if (written<=maxwritten) continue;
        maxwritten = written;
        bszpage = page;
    }
    if (maxwritten) {
        for (idx=0; idx<BSZBUCKETS; idx++) {
            bsz.data[idx] = pgm_read_dword(&flashbsz[bszpage].data[idx]);
        }
        bszpage = (bszpage+1)%BSZPAGES;
    } else {
        for (idx=0; idx<BSZBUCKETS; idx++) {
            bsz.data[idx] = 0;
        }
        bszpage = 0;
    }
#ifdef DEBUG_UART_FLASH
    p = text(p,"\np=");
    p = hex8(p,bszpage);
    p = text(p,"\n");
    ser_writes(dbg);
#endif
    bsz.data[BSZWRITTEN] = maxwritten+1;
    bsz.good = FLASHBSZGOOD;
    // Erase next flash page already here
    boot_page_erase(&flashbsz[bszpage]); // On the Attiny this halts the CPU until completion.

    i2c_init();

    set_sleep_mode(SLEEP_MODE_IDLE);
    sei();

    uint16_t lastuptimeslice = 0xFFFF;
    uint8_t lastwave = 0xFF, lastpoint = 0xFF, sendstate = 0;
    uint16_t hmcmax = 0, hmcmin = 0, peaksum = 0;
    uint8_t hmccount = 0, peaksumcount = 0;
 	while(1) {
#ifdef DEBUG_UART_SLEEP
        ser_writes("s");
#endif
        DEBUG_IDLE;
        sleep_mode(); // Sleep for some time.
        DEBUG_ACTIVE;
        cli();
        uint8_t milli = millis;
        uint16_t uptimeslice = uptimeslices;
        sei();
        if (uptimeslice!=lastuptimeslice) {
            // We are in a new slice. 
            // TODO: Finish off the previous one (if any)
            lastuptimeslice = uptimeslice;
            lastwave = 0xFF;
            peaksum = 0;
            peaksumcount = 0;
            hmc_init();
            if ((uptimeslice%SLICESPERSEND)==0) {
                sendstate = 1;
                mirf_startup(1);
            } else {
                sendstate = 0;
            }            
        }
        uint8_t thiswave = milli/WAVEMILLIS;
        if (thiswave!=lastwave) {
            if (lastwave!=0xFF) {
                if (hmccount>WAVEMILLIS/MILLISPERPOINT*3/4) { // SHoUld have 10 measurements, accept as from 8
                    peaksum += hmcmax-hmcmin;
                    peaksumcount++;
                }
#ifdef DEBUG_UART_HMC
                p = hex8(dbg,lastwave);
                p = hex8(p,hmccount);
                p = hex16(p,hmcmin);
                p = hex16(p,hmcmax);
                p = hex16(p,peaksum);
                p = hex8(p,peaksumcount);
                p = text(p,"\n");
                ser_writes(dbg);
#endif
                lastwave = 0xFF;
            }
        }        
        if (thiswave<WAVESPERSLICE-1) {
            if (thiswave!=lastwave) {
                lastwave = thiswave;
                hmccount = 0;
                hmcmin = 0xFFFF;
                hmcmax = 0;
                lastpoint = 0xFF;
            }
            uint8_t thispoint = (milli%WAVEMILLIS)/MILLISPERPOINT;
            if (thispoint!=lastpoint) {
                lastpoint = thispoint;
                uint16_t hmc = hmc_measure();
#ifdef DEBUG_UART_HMC_ALL
                p = hex8(dbg,thiswave);
                p = hex8(p,thispoint);
                p = hex16(p,hmc);
                p = text(p,"\n");
                ser_writes(dbg);
#endif
                if (hmc!=NOMEASURE) {
                    if (hmc>hmcmax) hmcmax = hmc;
                    if (hmc<hmcmin) hmcmin = hmc;
                    hmccount++;
                }
            }
        } else {
            if (peaksumcount) {
                peaksum /= peaksumcount;
#ifdef DEBUG_UART_BSZ
                p = hex16(dbg,eemyid);
                p = hex16(p,uptimeslice);
                p = text(p,":");
                p = hex16(p,peaksum);
                p = hex8(p,peaksumcount);
                p = text(p,",");
#endif                
                peaksum >>= 4; // Cut off noise. peaksum is now in 36mGauss/unit. 
                uint8_t bucket = 0;
                while (peaksum) {
                    bucket++;
                    peaksum >>= 1;
                }
                if (bucket>=10) bucket = 9; // Does not happen, just for protection
                bsz.data[bucket]++;
#ifdef DEBUG_UART_BSZ
                p = hex8(p,bucket);
                p = text(p,"=");
                p = hex16(p,bsz.data[bucket]>>16);
                p = hex16(p,bsz.data[bucket]&0xFFFF);
                p = text(p,",");
                p = hex8(p,vcc());
                p = text(p,"\n");
                ser_writes(dbg);
#endif
                peaksumcount = 0;
            }
            if (sendstate==1) {
                mirf_startup(2);
                bszsend();
                mirf_shutdown();
                sendstate = 2;
            }
        }
        
#ifdef DEBUG_UART_SLEEP
        ser_writes("\n");
#endif
    }        
}
