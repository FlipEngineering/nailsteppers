/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#include "w5100.h"

#if defined(__arm__) && defined(TEENSYDUINO)
// #include "SPIFIFO.h"
// #ifdef  HAS_SPIFIFO
// #define USE_SPIFIFO
// #endif
#endif

#define W5500_2K_BUFFERS
// #define W5500_4K_BUFFERS
// #define W5500_16K_BUFFERS


#define W5200_2K_BUFFERS
// #define W5200_4K_BUFFERS
//#define W5200_8K_BUFFERS        //MAX_SOCK_NUM has to be 2
// #define W5200_16K_BUFFERS        //MAX_SOCK_NUM has to be 1

// If the core library defines a SS pin, use it as the
// default.  Otherwise, default the default to pin 10.
// #if defined(PIN_SPI_SS)
// #define SS_PIN_DEFAULT  PIN_SPI_SS
// #elif defined(CORE_SS0_PIN)
// #define SS_PIN_DEFAULT  CORE_SS0_PIN
// #else
// #define SS_PIN_DEFAULT  10
// #endif

#define SS_PIN_DEFAULT  31//10


// W5100 controller instance
uint16_t W5100Class::SBASE[MAX_SOCK_NUM];
uint16_t W5100Class::RBASE[MAX_SOCK_NUM];
uint16_t W5100Class::CH_BASE;
uint16_t W5100Class::SSIZE;
uint16_t W5100Class::SMASK;
uint8_t  W5100Class::chip;
uint8_t  W5100Class::ss_pin = SS_PIN_DEFAULT;
W5100Class W5100;

// pointers and bitmasks for optimized SS pin
#if defined(__AVR__)
  volatile uint8_t * W5100Class::ss_pin_reg;
  uint8_t W5100Class::ss_pin_mask;
#elif defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK66FX1M0__) || defined(__MK64FX512__)
  volatile uint8_t * W5100Class::ss_pin_reg;
#elif defined(__MKL26Z64__)
  volatile uint8_t * W5100Class::ss_pin_reg;
  uint8_t W5100Class::ss_pin_mask;
#elif defined(__SAM3X8E__) || defined(__SAM3A8C__) || defined(__SAM3A4C__)
  volatile uint32_t * W5100Class::ss_pin_reg;
  uint32_t W5100Class::ss_pin_mask;
#elif defined(__PIC32MX__)
  volatile uint32_t * W5100Class::ss_pin_reg;
  uint32_t W5100Class::ss_pin_mask;
#elif defined(ARDUINO_ARCH_ESP8266)
  volatile uint32_t * W5100Class::ss_pin_reg;
  uint32_t W5100Class::ss_pin_mask;
#elif defined(__SAMD21G18A__)
  volatile uint32_t * W5100Class::ss_pin_reg;
  uint32_t W5100Class::ss_pin_mask;
#endif


uint8_t W5100Class::init(void)
{
	uint16_t TXBUF_BASE, RXBUF_BASE;
	uint8_t i;
  // SPIprt = new SPIClass(SPI);
  // ss_pin = 10;
	// Many Ethernet shields have a CAT811 or similar reset chip
	// connected to W5100 or W5200 chips.  The W5200 will not work at
	// all, and may even drive its MISO pin, until given an active low
	// reset pulse!  The CAT811 has a 240 ms typical pulse length, and
	// a 400 ms worst case maximum pulse length.  MAX811 has a worst
	// case maximum 560 ms pulse length.  This delay is meant to wait
	// until the reset pulse is ended.  If your hardware has a shorter
	// reset time, this can be edited or removed.
	delay(560);
	//Serial.println("w5100 init");

#ifdef USE_SPIFIFO
	SPI1.begin();
	SPIFIFO1.begin(ss_pin, SPI_CLOCK_24MHz);  // W5100 is 14 MHz max
#else
	SPI1.begin();
	initSS();
	resetSS();
#endif
	SPI1.beginTransaction(SPI_ETHERNET_SETTINGS);

	// Attempt W5200 detection first, because W5200 does not properly
	// reset its SPI state when CS goes high (inactive).  Communication
	// from detecting the other chips can leave the W5200 in a state
	// where it won't recover, unless given a reset pulse.
	if (isW5200()) {
		CH_BASE = 0x4000;
    #ifdef W5200_16K_BUFFERS
    SSIZE = 16384;
    #endif
    #ifdef W5200_8K_BUFFERS
    SSIZE = 8192;
    #endif
		#ifdef W5200_4K_BUFFERS
		SSIZE = 4096;
		#endif
    #ifdef W5200_2K_BUFFERS
		SSIZE = 2048;    // 2K buffers
		#endif
    SMASK = SSIZE-1;
		TXBUF_BASE = 0x8000;
		RXBUF_BASE = 0xC000;
		for (i=0; i<MAX_SOCK_NUM; i++) {
			writeSnRX_SIZE(i, SSIZE >> 10);
			writeSnTX_SIZE(i, SSIZE >> 10);
		}
		for (; i<8; i++) {
			writeSnRX_SIZE(i, 0);
			writeSnTX_SIZE(i, 0);
		}

	// Try W5500 next.  Wiznet finally seems to have implemented
	// SPI well with this chip.  It appears to be very resilient,
	// so try it after the fragile W5200
	} else if (isW5500()) {
		CH_BASE = 0x1000;
    #if defined(W5500_16K_BUFFERS)
    SSIZE = 16384;   // 16K buffers
		#elif defined(W5500_4K_BUFFERS)
		SSIZE = 4096;    // 4K buffers
		#elif defined(W5500_2K_BUFFERS)
		SSIZE = 2048;    // 2K buffers
		#endif
    SMASK = SSIZE - 1;
		TXBUF_BASE = 0x8000;
		RXBUF_BASE = 0xC000;
		#ifndef W5500_2K_BUFFERS
		for (i=0; i<MAX_SOCK_NUM; i++) {
			writeSnRX_SIZE(i, SSIZE >> 10);
			writeSnTX_SIZE(i, SSIZE >> 10);
		}
		for (; i<8; i++) {
			writeSnRX_SIZE(i, 0);
			writeSnTX_SIZE(i, 0);
		}
		#endif
	// Try W5100 last.  This simple chip uses fixed 4 byte frames
	// for every 8 bit access.  Terribly inefficient, but so simple
	// it recovers from "hearing" unsuccessful W5100 or W5200
	// communication.  W5100 is also the only chip without a VERSIONR
	// register for identification, so we check this last.
	} else if (isW5100()) {
		CH_BASE = 0x0400;
		SSIZE = 2048;
		SMASK = 0x07FF;
		TXBUF_BASE = 0x4000;
		RXBUF_BASE = 0x6000;
		writeTMSR(0x55);
		writeRMSR(0x55);
	// No hardware seems to be present.  Or it could be a W5200
	// that's heard other SPI communication if its chip select
	// pin wasn't high when a SD card or other SPI chip was used.
	} else {
		//Serial.println("no chip :-(");
		chip = 0;
		SPI1.endTransaction();
		return 0; // no known chip is responding :-(
	}
	SPI1.endTransaction();
	// Initialize the socket base addresses
	for (int i=0; i<MAX_SOCK_NUM; i++) {
		SBASE[i] = TXBUF_BASE + SSIZE * i;
		RBASE[i] = RXBUF_BASE + SSIZE * i;
	}
	return 1; // successful init
}

// Soft reset the Wiznet chip, by writing to its MR register reset bit
uint8_t W5100Class::softReset(void)
{
	uint16_t count=0;

	//Serial.println("Wiznet soft reset");
	// write to reset bit
	writeMR(0x80);
	// then wait for soft reset to complete
	do {
		uint8_t mr = readMR();
		//Serial.print("mr=");
		//Serial.println(mr, HEX);
		if (mr == 0) return 1;
		delay(1);
	} while (++count < 20);
	return 0;
}

uint8_t W5100Class::isW5100(void)
{
	chip = 51;
	//Serial.println("w5100.cpp: detect W5100 chip");
	if (!softReset()) return 0;
	writeMR(0x10);
	if (readMR() != 0x10) return 0;
	writeMR(0x12);
	if (readMR() != 0x12) return 0;
	writeMR(0x00);
	if (readMR() != 0x00) return 0;
	//Serial.println("chip is W5100");
	return 1;
}

uint8_t W5100Class::isW5200(void)
{
	chip = 52;
	//Serial.println("w5100.cpp: detect W5200 chip");
	if (!softReset()) return 0;
	writeMR(0x08);
	if (readMR() != 0x08) return 0;
	writeMR(0x10);
	if (readMR() != 0x10) return 0;
	writeMR(0x00);
	if (readMR() != 0x00) return 0;
	int ver = readVERSIONR_W5200();
	//Serial.print("version=");
	//Serial.println(ver);
	if (ver != 3) return 0;
	//Serial.println("chip is W5200");
	return 1;
}

uint8_t W5100Class::isW5500(void)
{
	chip = 55;
	//Serial.println("w5100.cpp: detect W5500 chip");
	if (!softReset()) return 0;
	writeMR(0x08);
	if (readMR() != 0x08) return 0;
	writeMR(0x10);
	if (readMR() != 0x10) return 0;
	writeMR(0x00);
	if (readMR() != 0x00) return 0;
	int ver = readVERSIONR_W5500();
	//Serial.print("version=");
	//Serial.println(ver);
	if (ver != 4) return 0;
	//Serial.println("chip is W5500");
	return 1;
}


#ifdef USE_SPIFIFO
uint16_t W5100Class::write(uint16_t addr, const uint8_t *buf, uint16_t len)
{
  uint32_t i;

  if (chip == 51) {
    for (i=0; i<len; i++) {
	SPIFIFO1.write16(0xF000 | (addr >> 8), SPI_CONTINUE);
	SPIFIFO1.write16((addr << 8) | buf[i]);
	addr++;
	SPIFIFO1.read();
	SPIFIFO1.read();
    }
  } else if (chip == 52) {
	SPIFIFO1.clear();
	SPIFIFO1.write16(addr, SPI_CONTINUE);
	SPIFIFO1.write16(len | 0x8000, SPI_CONTINUE);
	for (i=0; i<len; i++) {
		SPIFIFO1.write(buf[i], ((i+1<len) ? SPI_CONTINUE : 0));
    // //---------------remove this-------------------------
    // digitalWrite(13,HIGH);
    // delay(200);
    // digitalWrite(13,LOW);
    // delay(200);
    // //------------------------------------------
		SPIFIFO1.read();
	}
	SPIFIFO1.read();
	SPIFIFO1.read();
  } else {
	//SPIFIFO1.clear();
	SPIFIFO1.write16(addr, SPI_CONTINUE);
	if (addr < 0x100) {
		// common registers 00nn
		SPIFIFO1.write16(0x0400 | *buf++,
			((len > 1) ? SPI_CONTINUE : 0));
	} else if (addr < 0x8000) {
		// socket registers  10nn, 11nn, 12nn, 13nn, etc
		SPIFIFO1.write16(((addr << 5) & 0xE000) | 0x0C00 | *buf++,
			((len > 1) ? SPI_CONTINUE : 0));
	} else if (addr < 0xC000) {
		// transmit buffers  8000-87FF, 8800-8FFF, 9000-97FF, etc
    #if defined(W5500_16K_BUFFERS)
    SPIFIFO1.write16(((addr << 1) & 0x0000) | 0x1400 | *buf++, // 16K buffers
			((len > 1) ? SPI_CONTINUE : 0));
    #elif defined(W5500_8K_BUFFERS)
		SPIFIFO1.write16(((addr << 1) & 0x2000) | 0x1400 | *buf++, // 8K buffers
			((len > 1) ? SPI_CONTINUE : 0));
		#elif defined(W5500_4K_BUFFERS)
		SPIFIFO1.write16(((addr << 1) & 0x6000) | 0x1400 | *buf++, // 4K buffers
			((len > 1) ? SPI_CONTINUE : 0));
		#else
		SPIFIFO1.write16(((addr << 2) & 0xE000) | 0x1400 | *buf++, // 2K buffers
			((len > 1) ? SPI_CONTINUE : 0));
		#endif
	} else {
		// receive buffers
    #if defined(W5500_16K_BUFFERS)
    SPIFIFO1.write16(((addr << 1) & 0x0000) | 0x1C00 | *buf++, // 16K buffers
			((len > 1) ? SPI_CONTINUE : 0));
    #elif defined(W5500_8K_BUFFERS)
		SPIFIFO1.write16(((addr << 1) & 0x2000) | 0x1C00 | *buf++, // 8K buffers
			((len > 1) ? SPI_CONTINUE : 0));
		#elif defined(W5500_4K_BUFFERS)
		SPIFIFO1.write16(((addr << 1) & 0x6000) | 0x1C00 | *buf++, // 4K buffers
			((len > 1) ? SPI_CONTINUE : 0));
		#else
		SPIFIFO1.write16(((addr << 2) & 0xE000) | 0x1C00 | *buf++, // 2K buffers
			((len > 1) ? SPI_CONTINUE : 0));
		#endif
	}
	len--;
	while (len >= 2) {
		len -= 2;
		SPIFIFO1.write16((*buf << 8) | *(buf+1), (len == 0) ? 0 : SPI_CONTINUE);
		buf += 2;
		SPIFIFO1.read();
	}
	if (len) {
		SPIFIFO1.write(*buf);
		SPIFIFO1.read();
	}
	SPIFIFO1.read();
	SPIFIFO1.read();
  }
  return len;
}
#else
uint16_t W5100Class::write(uint16_t addr, const uint8_t *buf, uint16_t len)
{
  if (chip == 51) {
    for (uint16_t i=0; i<len; i++) {
      setSS();
      SPI1.transfer(0xF0);
      SPI1.transfer(addr >> 8);
      SPI1.transfer(addr & 0xFF);
      addr++;
      SPI1.transfer(buf[i]);
      resetSS();
    }
  } else if (chip == 52) {
    setSS();
    SPI1.transfer(addr >> 8);
    SPI1.transfer(addr & 0xFF);
    SPI1.transfer(((len >> 8) & 0x7F) | 0x80);
    SPI1.transfer(len & 0xFF);
    for (uint16_t i=0; i<len; i++) {
      SPI1.transfer(buf[i]);
    }
    resetSS();
  } else {
    setSS();
    if (addr < 0x100) {
      // common registers 00nn
      SPI1.transfer(0);
      SPI1.transfer(addr & 0xFF);
      SPI1.transfer(0x04);
    } else if (addr < 0x8000) {
      // socket registers  10nn, 11nn, 12nn, 13nn, etc
      SPI1.transfer(0);
      SPI1.transfer(addr & 0xFF);
      SPI1.transfer(((addr >> 3) & 0xE0) | 0x0C);
    } else if (addr < 0xC000) {
      // transmit buffers  8000-87FF, 8800-8FFF, 9000-97FF, etc
      //  10## #nnn nnnn nnnn
      SPI1.transfer(addr >> 8);
      SPI1.transfer(addr & 0xFF);
      #if defined(W5500_16K_BUFFERS)
      SPI1.transfer(((addr >> 7) & 0x00) | 0x14); // 16K buffers
      #elif defined(W5500_8K_BUFFERS)
      SPI1.transfer(((addr >> 7) & 0x20) | 0x14); // 8K buffers
      #elif defined(W5500_4K_BUFFERS)
      SPI1.transfer(((addr >> 7) & 0x60) | 0x14); // 4K buffers
      #else
      SPI1.transfer(((addr >> 6) & 0xE0) | 0x14); // 2K buffers
      #endif
    } else {
      // receive buffers
      SPI1.transfer(addr >> 8);
      SPI1.transfer(addr & 0xFF);
      #if defined(W5500_16K_BUFFERS)
      SPI1.transfer(((addr >> 7) & 0x00) | 0x1C); // 16K buffers
      #elif defined(W5500_8K_BUFFERS)
      SPI1.transfer(((addr >> 7) & 0x20) | 0x1C); // 8K buffers
      #elif defined(W5500_4K_BUFFERS)
      SPI1.transfer(((addr >> 7) & 0x60) | 0x1C); // 4K buffers
      #else
      SPI1.transfer(((addr >> 6) & 0xE0) | 0x1C); // 2K buffers
      #endif
    }
    for (uint16_t i=0; i<len; i++) {
      SPI1.transfer(buf[i]);
    }
    resetSS();
  }
  return len;
}
#endif







#ifdef USE_SPIFIFO
uint16_t W5100Class::read(uint16_t addr, uint8_t *buf, uint16_t len)
{
  uint32_t i;

  if (chip == 51) {
    for (i=0; i<len; i++) {
	#if 1
	SPIFIFO1.write(0x0F, SPI_CONTINUE);
	SPIFIFO1.write16(addr, SPI_CONTINUE);
	addr++;
	SPIFIFO1.read();
	SPIFIFO1.write(0);
	SPIFIFO1.read();
	buf[i] = SPIFIFO1.read();
	#endif
	#if 0
	// this does not work, but why?
	SPIFIFO1.write16(0x0F00 | (addr >> 8), SPI_CONTINUE);
	SPIFIFO1.write16(addr << 8);
	addr++;
	SPIFIFO1.read();
	buf[i] = SPIFIFO1.read();
	#endif
    }
  } else if (chip == 52) {
	// len = 1:  write header, write 1 byte, read
	// len = 2:  write header, write 2 byte, read
	// len = 3,5,7
	SPIFIFO1.clear();
	SPIFIFO1.write16(addr, SPI_CONTINUE);
	SPIFIFO1.write16(len & 0x7FFF, SPI_CONTINUE);
	SPIFIFO1.read();
	if (len == 1) {
		// read only 1 byte
		SPIFIFO1.write(0);
		SPIFIFO1.read();
		*buf = SPIFIFO1.read();
	} else if (len == 2) {
		// read only 2 bytes
		SPIFIFO1.write16(0);
		SPIFIFO1.read();
		uint32_t val = SPIFIFO1.read();
		*buf++ = val >> 8;
		*buf = val;
	} else if ((len & 1)) {
		// read 3 or more, odd length
  		//Serial.print("W5200 read, len=");
		//Serial.println(len);
		uint32_t count = len / 2;
		SPIFIFO1.write16(0, SPI_CONTINUE);
		SPIFIFO1.read();
		do {
			if (count > 1) SPIFIFO1.write16(0, SPI_CONTINUE);
			else SPIFIFO1.write(0);
			uint32_t val = SPIFIFO1.read();
			//TODO: WebClient_speedtest with READSIZE 7 is
			//dramatically faster with this Serial.print(),
			//and the 2 above, but not without both.  Why?!
			//Serial.println(val, HEX);
			*buf++ = val >> 8;
			*buf++ = val;
		} while (--count > 0);
		*buf = SPIFIFO1.read();
		//Serial.println(*buf, HEX);
	} else {
		// read 4 or more, even length
  		//Serial.print("W5200 read, len=");
		//Serial.println(len);
		uint32_t count = len / 2 - 1;
		SPIFIFO1.write16(0, SPI_CONTINUE);
		SPIFIFO1.read();
		do {
			SPIFIFO1.write16(0, (count > 1) ? SPI_CONTINUE : 0);
			uint32_t val = SPIFIFO1.read();
			*buf++ = val >> 8;
			*buf++ = val;
		} while (--count > 0);
		uint32_t val = SPIFIFO1.read();
		*buf++ = val >> 8;
		*buf++ = val;
	}
  } else {
	//SPIFIFO1.clear();
	SPIFIFO1.write16(addr, SPI_CONTINUE);
	if (addr < 0x100) {
		// common registers 00nn
		SPIFIFO1.write16(0,
			((len > 1) ? SPI_CONTINUE : 0));
	} else if (addr < 0x8000) {
		// socket registers  10nn, 11nn, 12nn, 13nn, etc
		SPIFIFO1.write16(((addr << 5) & 0xE000) | 0x0800,
			((len > 1) ? SPI_CONTINUE : 0));
	} else if (addr < 0xC000) {
		// transmit buffers  8000-87FF, 8800-8FFF, 9000-97FF, etc
    #if defined(W5500_16K_BUFFERS)
    SPIFIFO1.write16(((addr << 1) & 0x0000) | 0x1000, // 16K buffers
			((len > 1) ? SPI_CONTINUE : 0));
    #elif defined(W5500_8K_BUFFERS)
    SPIFIFO1.write16(((addr << 1) & 0x2000) | 0x1000, // 8K buffers
			((len > 1) ? SPI_CONTINUE : 0));
		#elif defined(W5500_4K_BUFFERS)
		SPIFIFO1.write16(((addr << 1) & 0x6000) | 0x1000, // 4K buffers
			((len > 1) ? SPI_CONTINUE : 0));
		#else
		SPIFIFO1.write16(((addr << 2) & 0xE000) | 0x1000, // 2K buffers
			((len > 1) ? SPI_CONTINUE : 0));
		#endif
	} else {
		// receive buffers
    #if defined(W5500_16K_BUFFERS)
    SPIFIFO1.write16(((addr << 1) & 0x0000) | 0x1800, // 16K buffers
			((len > 1) ? SPI_CONTINUE : 0));
    #elif defined(W5500_8K_BUFFERS)
    SPIFIFO1.write16(((addr << 1) & 0x2000) | 0x1800, // 8K buffers
			((len > 1) ? SPI_CONTINUE : 0));
		#elif defined(W5500_4K_BUFFERS)
		SPIFIFO1.write16(((addr << 1) & 0x6000) | 0x1800, // 4K buffers
			((len > 1) ? SPI_CONTINUE : 0));
		#else
		SPIFIFO1.write16(((addr << 2) & 0xE000) | 0x1800, // 2K buffers
			((len > 1) ? SPI_CONTINUE : 0));
		#endif
	}
	SPIFIFO1.read();
	if (len <= 1) {
		*buf++ = SPIFIFO1.read();
	} else if (len == 2) {
		SPIFIFO1.write(0);
		*buf++ = SPIFIFO1.read();
		*buf++ = SPIFIFO1.read();
	} else if (len & 1) {
		uint32_t count = len >> 1;
		SPIFIFO1.write16(0, (count > 1) ? SPI_CONTINUE : 0);
		*buf++ = SPIFIFO1.read();
		while (count > 1) {
			count--;
			SPIFIFO1.write16(0, (count > 1) ? SPI_CONTINUE : 0);
			uint32_t val = SPIFIFO1.read();
			*buf++ = val >> 8;
			*buf++ = val;
		}
		uint32_t val = SPIFIFO1.read();
		*buf++ = val >> 8;
		*buf++ = val;
	} else {
		SPIFIFO1.write16(0, SPI_CONTINUE);
		*buf++ = SPIFIFO1.read();
		uint32_t count = len >> 1;
		while (count > 1) {
			count--;
			if (count > 1) {
				SPIFIFO1.write16(0, SPI_CONTINUE);
			} else {
				SPIFIFO1.write(0, 0);
			}
			uint32_t val = SPIFIFO1.read();
			*buf++ = val >> 8;
			*buf++ = val;
		}
		*buf = SPIFIFO1.read();
	}
  }
  return len;
}
#else
uint16_t W5100Class::read(uint16_t addr, uint8_t *buf, uint16_t len)
{
  if (chip == 51) {
    for (uint16_t i=0; i<len; i++) {
      setSS();
      SPI1.transfer(0x0F);
      SPI1.transfer(addr >> 8);
      SPI1.transfer(addr & 0xFF);
      addr++;
      buf[i] = SPI1.transfer(0);
      resetSS();
    }
  } else if (chip == 52) {
    setSS();
    SPI1.transfer(addr >> 8);
    SPI1.transfer(addr & 0xFF);
    SPI1.transfer((len >> 8) & 0x7F);
    SPI1.transfer(len & 0xFF);
    for (uint16_t i=0; i<len; i++) {
      buf[i] = SPI1.transfer(0);
    }
    resetSS();
  } else {
    setSS();
    if (addr < 0x100) {
      // common registers 00nn
      SPI1.transfer(0);
      SPI1.transfer(addr & 0xFF);
      SPI1.transfer(0x00);
    } else if (addr < 0x8000) {
      // socket registers  10nn, 11nn, 12nn, 13nn, etc
      SPI1.transfer(0);
      SPI1.transfer(addr & 0xFF);
      SPI1.transfer(((addr >> 3) & 0xE0) | 0x08);
    } else if (addr < 0xC000) {
      // transmit buffers  8000-87FF, 8800-8FFF, 9000-97FF, etc
      //  10## #nnn nnnn nnnn
      SPI1.transfer(addr >> 8);
      SPI1.transfer(addr & 0xFF);
      #ifdef W5500_4K_BUFFERS
      SPI1.transfer(((addr >> 7) & 0x60) | 0x10); // 4K buffers
      #else
      SPI1.transfer(((addr >> 6) & 0xE0) | 0x10); // 2K buffers
      #endif
    } else {
      // receive buffers
      SPI1.transfer(addr >> 8);
      SPI1.transfer(addr & 0xFF);
      #if defined(W5500_16K_BUFFERS)
      SPI1.transfer(((addr >> 7) & 0x00) | 0x18); // 16K buffers
      #elif defined(W5500_8K_BUFFERS)
      SPI1.transfer(((addr >> 7) & 0x20) | 0x18); // 8K buffers
  		#elif defined(W5500_4K_BUFFERS)
      SPI1.transfer(((addr >> 7) & 0x60) | 0x18); // 4K buffers
      #else
      SPI1.transfer(((addr >> 6) & 0xE0) | 0x18); // 2K buffers
      #endif
    }
    for (uint16_t i=0; i<len; i++) {
      buf[i] = SPI1.transfer(0);
    }
    resetSS();
  }
  return len;
}
#endif

void W5100Class::execCmdSn(SOCKET s, SockCMD _cmd) {
  // Send command to socket
  writeSnCR(s, _cmd);
  // Wait for command to complete
  while (readSnCR(s))
    ;
}
