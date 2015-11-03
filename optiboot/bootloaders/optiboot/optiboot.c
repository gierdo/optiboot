/**********************************************************/
/* Optiboot bootloader for Arduino                        */
/*                                                        */
/* http://optiboot.googlecode.com                         */
/*                                                        */
/* Arduino-maintained version : See README.TXT            */
/* http://code.google.com/p/arduino/                      */
/*                                                        */
/* Heavily optimised bootloader that is faster and        */
/* smaller than the Arduino standard bootloader           */
/*                                                        */
/* Enhancements:                                          */
/*   Fits in 512 bytes, saving 1.5K of code space         */
/*   Background page erasing speeds up programming        */
/*   Higher baud rate speeds up programming               */
/*   Written almost entirely in C                         */
/*   Customisable timeout with accurate timeconstant      */
/*   Optional virtual UART. No hardware UART required.    */
/*   Optional virtual boot partition for devices without. */
/*                                                        */
/* What you lose:                                         */
/*   Implements a skeleton STK500 protocol which is       */
/*     missing several features including EEPROM          */
/*     programming and non-page-aligned writes            */
/*   High baud rate breaks compatibility with standard    */
/*     Arduino flash settings                             */
/*                                                        */
/* Fully supported:                                       */
/*   ATmega168 based devices  (Diecimila etc)             */
/*   ATmega328P based devices (Duemilanove etc)           */
/*                                                        */
/* Alpha test                                             */
/*   ATmega1280 based devices (Arduino Mega)              */
/*                                                        */
/* Work in progress:                                      */
/*   ATmega644P based devices (Sanguino)                  */
/*   ATtiny84 based devices (Luminet)                     */
/*                                                        */
/* Does not support:                                      */
/*   USB based devices (eg. Teensy)                       */
/*                                                        */
/* Assumptions:                                           */
/*   The code makes several assumptions that reduce the   */
/*   code size. They are all true after a hardware reset, */
/*   but may not be true if the bootloader is called by   */
/*   other means or on other hardware.                    */
/*     No interrupts can occur                            */
/*     UART and Timer 1 are set to their reset state      */
/*     SP points to RAMEND                                */
/*                                                        */
/* Code builds on code, libraries and optimisations from: */
/*   stk500boot.c          by Jason P. Kyle               */
/*   Arduino bootloader    http://arduino.cc              */
/*   Spiff's 1K bootloader http://spiffie.org/know/arduino_1k_bootloader/bootloader.shtml */
/*   avr-libc project      http://nongnu.org/avr-libc     */
/*   Adaboot               http://www.ladyada.net/library/arduino/bootloader.html */
/*   AVR305                Atmel Application Note         */
/*                                                        */
/* This program is free software; you can redistribute it */
/* and/or modify it under the terms of the GNU General    */
/* Public License as published by the Free Software       */
/* Foundation; either version 2 of the License, or        */
/* (at your option) any later version.                    */
/*                                                        */
/* This program is distributed in the hope that it will   */
/* be useful, but WITHOUT ANY WARRANTY; without even the  */
/* implied warranty of MERCHANTABILITY or FITNESS FOR A   */
/* PARTICULAR PURPOSE.  See the GNU General Public        */
/* License for more details.                              */
/*                                                        */
/* You should have received a copy of the GNU General     */
/* Public License along with this program; if not, write  */
/* to the Free Software Foundation, Inc.,                 */
/* 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA */
/*                                                        */
/* Licence can be viewed at                               */
/* http://www.fsf.org/licenses/gpl.txt                    */
/*                                                        */
/**********************************************************/


/**********************************************************/
/*                                                        */
/* Optional defines:                                      */
/*                                                        */
/**********************************************************/
/*                                                        */
/* BIG_BOOT:                                              */
/* Build a 1k bootloader, not 512 bytes. This turns on    */
/* extra functionality.                                   */
/*                                                        */
/* BAUD_RATE:                                             */
/* Set bootloader baud rate.                              */
/*                                                        */
/* LUDICROUS_SPEED:                                       */
/* 230400 baud :-)                                        */
/*                                                        */
/* SOFT_UART:                                             */
/* Use AVR305 soft-UART instead of hardware UART.         */
/*                                                        */
/* LED_START_FLASHES:                                     */
/* Number of LED flashes on bootup.                       */
/*                                                        */
/* LED_DATA_FLASH:                                        */
/* Flash LED when transferring data. For boards without   */
/* TX or RX LEDs, or for people who like blinky lights.   */
/*                                                        */
/* SUPPORT_EEPROM:                                        */
/* Support reading and writing from EEPROM. This is not   */
/* used by Arduino, so off by default.                    */
/*                                                        */
/* TIMEOUT_MS:                                            */
/* Bootloader timeout period, in milliseconds.            */
/* 500,1000,2000,4000,8000 supported.                     */
/*                                                        */
/**********************************************************/

/**********************************************************/
/* Version Numbers!                                       */
/*                                                        */
/* Arduino Optiboot now includes this Version number in   */
/* the source and object code.                            */
/*                                                        */
/* Version 3 was released as zip from the optiboot        */
/*  repository and was distributed with Arduino 0022.     */
/* Version 4 starts with the arduino repository commit    */
/*  that brought the arduino repository up-to-date with   */
/* the optiboot source tree changes since v3.             */
/*                                                        */
/**********************************************************/

/**********************************************************/
/* Edit History:					  */
/*							  */
/* Jan 2012:                                              */
/* 4.5 WestfW: fix NRWW value for m1284.                  */
/* 4.4 WestfW: use attribute OS_main instead of naked for */
/*             main().  This allows optimizations that we */
/*             count on, which are prohibited in naked    */
/*             functions due to PR42240.  (keeps us less  */
/*             than 512 bytes when compiler is gcc4.5     */
/*             (code from 4.3.2 remains the same.)        */
/* 4.4 WestfW and Maniacbug:  Add m1284 support.  This    */
/*             does not change the 328 binary, so the     */
/*             version number didn't change either. (?)   */
/* June 2011:                                             */
/* 4.4 WestfW: remove automatic soft_uart detect (didn't  */
/*             know what it was doing or why.)  Added a   */
/*             check of the calculated BRG value instead. */
/*             Version stays 4.4; existing binaries are   */
/*             not changed.                               */
/* 4.4 WestfW: add initialization of address to keep      */
/*             the compiler happy.  Change SC'ed targets. */
/*             Return the SW version via READ PARAM       */
/* 4.3 WestfW: catch framing errors in getch(), so that   */
/*             AVRISP works without HW kludges.           */
/*  http://code.google.com/p/arduino/issues/detail?id=368n*/
/* 4.2 WestfW: reduce code size, fix timeouts, change     */
/*             verifySpace to use WDT instead of appstart */
/* 4.1 WestfW: put version number in binary.		  */
/**********************************************************/

#define TRUE 1
#define FALSE 0

#define OPTIBOOT_MAJVER 4
#define OPTIBOOT_MINVER 5

#define MAKESTR(a) #a
#define MAKEVER(a, b) MAKESTR(a*256+b)

asm("  .section .version\n"
		"optiboot_version:  .word " MAKEVER(OPTIBOOT_MAJVER, OPTIBOOT_MINVER) "\n"
		"  .section .text\n");

#include <inttypes.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/twi.h>
#include <avr/eeprom.h>

// <avr/boot.h> uses sts instructions, but this version uses out instructions
// This saves cycles and program memory.
#include "boot.h"


// We don't use <avr/wdt.h> as those routines have interrupt overhead we don't need.

#include "pin_defs.h"
#include "stk500.h"

#define RS485_ENABLESEND_PORT PORTD	///< defines the PORT register of the send-enable pin
#define	RS485_ENABLESEND_DDR DDRD	///< defines the DDR register of the send-enable pin
#define RS485_ENABLESEND_PIN PIND	///< defines the PIN register of the send-enable pin
#define RS485_ENABLESEND PD4		///< defines the pin number of the send-enable pin

#ifndef LED_START_FLASHES
#define LED_START_FLASHES 0
#endif

#ifdef LUDICROUS_SPEED
#define BAUD_RATE 230400L
#endif

/* set the UART baud rate defaults */
#ifndef BAUD_RATE
#if F_CPU >= 8000000L
#define BAUD_RATE   19200L //
#elsif F_CPU >= 1000000L
#define BAUD_RATE   9600L   // 19200 also supported, but with significant error
#elsif F_CPU >= 128000L
#define BAUD_RATE   4800L   // Good for 128kHz internal RC
#else
#define BAUD_RATE 1200L     // Good even at 32768Hz
#endif
#endif

#if 0
/* Switch in soft UART for hard baud rates */
/*
 * I don't understand what this was supposed to accomplish, where the
 * constant "280" came from, or why automatically (and perhaps unexpectedly)
 * switching to a soft uart is a good thing, so I'm undoing this in favor
 * of a range check using the same calc used to config the BRG...
 */
#if (F_CPU/BAUD_RATE) > 280 // > 57600 for 16MHz
#ifndef SOFT_UART
#define SOFT_UART
#endif
#endif
#else // 0
#if (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 > 250
#error Unachievable baud rate (too slow) BAUD_RATE 
#endif // baud rate slow check
#if (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 < 3
#error Unachievable baud rate (too fast) BAUD_RATE 
#endif // baud rate fastn check
#endif

/* Watchdog settings */
#define WATCHDOG_OFF    (0)
#define WATCHDOG_16MS   (_BV(WDE))
#define WATCHDOG_32MS   (_BV(WDP0) | _BV(WDE))
#define WATCHDOG_64MS   (_BV(WDP1) | _BV(WDE))
#define WATCHDOG_125MS  (_BV(WDP1) | _BV(WDP0) | _BV(WDE))
#define WATCHDOG_250MS  (_BV(WDP2) | _BV(WDE))
#define WATCHDOG_500MS  (_BV(WDP2) | _BV(WDP0) | _BV(WDE))
#define WATCHDOG_1S     (_BV(WDP2) | _BV(WDP1) | _BV(WDE))
#define WATCHDOG_2S     (_BV(WDP2) | _BV(WDP1) | _BV(WDP0) | _BV(WDE))
#ifndef __AVR_ATmega8__
#define WATCHDOG_4S     (_BV(WDP3) | _BV(WDE))
#define WATCHDOG_8S     (_BV(WDP3) | _BV(WDP0) | _BV(WDE))
#endif

/* Function Prototypes */
/* The main function is in init9, which removes the interrupt vector table */
/* we don't need. It is also 'naked', which means the compiler does not    */
/* generate any entry or exit code itself. */
int main(void) __attribute__ ((OS_main)) __attribute__ ((section (".init9")));
void putch(char);
uint8_t getch(void);
static inline void getNch(uint8_t); /* "static inline" is a compiler hint to reduce code size */
uint8_t verifySpace();
static inline void watchdogReset();
void watchdogConfig(uint8_t x);
#ifdef SOFT_UART
void uartDelay() __attribute__ ((naked));
#endif
void appStart() __attribute__ ((naked));



/* i2c interface */

#define TW_READ_BIT  0       // Bit position for R/W bit in "address byte".
#define TW_ADR_BITS  1       // Bit position for LSB of the slave address bits in the init byte.



typedef enum
{
	i2cAddress,
	i2cData1,
	i2cData2,
	i2cData3,
	i2cData4,
	i2cData5,
	i2cData6,
	i2cData7,
	i2cData8,
	i2cData9,
	i2cData10,
	i2cData11,
	TW_BUFFER_SIZE	// Has to be at the end of the enum list
}i2cPosition_t;

typedef enum
{
	i2cGeneralCall,
	i2cCBUS,
	i2cReserved0,
	i2cReserved1,
	i2cLeddriver
}i2cAddress_t;

typedef enum
{
	i2cNone,
	i2cRequestPerformLightshow
}i2cInstruction_t;

/**
 * @brief Union to store colour and brightness of a pixel in an efficient way.
 */
typedef union
{
	uint16_t wholeBytes;	//!< Direct access to the 2 used bytes
	struct
	{
		uint16_t g :3;	//!< Green value. Max: 7
		uint16_t r :3;	//!< Red value. Max: 7
		uint16_t b :3;	//!< Blue value. Max: 7
		uint16_t brightness :5;	//!< Brightness value. Max: 31
	};
} LEDs_colour_t;

/**
 * @brief Struct to store the instruction for a ring.
 */
typedef struct
{
	union
	{
		uint32_t instructionTypeWholeBytes;	//!< Direct access to the 4 used bytes of the instruction type
		struct
		{
			uint32_t setPixelValues :2;	//!< set direct pixel values of specified ring, Max: 3
			uint32_t ring1_instruction :5;	//!< instruction, Max: 32
			uint32_t ring1_pixel :5;	//!< glowing pixel id, Max: 32
			uint32_t ring2_instruction :5;	//!< instruction, Max: 32
			uint32_t ring2_pixel :5;	//!< glowing pixel id, Max: 32
			uint32_t ring3_instruction :5;	//!< instruction, Max: 32
			uint32_t ring3_pixel :5;	//!< glowing pixel id, Max: 32
		};
	};
	LEDs_colour_t ring1_colour;
	LEDs_colour_t ring2_colour;
	LEDs_colour_t ring3_colour;
} LEDs_ringInstruction_t;


typedef enum
{
	LEDs_modePulse,      //!< LEDs_modePulse
	LEDs_modeRotateLeft, //!< LEDs_modeRotateLeft
	LEDs_modeRotateRight,//!< LEDs_modeRotateRight
	LEDs_modeIndicate,	 //!< LEDs_modeIndicate
	LEDs_modeRotateDouble, //!< LEDs_modeRotateDouble
	LEDs_modeStatic      //!< LEDs_modeStatic
}LEDs_operationMode_t;

volatile LEDs_colour_t colourBoot;	// colour used while booting
volatile LEDs_colour_t colourFlash;	// colour used while flashing

typedef struct {
	uint8_t initVersion; // init version of the struct in the eeprom
	uint8_t ownAddress;
}EEPROM_CONFIG;

typedef struct {
	EEPROM_CONFIG config;
	unsigned char data[3];
	uint8_t structVersion;
	uint8_t brakeIdConfig[2];
	uint8_t LEDRingIdConfig[3];
	uint8_t switchIdConfig[6];
	LEDs_colour_t colourBoot[2];
}EEPROM_STRUCT;


EEPROM_STRUCT EEMEM eeprom_struct;


#define TW_CMD_MASK 			0x0F
#define TWI_OK					0x00
#define TWI_ERROR_NODEV			0x01

void TW_init( void );
void TW_reset( void );
void TW_start_cond( void );
void TW_stop_cond( void );
void TW_wait_compl( void );
void TW_send_byte( uint8_t data );
void TW_transceiveFrame( unsigned char * , unsigned char );
void TW_bootloaderActive();
void TW_flashmodeActive();
void TW_flashMe();
void TW_initColours();

/*
 * NRWW memory
 * Addresses below NRWW (Non-Read-While-Write) can be programmed while
 * continuing to run code from flash, slightly speeding up programming
 * time.  Beware that Atmel data sheets specify this as a WORD address,
 * while optiboot will be comparing against a 16-bit byte address.  This
 * means that on a part with 128kB of memory, the upper part of the lower
 * 64k will get NRWW processing as well, even though it doesn't need it.
 * That's OK.  In fact, you can disable the overlapping processing for
 * a part entirely by setting NRWWSTART to zero.  This reduces code
 * space a bit, at the expense of being slightly slower, overall.
 *
 * RAMSTART should be self-explanatory.  It's bigger on parts with a
 * lot of peripheral registers.
 */
#if defined(__AVR_ATmega168__)
#define RAMSTART (0x100)
#define NRWWSTART (0x3800)
#elif defined(__AVR_ATmega328P__)
#define RAMSTART (0x100)
#define NRWWSTART (0x7000)
#elif defined (__AVR_ATmega644P__)
#define RAMSTART (0x100)
#define NRWWSTART (0xE000)
#elif defined (__AVR_ATmega1284P__)
#define RAMSTART (0x100)
#define NRWWSTART (0xE000)
#elif defined(__AVR_ATtiny84__)
#define RAMSTART (0x100)
#define NRWWSTART (0x0000)
#elif defined(__AVR_ATmega1280__)
#define RAMSTART (0x200)
#define NRWWSTART (0xE000)
#elif defined(__AVR_ATmega8__) || defined(__AVR_ATmega88__)
#define RAMSTART (0x100)
#define NRWWSTART (0x1800)
#endif

/* C zero initialises all global variables. However, that requires */
/* These definitions are NOT zero initialised, but that doesn't matter */
/* This allows us to drop the zero init code, saving us memory */
#define buff    ((uint8_t*)(RAMSTART))
#ifdef VIRTUAL_BOOT_PARTITION
#define rstVect (*(uint16_t*)(RAMSTART+SPM_PAGESIZE*2+4))
#define wdtVect (*(uint16_t*)(RAMSTART+SPM_PAGESIZE*2+6))
#endif

/* main program starts here */
int main(void) {
	uint8_t wasInvokedByMessage = 0;
	uint8_t ch;

	uint8_t bootloaderStartCondition =0;
	EEPROM_STRUCT eepromStruct;
	/*
	 * Making these local and in registers prevents the need for initializing
	 * them, and also saves space because code no longer stores to memory.
	 * (initializing address keeps the compiler happy, but isn't really
	 *  necessary, and uses 4 bytes of flash.)
	 */
	register uint16_t address = 0;
	register uint8_t  length;

	// After the zero init loop, this is the first code to run.
	//
	// This code makes the following assumptions:
	//  No interrupts will execute
	//  SP points to RAMEND
	//  r1 contains zero
	//
	// If not, uncomment the following instructions:
	// cli();
	asm volatile ("clr __zero_reg__");
#ifdef __AVR_ATmega8__
	SP=RAMEND;  // This is done by hardware reset
#endif

	eeprom_read_block(&eepromStruct, &eeprom_struct, sizeof(EEPROM_STRUCT));
	eeprom_read_block(&colourBoot, &eeprom_struct.colourBoot[0], sizeof(LEDs_colour_t));
	eeprom_read_block(&colourFlash, &eeprom_struct.colourBoot[1], sizeof(LEDs_colour_t));

	if (eepromStruct.config.initVersion != 1) //eeprom not initialized
	{
		eepromStruct.config.initVersion = 1;
		eepromStruct. config.ownAddress = 0;
		eeprom_update_block(&eepromStruct.config, &eeprom_struct.config, sizeof(EEPROM_CONFIG));

		colourBoot.r = 6;
		colourBoot.g = 2;
		colourBoot.b = 0;
		colourBoot.brightness = 4;
		eeprom_update_block(&colourBoot, &eeprom_struct.colourBoot[0], sizeof(LEDs_colour_t));

		colourFlash.r = 0;
		colourFlash.g = 7;
		colourFlash.b = 0;
		colourFlash.brightness = 4;
		eeprom_update_block(&colourFlash, &eeprom_struct.colourBoot[1], sizeof(LEDs_colour_t));
	}

	/* See if update was invoked by a message and reset eeprom struct */
	wasInvokedByMessage = eepromStruct.data[1];
	eepromStruct.data[1] = 0;
	eeprom_update_block(&eepromStruct.data[1], &eeprom_struct.data[1], sizeof(unsigned char));

	// Adaboot no-wait mod
	ch = MCUSR;
	MCUSR = 0;
	if (((ch & _BV(PORF)) != 1) && (wasInvokedByMessage == 0))
	{
		appStart();
	}

	// Wait for i2C slave to boot
	_delay_ms(50);

	TW_init();
	TW_bootloaderActive();

	// Configure rs485_enablesend pin as output, off
	RS485_ENABLESEND_DDR |= (1<<RS485_ENABLESEND);
	RS485_ENABLESEND_PORT &= ~(1<<RS485_ENABLESEND);

#ifndef SOFT_UART
#ifdef __AVR_ATmega8__
	UCSRA = _BV(U2X); //Double speed mode USART
	UCSRB = _BV(RXEN) | _BV(TXEN);  // enable Rx & Tx
	UCSRC = _BV(URSEL) | _BV(UCSZ1) | _BV(UCSZ0);  // config USART; 8N1
	UBRRL = (uint8_t)( (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 );
#else
	UCSR0A = _BV(U2X0); //Double speed mode USART0
	UCSR0B = _BV(RXEN0) | _BV(TXEN0);
	UCSR0C = _BV(UCSZ00) | _BV(UCSZ01);
	UBRR0L = (uint8_t)( (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 );
#endif
#endif

	// Set up watchdog to trigger after 4s, if flashing was not invoked by message
	if (wasInvokedByMessage == 0)
	{
		watchdogConfig(WATCHDOG_4S);
	}
	else
	{
		watchdogConfig(WATCHDOG_OFF);
	}

	/* Set LED pin as output */
	//  LED_DDR |= _BV(LED);

#ifdef SOFT_UART
	/* Set TX pin as output */
	UART_DDR |= _BV(UART_TX_BIT);
#endif

	for (;;) /* Forever loop 1 */
	{
		flashmode_waitloop:

		if(bootloaderStartCondition == 1)	// We come from being flashed -> stay in the loop
		{
			bootloaderStartCondition = 0;
		}

		ch = getch();

		/*
		 * if "flashmode" -> activate flashmode
		 * if "flashmode"
		 */
		if (bootloaderStartCondition == 0)
		{
			if(ch == 'f')
			{
				bootloaderStartCondition = 2;
			}
			else
			{
				bootloaderStartCondition = 0;
			}
		}
		else if (bootloaderStartCondition == 2)
		{
			if(ch == 'l')
			{
				bootloaderStartCondition = 3;
			}
			else
			{
				bootloaderStartCondition = 0;
			}
		}
		else if (bootloaderStartCondition == 3)
		{
			if(ch == 'a')
			{
				bootloaderStartCondition = 4;
			}
			else
			{
				bootloaderStartCondition = 0;
			}
		}
		else if (bootloaderStartCondition == 4)
		{
			if(ch == 's')
			{
				bootloaderStartCondition = 5;
			}
			else
			{
				bootloaderStartCondition = 0;
			}
		}
		else if (bootloaderStartCondition == 5)
		{
			if(ch == 'h')
			{
				bootloaderStartCondition = 6;
			}
			else
			{
				bootloaderStartCondition = 0;
			}
		}
		else if (bootloaderStartCondition == 6)
		{
			if(ch == 'm')
			{
				bootloaderStartCondition = 7;
			}
			else
			{
				bootloaderStartCondition = 0;
			}
		}
		else if (bootloaderStartCondition == 7)
		{
			if(ch == 'o')
			{
				bootloaderStartCondition = 8;
			}
			else
			{
				bootloaderStartCondition = 0;
			}
		}
		else if (bootloaderStartCondition == 8)
		{
			if(ch == 'd')
			{
				bootloaderStartCondition = 9;
			}
			else
			{
				bootloaderStartCondition = 0;
			}
		}
		else if (bootloaderStartCondition == 9)
		{
			if(ch == 'e')
			{
				bootloaderStartCondition = 10;
				watchdogConfig(WATCHDOG_OFF);
			}
			else
			{
				bootloaderStartCondition = 0;
			}
		}
		// Bootloader active, wait for address

		else if (bootloaderStartCondition == 10)
		{
			if(ch == eepromStruct.config.ownAddress) // We are being flashed!
			{
				bootloaderStartCondition = 1;
				TW_flashMe();
			}
			else	// Somebody else is being flashed!
			{
				bootloaderStartCondition = 0;
				TW_flashmodeActive();
			}
		}

		if(bootloaderStartCondition == 1)
		{
			/* Forever loop 2 */
			for (;;) {
				/* get character from UART */
				ch = getch();

				if(ch == STK_GET_PARAMETER) {
					unsigned char which = getch();
					if (verifySpace())
						{
						TW_flashmodeActive();
						goto flashmode_waitloop;
						}

					if (which == 0x82) {
						/*
						 * Send optiboot version as "minor SW version"
						 */
						putch(OPTIBOOT_MINVER);
					} else if (which == 0x81) {
						putch(OPTIBOOT_MAJVER);
					} else {
						/*
						 * GET PARAMETER returns a generic 0x03 reply for
						 * other parameters - enough to keep Avrdude happy
						 */
						putch(0x03);
					}
				}
				else if(ch == STK_SET_DEVICE) {
					// SET DEVICE is ignored
					getNch(20);
				}
				else if(ch == STK_SET_DEVICE_EXT) {
					// SET DEVICE EXT is ignored
					getNch(5);
				}
				else if(ch == STK_LOAD_ADDRESS) {
					// LOAD ADDRESS
					uint16_t newAddress;
					newAddress = getch();
					newAddress = (newAddress & 0xff) | (getch() << 8);
#ifdef RAMPZ
					// Transfer top bit to RAMPZ
					RAMPZ = (newAddress & 0x8000) ? 1 : 0;
#endif
					newAddress += newAddress; // Convert from word address to byte address
					address = newAddress;
					if (verifySpace())
						{
						TW_flashmodeActive();
						goto flashmode_waitloop;
						}
				}
				else if(ch == STK_UNIVERSAL) {
					// UNIVERSAL command is ignored
					getNch(4);
					putch(0x00);
				}
				/* Write memory, length is big endian and is in bytes */
				else if(ch == STK_PROG_PAGE) {
					// PROGRAM PAGE - we support flash programming only, not EEPROM
					uint8_t *bufPtr;
					uint16_t addrPtr;

					getch();			/* getlen() */
					length = getch();
					getch();

					// If we are in RWW section, immediately start page erase
					if (address < NRWWSTART) __boot_page_erase_short((uint16_t)(void*)address);

					// While that is going on, read in page contents
					bufPtr = buff;
					do *bufPtr++ = getch();
					while (--length);

					// If we are in NRWW section, page erase has to be delayed until now.
					// Todo: Take RAMPZ into account
					if (address >= NRWWSTART) __boot_page_erase_short((uint16_t)(void*)address);

					// Read command terminator, start reply
					if (verifySpace())
						{
						TW_flashmodeActive();
						goto flashmode_waitloop;
						}

					// If only a partial page is to be programmed, the erase might not be complete.
					// So check that here
					boot_spm_busy_wait();

#ifdef VIRTUAL_BOOT_PARTITION
					if ((uint16_t)(void*)address == 0) {
						// This is the reset vector page. We need to live-patch the code so the
						// bootloader runs.
						//
						// Move RESET vector to WDT vector
						uint16_t vect = buff[0] | (buff[1]<<8);
						rstVect = vect;
						wdtVect = buff[8] | (buff[9]<<8);
						vect -= 4; // Instruction is a relative jump (rjmp), so recalculate.
						buff[8] = vect & 0xff;
						buff[9] = vect >> 8;

						// Add jump to bootloader at RESET vector
						buff[0] = 0x7f;
						buff[1] = 0xce; // rjmp 0x1d00 instruction
					}
#endif

					// Copy buffer into programming buffer
					bufPtr = buff;
					addrPtr = (uint16_t)(void*)address;
					ch = SPM_PAGESIZE / 2;
					do {
						uint16_t a;
						a = *bufPtr++;
						a |= (*bufPtr++) << 8;
						__boot_page_fill_short((uint16_t)(void*)addrPtr,a);
						addrPtr += 2;
					} while (--ch);

					// Write from programming buffer
					__boot_page_write_short((uint16_t)(void*)address);
					boot_spm_busy_wait();

#if defined(RWWSRE)
					// Reenable read access to flash
					boot_rww_enable();
#endif

				}
				/* Read memory block mode, length is big endian.  */
				else if(ch == STK_READ_PAGE) {
					// READ PAGE - we only read flash
					getch();			/* getlen() */
					length = getch();
					getch();

					if (verifySpace())
						{
						TW_flashmodeActive();
						goto flashmode_waitloop;
						}
#ifdef VIRTUAL_BOOT_PARTITION
					do {
						// Undo vector patch in bottom page so verify passes
						if (address == 0)       ch=rstVect & 0xff;
						else if (address == 1)  ch=rstVect >> 8;
						else if (address == 8)  ch=wdtVect & 0xff;
						else if (address == 9) ch=wdtVect >> 8;
						else ch = pgm_read_byte_near(address);
						address++;
						putch(ch);
					} while (--length);
#else
#ifdef RAMPZ
					// Since RAMPZ should already be set, we need to use EPLM directly.
					//      do putch(pgm_read_byte_near(address++));
					//      while (--length);
					do {
						uint8_t result;
						__asm__ ("elpm %0,Z\n":"=r"(result):"z"(address));
						putch(result);
						address++;
					}
					while (--length);
#else
					do putch(pgm_read_byte_near(address++));
					while (--length);
#endif
#endif
				}

				/* Get device signature bytes  */
				else if(ch == STK_READ_SIGN) {
					// READ SIGN - return what Avrdude wants to hear
					if (verifySpace())
						{
						TW_flashmodeActive();
						goto flashmode_waitloop;
						}
					putch(SIGNATURE_0);
					putch(SIGNATURE_1);
					putch(SIGNATURE_2);
				}
				else if (ch == STK_LEAVE_PROGMODE) { /* 'Q' */
					// Adaboot no-wait mod
//					watchdogConfig(WATCHDOG_16MS);
					if (verifySpace())
						{
						goto flashmode_waitloop;
						}
					putch(STK_OK);
					TW_flashmodeActive();
					goto flashmode_waitloop;
				}
				else {
					// This covers the response to commands like STK_ENTER_PROGMODE
					if (verifySpace())
						{
						TW_flashmodeActive();
						goto flashmode_waitloop;
						}
				}
				putch(STK_OK);
			}
		}
	}
}

void putch(char ch) {
#ifndef SOFT_UART
	while (!(UCSR0A & _BV(UDRE0)));
	RS485_ENABLESEND_PORT |= (1<<RS485_ENABLESEND);
	UCSR0A |= (1 << TXC0);		// Reset transmit complete bit
	UDR0 = ch;
	while (!(UCSR0A & (1 << TXC0)));
	RS485_ENABLESEND_PORT &= ~(1<<RS485_ENABLESEND);

#else
	__asm__ __volatile__ (
			"   com %[ch]\n" // ones complement, carry set
			"   sec\n"
			"1: brcc 2f\n"
			"   cbi %[uartPort],%[uartBit]\n"
			"   rjmp 3f\n"
			"2: sbi %[uartPort],%[uartBit]\n"
			"   nop\n"
			"3: rcall uartDelay\n"
			"   rcall uartDelay\n"
			"   lsr %[ch]\n"
			"   dec %[bitcnt]\n"
			"   brne 1b\n"
			:
			:
			[bitcnt] "d" (10),
			[ch] "r" (ch),
			[uartPort] "I" (_SFR_IO_ADDR(UART_PORT)),
			[uartBit] "I" (UART_TX_BIT)
			:
			"r25"
	);
#endif
}

uint8_t getch(void) {
	uint8_t ch;

#ifdef LED_DATA_FLASH
#ifdef __AVR_ATmega8__
	LED_PORT ^= _BV(LED);
#else
	LED_PIN |= _BV(LED);
#endif
#endif

#ifdef SOFT_UART
	__asm__ __volatile__ (
			"1: sbic  %[uartPin],%[uartBit]\n"  // Wait for start edge
			"   rjmp  1b\n"
			"   rcall uartDelay\n"          // Get to middle of start bit
			"2: rcall uartDelay\n"              // Wait 1 bit period
			"   rcall uartDelay\n"              // Wait 1 bit period
			"   clc\n"
			"   sbic  %[uartPin],%[uartBit]\n"
			"   sec\n"
			"   dec   %[bitCnt]\n"
			"   breq  3f\n"
			"   ror   %[ch]\n"
			"   rjmp  2b\n"
			"3:\n"
			:
			[ch] "=r" (ch)
			:
			[bitCnt] "d" (9),
			[uartPin] "I" (_SFR_IO_ADDR(UART_PIN)),
			[uartBit] "I" (UART_RX_BIT)
			:
			"r25"
	);
#else
	while(!(UCSR0A & _BV(RXC0)))
		;
	if (!(UCSR0A & _BV(FE0))) {
		/*
		 * A Framing Error indicates (probably) that something is talking
		 * to us at the wrong bit rate.  Assume that this is because it
		 * expects to be talking to the application, and DON'T reset the
		 * watchdog.  This should cause the bootloader to abort and run
		 * the application "soon", if it keeps happening.  (Note that we
		 * don't care that an invalid char is returned...)
		 */
		watchdogReset();
	}

	ch = UDR0;
#endif

#ifdef LED_DATA_FLASH
#ifdef __AVR_ATmega8__
	LED_PORT ^= _BV(LED);
#else
	LED_PIN |= _BV(LED);
#endif
#endif

	return ch;
}

#ifdef SOFT_UART
// AVR305 equation: #define UART_B_VALUE (((F_CPU/BAUD_RATE)-23)/6)
// Adding 3 to numerator simulates nearest rounding for more accurate baud rates
#define UART_B_VALUE (((F_CPU/BAUD_RATE)-20)/6)
#if UART_B_VALUE > 255
#error Baud rate too slow for soft UART
#endif

void uartDelay() {
	__asm__ __volatile__ (
			"ldi r25,%[count]\n"
			"1:dec r25\n"
			"brne 1b\n"
			"ret\n"
			::[count] "M" (UART_B_VALUE)
	);
}
#endif

void getNch(uint8_t count) {
	do getch(); while (--count);
	verifySpace();
}

uint8_t verifySpace() {
	if (getch() != CRC_EOP) {
		// The original function went into a watchdog-reboot-loop here. Not a good idea if you might want to flash other systems on a bus / try again.
		return 1;
	}
	putch(STK_INSYNC);
	return 0;
}

#if LED_START_FLASHES > 0
void flash_led(uint8_t count) {
	do {
		TCNT1 = -(F_CPU/(1024*16));
		TIFR1 = _BV(TOV1);
		while(!(TIFR1 & _BV(TOV1)));
#ifdef __AVR_ATmega8__
		LED_PORT ^= _BV(LED);
#else
		LED_PIN |= _BV(LED);
#endif
		watchdogReset();
	} while (--count);
}
#endif

// Watchdog functions. These are only safe with interrupts turned off.
void watchdogReset() {
	__asm__ __volatile__ (
			"wdr\n"
	);
}

void watchdogConfig(uint8_t x) {
	WDTCSR = _BV(WDCE) | _BV(WDE);
	WDTCSR = x;
}

void appStart() {
	watchdogConfig(WATCHDOG_OFF);
	__asm__ __volatile__ (
#ifdef VIRTUAL_BOOT_PARTITION
			// Jump to WDT vector
			"ldi r30,4\n"
			"clr r31\n"
#else
			// Jump to RST vector
			"clr r30\n"
			"clr r31\n"
#endif
			"ijmp\n"
	);
}

void TW_init(void)
{
	TWBR = 72;                                  // Set bit rate register (Baudrate).
	TWSR |= 0x00;                                  // No Prescaler
	TWCR = (1<<TWEN)|                                 // Enable TWI-interface and release TWI pins.
		 (0<<TWIE)|(0<<TWINT)|                      // Disable Interrupt, clear flag
		 (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // No Signal requests.
		 (0<<TWWC);                                 //
}

void TW_reset( void )
{
	TWDR = 0xFF;                                      // Default content = SDA released.
	TWCR = 0;
	TWCR = (1<<TWEN)|                                 // TWI Interface enabled
	(0<<TWIE)|(1<<TWINT)|                      // Disable TWI Interrupt and clear the flag
	(0<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|           // Initiate a STOP condition.
	(0<<TWWC);                                 //
}

void TW_start_cond()
{
	TWCR = (( TWCR & TW_CMD_MASK) | _BV(TWINT) | _BV(TWSTA));
}

void TW_stop_cond()
{
	TWCR = (TWCR & TW_CMD_MASK) | _BV(TWINT) | _BV(TWEA) | _BV(TWSTO);
}

void TW_wait_compl()
{
	while( (TWCR & _BV(TWINT)) == 0 );
}

void TW_send_byte(uint8_t data)
{
	// save data to TWDR
	TWDR = data;
	// begin send
	TWCR = ((TWCR & TW_CMD_MASK) | _BV(TWINT));
}

void TW_transceiveFrame( unsigned char *msg, unsigned char msgSize )
{
	// send start condition
	TW_start_cond();
	TW_wait_compl();

	// check if device is present and live
	// sending data
	while(msgSize)
	{
		TW_send_byte( *msg++ );
		TW_wait_compl();
		msgSize--;
	}

	// transmit stop condition
	// leave with TWEA on for slave receiving
	TW_stop_cond();
//	while( (TWCR & _BV(TWSTO)) == 0);
}

void TW_bootloaderActive()
{
	LEDs_ringInstruction_t instruction;
	uint8_t i2cMessageBuffer[i2cData11+1];

	TW_initColours();

	instruction.setPixelValues = 0;

	instruction.ring1_colour = colourBoot;
	instruction.ring1_instruction = LEDs_modePulse;
	instruction.ring1_pixel = 3;

	instruction.ring2_colour = colourBoot;
	instruction.ring2_instruction = LEDs_modePulse;
	instruction.ring2_pixel = 3;

	instruction.ring3_colour = colourBoot;
	instruction.ring3_instruction = LEDs_modePulse;
	instruction.ring3_pixel = 3;

	i2cMessageBuffer[i2cAddress] = ((i2cLeddriver<<TW_ADR_BITS)|(FALSE<<TW_READ_BIT));
	i2cMessageBuffer[i2cData1] = 1;
	i2cMessageBuffer[i2cData2] = (uint8_t)(instruction.instructionTypeWholeBytes >>24);
	i2cMessageBuffer[i2cData3] = (uint8_t)(instruction.instructionTypeWholeBytes >>16);
	i2cMessageBuffer[i2cData4] = (uint8_t)(instruction.instructionTypeWholeBytes >>8);
	i2cMessageBuffer[i2cData5] = (uint8_t)(instruction.instructionTypeWholeBytes);
	i2cMessageBuffer[i2cData6] = (uint8_t)(instruction.ring1_colour.wholeBytes >>8);
	i2cMessageBuffer[i2cData7] = (uint8_t)(instruction.ring1_colour.wholeBytes);
	i2cMessageBuffer[i2cData8] = (uint8_t)(instruction.ring2_colour.wholeBytes >>8);
	i2cMessageBuffer[i2cData9] = (uint8_t)(instruction.ring2_colour.wholeBytes);
	i2cMessageBuffer[i2cData10] = (uint8_t)(instruction.ring3_colour.wholeBytes >>8);
	i2cMessageBuffer[i2cData11] = (uint8_t)(instruction.ring3_colour.wholeBytes);
	TW_transceiveFrame( i2cMessageBuffer, (i2cData11 +1) );
}

void TW_flashmodeActive()
{
	LEDs_ringInstruction_t instruction;
	uint8_t i2cMessageBuffer[i2cData11+1];

	TW_initColours();

	instruction.setPixelValues = 0;

	instruction.ring1_colour = colourBoot;
	instruction.ring1_instruction = LEDs_modeRotateLeft;
	instruction.ring1_pixel = 3;

	instruction.ring2_colour = colourBoot;
	instruction.ring2_instruction = LEDs_modeRotateLeft;
	instruction.ring2_pixel = 3;

	instruction.ring3_colour = colourBoot;
	instruction.ring3_instruction = LEDs_modeRotateLeft;
	instruction.ring3_pixel = 3;

	i2cMessageBuffer[i2cAddress] = ((i2cLeddriver<<TW_ADR_BITS)|(FALSE<<TW_READ_BIT));
	i2cMessageBuffer[i2cData1] = 1;
	i2cMessageBuffer[i2cData2] = (uint8_t)(instruction.instructionTypeWholeBytes >>24);
	i2cMessageBuffer[i2cData3] = (uint8_t)(instruction.instructionTypeWholeBytes >>16);
	i2cMessageBuffer[i2cData4] = (uint8_t)(instruction.instructionTypeWholeBytes >>8);
	i2cMessageBuffer[i2cData5] = (uint8_t)(instruction.instructionTypeWholeBytes);
	i2cMessageBuffer[i2cData6] = (uint8_t)(instruction.ring1_colour.wholeBytes >>8);
	i2cMessageBuffer[i2cData7] = (uint8_t)(instruction.ring1_colour.wholeBytes);
	i2cMessageBuffer[i2cData8] = (uint8_t)(instruction.ring2_colour.wholeBytes >>8);
	i2cMessageBuffer[i2cData9] = (uint8_t)(instruction.ring2_colour.wholeBytes);
	i2cMessageBuffer[i2cData10] = (uint8_t)(instruction.ring3_colour.wholeBytes >>8);
	i2cMessageBuffer[i2cData11] = (uint8_t)(instruction.ring3_colour.wholeBytes);
	TW_transceiveFrame( i2cMessageBuffer, (i2cData11 +1) );
}

void TW_flashMe()
{
	LEDs_ringInstruction_t instruction;
	uint8_t i2cMessageBuffer[i2cData11+1];

	TW_initColours();

	instruction.setPixelValues = 0;

	instruction.ring1_colour = colourFlash;
	instruction.ring1_instruction = LEDs_modeRotateRight;
	instruction.ring1_pixel = 3;

	instruction.ring2_colour = colourFlash;
	instruction.ring2_instruction = LEDs_modeRotateRight;
	instruction.ring2_pixel = 3;

	instruction.ring3_colour = colourFlash;
	instruction.ring3_instruction = LEDs_modeRotateRight;
	instruction.ring3_pixel = 3;

	i2cMessageBuffer[i2cAddress] = ((i2cLeddriver<<TW_ADR_BITS)|(FALSE<<TW_READ_BIT));
	i2cMessageBuffer[i2cData1] = 1;
	i2cMessageBuffer[i2cData2] = (uint8_t)(instruction.instructionTypeWholeBytes >>24);
	i2cMessageBuffer[i2cData3] = (uint8_t)(instruction.instructionTypeWholeBytes >>16);
	i2cMessageBuffer[i2cData4] = (uint8_t)(instruction.instructionTypeWholeBytes >>8);
	i2cMessageBuffer[i2cData5] = (uint8_t)(instruction.instructionTypeWholeBytes);
	i2cMessageBuffer[i2cData6] = (uint8_t)(instruction.ring1_colour.wholeBytes >>8);
	i2cMessageBuffer[i2cData7] = (uint8_t)(instruction.ring1_colour.wholeBytes);
	i2cMessageBuffer[i2cData8] = (uint8_t)(instruction.ring2_colour.wholeBytes >>8);
	i2cMessageBuffer[i2cData9] = (uint8_t)(instruction.ring2_colour.wholeBytes);
	i2cMessageBuffer[i2cData10] = (uint8_t)(instruction.ring3_colour.wholeBytes >>8);
	i2cMessageBuffer[i2cData11] = (uint8_t)(instruction.ring3_colour.wholeBytes);
	TW_transceiveFrame( i2cMessageBuffer, (i2cData11 +1) );
}

void TW_initColours()
{
	eeprom_read_block(&colourBoot, &eeprom_struct.colourBoot[0], sizeof(LEDs_colour_t));
	eeprom_read_block(&colourFlash, &eeprom_struct.colourBoot[1], sizeof(LEDs_colour_t));
}
