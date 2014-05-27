/*==============================================================================

  TinyDebugSerial.h - Tiny write-only software serial.

  Copyright 2010 Rowdy Dog Software.

  This file is part of Arduino-Tiny.

  Arduino-Tiny is free software: you can redistribute it and/or modify it
  under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation, either version 3 of the License, or (at your
  option) any later version.

  Arduino-Tiny is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
  or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
  License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with Arduino-Tiny.  If not, see <http://www.gnu.org/licenses/>.

==============================================================================*/

#include <avr/io.h>

#define SERPORT 0x1B
#define SERPIN PINA2

	void ser_init () {
		asm volatile (
			"sbi   %[port]-1, %[pin]"            "\n\t"
			"sbi   %[port], %[pin]"              "\n\t"
			:
			:
			[port] "I" ( SERPORT ),
			[pin] "I" ( SERPIN )
			:
		);
	}
    

	uint8_t ser_write_1_115200 (uint8_t value) {
		asm volatile (
			"cli"                                     "\n\t"

			"cbi   %[port], %[pin]"              "\n\t"      /* 2  <--- 0 */
			"ror   %[value]"                          "\n\t"      /* 1 */
			"nop"                                     "\n\t"      /* 1 */
			"nop"                                     "\n\t"      /* 1 */
			"nop"                                     "\n\t"      /* 1 */
			"nop"                                     "\n\t"      /* 1 */
		
			"brcs  L%=b0h"                            "\n\t"      /* 1  (not taken) */
			"nop"                                     "\n\t"      /* 1 */
			"cbi   %[port], %[pin]"              "\n\t"      /* 2  <--- st is 9 cycles */
			"rjmp  L%=b0z"                            "\n\t"      /* 2 */
			"L%=b0h: "                                              /* 2  (taken) */
			"sbi   %[port], %[pin]"              "\n\t"      /* 2  <--- st is 9 cycles */
			"nop"                                     "\n\t"      /* 1 */
			"nop"                                     "\n\t"      /* 1 */
			"L%=b0z: "
			"ror   %[value]"                          "\n\t"      /* 1 */

			"nop"                                     "\n\t"      /* 1 */

			"brcs  L%=b1h"                            "\n\t"      /* 1  (not taken) */
			"nop"                                     "\n\t"      /* 1 */
			"cbi   %[port], %[pin]"              "\n\t"      /* 2  <--- b0 is 8 cycles */
			"rjmp  L%=b1z"                            "\n\t"      /* 2 */
			"L%=b1h: "                                              /* 2  (taken) */
			"sbi   %[port], %[pin]"              "\n\t"      /* 2  <--- b0 is 8 cycles */
			"nop"                                     "\n\t"      /* 1 */
			"nop"                                     "\n\t"      /* 1 */
			"L%=b1z: "
			"ror   %[value]"                          "\n\t"      /* 1 */

			"nop"                                     "\n\t"      /* 1 */
			"nop"                                     "\n\t"      /* 1 */

			"brcs  L%=b2h"                            "\n\t"      /* 1  (not taken) */
			"nop"                                     "\n\t"      /* 1 */
			"cbi   %[port], %[pin]"              "\n\t"      /* 2  <--- b1 is 9 cycles */
			"rjmp  L%=b2z"                            "\n\t"      /* 2 */
			"L%=b2h: "                                              /* 2  (taken) */
			"sbi   %[port], %[pin]"              "\n\t"      /* 2  <--- b1 is 9 cycles */
			"nop"                                     "\n\t"      /* 1 */
			"nop"                                     "\n\t"      /* 1 */
			"L%=b2z: "
			"ror   %[value]"                          "\n\t"      /* 1 */

			"nop"                                     "\n\t"      /* 1 */
			"nop"                                     "\n\t"      /* 1 */

			"brcs  L%=b3h"                            "\n\t"      /* 1  (not taken) */
			"nop"                                     "\n\t"      /* 1 */
			"cbi   %[port], %[pin]"              "\n\t"      /* 2  <--- b2 is 9 cycles */
			"rjmp  L%=b3z"                            "\n\t"      /* 2 */
			"L%=b3h: "                                              /* 2  (taken) */
			"sbi   %[port], %[pin]"              "\n\t"      /* 2  <--- b2 is 9 cycles */
			"nop"                                     "\n\t"      /* 1 */
			"nop"                                     "\n\t"      /* 1 */
			"L%=b3z: "
			"ror   %[value]"                          "\n\t"      /* 1 */

			"nop"                                     "\n\t"      /* 1 */

			"brcs  L%=b4h"                            "\n\t"      /* 1  (not taken) */
			"nop"                                     "\n\t"      /* 1 */
			"cbi   %[port], %[pin]"              "\n\t"      /* 2  <--- b3 is 8 cycles */
			"rjmp  L%=b4z"                            "\n\t"      /* 2 */
			"L%=b4h: "                                              /* 2  (taken) */
			"sbi   %[port], %[pin]"              "\n\t"      /* 2  <--- b3 is 8 cycles */
			"nop"                                     "\n\t"      /* 1 */
			"nop"                                     "\n\t"      /* 1 */
			"L%=b4z: "
			"ror   %[value]"                          "\n\t"      /* 1 */

			"nop"                                     "\n\t"      /* 1 */
			"nop"                                     "\n\t"      /* 1 */

			"brcs  L%=b5h"                            "\n\t"      /* 1  (not taken) */
			"nop"                                     "\n\t"      /* 1 */
			"cbi   %[port], %[pin]"              "\n\t"      /* 2  <--- b4 is 9 cycles */
			"rjmp  L%=b5z"                            "\n\t"      /* 2 */
			"L%=b5h: "                                              /* 2  (taken) */
			"sbi   %[port], %[pin]"              "\n\t"      /* 2  <--- b4 is 9 cycles */
			"nop"                                     "\n\t"      /* 1 */
			"nop"                                     "\n\t"      /* 1 */
			"L%=b5z: "
			"ror   %[value]"                          "\n\t"      /* 1 */

			"nop"                                     "\n\t"      /* 1 */
			"nop"                                     "\n\t"      /* 1 */

			"brcs  L%=b6h"                            "\n\t"      /* 1  (not taken) */
			"nop"                                     "\n\t"      /* 1 */
			"cbi   %[port], %[pin]"              "\n\t"      /* 2  <--- b5 is 9 cycles */
			"rjmp  L%=b6z"                            "\n\t"      /* 2 */
			"L%=b6h: "                                              /* 2  (taken) */
			"sbi   %[port], %[pin]"              "\n\t"      /* 2  <--- b5 is 9 cycles */
			"nop"                                     "\n\t"      /* 1 */
			"nop"                                     "\n\t"      /* 1 */
			"L%=b6z: "
			"ror   %[value]"                          "\n\t"      /* 1 */

			"nop"                                     "\n\t"      /* 1 */

			"brcs  L%=b7h"                            "\n\t"      /* 1  (not taken) */
			"nop"                                     "\n\t"      /* 1 */
			"cbi   %[port], %[pin]"              "\n\t"      /* 2  <--- b6 is 8 cycles */
			"rjmp  L%=b7z"                            "\n\t"      /* 2 */
			"L%=b7h: "                                              /* 2  (taken) */
			"sbi   %[port], %[pin]"              "\n\t"      /* 2  <--- b6 is 8 cycles */
			"nop"                                     "\n\t"      /* 1 */
			"nop"                                     "\n\t"      /* 1 */
			"L%=b7z: "
			"nop"                                     "\n\t"      /* 1 */

			"nop"                                     "\n\t"      /* 1 */
			"nop"                                     "\n\t"      /* 1 */

			"nop"                                     "\n\t"      /* 1 */
			"nop"                                     "\n\t"      /* 1 */
			"sbi   %[port], %[pin]"              "\n\t"      /* 2  <--- b7 is 9 cycles */
			"nop"                                     "\n\t"      /* 1 */
			"nop"                                     "\n\t"      /* 1 */
			"nop"                                     "\n\t"      /* 1 */
			"nop"                                     "\n\t"      /* 1 */
			"nop"                                     "\n\t"      /* 1 */
			"nop"                                     "\n\t"      /* 1 */
			"nop"                                     "\n\t"      /* 1 */
			/*    <---sp is 9 cycles */

			"sei"                                     "\n\t"

			:
			:
			[value] "r" ( value ),
			[port] "I" ( SERPORT ),
			[pin] "I" ( SERPIN )
		);
	return 1;
	}

__attribute__((always_inline, unused)) static inline void TinyDebugSerialWriterBangOneByte( uint8_t value, uint8_t lom, uint8_t him, uint8_t oloops, uint8_t iloops, uint8_t nops ) {
        uint8_t i;
        uint8_t j;
        uint8_t ol;
        uint8_t il;
        uint8_t b;  // Initialized to the low bits
        uint8_t hib;
        uint8_t m;
        
        b   = ((value << 1) & 0x1F);
        hib = ((value >> 4) & 0x1F) | 0x10;
        
        asm volatile
        (
        "ldi   %[j], 2"                           "\n\t"
        "ldi   %[i], 5"                           "\n\t"
        "ldi   %[m], %[lom]"                      "\n\t"

        // Note: 8 MHz, 9600 baud ---> disabling interrupts does not appear to be necessary

        "cli"                                     "\n\t"

        "rjmp  L%=ntop"                           "\n\t"

        "L%=btop: "
        "nop"                                     "\n\t"      // ---> 7
        "nop"                                     "\n\t"      //
        "nop"                                     "\n\t"      //
        "nop"                                     "\n\t"      //
        "nop"                                     "\n\t"      //
        "nop"                                     "\n\t"      //
        "nop"                                     "\n\t"      //

        "L%=ntop: "
        "ror   %[b]"                              "\n\t"      // ---> 1
        
        "brcs  L%=bxh"                            "\n\t"      // 1  (not taken)
        "cbi   %[serreg], %[serbit]"              "\n\t"      // 2
        "rjmp  L%=bxz"                            "\n\t"      // 2
        
        "L%=bxh: "                                              // 2  (taken)
        "sbi   %[serreg], %[serbit]"              "\n\t"      // 2
        "nop"                                     "\n\t"      // 1

        // ---> 5
        "L%=bxz: "

        "ror   %[m]"                              "\n\t"      // ---> 3 or 4
        "brcc  L%=bnoe"                           "\n\t"      //
        "nop"                                     "\n\t"      //
        "nop"                                     "\n\t"      //
        "L%=bnoe: "

        // ---> 1
        ".if %[oloops] >= 1"                        "\n\t"      // if oloops >= 1 then...
        "ldi   %[ol], %[oloops]"                  "\n\t"      // 4*oloops + oloops*(3*iloops) or oloops*((3*iloops)+4)
        "L%=odelay: "                               "\n\t"
        ".endif"                                    "\n\t"
        "ldi   %[il], %[iloops]"                  "\n\t"      // if oloops == 0 then...
        "L%=idelay: "                               "\n\t"      // (3*iloops)
        "dec   %[il]"                             "\n\t"
        "brne  L%=idelay"                         "\n\t"
        "nop"                                     "\n\t"
        ".if %[oloops] >= 1"                        "\n\t"
        "dec   %[ol]"                             "\n\t"
        "brne  L%=odelay"                         "\n\t"
        "nop"                                     "\n\t"
        ".endif"                                    "\n\t"

        ".if %[nops] >= 1"                          "\n\t"
        "nop"                                     "\n\t"      //
        ".endif"                                    "\n\t"
        ".if %[nops] >= 2"                          "\n\t"
        "nop"                                     "\n\t"      //
        ".endif"                                    "\n\t"

        "dec   %[i]"                              "\n\t"      // ---> 3
        "brne  L%=btop"                           "\n\t"      //
        "nop"                                     "\n\t"      //

        "dec   %[j]"                              "\n\t"      // ---> 7
        "breq  L%=bfin"                           "\n\t"      //
        "ldi   %[i], 5"                           "\n\t"      //
        "mov   %[b], %[hib]"                      "\n\t"      //
        "ldi   %[m], %[him]"                      "\n\t"      //
        "rjmp  L%=ntop"                           "\n\t"      //

        "L%=bfin: "

        "sei"                                     "\n\t"
        :
        [i] "=&r" ( i ),
        [j] "=&r" ( j ),
        [ol] "=&r" ( ol ),
        [il] "=&r" ( il ),
        [m] "=&r" ( m )
        :
        [b] "r" ( b ),
        [hib] "r" ( hib ),
        [serreg] "I" ( SERPORT ),
        [serbit] "M" ( SERPIN ),
        [lom] "M" ( lom ),
        [him] "M" ( him ),
        [oloops] "M" ( oloops ),
        [iloops] "M" ( iloops ),
        [nops] "M" ( nops )
        :
        "r31",
        "r30"
        );
    }

#define ser_write_16_115200(value) TinyDebugSerialWriterBangOneByte(value,0b11110,0b11111,0,39,1)
#define ser_write_8_115200(value) TinyDebugSerialWriterBangOneByte(value,0b01010,0b10100,0,16,1)

#define ser_write(value) ser_write_8_115200(value)

