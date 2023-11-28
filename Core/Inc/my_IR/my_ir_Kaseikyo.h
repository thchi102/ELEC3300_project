#ifndef _IR_KASEIKYO_H
#define _IR_KASEIKYO_H

#define KASEIKYO_VENDOR_ID_BITS     16
#define KASEIKYO_VENDOR_ID_PARITY_BITS   4
#define KASEIKYO_ADDRESS_BITS       12
#define KASEIKYO_COMMAND_BITS       8
#define KASEIKYO_PARITY_BITS        8
#define KASEIKYO_BITS               (KASEIKYO_VENDOR_ID_BITS + KASEIKYO_VENDOR_ID_PARITY_BITS + KASEIKYO_ADDRESS_BITS + KASEIKYO_COMMAND_BITS + KASEIKYO_PARITY_BITS) // 48
#define KASEIKYO_UNIT               432 // 16 pulses of 37 kHz (432,432432)  - Pronto 0x70 | 0x10

#define KASEIKYO_HEADER_MARK        (8 * KASEIKYO_UNIT) // 3456
#define KASEIKYO_HEADER_SPACE       (4 * KASEIKYO_UNIT) // 1728

#define KASEIKYO_BIT_MARK           KASEIKYO_UNIT
#define KASEIKYO_ONE_SPACE          (3 * KASEIKYO_UNIT) // 1296
#define KASEIKYO_ZERO_SPACE         KASEIKYO_UNIT

#define KASEIKYO_AVERAGE_DURATION   56000
#define KASEIKYO_REPEAT_PERIOD      130000
#define KASEIKYO_REPEAT_DISTANCE    (KASEIKYO_REPEAT_PERIOD - KASEIKYO_AVERAGE_DURATION) // 74 ms
#define KASEIKYO_MAXIMUM_REPEAT_DISTANCE    (KASEIKYO_REPEAT_DISTANCE + (KASEIKYO_REPEAT_DISTANCE / 4)) // Just a guess

#define PANASONIC_VENDOR_ID_CODE    0x2002
#define DENON_VENDOR_ID_CODE        0x3254
#define MITSUBISHI_VENDOR_ID_CODE   0xCB23
#define SHARP_VENDOR_ID_CODE        0x5AAA
#define JVC_VENDOR_ID_CODE          0x0103

struct PulseDistanceWidthProtocolConstants KaseikyoProtocolConstants = { KASEIKYO, KASEIKYO_KHZ, KASEIKYO_HEADER_MARK,
KASEIKYO_HEADER_SPACE, KASEIKYO_BIT_MARK, KASEIKYO_ONE_SPACE, KASEIKYO_BIT_MARK, KASEIKYO_ZERO_SPACE, PROTOCOL_IS_LSB_FIRST
       , (KASEIKYO_REPEAT_PERIOD / MICROS_IN_ONE_MILLI), NULL };




#endif