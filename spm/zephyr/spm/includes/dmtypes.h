
#ifndef DM_TYPES_H
#define DM_TYPES_H

#include <zephyr/kernel.h>
#include <stddef.h>

//-----------------------------------------------------------------------------
// Platform built-in data types
//-----------------------------------------------------------------------------

// Unsigned integer types
typedef unsigned char       UINT8;
typedef unsigned short      UINT16;
typedef unsigned int        UINT32;
typedef unsigned long long  UINT64;

// Signed integer types - mention 'signed' explicitly or bitfields will malfunction
typedef signed char         INT8;
typedef signed short int    INT16;
typedef signed int          INT32;
typedef signed long long    INT64;

// Floating point types
typedef float               REAL32;
typedef double              REAL64;

//-----------------------------------------------------------------------------
// User-defined data types
//-----------------------------------------------------------------------------

// Boolean
typedef unsigned char BOOL;

// Byte
typedef unsigned char BYTE;

// Byte and bit access in unions
typedef union
{
    UINT16 iValue;
    struct
    {
        BYTE b0;
        BYTE b1;
    } Bytes;
} UInt16Bytes_t;

typedef union
{
    UINT32 iValue;
    struct
    {
        BYTE b0;
        BYTE b1;
        BYTE b2;
        BYTE b3;
    } Bytes;
} UInt32Bytes_t;

typedef union
{
    BYTE bValue;
    struct
    {
        BYTE b0 : 1;
        BYTE b1 : 1;
        BYTE b2 : 1;
        BYTE b3 : 1;
        BYTE b4 : 1;
        BYTE b5 : 1;
        BYTE b6 : 1;
        BYTE b7 : 1;
    } Bits;
} ByteBits_t;

#pragma pack(push,1)
typedef union
{
    UINT64 iU64;
    INT64  i64;
    REAL64 r64;
    UINT32 aiU32[2];
    INT32  ai32[2];
    REAL32 ar32[2];
    UINT16 aiU16[4];
    INT16  ai16[4];
    BYTE   ab[8];
    INT8   ai8[8];
} Pack_t;
#pragma pack(pop)

//-----------------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------------
// Values to be used with boolean (BOOL) variables
#ifdef FALSE
#undef FALSE
#endif
#define FALSE  0

#ifdef TRUE
#undef TRUE
#endif
#define TRUE   1

// Bit operations
#define SETBIT(var, bit)           var |= ((UINT32)1 << (bit))
#define SETBIT2(var, bit1, bit2)   var |= (((UINT32)1 << (bit1)) + ((UINT32)1 << (bit2)))

#define WRBIT(var, bit)            var = ((UINT32)1 << (bit))
#define WRBIT2(var, bit1, bit2)    var = (((UINT32)1 << (bit1)) + ((UINT32)1 << (bit2)))

#define CLRBIT(var, bit)           var &= (~(((UINT32)1 << (bit))))
#define CLRBIT2(var, bit1, bit2)   var &= (~(((UINT32)1 << (bit1)) + ((UINT32)1 << (bit2))))

#define SETCLRBIT(var, bit1, bit2) var = ((var) & ~(((UINT32)1 << (bit2)))) | ((UINT32)1 << (bit1))
#define CLRSETBIT(var, bit1, bit2) var = ((var) & ~(((UINT32)1 << (bit1)))) | ((UINT32)1 << (bit2))

#define MASKBIT(var, bit) ((var) & ((UINT32)1 << (bit)))

#define BINARY(b7, b6, b5, b4, b3, b2, b1, b0) ((b7) * 128 + (b6) * 64 + (b5) * 32 + (b4) * 16 + (b3) * 8 + (b2) * 4 + (b1) * 2 + (b0))

#define CLRMASK(var, mask)  var &= (~(mask))
#define SETMASK(var, mask)  var |= (mask)

// This macro will generate an error at compile time, and may contain sizeof(), which is illegal in #if's.
// Unfortunately it can't print a nice error message...
#define COMPILE_TIME_ASSERT(pred) typedef char static_assert_failed[(pred) ? 1 : -1]


// bigsmall parameter macros
#define HRMIN_BYTE_TO_SECS(x)   ((x) & 0x80 ? (((x) & 0x7F) * 3600) : ((x) * 60))
#define HRMIN_BYTE_TO_MIN(x)    ((x) & 0x80 ? (((x) & 0x7F) * 60) : (x))
#define MINSEC_BYTE_TO_SECS(x)  ((x) & 0x80 ? (((x) & 0x7F) * 60) : (x))

// Validation helper macros
#define CHECK_MIN(var, min)                do{ if ((var) < (min)) (var) = (min); } while(0)
#define CHECK_MAX(var, max)                do{ if ((var) > (max)) (var) = (max); } while(0)
#define CHECK_RANGE(var, min, max)         do{ CHECK_MIN(var, min);              \
                                               CHECK_MAX(var, max);              } while(0)
#define CHECK_MIN_OR(var, min, def)        do{ if ((var) < (min)) (var) = (def); } while(0)
#define CHECK_MAX_OR(var, max, def)        do{ if ((var) > (max)) (var) = (def); } while(0)
#define CHECK_RANGE_OR(var, min, max, def) do{ CHECK_MIN_OR(var, min, def);      \
                                               CHECK_MAX_OR(var, max, def);      } while(0)

// Finds container of given member
#define container_of(ptr, type, member) ((type*) ((BYTE*)(ptr) - offsetof(type, member)))

// Checks if x is a positive power of 2
#define IS_PWR_OF_2(u32) ((((UINT32)(u32)) != 0) && ((((UINT32)(u32)) & (~((UINT32)(u32)) + 1)) == ((UINT32)(u32))))

// Atomic function for Zephyr
#define ATOMIC_ZEPHYR(stmt) {INT32 iIrqKey = irq_lock(); stmt; irq_unlock(iIrqKey);} 

#endif // DM_TYPES_H

// End of file

