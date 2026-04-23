

#define __LZO_IN_MINILZO 1

#if defined(LZO_CFG_FREESTANDING)
#  define LZO_LIBC_FREESTANDING 1
#  define LZO_OS_FREESTANDING 1
#endif

#include <limits.h>
#include <stddef.h>

#include "minilzo.h"

#ifndef __LZO_CONF_H
#define __LZO_CONF_H 1

#  pragma warning(disable: 4702)
#  pragma warning(disable: 4127 4701)
#  pragma warning(disable: 4514 4710 4711)
#  pragma warning(disable: 4820)
#  pragma warning(disable: 4746)

#if (LZO_CC_SUNPROC)
#if !defined(__cplusplus)
#  pragma error_messages(off,E_END_OF_LOOP_CODE_NOT_REACHED)
#  pragma error_messages(off,E_LOOP_NOT_ENTERED_AT_TOP)
#  pragma error_messages(off,E_STATEMENT_NOT_REACHED)
#endif
#endif

#if !defined(__LZO_NOEXPORT1)
#  define __LZO_NOEXPORT1       /*empty*/
#endif
#if !defined(__LZO_NOEXPORT2)
#  define __LZO_NOEXPORT2       /*empty*/
#endif

#if 1
#  define LZO_PUBLIC_DECL(r)    LZO_EXTERN(r)
#endif
#if 1
#  define LZO_PUBLIC_IMPL(r)    LZO_PUBLIC(r)
#endif
#if !defined(LZO_LOCAL_DECL)
#  define LZO_LOCAL_DECL(r)     __LZO_EXTERN_C LZO_LOCAL_IMPL(r)
#endif
#if !defined(LZO_LOCAL_IMPL)
#  define LZO_LOCAL_IMPL(r)     __LZO_NOEXPORT1 r __LZO_NOEXPORT2 __LZO_CDECL
#endif
#if 1
#  define LZO_STATIC_DECL(r)    LZO_PRIVATE(r)
#endif
#if 1
#  define LZO_STATIC_IMPL(r)    LZO_PRIVATE(r)
#endif

#if 1
#elif 1
#  include <string.h>
#else
#  define LZO_WANT_ACC_INCD_H 1
#endif
#if defined(LZO_HAVE_CONFIG_H)
#  define LZO_CFG_NO_CONFIG_HEADER 1
#endif

#if 1 && !defined(LZO_CFG_FREESTANDING)
#if 1 && !defined(HAVE_STRING_H)
#define HAVE_STRING_H 1
#endif

#define HAVE_MEMCMP 1
#define HAVE_MEMCPY 1
#define HAVE_MEMMOVE 1
#define HAVE_MEMSET 1

#endif

#if 1 && defined(HAVE_STRING_H)

#include <string.h>

#endif

#if 1 || defined(lzo_int8_t) || defined(lzo_uint8_t)
LZO_COMPILE_TIME_ASSERT_HEADER(sizeof(lzo_int8_t) == 1)
LZO_COMPILE_TIME_ASSERT_HEADER(sizeof(lzo_uint8_t) == 1)
#endif
#if 1 || defined(lzo_int16_t) || defined(lzo_uint16_t)
LZO_COMPILE_TIME_ASSERT_HEADER(sizeof(lzo_int16_t) == 2)
LZO_COMPILE_TIME_ASSERT_HEADER(sizeof(lzo_uint16_t) == 2)
#endif
#if 1 || defined(lzo_int32_t) || defined(lzo_uint32_t)
LZO_COMPILE_TIME_ASSERT_HEADER(sizeof(lzo_int32_t) == 4)
LZO_COMPILE_TIME_ASSERT_HEADER(sizeof(lzo_uint32_t) == 4)
#endif
#if defined(lzo_int64_t) || defined(lzo_uint64_t)
LZO_COMPILE_TIME_ASSERT_HEADER(sizeof(lzo_int64_t)  == 8)
LZO_COMPILE_TIME_ASSERT_HEADER(sizeof(lzo_uint64_t) == 8)
#endif

#if (LZO_CFG_FREESTANDING)
#  undef HAVE_MEMCMP
#  undef HAVE_MEMCPY
#  undef HAVE_MEMMOVE
#  undef HAVE_MEMSET
#endif

#if (HAVE_MEMCMP)
#  undef lzo_memcmp
#  define lzo_memcmp(a, b, c)     memcmp(a,b,c)
#endif
#if (HAVE_MEMCPY)
#  undef lzo_memcpy
#  define lzo_memcpy(a, b, c)     memcpy(a,b,c)
#endif
#if (HAVE_MEMMOVE)
#  undef lzo_memmove
#  define lzo_memmove(a, b, c)    memmove(a,b,c)
#endif
#if (HAVE_MEMSET)
#  undef lzo_memset
#  define lzo_memset(a, b, c)     memset(a,b,c)
#endif

#undef NDEBUG
#if (LZO_CFG_FREESTANDING)
#  undef LZO_DEBUG
#  define NDEBUG 1
#  undef assert
#  define assert(e) ((void)0)
#else
#  if !defined(LZO_DEBUG)
#    define NDEBUG 1
#  endif

#  include <assert.h>

#endif

#define BOUNDS_CHECKING_OFF_DURING(stmt)      stmt
#define BOUNDS_CHECKING_OFF_IN_EXPR(expr)     (expr)

#if (LZO_CFG_PGO)
#  undef __lzo_likely
#  undef __lzo_unlikely
#  define __lzo_likely(e)       (e)
#  define __lzo_unlikely(e)     (e)
#endif

#undef _
#undef __
#undef ___
#undef ____
#undef _p0
#undef _p1
#undef _p2
#undef _p3
#undef _p4
#undef _s0
#undef _s1
#undef _s2
#undef _s3
#undef _s4
#undef _ww

#if 1
#  define LZO_BYTE(x)       ((unsigned char) (x))
#else
#  define LZO_BYTE(x)       ((unsigned char) ((x) & 0xff))
#endif

#define LZO_MAX(a, b)        ((a) >= (b) ? (a) : (b))
#define LZO_MIN(a, b)        ((a) <= (b) ? (a) : (b))
#define LZO_MAX3(a, b, c)     ((a) >= (b) ? LZO_MAX(a,c) : LZO_MAX(b,c))
#define LZO_MIN3(a, b, c)     ((a) <= (b) ? LZO_MIN(a,c) : LZO_MIN(b,c))

#define lzo_sizeof(type)    ((lzo_uint) (sizeof(type)))

#define LZO_HIGH(array)     ((lzo_uint) (sizeof(array)/sizeof(*(array))))

#define LZO_SIZE(bits)      (1u << (bits))
#define LZO_MASK(bits)      (LZO_SIZE(bits) - 1)

#define LZO_USIZE(bits)     ((lzo_uint) 1 << (bits))
#define LZO_UMASK(bits)     (LZO_USIZE(bits) - 1)

#if !defined(DMUL)

#define DMUL(a, b) ((lzo_xint) ((a) * (b)))

#endif

#ifndef __LZO_FUNC_H
#define __LZO_FUNC_H 1

#if !defined(LZO_BITOPS_USE_ASM_BITSCAN) && !defined(LZO_BITOPS_USE_GNUC_BITSCAN) && !defined(LZO_BITOPS_USE_MSC_BITSCAN)
#if 1 && (LZO_ARCH_AMD64) && (LZO_CC_GNUC && (LZO_CC_GNUC < 0x040000ul)) && (LZO_ASM_SYNTAX_GNUC)
#define LZO_BITOPS_USE_ASM_BITSCAN 1
#elif (LZO_CC_CLANG || (LZO_CC_GNUC >= 0x030400ul) || (LZO_CC_INTELC_GNUC && (__INTEL_COMPILER >= 1000)) || (LZO_CC_LLVM && (!defined(__llvm_tools_version__) || (__llvm_tools_version__ + 0 >= 0x010500ul))))
#define LZO_BITOPS_USE_GNUC_BITSCAN 1
#elif (LZO_OS_WIN32 || LZO_OS_WIN64) && ((LZO_CC_INTELC_MSC && (__INTEL_COMPILER >= 1010)) || (LZO_CC_MSC && (_MSC_VER >= 1400)))
#define LZO_BITOPS_USE_MSC_BITSCAN 1
#if (LZO_CC_MSC) && (LZO_ARCH_AMD64 || LZO_ARCH_I386)
#include <intrin.h>
#endif
#if (LZO_CC_MSC) && (LZO_ARCH_AMD64 || LZO_ARCH_I386)
#pragma intrinsic(_BitScanReverse)
#pragma intrinsic(_BitScanForward)
#endif
#if (LZO_CC_MSC) && (LZO_ARCH_AMD64)
#pragma intrinsic(_BitScanReverse64)
#pragma intrinsic(_BitScanForward64)
#endif
#endif
#endif

__lzo_static_forceinline unsigned lzo_bitops_ctlz32_func(lzo_uint32_t v) {
#if (LZO_BITOPS_USE_MSC_BITSCAN) && (LZO_ARCH_AMD64 || LZO_ARCH_I386)
    unsigned long r; (void) _BitScanReverse(&r, v); return (unsigned) r ^ 31;
#define lzo_bitops_ctlz32(v)    lzo_bitops_ctlz32_func(v)
#elif (LZO_BITOPS_USE_ASM_BITSCAN) && (LZO_ARCH_AMD64 || LZO_ARCH_I386) && (LZO_ASM_SYNTAX_GNUC)
    lzo_uint32_t r;
    __asm__("bsr %1,%0" : "=r" (r) : "rm" (v) __LZO_ASM_CLOBBER_LIST_CC);
    return (unsigned) r ^ 31;
#define lzo_bitops_ctlz32(v)    lzo_bitops_ctlz32_func(v)
#elif (LZO_BITOPS_USE_GNUC_BITSCAN) && (LZO_SIZEOF_INT == 4)
    unsigned r; r = (unsigned) __builtin_clz(v); return r;
#define lzo_bitops_ctlz32(v)    ((unsigned) __builtin_clz(v))
#elif (LZO_BITOPS_USE_GNUC_BITSCAN) && (LZO_SIZEOF_LONG == 8) && (LZO_WORDSIZE >= 8)
    unsigned r; r = (unsigned) __builtin_clzl(v); return r ^ 32;
#define lzo_bitops_ctlz32(v)    (((unsigned) __builtin_clzl(v)) ^ 32)
#else
    LZO_UNUSED(v);
    return 0;
#endif
}

#if defined(lzo_uint64_t)
__lzo_static_forceinline unsigned lzo_bitops_ctlz64_func(lzo_uint64_t v)
{
#if (LZO_BITOPS_USE_MSC_BITSCAN) && (LZO_ARCH_AMD64)
    unsigned long r; (void) _BitScanReverse64(&r, v); return (unsigned) r ^ 63;
#define lzo_bitops_ctlz64(v)    lzo_bitops_ctlz64_func(v)
#elif (LZO_BITOPS_USE_ASM_BITSCAN) && (LZO_ARCH_AMD64) && (LZO_ASM_SYNTAX_GNUC)
    lzo_uint64_t r;
    __asm__("bsr %1,%0" : "=r" (r) : "rm" (v) __LZO_ASM_CLOBBER_LIST_CC);
    return (unsigned) r ^ 63;
#define lzo_bitops_ctlz64(v)    lzo_bitops_ctlz64_func(v)
#elif (LZO_BITOPS_USE_GNUC_BITSCAN) && (LZO_SIZEOF_LONG == 8) && (LZO_WORDSIZE >= 8)
    unsigned r; r = (unsigned) __builtin_clzl(v); return r;
#define lzo_bitops_ctlz64(v)    ((unsigned) __builtin_clzl(v))
#elif (LZO_BITOPS_USE_GNUC_BITSCAN) && (LZO_SIZEOF_LONG_LONG == 8) && (LZO_WORDSIZE >= 8)
    unsigned r; r = (unsigned) __builtin_clzll(v); return r;
#define lzo_bitops_ctlz64(v)    ((unsigned) __builtin_clzll(v))
#else
    LZO_UNUSED(v); return 0;
#endif
}
#endif

__lzo_static_forceinline unsigned lzo_bitops_cttz32_func(lzo_uint32_t v) {
#if (LZO_BITOPS_USE_MSC_BITSCAN) && (LZO_ARCH_AMD64 || LZO_ARCH_I386)
    unsigned long r; (void) _BitScanForward(&r, v); return (unsigned) r;
#define lzo_bitops_cttz32(v)    lzo_bitops_cttz32_func(v)
#elif (LZO_BITOPS_USE_ASM_BITSCAN) && (LZO_ARCH_AMD64 || LZO_ARCH_I386) && (LZO_ASM_SYNTAX_GNUC)
    lzo_uint32_t r;
    __asm__("bsf %1,%0" : "=r" (r) : "rm" (v) __LZO_ASM_CLOBBER_LIST_CC);
    return (unsigned) r;
#define lzo_bitops_cttz32(v)    lzo_bitops_cttz32_func(v)
#elif (LZO_BITOPS_USE_GNUC_BITSCAN) && (LZO_SIZEOF_INT >= 4)
    unsigned r; r = (unsigned) __builtin_ctz(v); return r;
#define lzo_bitops_cttz32(v)    ((unsigned) __builtin_ctz(v))
#else
    LZO_UNUSED(v);
    return 0;
#endif
}

#if defined(lzo_uint64_t)
__lzo_static_forceinline unsigned lzo_bitops_cttz64_func(lzo_uint64_t v)
{
#if (LZO_BITOPS_USE_MSC_BITSCAN) && (LZO_ARCH_AMD64)
    unsigned long r; (void) _BitScanForward64(&r, v); return (unsigned) r;
#define lzo_bitops_cttz64(v)    lzo_bitops_cttz64_func(v)
#elif (LZO_BITOPS_USE_ASM_BITSCAN) && (LZO_ARCH_AMD64) && (LZO_ASM_SYNTAX_GNUC)
    lzo_uint64_t r;
    __asm__("bsf %1,%0" : "=r" (r) : "rm" (v) __LZO_ASM_CLOBBER_LIST_CC);
    return (unsigned) r;
#define lzo_bitops_cttz64(v)    lzo_bitops_cttz64_func(v)
#elif (LZO_BITOPS_USE_GNUC_BITSCAN) && (LZO_SIZEOF_LONG >= 8) && (LZO_WORDSIZE >= 8)
    unsigned r; r = (unsigned) __builtin_ctzl(v); return r;
#define lzo_bitops_cttz64(v)    ((unsigned) __builtin_ctzl(v))
#elif (LZO_BITOPS_USE_GNUC_BITSCAN) && (LZO_SIZEOF_LONG_LONG >= 8) && (LZO_WORDSIZE >= 8)
    unsigned r; r = (unsigned) __builtin_ctzll(v); return r;
#define lzo_bitops_cttz64(v)    ((unsigned) __builtin_ctzll(v))
#else
    LZO_UNUSED(v); return 0;
#endif
}
#endif

lzo_unused_funcs_impl(void, lzo_bitops_unused_funcs)(void) {
    LZO_UNUSED_FUNC(lzo_bitops_unused_funcs);
    LZO_UNUSED_FUNC(lzo_bitops_ctlz32_func);
    LZO_UNUSED_FUNC(lzo_bitops_cttz32_func);
#if defined(lzo_uint64_t)
    LZO_UNUSED_FUNC(lzo_bitops_ctlz64_func);
    LZO_UNUSED_FUNC(lzo_bitops_cttz64_func);
#endif
}

#if defined(__lzo_alignof) && !(LZO_CFG_NO_UNALIGNED)
#if !defined(lzo_memops_tcheck__) && 0
#define lzo_memops_tcheck__(t,a,b) ((void)0, sizeof(t) == (a) && __lzo_alignof(t) == (b))
#endif
#endif
#ifndef lzo_memops_TU0p
#define lzo_memops_TU0p void __LZO_MMODEL *
#endif
#ifndef lzo_memops_TU1p
#define lzo_memops_TU1p unsigned char __LZO_MMODEL *
#endif
#ifndef lzo_memops_TU2p
#if (LZO_OPT_UNALIGNED16)
typedef lzo_uint16_t __lzo_may_alias lzo_memops_TU2;
#define lzo_memops_TU2p volatile lzo_memops_TU2 *
#elif defined(__lzo_byte_struct)
__lzo_byte_struct(lzo_memops_TU2_struct,2)
typedef struct lzo_memops_TU2_struct lzo_memops_TU2;
#else
struct lzo_memops_TU2_struct {
    unsigned char a[2];
} __lzo_may_alias;
typedef struct lzo_memops_TU2_struct lzo_memops_TU2;
#endif
#ifndef lzo_memops_TU2p
#define lzo_memops_TU2p lzo_memops_TU2 *
#endif
#endif
#ifndef lzo_memops_TU4p
#if (LZO_OPT_UNALIGNED32)
typedef lzo_uint32_t __lzo_may_alias lzo_memops_TU4;
#define lzo_memops_TU4p volatile lzo_memops_TU4 __LZO_MMODEL *
#elif defined(__lzo_byte_struct)
__lzo_byte_struct(lzo_memops_TU4_struct,4)
typedef struct lzo_memops_TU4_struct lzo_memops_TU4;
#else
struct lzo_memops_TU4_struct {
    unsigned char a[4];
} __lzo_may_alias;
typedef struct lzo_memops_TU4_struct lzo_memops_TU4;
#endif
#ifndef lzo_memops_TU4p
#define lzo_memops_TU4p lzo_memops_TU4 __LZO_MMODEL *
#endif
#endif
#ifndef lzo_memops_TU8p
#if (LZO_OPT_UNALIGNED64)
typedef lzo_uint64_t __lzo_may_alias lzo_memops_TU8;
#define lzo_memops_TU8p volatile lzo_memops_TU8 __LZO_MMODEL *
#elif defined(__lzo_byte_struct)
__lzo_byte_struct(lzo_memops_TU8_struct,8)
typedef struct lzo_memops_TU8_struct lzo_memops_TU8;
#else
struct lzo_memops_TU8_struct {
    unsigned char a[8];
} __lzo_may_alias;
typedef struct lzo_memops_TU8_struct lzo_memops_TU8;
#endif
#ifndef lzo_memops_TU8p
#define lzo_memops_TU8p lzo_memops_TU8 __LZO_MMODEL *
#endif
#endif
#ifndef lzo_memops_set_TU1p
#define lzo_memops_set_TU1p     volatile lzo_memops_TU1p
#endif
#ifndef lzo_memops_move_TU1p
#define lzo_memops_move_TU1p    lzo_memops_TU1p
#endif
#define LZO_MEMOPS_SET1(dd, cc) \
    LZO_BLOCK_BEGIN \
    lzo_memops_set_TU1p d__1 = (lzo_memops_set_TU1p) (lzo_memops_TU0p) (dd); \
    d__1[0] = LZO_BYTE(cc); \
    LZO_BLOCK_END
#define LZO_MEMOPS_SET2(dd, cc) \
    LZO_BLOCK_BEGIN \
    lzo_memops_set_TU1p d__2 = (lzo_memops_set_TU1p) (lzo_memops_TU0p) (dd); \
    d__2[0] = LZO_BYTE(cc); d__2[1] = LZO_BYTE(cc); \
    LZO_BLOCK_END
#define LZO_MEMOPS_SET3(dd, cc) \
    LZO_BLOCK_BEGIN \
    lzo_memops_set_TU1p d__3 = (lzo_memops_set_TU1p) (lzo_memops_TU0p) (dd); \
    d__3[0] = LZO_BYTE(cc); d__3[1] = LZO_BYTE(cc); d__3[2] = LZO_BYTE(cc); \
    LZO_BLOCK_END
#define LZO_MEMOPS_SET4(dd, cc) \
    LZO_BLOCK_BEGIN \
    lzo_memops_set_TU1p d__4 = (lzo_memops_set_TU1p) (lzo_memops_TU0p) (dd); \
    d__4[0] = LZO_BYTE(cc); d__4[1] = LZO_BYTE(cc); d__4[2] = LZO_BYTE(cc); d__4[3] = LZO_BYTE(cc); \
    LZO_BLOCK_END
#define LZO_MEMOPS_MOVE1(dd, ss) \
    LZO_BLOCK_BEGIN \
    lzo_memops_move_TU1p d__1 = (lzo_memops_move_TU1p) (lzo_memops_TU0p) (dd); \
    const lzo_memops_move_TU1p s__1 = (const lzo_memops_move_TU1p) (const lzo_memops_TU0p) (ss); \
    d__1[0] = s__1[0]; \
    LZO_BLOCK_END
#define LZO_MEMOPS_MOVE2(dd, ss) \
    LZO_BLOCK_BEGIN \
    lzo_memops_move_TU1p d__2 = (lzo_memops_move_TU1p) (lzo_memops_TU0p) (dd); \
    const lzo_memops_move_TU1p s__2 = (const lzo_memops_move_TU1p) (const lzo_memops_TU0p) (ss); \
    d__2[0] = s__2[0]; d__2[1] = s__2[1]; \
    LZO_BLOCK_END
#define LZO_MEMOPS_MOVE3(dd, ss) \
    LZO_BLOCK_BEGIN \
    lzo_memops_move_TU1p d__3 = (lzo_memops_move_TU1p) (lzo_memops_TU0p) (dd); \
    const lzo_memops_move_TU1p s__3 = (const lzo_memops_move_TU1p) (const lzo_memops_TU0p) (ss); \
    d__3[0] = s__3[0]; d__3[1] = s__3[1]; d__3[2] = s__3[2]; \
    LZO_BLOCK_END
#define LZO_MEMOPS_MOVE4(dd, ss) \
    LZO_BLOCK_BEGIN \
    lzo_memops_move_TU1p d__4 = (lzo_memops_move_TU1p) (lzo_memops_TU0p) (dd); \
    const lzo_memops_move_TU1p s__4 = (const lzo_memops_move_TU1p) (const lzo_memops_TU0p) (ss); \
    d__4[0] = s__4[0]; d__4[1] = s__4[1]; d__4[2] = s__4[2]; d__4[3] = s__4[3]; \
    LZO_BLOCK_END
#define LZO_MEMOPS_MOVE8(dd, ss) \
    LZO_BLOCK_BEGIN \
    lzo_memops_move_TU1p d__8 = (lzo_memops_move_TU1p) (lzo_memops_TU0p) (dd); \
    const lzo_memops_move_TU1p s__8 = (const lzo_memops_move_TU1p) (const lzo_memops_TU0p) (ss); \
    d__8[0] = s__8[0]; d__8[1] = s__8[1]; d__8[2] = s__8[2]; d__8[3] = s__8[3]; \
    d__8[4] = s__8[4]; d__8[5] = s__8[5]; d__8[6] = s__8[6]; d__8[7] = s__8[7]; \
    LZO_BLOCK_END
LZO_COMPILE_TIME_ASSERT_HEADER(sizeof(*(lzo_memops_TU1p) 0) == 1)
#define LZO_MEMOPS_COPY1(dd, ss) LZO_MEMOPS_MOVE1(dd,ss)
#if (LZO_OPT_UNALIGNED16)
LZO_COMPILE_TIME_ASSERT_HEADER(sizeof(*(lzo_memops_TU2p)0)==2)
#define LZO_MEMOPS_COPY2(dd,ss) \
    * (lzo_memops_TU2p) (lzo_memops_TU0p) (dd) = * (const lzo_memops_TU2p) (const lzo_memops_TU0p) (ss)
#elif defined(lzo_memops_tcheck__)
#define LZO_MEMOPS_COPY2(dd,ss) \
    LZO_BLOCK_BEGIN if (lzo_memops_tcheck__(lzo_memops_TU2,2,1)) { \
        * (lzo_memops_TU2p) (lzo_memops_TU0p) (dd) = * (const lzo_memops_TU2p) (const lzo_memops_TU0p) (ss); \
    } else { LZO_MEMOPS_MOVE2(dd,ss); } LZO_BLOCK_END
#else
#define LZO_MEMOPS_COPY2(dd, ss) LZO_MEMOPS_MOVE2(dd,ss)
#endif
#if (LZO_OPT_UNALIGNED32)
LZO_COMPILE_TIME_ASSERT_HEADER(sizeof(*(lzo_memops_TU4p)0)==4)
#define LZO_MEMOPS_COPY4(dd,ss) \
    * (lzo_memops_TU4p) (lzo_memops_TU0p) (dd) = * (const lzo_memops_TU4p) (const lzo_memops_TU0p) (ss)
#elif defined(lzo_memops_tcheck__)
#define LZO_MEMOPS_COPY4(dd,ss) \
    LZO_BLOCK_BEGIN if (lzo_memops_tcheck__(lzo_memops_TU4,4,1)) { \
        * (lzo_memops_TU4p) (lzo_memops_TU0p) (dd) = * (const lzo_memops_TU4p) (const lzo_memops_TU0p) (ss); \
    } else { LZO_MEMOPS_MOVE4(dd,ss); } LZO_BLOCK_END
#else
#define LZO_MEMOPS_COPY4(dd, ss) LZO_MEMOPS_MOVE4(dd,ss)
#endif
#if (LZO_WORDSIZE != 8)
#define LZO_MEMOPS_COPY8(dd, ss) \
    LZO_BLOCK_BEGIN LZO_MEMOPS_COPY4(dd,ss); LZO_MEMOPS_COPY4((lzo_memops_TU1p)(lzo_memops_TU0p)(dd)+4,(const lzo_memops_TU1p)(const lzo_memops_TU0p)(ss)+4); LZO_BLOCK_END
#else
#if (LZO_OPT_UNALIGNED64)
LZO_COMPILE_TIME_ASSERT_HEADER(sizeof(*(lzo_memops_TU8p)0)==8)
#define LZO_MEMOPS_COPY8(dd,ss) \
    * (lzo_memops_TU8p) (lzo_memops_TU0p) (dd) = * (const lzo_memops_TU8p) (const lzo_memops_TU0p) (ss)
#elif (LZO_OPT_UNALIGNED32)
#define LZO_MEMOPS_COPY8(dd,ss) \
    LZO_BLOCK_BEGIN LZO_MEMOPS_COPY4(dd,ss); LZO_MEMOPS_COPY4((lzo_memops_TU1p)(lzo_memops_TU0p)(dd)+4,(const lzo_memops_TU1p)(const lzo_memops_TU0p)(ss)+4); LZO_BLOCK_END
#elif defined(lzo_memops_tcheck__)
#define LZO_MEMOPS_COPY8(dd,ss) \
    LZO_BLOCK_BEGIN if (lzo_memops_tcheck__(lzo_memops_TU8,8,1)) { \
        * (lzo_memops_TU8p) (lzo_memops_TU0p) (dd) = * (const lzo_memops_TU8p) (const lzo_memops_TU0p) (ss); \
    } else { LZO_MEMOPS_MOVE8(dd,ss); } LZO_BLOCK_END
#else
#define LZO_MEMOPS_COPY8(dd,ss) LZO_MEMOPS_MOVE8(dd,ss)
#endif
#endif
#define LZO_MEMOPS_COPYN(dd, ss, nn) \
    LZO_BLOCK_BEGIN \
    lzo_memops_TU1p d__n = (lzo_memops_TU1p) (lzo_memops_TU0p) (dd); \
    const lzo_memops_TU1p s__n = (const lzo_memops_TU1p) (const lzo_memops_TU0p) (ss); \
    lzo_uint n__n = (nn); \
    while ((void)0, n__n >= 8) { LZO_MEMOPS_COPY8(d__n, s__n); d__n += 8; s__n += 8; n__n -= 8; } \
    if ((void)0, n__n >= 4) { LZO_MEMOPS_COPY4(d__n, s__n); d__n += 4; s__n += 4; n__n -= 4; } \
    if ((void)0, n__n > 0) do { *d__n++ = *s__n++; } while (--n__n > 0); \
    LZO_BLOCK_END

__lzo_static_forceinline lzo_uint16_t lzo_memops_get_le16(const lzo_voidp ss) {
    lzo_uint16_t v;
#if (LZO_ABI_LITTLE_ENDIAN)
    LZO_MEMOPS_COPY2(&v, ss);
#elif (LZO_OPT_UNALIGNED16 && LZO_ARCH_POWERPC && LZO_ABI_BIG_ENDIAN) && (LZO_ASM_SYNTAX_GNUC)
    const lzo_memops_TU2p s = (const lzo_memops_TU2p) ss;
    unsigned long vv;
    __asm__("lhbrx %0,0,%1" : "=r" (vv) : "r" (s), "m" (*s));
    v = (lzo_uint16_t) vv;
#else
    const lzo_memops_TU1p s = (const lzo_memops_TU1p) ss;
    v = (lzo_uint16_t)(((lzo_uint16_t) s[0]) | ((lzo_uint16_t) s[1] << 8));
#endif
    return v;
}

#if (LZO_OPT_UNALIGNED16) && (LZO_ABI_LITTLE_ENDIAN)
#define LZO_MEMOPS_GET_LE16(ss)    (* (const lzo_memops_TU2p) (const lzo_memops_TU0p) (ss))
#else
#define LZO_MEMOPS_GET_LE16(ss)    lzo_memops_get_le16(ss)
#endif

__lzo_static_forceinline lzo_uint32_t lzo_memops_get_le32(const lzo_voidp ss) {
    lzo_uint32_t v;
#if (LZO_ABI_LITTLE_ENDIAN)
    LZO_MEMOPS_COPY4(&v, ss);
#elif (LZO_OPT_UNALIGNED32 && LZO_ARCH_POWERPC && LZO_ABI_BIG_ENDIAN) && (LZO_ASM_SYNTAX_GNUC)
    const lzo_memops_TU4p s = (const lzo_memops_TU4p) ss;
    unsigned long vv;
    __asm__("lwbrx %0,0,%1" : "=r" (vv) : "r" (s), "m" (*s));
    v = (lzo_uint32_t) vv;
#else
    const lzo_memops_TU1p s = (const lzo_memops_TU1p) ss;
    v = (lzo_uint32_t)(((lzo_uint32_t) s[0]) | ((lzo_uint32_t) s[1] << 8) | ((lzo_uint32_t) s[2] << 16) |
                       ((lzo_uint32_t) s[3] << 24));
#endif
    return v;
}

#if (LZO_OPT_UNALIGNED32) && (LZO_ABI_LITTLE_ENDIAN)
#define LZO_MEMOPS_GET_LE32(ss)    (* (const lzo_memops_TU4p) (const lzo_memops_TU0p) (ss))
#else
#define LZO_MEMOPS_GET_LE32(ss)    lzo_memops_get_le32(ss)
#endif

#if (LZO_OPT_UNALIGNED64) && (LZO_ABI_LITTLE_ENDIAN)
#define LZO_MEMOPS_GET_LE64(ss)    (* (const lzo_memops_TU8p) (const lzo_memops_TU0p) (ss))
#endif

__lzo_static_forceinline lzo_uint16_t lzo_memops_get_ne16(const lzo_voidp ss) {
    lzo_uint16_t v;
    LZO_MEMOPS_COPY2(&v, ss);
    return v;
}

#if (LZO_OPT_UNALIGNED16)
LZO_COMPILE_TIME_ASSERT_HEADER(sizeof(*(lzo_memops_TU2p)0)==2)
#define LZO_MEMOPS_GET_NE16(ss)    (* (const lzo_memops_TU2p) (const lzo_memops_TU0p) (ss))
#else
#define LZO_MEMOPS_GET_NE16(ss)    lzo_memops_get_ne16(ss)
#endif

__lzo_static_forceinline lzo_uint32_t lzo_memops_get_ne32(const lzo_voidp ss) {
    lzo_uint32_t v;
    LZO_MEMOPS_COPY4(&v, ss);
    return v;
}

#if (LZO_OPT_UNALIGNED32)
LZO_COMPILE_TIME_ASSERT_HEADER(sizeof(*(lzo_memops_TU4p)0)==4)
#define LZO_MEMOPS_GET_NE32(ss)    (* (const lzo_memops_TU4p) (const lzo_memops_TU0p) (ss))
#else
#define LZO_MEMOPS_GET_NE32(ss)    lzo_memops_get_ne32(ss)
#endif

#if (LZO_OPT_UNALIGNED64)
LZO_COMPILE_TIME_ASSERT_HEADER(sizeof(*(lzo_memops_TU8p)0)==8)
#define LZO_MEMOPS_GET_NE64(ss)    (* (const lzo_memops_TU8p) (const lzo_memops_TU0p) (ss))
#endif

__lzo_static_forceinline void lzo_memops_put_le16(lzo_voidp dd, lzo_uint16_t vv) {
#if (LZO_ABI_LITTLE_ENDIAN)
    LZO_MEMOPS_COPY2(dd, &vv);
#elif (LZO_OPT_UNALIGNED16 && LZO_ARCH_POWERPC && LZO_ABI_BIG_ENDIAN) && (LZO_ASM_SYNTAX_GNUC)
    lzo_memops_TU2p d = (lzo_memops_TU2p) dd;
    unsigned long v = vv;
    __asm__("sthbrx %2,0,%1" : "=m" (*d) : "r" (d), "r" (v));
#else
    lzo_memops_TU1p d = (lzo_memops_TU1p) dd;
    d[0] = LZO_BYTE((vv) & 0xff);
    d[1] = LZO_BYTE((vv >> 8) & 0xff);
#endif
}

#if (LZO_OPT_UNALIGNED16) && (LZO_ABI_LITTLE_ENDIAN)
#define LZO_MEMOPS_PUT_LE16(dd,vv) (* (lzo_memops_TU2p) (lzo_memops_TU0p) (dd) = (vv))
#else
#define LZO_MEMOPS_PUT_LE16(dd, vv) lzo_memops_put_le16(dd,vv)
#endif

__lzo_static_forceinline void lzo_memops_put_le32(lzo_voidp dd, lzo_uint32_t vv) {
#if (LZO_ABI_LITTLE_ENDIAN)
    LZO_MEMOPS_COPY4(dd, &vv);
#elif (LZO_OPT_UNALIGNED32 && LZO_ARCH_POWERPC && LZO_ABI_BIG_ENDIAN) && (LZO_ASM_SYNTAX_GNUC)
    lzo_memops_TU4p d = (lzo_memops_TU4p) dd;
    unsigned long v = vv;
    __asm__("stwbrx %2,0,%1" : "=m" (*d) : "r" (d), "r" (v));
#else
    lzo_memops_TU1p d = (lzo_memops_TU1p) dd;
    d[0] = LZO_BYTE((vv) & 0xff);
    d[1] = LZO_BYTE((vv >> 8) & 0xff);
    d[2] = LZO_BYTE((vv >> 16) & 0xff);
    d[3] = LZO_BYTE((vv >> 24) & 0xff);
#endif
}

#if (LZO_OPT_UNALIGNED32) && (LZO_ABI_LITTLE_ENDIAN)
#define LZO_MEMOPS_PUT_LE32(dd,vv) (* (lzo_memops_TU4p) (lzo_memops_TU0p) (dd) = (vv))
#else
#define LZO_MEMOPS_PUT_LE32(dd, vv) lzo_memops_put_le32(dd,vv)
#endif

__lzo_static_forceinline void lzo_memops_put_ne16(lzo_voidp dd, lzo_uint16_t vv) {
    LZO_MEMOPS_COPY2(dd, &vv);
}

#if (LZO_OPT_UNALIGNED16)
#define LZO_MEMOPS_PUT_NE16(dd,vv) (* (lzo_memops_TU2p) (lzo_memops_TU0p) (dd) = (vv))
#else
#define LZO_MEMOPS_PUT_NE16(dd, vv) lzo_memops_put_ne16(dd,vv)
#endif

__lzo_static_forceinline void lzo_memops_put_ne32(lzo_voidp dd, lzo_uint32_t vv) {
    LZO_MEMOPS_COPY4(dd, &vv);
}

#if (LZO_OPT_UNALIGNED32)
#define LZO_MEMOPS_PUT_NE32(dd,vv) (* (lzo_memops_TU4p) (lzo_memops_TU0p) (dd) = (vv))
#else
#define LZO_MEMOPS_PUT_NE32(dd, vv) lzo_memops_put_ne32(dd,vv)
#endif

lzo_unused_funcs_impl(void, lzo_memops_unused_funcs)(void) {
    LZO_UNUSED_FUNC(lzo_memops_unused_funcs);
    LZO_UNUSED_FUNC(lzo_memops_get_le16);
    LZO_UNUSED_FUNC(lzo_memops_get_le32);
    LZO_UNUSED_FUNC(lzo_memops_get_ne16);
    LZO_UNUSED_FUNC(lzo_memops_get_ne32);
    LZO_UNUSED_FUNC(lzo_memops_put_le16);
    LZO_UNUSED_FUNC(lzo_memops_put_le32);
    LZO_UNUSED_FUNC(lzo_memops_put_ne16);
    LZO_UNUSED_FUNC(lzo_memops_put_ne32);
}

#endif

#ifndef UA_SET1
#define UA_SET1             LZO_MEMOPS_SET1
#endif
#ifndef UA_SET2
#define UA_SET2             LZO_MEMOPS_SET2
#endif
#ifndef UA_SET3
#define UA_SET3             LZO_MEMOPS_SET3
#endif
#ifndef UA_SET4
#define UA_SET4             LZO_MEMOPS_SET4
#endif
#ifndef UA_MOVE1
#define UA_MOVE1            LZO_MEMOPS_MOVE1
#endif
#ifndef UA_MOVE2
#define UA_MOVE2            LZO_MEMOPS_MOVE2
#endif
#ifndef UA_MOVE3
#define UA_MOVE3            LZO_MEMOPS_MOVE3
#endif
#ifndef UA_MOVE4
#define UA_MOVE4            LZO_MEMOPS_MOVE4
#endif
#ifndef UA_MOVE8
#define UA_MOVE8            LZO_MEMOPS_MOVE8
#endif
#ifndef UA_COPY1
#define UA_COPY1            LZO_MEMOPS_COPY1
#endif
#ifndef UA_COPY2
#define UA_COPY2            LZO_MEMOPS_COPY2
#endif
#ifndef UA_COPY3
#define UA_COPY3            LZO_MEMOPS_COPY3
#endif
#ifndef UA_COPY4
#define UA_COPY4            LZO_MEMOPS_COPY4
#endif
#ifndef UA_COPY8
#define UA_COPY8            LZO_MEMOPS_COPY8
#endif
#ifndef UA_COPYN
#define UA_COPYN            LZO_MEMOPS_COPYN
#endif
#ifndef UA_COPYN_X
#define UA_COPYN_X          LZO_MEMOPS_COPYN
#endif
#ifndef UA_GET_LE16
#define UA_GET_LE16         LZO_MEMOPS_GET_LE16
#endif
#ifndef UA_GET_LE32
#define UA_GET_LE32         LZO_MEMOPS_GET_LE32
#endif
#ifdef LZO_MEMOPS_GET_LE64
#ifndef UA_GET_LE64
#define UA_GET_LE64         LZO_MEMOPS_GET_LE64
#endif
#endif
#ifndef UA_GET_NE16
#define UA_GET_NE16         LZO_MEMOPS_GET_NE16
#endif
#ifndef UA_GET_NE32
#define UA_GET_NE32         LZO_MEMOPS_GET_NE32
#endif
#ifdef LZO_MEMOPS_GET_NE64
#ifndef UA_GET_NE64
#define UA_GET_NE64         LZO_MEMOPS_GET_NE64
#endif
#endif
#ifndef UA_PUT_LE16
#define UA_PUT_LE16         LZO_MEMOPS_PUT_LE16
#endif
#ifndef UA_PUT_LE32
#define UA_PUT_LE32         LZO_MEMOPS_PUT_LE32
#endif
#ifndef UA_PUT_NE16
#define UA_PUT_NE16         LZO_MEMOPS_PUT_NE16
#endif
#ifndef UA_PUT_NE32
#define UA_PUT_NE32         LZO_MEMOPS_PUT_NE32
#endif

#define MEMCPY8_DS(dest, src, len) \
    lzo_memcpy(dest,src,len); dest += len; src += len

#define BZERO8_PTR(s, l, n) \
    lzo_memset((lzo_voidp)(s),0,(lzo_uint)(l)*(n))

#define MEMCPY_DS(dest, src, len) \
    do *dest++ = *src++; while (--len > 0)

LZO_EXTERN(const lzo_bytep)lzo_copyright(void);

#ifndef __LZO_PTR_H
#define __LZO_PTR_H 1

#ifdef __cplusplus
extern "C" {
#endif

#if (LZO_ARCH_I086)
#error "LZO_ARCH_I086 is unsupported"
#elif (LZO_MM_PVP)
#error "LZO_MM_PVP is unsupported"
#else
#define PTR(a)              ((lzo_uintptr_t) (a))
#define PTR_LINEAR(a)       PTR(a)
#define PTR_ALIGNED_4(a)    ((PTR_LINEAR(a) & 3) == 0)
#define PTR_ALIGNED_8(a)    ((PTR_LINEAR(a) & 7) == 0)
#define PTR_ALIGNED2_4(a, b) (((PTR_LINEAR(a) | PTR_LINEAR(b)) & 3) == 0)
#define PTR_ALIGNED2_8(a, b) (((PTR_LINEAR(a) | PTR_LINEAR(b)) & 7) == 0)
#endif

#define PTR_LT(a, b)         (PTR(a) < PTR(b))
#define PTR_GE(a, b)         (PTR(a) >= PTR(b))
#define PTR_DIFF(a, b)       (PTR(a) - PTR(b))
#define pd(a, b)             ((lzo_uint) ((a)-(b)))

LZO_EXTERN(lzo_uintptr_t)
__lzo_ptr_linear(const lzo_voidp ptr);

typedef union {
    char a_char;
    unsigned char a_uchar;
    short a_short;
    unsigned short a_ushort;
    int a_int;
    unsigned int a_uint;
    long a_long;
    unsigned long a_ulong;
    lzo_int a_lzo_int;
    lzo_uint a_lzo_uint;
    lzo_xint a_lzo_xint;
    lzo_int16_t a_lzo_int16_t;
    lzo_uint16_t a_lzo_uint16_t;
    lzo_int32_t a_lzo_int32_t;
    lzo_uint32_t a_lzo_uint32_t;
#if defined(lzo_uint64_t)
    lzo_int64_t     a_lzo_int64_t;
    lzo_uint64_t    a_lzo_uint64_t;
#endif
    size_t a_size_t;
    ptrdiff_t a_ptrdiff_t;
    lzo_uintptr_t a_lzo_uintptr_t;
    void *a_void_p;
    char *a_char_p;
    unsigned char *a_uchar_p;
    const void *a_c_void_p;
    const char *a_c_char_p;
    const unsigned char *a_c_uchar_p;
    lzo_voidp a_lzo_voidp;
    lzo_bytep a_lzo_bytep;
    const lzo_voidp a_c_lzo_voidp;
    const lzo_bytep a_c_lzo_bytep;
}
        lzo_full_align_t;

#ifdef __cplusplus
}
#endif

#endif

#ifndef LZO_DETERMINISTIC
#define LZO_DETERMINISTIC 1
#endif

#ifndef LZO_DICT_USE_PTR
#define LZO_DICT_USE_PTR 1
#endif

#if (LZO_DICT_USE_PTR)
#  define lzo_dict_t    const lzo_bytep
#  define lzo_dict_p    lzo_dict_t *
#else
#  define lzo_dict_t    lzo_uint
#  define lzo_dict_p    lzo_dict_t *
#endif

#endif

#if !defined(MINILZO_CFG_SKIP_LZO_PTR)

LZO_PUBLIC(lzo_uintptr_t)
__lzo_ptr_linear(const lzo_voidp ptr) {
    lzo_uintptr_t p;

#if (LZO_ARCH_I086)
#error "LZO_ARCH_I086 is unsupported"
#elif (LZO_MM_PVP)
#error "LZO_MM_PVP is unsupported"
#else
    p = (lzo_uintptr_t) PTR_LINEAR(ptr);
#endif

    return p;
}

LZO_PUBLIC(unsigned)
__lzo_align_gap(const lzo_voidp ptr, lzo_uint size) {
#if (__LZO_UINTPTR_T_IS_POINTER)
#error "__LZO_UINTPTR_T_IS_POINTER is unsupported"
#else
    lzo_uintptr_t p, n;
    if (size < 2) return 0;
    p = __lzo_ptr_linear(ptr);
#if 0
    n = (((p + size - 1) / size) * size) - p;
#else
    if ((size & (size - 1)) != 0)
        return 0;
    n = size;
    n = ((p + n - 1) & ~(n - 1)) - p;
#endif
#endif
    assert((long) n >= 0);
    assert(n <= size);
    return (unsigned) n;
}

#endif
#if !defined(MINILZO_CFG_SKIP_LZO_UTIL)

/* If you use the LZO library in a product, I would appreciate that you
 * keep this copyright string in the executable of your product.
 */





#define LZO_BASE 65521u
#define LZO_NMAX 5552

#define LZO_DO1(buf, i)  s1 += buf[i]; s2 += s1
#define LZO_DO2(buf, i)  LZO_DO1(buf,i); LZO_DO1(buf,i+1)
#define LZO_DO4(buf, i)  LZO_DO2(buf,i); LZO_DO2(buf,i+2)
#define LZO_DO8(buf, i)  LZO_DO4(buf,i); LZO_DO4(buf,i+4)
#define LZO_DO16(buf, i) LZO_DO8(buf,i); LZO_DO8(buf,i+8)

LZO_PUBLIC(lzo_uint32_t)
lzo_adler32(lzo_uint32_t adler, const lzo_bytep buf, lzo_uint len) {
    lzo_uint32_t s1 = adler & 0xffff;
    lzo_uint32_t s2 = (adler >> 16) & 0xffff;
    unsigned k;

    if (buf == NULL)
        return 1;

    while (len > 0) {
        k = len < LZO_NMAX ? (unsigned) len : LZO_NMAX;
        len -= k;
        if (k >= 16)
            do {
                LZO_DO16(buf, 0);
                buf += 16;
                k -= 16;
            } while (k >= 16);
        if (k != 0)
            do {
                s1 += *buf++;
                s2 += s1;
            } while (--k > 0);
        s1 %= LZO_BASE;
        s2 %= LZO_BASE;
    }
    return (s2 << 16) | s1;
}

#undef LZO_DO1
#undef LZO_DO2
#undef LZO_DO4
#undef LZO_DO8
#undef LZO_DO16

#endif
#if !defined(MINILZO_CFG_SKIP_LZO_STRING)
#undef lzo_memcmp
#undef lzo_memcpy
#undef lzo_memmove
#if !defined(__LZO_MMODEL_HUGE)
#  undef LZO_HAVE_MM_HUGE_PTR
#endif

#define LZOLIB_PUBLIC(r, f)      LZO_PUBLIC(r) f

#define __LZOLIB_HMEMCPY_CH_INCLUDED 1
#if !defined(LZOLIB_PUBLIC)
#  define LZOLIB_PUBLIC(r,f)    r __LZOLIB_FUNCNAME(f)
#endif

#undef LZOLIB_PUBLIC
#endif
#if !defined(MINILZO_CFG_SKIP_LZO_INIT)

union lzo_config_check_union {
    lzo_uint a[2];
    unsigned char b[2 * LZO_MAX(8, sizeof(lzo_uint))];
#if defined(lzo_uint64_t)
    lzo_uint64_t c[2];
#endif
};

static __lzo_noinline lzo_voidp u2p(lzo_voidp ptr, lzo_uint off) {
    return (lzo_voidp) ((lzo_bytep) ptr + off);
}

LZO_PUBLIC(int)
_lzo_config_check(void) {
    union lzo_config_check_union u;
    lzo_voidp p;
    unsigned r = 1;

    u.a[0] = u.a[1] = 0;
    p = u2p(&u, 0);
    r &= ((*(lzo_bytep) p) == 0);
#if !(LZO_CFG_NO_CONFIG_CHECK)
#if (LZO_ABI_BIG_ENDIAN)
    u.a[0] = u.a[1] = 0; u.b[sizeof(lzo_uint) - 1] = 128;
    p = u2p(&u, 0);
    r &= ((* (lzo_uintp) p) == 128);
#endif
#if (LZO_ABI_LITTLE_ENDIAN)
    u.a[0] = u.a[1] = 0; u.b[0] = 128;
    p = u2p(&u, 0);
    r &= ((* (lzo_uintp) p) == 128);
#endif
    u.a[0] = u.a[1] = 0;
    u.b[0] = 1;
    u.b[3] = 2;
    p = u2p(&u, 1);
    r &= UA_GET_NE16(p) == 0;
    r &= UA_GET_LE16(p) == 0;
    u.b[1] = 128;
    r &= UA_GET_LE16(p) == 128;
    u.b[2] = 129;
    r &= UA_GET_LE16(p) == LZO_UINT16_C(0x8180);
#if (LZO_ABI_BIG_ENDIAN)
    r &= UA_GET_NE16(p) == LZO_UINT16_C(0x8081);
#endif
#if (LZO_ABI_LITTLE_ENDIAN)
    r &= UA_GET_NE16(p) == LZO_UINT16_C(0x8180);
#endif
    u.a[0] = u.a[1] = 0;
    u.b[0] = 3;
    u.b[5] = 4;
    p = u2p(&u, 1);
    r &= UA_GET_NE32(p) == 0;
    r &= UA_GET_LE32(p) == 0;
    u.b[1] = 128;
    r &= UA_GET_LE32(p) == 128;
    u.b[2] = 129;
    u.b[3] = 130;
    u.b[4] = 131;
    r &= UA_GET_LE32(p) == LZO_UINT32_C(0x83828180);
#if (LZO_ABI_BIG_ENDIAN)
    r &= UA_GET_NE32(p) == LZO_UINT32_C(0x80818283);
#endif
#if (LZO_ABI_LITTLE_ENDIAN)
    r &= UA_GET_NE32(p) == LZO_UINT32_C(0x83828180);
#endif
#if defined(UA_GET_NE64)
    u.c[0] = u.c[1] = 0;
    u.b[0] = 5; u.b[9] = 6;
    p = u2p(&u, 1);
    u.c[0] = u.c[1] = 0;
    r &= UA_GET_NE64(p) == 0;
#if defined(UA_GET_LE64)
    r &= UA_GET_LE64(p) == 0;
    u.b[1] = 128;
    r &= UA_GET_LE64(p) == 128;
#endif
#endif
#if defined(lzo_bitops_ctlz32)
    { unsigned i = 0; lzo_uint32_t v;
    for (v = 1; v != 0 && r == 1; v <<= 1, i++) {
        r &= lzo_bitops_ctlz32(v) == 31 - i;
        r &= lzo_bitops_ctlz32_func(v) == 31 - i;
    }}
#endif
#if defined(lzo_bitops_ctlz64)
    { unsigned i = 0; lzo_uint64_t v;
    for (v = 1; v != 0 && r == 1; v <<= 1, i++) {
        r &= lzo_bitops_ctlz64(v) == 63 - i;
        r &= lzo_bitops_ctlz64_func(v) == 63 - i;
    }}
#endif
#if defined(lzo_bitops_cttz32)
    { unsigned i = 0; lzo_uint32_t v;
    for (v = 1; v != 0 && r == 1; v <<= 1, i++) {
        r &= lzo_bitops_cttz32(v) == i;
        r &= lzo_bitops_cttz32_func(v) == i;
    }}
#endif
#if defined(lzo_bitops_cttz64)
    { unsigned i = 0; lzo_uint64_t v;
    for (v = 1; v != 0 && r == 1; v <<= 1, i++) {
        r &= lzo_bitops_cttz64(v) == i;
        r &= lzo_bitops_cttz64_func(v) == i;
    }}
#endif
#endif
    LZO_UNUSED_FUNC(lzo_bitops_unused_funcs);

    return r == 1 ? LZO_E_OK : LZO_E_ERROR;
}

LZO_PUBLIC(int)
__lzo_init_v2(unsigned v, int s1, int s2, int s3, int s4, int s5,
              int s6, int s7, int s8) {
    int r;

    if (v == 0)
        return LZO_E_ERROR;

    r = (s1 == -1 || s1 == (int) sizeof(short)) &&
        (s2 == -1 || s2 == (int) sizeof(int)) &&
        (s3 == -1 || s3 == (int) sizeof(long)) &&
        (s4 == -1 || s4 == (int) sizeof(lzo_uint32_t)) &&
        (s5 == -1 || s5 == (int) sizeof(lzo_uint)) &&
        (s6 == -1 || s6 == (int) lzo_sizeof_dict_t) &&
        (s7 == -1 || s7 == (int) sizeof(char *)) &&
        (s8 == -1 || s8 == (int) sizeof(lzo_voidp));
    if (!r)
        return LZO_E_ERROR;

    r = _lzo_config_check();
    if (r != LZO_E_OK)
        return r;

    return r;
}

#endif

#define LZO1X           1
#define LZO_EOF_CODE    1
#define M2_MAX_OFFSET   0x0800

#if !defined(MINILZO_CFG_SKIP_LZO1X_1_COMPRESS)

#if 1 && defined(UA_GET_LE32)
#undef  LZO_DICT_USE_PTR
#define LZO_DICT_USE_PTR 0
#undef  lzo_dict_t
#define lzo_dict_t lzo_uint16_t
#endif

#define LZO_NEED_DICT_H 1
#ifndef D_BITS
#define D_BITS          14
#endif
#define D_INDEX1(d, p)       d = DM(DMUL(0x21,DX3(p,5,5,6)) >> 5)
#define D_INDEX2(d, p)       d = (d & (D_MASK & 0x7ff)) ^ (D_HIGH | 0x1f)
#if 1
#define DINDEX(dv, p)        DM(((DMUL(0x1824429d,dv)) >> (32-D_BITS)))
#else
#define DINDEX(dv,p)        DM((dv) + ((dv) >> (32-D_BITS)))
#endif

#ifndef __LZO_CONFIG1X_H
#define __LZO_CONFIG1X_H 1

#if !defined(LZO1X) && !defined(LZO1Y) && !defined(LZO1Z)
#  define LZO1X 1
#endif

#ifndef LZO_EOF_CODE
#define LZO_EOF_CODE 1
#endif
#undef LZO_DETERMINISTIC

#define M1_MAX_OFFSET   0x0400
#ifndef M2_MAX_OFFSET
#define M2_MAX_OFFSET   0x0800
#endif
#define M3_MAX_OFFSET   0x4000
#define M4_MAX_OFFSET   0xbfff

#define MX_MAX_OFFSET   (M1_MAX_OFFSET + M2_MAX_OFFSET)

#define M1_MIN_LEN      2
#define M1_MAX_LEN      2
#define M2_MIN_LEN      3
#ifndef M2_MAX_LEN
#define M2_MAX_LEN      8
#endif
#define M3_MIN_LEN      3
#define M3_MAX_LEN      33
#define M4_MIN_LEN      3
#define M4_MAX_LEN      9

#define M1_MARKER       0
#define M2_MARKER       64
#define M3_MARKER       32
#define M4_MARKER       16

#ifndef MIN_LOOKAHEAD
#define MIN_LOOKAHEAD       (M2_MAX_LEN + 1)
#endif

#if defined(LZO_NEED_DICT_H)

#ifndef LZO_HASH
#define LZO_HASH            LZO_HASH_LZO_INCREMENTAL_B
#endif
#define DL_MIN_LEN          M2_MIN_LEN

#ifndef __LZO_DICT_H
#define __LZO_DICT_H 1

#ifdef __cplusplus
extern "C" {
#endif

#if !defined(D_BITS) && defined(DBITS)
#  define D_BITS        DBITS
#endif
#if !defined(D_BITS)
#  error "D_BITS is not defined"
#endif
#if (D_BITS < 16)
#  define D_SIZE        LZO_SIZE(D_BITS)
#  define D_MASK        LZO_MASK(D_BITS)
#else
#  define D_SIZE        LZO_USIZE(D_BITS)
#  define D_MASK        LZO_UMASK(D_BITS)
#endif
#define D_HIGH          ((D_MASK >> 1) + 1)

#if !defined(DD_BITS)
#  define DD_BITS       0
#endif
#define DD_SIZE         LZO_SIZE(DD_BITS)
#define DD_MASK         LZO_MASK(DD_BITS)

#if !defined(DL_BITS)
#  define DL_BITS       (D_BITS - DD_BITS)
#endif
#if (DL_BITS < 16)
#  define DL_SIZE       LZO_SIZE(DL_BITS)
#  define DL_MASK       LZO_MASK(DL_BITS)
#else
#  define DL_SIZE       LZO_USIZE(DL_BITS)
#  define DL_MASK       LZO_UMASK(DL_BITS)
#endif

#if (D_BITS != DL_BITS + DD_BITS)
#  error "D_BITS does not match"
#endif
#if (D_BITS < 6 || D_BITS > 18)
#  error "invalid D_BITS"
#endif
#if (DL_BITS < 6 || DL_BITS > 20)
#  error "invalid DL_BITS"
#endif
#if (DD_BITS < 0 || DD_BITS > 6)
#  error "invalid DD_BITS"
#endif

#if !defined(DL_MIN_LEN)
#  define DL_MIN_LEN    3
#endif
#if !defined(DL_SHIFT)
#  define DL_SHIFT      ((DL_BITS + (DL_MIN_LEN - 1)) / DL_MIN_LEN)
#endif

#define LZO_HASH_GZIP                   1
#define LZO_HASH_GZIP_INCREMENTAL       2
#define LZO_HASH_LZO_INCREMENTAL_A      3
#define LZO_HASH_LZO_INCREMENTAL_B      4

#if !defined(LZO_HASH)
#  error "choose a hashing strategy"
#endif

#undef DM
#undef DX

#if (DL_MIN_LEN == 3)
#  define _DV2_A(p, shift1, shift2) \
        (((( (lzo_xint)((p)[0]) << shift1) ^ (p)[1]) << shift2) ^ (p)[2])
#  define _DV2_B(p, shift1, shift2) \
        (((( (lzo_xint)((p)[2]) << shift1) ^ (p)[1]) << shift2) ^ (p)[0])
#  define _DV3_B(p, shift1, shift2, shift3) \
        ((_DV2_B((p)+1,shift1,shift2) << (shift3)) ^ (p)[0])
#elif (DL_MIN_LEN == 2)
#  define _DV2_A(p,shift1,shift2) \
        (( (lzo_xint)(p[0]) << shift1) ^ p[1])
#  define _DV2_B(p,shift1,shift2) \
        (( (lzo_xint)(p[1]) << shift1) ^ p[2])
#else
#  error "invalid DL_MIN_LEN"
#endif
#define _DV_A(p, shift)      _DV2_A(p,shift,shift)
#define _DV_B(p, shift)      _DV2_B(p,shift,shift)
#define DA2(p, s1, s2) \
        (((((lzo_xint)((p)[2]) << (s2)) + (p)[1]) << (s1)) + (p)[0])
#define DS2(p, s1, s2) \
        (((((lzo_xint)((p)[2]) << (s2)) - (p)[1]) << (s1)) - (p)[0])
#define DX2(p, s1, s2) \
        (((((lzo_xint)((p)[2]) << (s2)) ^ (p)[1]) << (s1)) ^ (p)[0])
#define DA3(p, s1, s2, s3) ((DA2((p)+1,s2,s3) << (s1)) + (p)[0])
#define DS3(p, s1, s2, s3) ((DS2((p)+1,s2,s3) << (s1)) - (p)[0])
#define DX3(p, s1, s2, s3) ((DX2((p)+1,s2,s3) << (s1)) ^ (p)[0])
#define DMS(v, s)        ((lzo_uint) (((v) & (D_MASK >> (s))) << (s)))
#define DM(v)           DMS(v,0)

#if (LZO_HASH == LZO_HASH_GZIP)
#  define _DINDEX(dv,p)     (_DV_A((p),DL_SHIFT))

#elif (LZO_HASH == LZO_HASH_GZIP_INCREMENTAL)
#  define __LZO_HASH_INCREMENTAL 1
#  define DVAL_FIRST(dv,p)  dv = _DV_A((p),DL_SHIFT)
#  define DVAL_NEXT(dv,p)   dv = (((dv) << DL_SHIFT) ^ p[2])
#  define _DINDEX(dv,p)     (dv)
#  define DVAL_LOOKAHEAD    DL_MIN_LEN

#elif (LZO_HASH == LZO_HASH_LZO_INCREMENTAL_A)
#  define __LZO_HASH_INCREMENTAL 1
#  define DVAL_FIRST(dv,p)  dv = _DV_A((p),5)
#  define DVAL_NEXT(dv,p) \
                dv ^= (lzo_xint)(p[-1]) << (2*5); dv = (((dv) << 5) ^ p[2])
#  define _DINDEX(dv,p)     ((DMUL(0x9f5f,dv)) >> 5)
#  define DVAL_LOOKAHEAD    DL_MIN_LEN

#elif (LZO_HASH == LZO_HASH_LZO_INCREMENTAL_B)
#  define __LZO_HASH_INCREMENTAL 1
#  define DVAL_FIRST(dv, p)  dv = _DV_B((p),5)
#  define DVAL_NEXT(dv, p) \
                dv ^= p[-1]; dv = (((dv) >> 5) ^ ((lzo_xint)(p[2]) << (2*5)))
#  define _DINDEX(dv, p)     ((DMUL(0x9f5f,dv)) >> 5)
#  define DVAL_LOOKAHEAD    DL_MIN_LEN

#else
#  error "choose a hashing strategy"
#endif

#ifndef DINDEX
#define DINDEX(dv,p)        ((lzo_uint)((_DINDEX(dv,p)) & DL_MASK) << DD_BITS)
#endif
#if !defined(DINDEX1) && defined(D_INDEX1)
#define DINDEX1             D_INDEX1
#endif
#if !defined(DINDEX2) && defined(D_INDEX2)
#define DINDEX2             D_INDEX2
#endif

#if !defined(__LZO_HASH_INCREMENTAL)
#  define DVAL_FIRST(dv,p)  ((void) 0)
#  define DVAL_NEXT(dv,p)   ((void) 0)
#  define DVAL_LOOKAHEAD    0
#endif

#if !defined(DVAL_ASSERT)
#if defined(__LZO_HASH_INCREMENTAL) && !defined(NDEBUG)
#if 1 && (LZO_CC_ARMCC_GNUC || LZO_CC_CLANG || (LZO_CC_GNUC >= 0x020700ul) || LZO_CC_INTELC_GNUC || LZO_CC_LLVM || LZO_CC_PATHSCALE || LZO_CC_PGI)
static void __attribute__((__unused__))
#else
static void
#endif
DVAL_ASSERT(lzo_xint dv, const lzo_bytep p)
{
    lzo_xint df;
    DVAL_FIRST(df,(p));
    assert(DINDEX(dv,p) == DINDEX(df,p));
}
#else
#  define DVAL_ASSERT(dv, p) ((void) 0)
#endif
#endif

#if (LZO_DICT_USE_PTR)
#  define DENTRY(p,in)                          (p)
#  define GINDEX(m_pos,m_off,dict,dindex,in)    m_pos = dict[dindex]
#else
#  define DENTRY(p, in)                          ((lzo_dict_t) pd(p, in))
#  define GINDEX(m_pos, m_off, dict, dindex, in)    m_off = dict[dindex]
#endif

#if (DD_BITS == 0)

#  define UPDATE_D(dict, drun, dv, p, in)       dict[ DINDEX(dv,p) ] = DENTRY(p,in)
#  define UPDATE_I(dict, drun, index, p, in)    dict[index] = DENTRY(p,in)
#  define UPDATE_P(ptr, drun, p, in)           (ptr)[0] = DENTRY(p,in)

#else

#  define UPDATE_D(dict,drun,dv,p,in)   \
        dict[ DINDEX(dv,p) + drun++ ] = DENTRY(p,in); drun &= DD_MASK
#  define UPDATE_I(dict,drun,index,p,in)    \
        dict[ (index) + drun++ ] = DENTRY(p,in); drun &= DD_MASK
#  define UPDATE_P(ptr,drun,p,in)   \
        (ptr) [ drun++ ] = DENTRY(p,in); drun &= DD_MASK

#endif

#if (LZO_DICT_USE_PTR)

#define LZO_CHECK_MPOS_DET(m_pos,m_off,in,ip,max_offset) \
        (m_pos == NULL || (m_off = pd(ip, m_pos)) > max_offset)

#define LZO_CHECK_MPOS_NON_DET(m_pos,m_off,in,ip,max_offset) \
    (BOUNDS_CHECKING_OFF_IN_EXPR(( \
        m_pos = ip - (lzo_uint) PTR_DIFF(ip,m_pos), \
        PTR_LT(m_pos,in) || \
        (m_off = (lzo_uint) PTR_DIFF(ip,m_pos)) == 0 || \
         m_off > max_offset )))

#else

#define LZO_CHECK_MPOS_DET(m_pos, m_off, in, ip, max_offset) \
        (m_off == 0 || \
         ((m_off = pd(ip, in) - m_off) > max_offset) || \
         (m_pos = (ip) - (m_off), 0) )

#define LZO_CHECK_MPOS_NON_DET(m_pos, m_off, in, ip, max_offset) \
        (pd(ip, in) <= m_off || \
         ((m_off = pd(ip, in) - m_off) > max_offset) || \
         (m_pos = (ip) - (m_off), 0) )

#endif

#if (LZO_DETERMINISTIC)
#  define LZO_CHECK_MPOS    LZO_CHECK_MPOS_DET
#else
#  define LZO_CHECK_MPOS    LZO_CHECK_MPOS_NON_DET
#endif

#ifdef __cplusplus
}
#endif

#endif

#endif

#endif

#define LZO_DETERMINISTIC !(LZO_DICT_USE_PTR)

static __lzo_noinline lzo_uint
lzo1x_1_compress_core(const lzo_bytep in, lzo_uint in_len,
                      lzo_bytep out, lzo_uintp out_len,
                      lzo_uint ti, lzo_voidp wrkmem) {
    const lzo_bytep ip;
    lzo_bytep op;
    const lzo_bytep const in_end = in + in_len;
    const lzo_bytep const ip_end = in + in_len - 20;
    const lzo_bytep ii;
    lzo_dict_p const dict = (lzo_dict_p) wrkmem;

    op = out;
    ip = in;
    ii = ip;

    ip += ti < 4 ? 4 - ti : 0;
    for (;;) {
        const lzo_bytep m_pos;
#if !(LZO_DETERMINISTIC)
        LZO_DEFINE_UNINITIALIZED_VAR(lzo_uint, m_off, 0);
        lzo_uint m_len;
        lzo_uint dindex;
next:
        if __lzo_unlikely(ip >= ip_end)
            break;
        DINDEX1(dindex,ip);
        GINDEX(m_pos,m_off,dict,dindex,in);
        if (LZO_CHECK_MPOS_NON_DET(m_pos,m_off,in,ip,M4_MAX_OFFSET))
            goto literal;
#if 1
        if (m_off <= M2_MAX_OFFSET || m_pos[3] == ip[3])
            goto try_match;
        DINDEX2(dindex,ip);
#endif
        GINDEX(m_pos,m_off,dict,dindex,in);
        if (LZO_CHECK_MPOS_NON_DET(m_pos,m_off,in,ip,M4_MAX_OFFSET))
            goto literal;
        if (m_off <= M2_MAX_OFFSET || m_pos[3] == ip[3])
            goto try_match;
        goto literal;

try_match:
#if (LZO_OPT_UNALIGNED32)
        if (UA_GET_NE32(m_pos) != UA_GET_NE32(ip))
#else
        if (m_pos[0] != ip[0] || m_pos[1] != ip[1] || m_pos[2] != ip[2] || m_pos[3] != ip[3])
#endif
        {
literal:
            UPDATE_I(dict,0,dindex,ip,in);
            ip += 1 + ((ip - ii) >> 5);
            continue;
        }
        UPDATE_I(dict,0,dindex,ip,in);
#else
        lzo_uint m_off;
        lzo_uint m_len;
        {
            lzo_uint32_t dv;
            lzo_uint dindex;
            literal:
            ip += 1 + ((ip - ii) >> 5);
            next:
            if __lzo_unlikely(ip >= ip_end)
                break;
            dv = UA_GET_LE32(ip);
            dindex = DINDEX(dv, ip);
            GINDEX(m_off, m_pos, in + dict, dindex, in);
            UPDATE_I(dict, 0, dindex, ip, in);
            if __lzo_unlikely(dv != UA_GET_LE32(m_pos))
                goto literal;
        }
#endif

        ii -= ti;
        ti = 0;
        {
            lzo_uint t = pd(ip, ii);
            if (t != 0) {
                if (t <= 3) {
                    op[-2] = LZO_BYTE(op[-2] | t);
#if (LZO_OPT_UNALIGNED32)
                    UA_COPY4(op, ii);
                    op += t;
#else
                    { do *op++ = *ii++; while (--t > 0); }
#endif
                }
#if (LZO_OPT_UNALIGNED32) || (LZO_OPT_UNALIGNED64)
                    else if (t <= 16)
                    {
                        *op++ = LZO_BYTE(t - 3);
                        UA_COPY8(op, ii);
                        UA_COPY8(op+8, ii+8);
                        op += t;
                    }
#endif
                else {
                    if (t <= 18)
                        *op++ = LZO_BYTE(t - 3);
                    else {
                        lzo_uint tt = t - 18;
                        *op++ = 0;
                        while __lzo_unlikely(tt > 255) {
                            tt -= 255;
                                    UA_SET1(op, 0);
                            op++;
                        }
                        assert(tt > 0);
                        *op++ = LZO_BYTE(tt);
                    }
#if (LZO_OPT_UNALIGNED32) || (LZO_OPT_UNALIGNED64)
                    do {
                        UA_COPY8(op, ii);
                        UA_COPY8(op+8, ii+8);
                        op += 16; ii += 16; t -= 16;
                    } while (t >= 16); if (t > 0)
#endif
                    { do *op++ = *ii++; while (--t > 0); }
                }
            }
        }
        m_len = 4;
        {
#if (LZO_OPT_UNALIGNED64)
            lzo_uint64_t v;
            v = UA_GET_NE64(ip + m_len) ^ UA_GET_NE64(m_pos + m_len);
            if __lzo_unlikely(v == 0) {
                do {
                    m_len += 8;
                    v = UA_GET_NE64(ip + m_len) ^ UA_GET_NE64(m_pos + m_len);
                    if __lzo_unlikely(ip + m_len >= ip_end)
                        goto m_len_done;
                } while (v == 0);
            }
#if (LZO_ABI_BIG_ENDIAN) && defined(lzo_bitops_ctlz64)
            m_len += lzo_bitops_ctlz64(v) / CHAR_BIT;
#elif (LZO_ABI_BIG_ENDIAN)
            if ((v >> (64 - CHAR_BIT)) == 0) do {
                v <<= CHAR_BIT;
                m_len += 1;
            } while ((v >> (64 - CHAR_BIT)) == 0);
#elif (LZO_ABI_LITTLE_ENDIAN) && defined(lzo_bitops_cttz64)
            m_len += lzo_bitops_cttz64(v) / CHAR_BIT;
#elif (LZO_ABI_LITTLE_ENDIAN)
            if ((v & UCHAR_MAX) == 0) do {
                v >>= CHAR_BIT;
                m_len += 1;
            } while ((v & UCHAR_MAX) == 0);
#else
            if (ip[m_len] == m_pos[m_len]) do {
                m_len += 1;
            } while (ip[m_len] == m_pos[m_len]);
#endif
#elif (LZO_OPT_UNALIGNED32)
            lzo_uint32_t v;
            v = UA_GET_NE32(ip + m_len) ^ UA_GET_NE32(m_pos + m_len);
            if __lzo_unlikely(v == 0) {
                do {
                    m_len += 4;
                    v = UA_GET_NE32(ip + m_len) ^ UA_GET_NE32(m_pos + m_len);
                    if (v != 0)
                        break;
                    m_len += 4;
                    v = UA_GET_NE32(ip + m_len) ^ UA_GET_NE32(m_pos + m_len);
                    if __lzo_unlikely(ip + m_len >= ip_end)
                        goto m_len_done;
                } while (v == 0);
            }
#if (LZO_ABI_BIG_ENDIAN) && defined(lzo_bitops_ctlz32)
            m_len += lzo_bitops_ctlz32(v) / CHAR_BIT;
#elif (LZO_ABI_BIG_ENDIAN)
            if ((v >> (32 - CHAR_BIT)) == 0) do {
                v <<= CHAR_BIT;
                m_len += 1;
            } while ((v >> (32 - CHAR_BIT)) == 0);
#elif (LZO_ABI_LITTLE_ENDIAN) && defined(lzo_bitops_cttz32)
            m_len += lzo_bitops_cttz32(v) / CHAR_BIT;
#elif (LZO_ABI_LITTLE_ENDIAN)
            if ((v & UCHAR_MAX) == 0) do {
                v >>= CHAR_BIT;
                m_len += 1;
            } while ((v & UCHAR_MAX) == 0);
#else
            if (ip[m_len] == m_pos[m_len]) do {
                m_len += 1;
            } while (ip[m_len] == m_pos[m_len]);
#endif
#else
            if __lzo_unlikely(ip[m_len] == m_pos[m_len]) {
                do {
                    m_len += 1;
                    if (ip[m_len] != m_pos[m_len])
                        break;
                    m_len += 1;
                    if (ip[m_len] != m_pos[m_len])
                        break;
                    m_len += 1;
                    if (ip[m_len] != m_pos[m_len])
                        break;
                    m_len += 1;
                    if (ip[m_len] != m_pos[m_len])
                        break;
                    m_len += 1;
                    if (ip[m_len] != m_pos[m_len])
                        break;
                    m_len += 1;
                    if (ip[m_len] != m_pos[m_len])
                        break;
                    m_len += 1;
                    if (ip[m_len] != m_pos[m_len])
                        break;
                    m_len += 1;
                    if __lzo_unlikely(ip + m_len >= ip_end)
                        goto m_len_done;
                } while (ip[m_len] == m_pos[m_len]);
            }
#endif
        }
        m_len_done:
        m_off = pd(ip, m_pos);
        ip += m_len;
        ii = ip;
        if (m_len <= M2_MAX_LEN && m_off <= M2_MAX_OFFSET) {
            m_off -= 1;
#if defined(LZO1X)
            *op++ = LZO_BYTE(((m_len - 1) << 5) | ((m_off & 7) << 2));
            *op++ = LZO_BYTE(m_off >> 3);
#elif defined(LZO1Y)
            *op++ = LZO_BYTE(((m_len + 1) << 4) | ((m_off & 3) << 2));
            *op++ = LZO_BYTE(m_off >> 2);
#endif
        } else if (m_off <= M3_MAX_OFFSET) {
            m_off -= 1;
            if (m_len <= M3_MAX_LEN)
                *op++ = LZO_BYTE(M3_MARKER | (m_len - 2));
            else {
                m_len -= M3_MAX_LEN;
                *op++ = M3_MARKER | 0;
                while __lzo_unlikely(m_len > 255) {
                    m_len -= 255;
                            UA_SET1(op, 0);
                    op++;
                }
                *op++ = LZO_BYTE(m_len);
            }
            *op++ = LZO_BYTE(m_off << 2);
            *op++ = LZO_BYTE(m_off >> 6);
        } else {
            m_off -= 0x4000;
            if (m_len <= M4_MAX_LEN)
                *op++ = LZO_BYTE(M4_MARKER | ((m_off >> 11) & 8) | (m_len - 2));
            else {
                m_len -= M4_MAX_LEN;
                *op++ = LZO_BYTE(M4_MARKER | ((m_off >> 11) & 8));
                while __lzo_unlikely(m_len > 255) {
                    m_len -= 255;
                            UA_SET1(op, 0);
                    op++;
                }
                *op++ = LZO_BYTE(m_len);
            }
            *op++ = LZO_BYTE(m_off << 2);
            *op++ = LZO_BYTE(m_off >> 6);
        }
        goto next;
    }

    *out_len = pd(op, out);
    return pd(in_end, ii - ti);
}

LZO_PUBLIC(int)
lzo1x_1_compress(const lzo_bytep in, lzo_uint in_len,
                 lzo_bytep out, lzo_uintp out_len,
                 lzo_voidp wrkmem) {
    const lzo_bytep ip = in;
    lzo_bytep op = out;
    lzo_uint l = in_len;
    lzo_uint t = 0;

    while (l > 20) {
        lzo_uint ll = l;
        lzo_uintptr_t ll_end;
#if 0 || (LZO_DETERMINISTIC)
        ll = LZO_MIN(ll, 49152);
#endif
        ll_end = (lzo_uintptr_t) ip + ll;
        if ((ll_end + ((t + ll) >> 5)) <= ll_end || (const lzo_bytep) (ll_end + ((t + ll) >> 5)) <= ip + ll)
            break;
#if (LZO_DETERMINISTIC)
        lzo_memset(wrkmem, 0, ((lzo_uint) 1 << D_BITS) * sizeof(lzo_dict_t));
#endif
        t = lzo1x_1_compress_core(ip, ll, op, out_len, t, wrkmem);
        ip += ll;
        op += *out_len;
        l -= ll;
    }
    t += l;

    if (t > 0) {
        const lzo_bytep ii = in + in_len - t;

        if (op == out && t <= 238)
            *op++ = LZO_BYTE(17 + t);
        else if (t <= 3)
            op[-2] = LZO_BYTE(op[-2] | t);
        else if (t <= 18)
            *op++ = LZO_BYTE(t - 3);
        else {
            lzo_uint tt = t - 18;

            *op++ = 0;
            while (tt > 255) {
                tt -= 255;
                        UA_SET1(op, 0);
                op++;
            }
            assert(tt > 0);
            *op++ = LZO_BYTE(tt);
        }
                UA_COPYN(op, ii, t);
        op += t;
    }

    *op++ = M4_MARKER | 1;
    *op++ = 0;
    *op++ = 0;

    *out_len = pd(op, out);
    return LZO_E_OK;
}

#endif

#undef LZO_HASH

#undef LZO_TEST_OVERRUN

#if !defined(MINILZO_CFG_SKIP_LZO1X_DECOMPRESS)

#if defined(LZO_TEST_OVERRUN)
#  if !defined(LZO_TEST_OVERRUN_INPUT)
#    define LZO_TEST_OVERRUN_INPUT       2
#  endif
#  if !defined(LZO_TEST_OVERRUN_OUTPUT)
#    define LZO_TEST_OVERRUN_OUTPUT      2
#  endif
#  if !defined(LZO_TEST_OVERRUN_LOOKBEHIND)
#    define LZO_TEST_OVERRUN_LOOKBEHIND  1
#  endif
#endif

#undef TEST_IP
#undef TEST_OP
#undef TEST_IP_AND_TEST_OP
#undef TEST_LB
#undef TEST_LBO
#undef NEED_IP
#undef NEED_OP
#undef TEST_IV
#undef TEST_OV
#undef HAVE_TEST_IP
#undef HAVE_TEST_OP
#undef HAVE_NEED_IP
#undef HAVE_NEED_OP
#undef HAVE_ANY_IP
#undef HAVE_ANY_OP

#if defined(LZO_TEST_OVERRUN_INPUT)
#  if (LZO_TEST_OVERRUN_INPUT >= 1)
#    define TEST_IP             (ip < ip_end)
#  endif
#  if (LZO_TEST_OVERRUN_INPUT >= 2)
#    define NEED_IP(x) \
            if ((lzo_uint)(ip_end - ip) < (lzo_uint)(x))  goto input_overrun
#    define TEST_IV(x)          if ((x) >  (lzo_uint)0 - (511)) goto input_overrun
#  endif
#endif

#if defined(LZO_TEST_OVERRUN_OUTPUT)
#  if (LZO_TEST_OVERRUN_OUTPUT >= 1)
#    define TEST_OP             (op <= op_end)
#  endif
#  if (LZO_TEST_OVERRUN_OUTPUT >= 2)
#    undef TEST_OP
#    define NEED_OP(x) \
            if ((lzo_uint)(op_end - op) < (lzo_uint)(x))  goto output_overrun
#    define TEST_OV(x)          if ((x) >  (lzo_uint)0 - (511)) goto output_overrun
#  endif
#endif

#if defined(LZO_TEST_OVERRUN_LOOKBEHIND)
#  define TEST_LB(m_pos)        if (PTR_LT(m_pos,out) || PTR_GE(m_pos,op)) goto lookbehind_overrun
#  define TEST_LBO(m_pos,o)     if (PTR_LT(m_pos,out) || PTR_GE(m_pos,op-(o))) goto lookbehind_overrun
#else
#  define TEST_LB(m_pos)        ((void) 0)
#  define TEST_LBO(m_pos, o)     ((void) 0)
#endif

#if !defined(LZO_EOF_CODE) && !defined(TEST_IP)
#  define TEST_IP               (ip < ip_end)
#endif

#if defined(TEST_IP)
#  define HAVE_TEST_IP 1
#else
#  define TEST_IP               1
#endif
#if defined(TEST_OP)
#  define HAVE_TEST_OP 1
#else
#  define TEST_OP               1
#endif

#if defined(HAVE_TEST_IP) && defined(HAVE_TEST_OP)
#  define TEST_IP_AND_TEST_OP   (TEST_IP && TEST_OP)
#elif defined(HAVE_TEST_IP)
#  define TEST_IP_AND_TEST_OP   TEST_IP
#elif defined(HAVE_TEST_OP)
#  define TEST_IP_AND_TEST_OP   TEST_OP
#else
#  define TEST_IP_AND_TEST_OP   1
#endif

#if defined(NEED_IP)
#  define HAVE_NEED_IP 1
#else
#  define NEED_IP(x)            ((void) 0)
#  define TEST_IV(x)            ((void) 0)
#endif
#if defined(NEED_OP)
#  define HAVE_NEED_OP 1
#else
#  define NEED_OP(x)            ((void) 0)
#  define TEST_OV(x)            ((void) 0)
#endif

#if defined(HAVE_TEST_IP) || defined(HAVE_NEED_IP)
#  define HAVE_ANY_IP 1
#endif
#if defined(HAVE_TEST_OP) || defined(HAVE_NEED_OP)
#  define HAVE_ANY_OP 1
#endif

LZO_PUBLIC(int)
lzo1x_decompress(const lzo_bytep in, lzo_uint in_len,
                 lzo_bytep out, lzo_uintp out_len)
#endif
{
    lzo_bytep op;
    const lzo_bytep ip;
    lzo_uint t;
#if defined(COPY_DICT)
    lzo_uint m_off;
    const lzo_bytep dict_end;
#else
    const lzo_bytep m_pos;
#endif

    const lzo_bytep const ip_end = in + in_len;
#if defined(HAVE_ANY_OP)
    lzo_bytep const op_end = out + *out_len;
#endif
#if defined(LZO1Z)
    lzo_uint last_m_off = 0;
#endif

#if defined(COPY_DICT)
    if (dict)
    {
        if (dict_len > M4_MAX_OFFSET)
        {
            dict += dict_len - M4_MAX_OFFSET;
            dict_len = M4_MAX_OFFSET;
        }
        dict_end = dict + dict_len;
    }
    else
    {
        dict_len = 0;
        dict_end = NULL;
    }
#endif

    *out_len = 0;

    op = out;
    ip = in;

    NEED_IP(1);
    if (*ip > 17) {
        t = *ip++ - 17;
        if (t < 4)
            goto match_next;
        assert(t > 0);
        NEED_OP(t);
        NEED_IP(t + 3);
        do *op++ = *ip++; while (--t > 0);
        goto first_literal_run;
    }

    for (;;) {
        NEED_IP(3);
        t = *ip++;
        if (t >= 16)
            goto match;
        if (t == 0) {
            while (*ip == 0) {
                t += 255;
                ip++;
                TEST_IV(t);
                NEED_IP(1);
            }
            t += 15 + *ip++;
        }
        assert(t > 0);
        NEED_OP(t + 3);
        NEED_IP(t + 6);
#if (LZO_OPT_UNALIGNED64) && (LZO_OPT_UNALIGNED32)
        t += 3;
        if (t >= 8) do
        {
            UA_COPY8(op,ip);
            op += 8; ip += 8; t -= 8;
        } while (t >= 8);
        if (t >= 4)
        {
            UA_COPY4(op,ip);
            op += 4; ip += 4; t -= 4;
        }
        if (t > 0)
        {
            *op++ = *ip++;
            if (t > 1) { *op++ = *ip++; if (t > 2) { *op++ = *ip++; } }
        }
#elif (LZO_OPT_UNALIGNED32) || (LZO_ALIGNED_OK_4)
#if !(LZO_OPT_UNALIGNED32)
        if (PTR_ALIGNED2_4(op,ip))
        {
#endif
        UA_COPY4(op,ip);
        op += 4; ip += 4;
        if (--t > 0)
        {
            if (t >= 4)
            {
                do {
                    UA_COPY4(op,ip);
                    op += 4; ip += 4; t -= 4;
                } while (t >= 4);
                if (t > 0) do *op++ = *ip++; while (--t > 0);
            }
            else
                do *op++ = *ip++; while (--t > 0);
        }
#if !(LZO_OPT_UNALIGNED32)
        }
        else
#endif
#endif
#if !(LZO_OPT_UNALIGNED32)
        {
            *op++ = *ip++;
            *op++ = *ip++;
            *op++ = *ip++;
            do *op++ = *ip++; while (--t > 0);
        }
#endif

        first_literal_run:

        t = *ip++;
        if (t >= 16)
            goto match;
#if defined(COPY_DICT)
#if defined(LZO1Z)
        m_off = (1 + M2_MAX_OFFSET) + (t << 6) + (*ip++ >> 2);
        last_m_off = m_off;
#else
        m_off = (1 + M2_MAX_OFFSET) + (t >> 2) + (*ip++ << 2);
#endif
        NEED_OP(3);
        t = 3; COPY_DICT(t,m_off)
#else
#if defined(LZO1Z)
        t = (1 + M2_MAX_OFFSET) + (t << 6) + (*ip++ >> 2);
        m_pos = op - t;
        last_m_off = t;
#else
        m_pos = op - (1 + M2_MAX_OFFSET);
        m_pos -= t >> 2;
        m_pos -= *ip++ << 2;
#endif
        TEST_LB(m_pos);
        NEED_OP(3);
        *op++ = *m_pos++;
        *op++ = *m_pos++;
        *op++ = *m_pos;
#endif
        goto match_done;

        for (;;) {
            match:
            if (t >= 64) {
#if defined(COPY_DICT)
#if defined(LZO1X)
                m_off = 1 + ((t >> 2) & 7) + (*ip++ << 3);
                t = (t >> 5) - 1;
#elif defined(LZO1Y)
                m_off = 1 + ((t >> 2) & 3) + (*ip++ << 2);
                t = (t >> 4) - 3;
#elif defined(LZO1Z)
                m_off = t & 0x1f;
                if (m_off >= 0x1c)
                    m_off = last_m_off;
                else
                {
                    m_off = 1 + (m_off << 6) + (*ip++ >> 2);
                    last_m_off = m_off;
                }
                t = (t >> 5) - 1;
#endif
#else
#if defined(LZO1X)
                m_pos = op - 1;
                m_pos -= (t >> 2) & 7;
                m_pos -= *ip++ << 3;
                t = (t >> 5) - 1;
#elif defined(LZO1Y)
                m_pos = op - 1;
                m_pos -= (t >> 2) & 3;
                m_pos -= *ip++ << 2;
                t = (t >> 4) - 3;
#elif defined(LZO1Z)
                {
                    lzo_uint off = t & 0x1f;
                    m_pos = op;
                    if (off >= 0x1c)
                    {
                        assert(last_m_off > 0);
                        m_pos -= last_m_off;
                    }
                    else
                    {
                        off = 1 + (off << 6) + (*ip++ >> 2);
                        m_pos -= off;
                        last_m_off = off;
                    }
                }
                t = (t >> 5) - 1;
#endif
                TEST_LB(m_pos);
                assert(t > 0);
                NEED_OP(t + 3 - 1);
                goto copy_match;
#endif
            } else if (t >= 32) {
                t &= 31;
                if (t == 0) {
                    while (*ip == 0) {
                        t += 255;
                        ip++;
                        TEST_OV(t);
                        NEED_IP(1);
                    }
                    t += 31 + *ip++;
                    NEED_IP(2);
                }
#if defined(COPY_DICT)
#if defined(LZO1Z)
                m_off = 1 + (ip[0] << 6) + (ip[1] >> 2);
                last_m_off = m_off;
#else
                m_off = 1 + (ip[0] >> 2) + (ip[1] << 6);
#endif
#else
#if defined(LZO1Z)
                {
                    lzo_uint off = 1 + (ip[0] << 6) + (ip[1] >> 2);
                    m_pos = op - off;
                    last_m_off = off;
                }
#elif (LZO_OPT_UNALIGNED16) && (LZO_ABI_LITTLE_ENDIAN)
                m_pos = op - 1;
                m_pos -= UA_GET_LE16(ip) >> 2;
#else
                m_pos = op - 1;
                m_pos -= (ip[0] >> 2) + (ip[1] << 6);
#endif
#endif
                ip += 2;
            } else if (t >= 16) {
#if defined(COPY_DICT)
                m_off = (t & 8) << 11;
#else
                m_pos = op;
                m_pos -= (t & 8) << 11;
#endif
                t &= 7;
                if (t == 0) {
                    while (*ip == 0) {
                        t += 255;
                        ip++;
                        TEST_OV(t);
                        NEED_IP(1);
                    }
                    t += 7 + *ip++;
                    NEED_IP(2);
                }
#if defined(COPY_DICT)
#if defined(LZO1Z)
                m_off += (ip[0] << 6) + (ip[1] >> 2);
#else
                m_off += (ip[0] >> 2) + (ip[1] << 6);
#endif
                ip += 2;
                if (m_off == 0)
                    goto eof_found;
                m_off += 0x4000;
#if defined(LZO1Z)
                last_m_off = m_off;
#endif
#else
#if defined(LZO1Z)
                m_pos -= (ip[0] << 6) + (ip[1] >> 2);
#elif (LZO_OPT_UNALIGNED16) && (LZO_ABI_LITTLE_ENDIAN)
                m_pos -= UA_GET_LE16(ip) >> 2;
#else
                m_pos -= (ip[0] >> 2) + (ip[1] << 6);
#endif
                ip += 2;
                if (m_pos == op)
                    goto eof_found;
                m_pos -= 0x4000;
#if defined(LZO1Z)
                last_m_off = pd((const lzo_bytep)op, m_pos);
#endif
#endif
            } else {
#if defined(COPY_DICT)
#if defined(LZO1Z)
                m_off = 1 + (t << 6) + (*ip++ >> 2);
                last_m_off = m_off;
#else
                m_off = 1 + (t >> 2) + (*ip++ << 2);
#endif
                NEED_OP(2);
                t = 2; COPY_DICT(t,m_off)
#else
#if defined(LZO1Z)
                t = 1 + (t << 6) + (*ip++ >> 2);
                m_pos = op - t;
                last_m_off = t;
#else
                m_pos = op - 1;
                m_pos -= t >> 2;
                m_pos -= *ip++ << 2;
#endif
                TEST_LB(m_pos);
                NEED_OP(2);
                *op++ = *m_pos++;
                *op++ = *m_pos;
#endif
                goto match_done;
            }

#if defined(COPY_DICT)

            NEED_OP(t+3-1);
            t += 3-1; COPY_DICT(t,m_off)

#else

            TEST_LB(m_pos);
            assert(t > 0);
            NEED_OP(t + 3 - 1);
#if (LZO_OPT_UNALIGNED64) && (LZO_OPT_UNALIGNED32)
            if (op - m_pos >= 8)
            {
                t += (3 - 1);
                if (t >= 8) do
                {
                    UA_COPY8(op,m_pos);
                    op += 8; m_pos += 8; t -= 8;
                } while (t >= 8);
                if (t >= 4)
                {
                    UA_COPY4(op,m_pos);
                    op += 4; m_pos += 4; t -= 4;
                }
                if (t > 0)
                {
                    *op++ = m_pos[0];
                    if (t > 1) { *op++ = m_pos[1]; if (t > 2) { *op++ = m_pos[2]; } }
                }
            }
            else
#elif (LZO_OPT_UNALIGNED32) || (LZO_ALIGNED_OK_4)
#if !(LZO_OPT_UNALIGNED32)
            if (t >= 2 * 4 - (3 - 1) && PTR_ALIGNED2_4(op,m_pos))
            {
                assert((op - m_pos) >= 4);
#else
            if (t >= 2 * 4 - (3 - 1) && (op - m_pos) >= 4)
            {
#endif
                UA_COPY4(op,m_pos);
                op += 4; m_pos += 4; t -= 4 - (3 - 1);
                do {
                    UA_COPY4(op,m_pos);
                    op += 4; m_pos += 4; t -= 4;
                } while (t >= 4);
                if (t > 0) do *op++ = *m_pos++; while (--t > 0);
            }
            else
#endif
            {
                copy_match:
                *op++ = *m_pos++;
                *op++ = *m_pos++;
                do *op++ = *m_pos++; while (--t > 0);
            }

#endif

            match_done:
#if defined(LZO1Z)
            t = ip[-1] & 3;
#else
            t = ip[-2] & 3;
#endif
            if (t == 0)
                break;

            match_next:
            assert(t > 0);
            assert(t < 4);
            NEED_OP(t);
            NEED_IP(t + 3);
#if 0
            do *op++ = *ip++; while (--t > 0);
#else
            *op++ = *ip++;
            if (t > 1) {
                *op++ = *ip++;
                if (t > 2) { *op++ = *ip++; }
            }
#endif
            t = *ip++;
        }
    }

    eof_found:
    *out_len = pd(op, out);
    return (ip == ip_end ? LZO_E_OK :
            (ip < ip_end ? LZO_E_INPUT_NOT_CONSUMED : LZO_E_INPUT_OVERRUN));

#if defined(HAVE_NEED_IP)
    input_overrun:
        *out_len = pd(op, out);
        return LZO_E_INPUT_OVERRUN;
#endif

#if defined(HAVE_NEED_OP)
    output_overrun:
        *out_len = pd(op, out);
        return LZO_E_OUTPUT_OVERRUN;
#endif

#if defined(LZO_TEST_OVERRUN_LOOKBEHIND)
    lookbehind_overrun:
        *out_len = pd(op, out);
        return LZO_E_LOOKBEHIND_OVERRUN;
#endif
}

#define LZO_TEST_OVERRUN 1

#if !defined(MINILZO_CFG_SKIP_LZO1X_DECOMPRESS_SAFE)

#if defined(LZO_TEST_OVERRUN)
#  if !defined(LZO_TEST_OVERRUN_INPUT)
#    define LZO_TEST_OVERRUN_INPUT       2
#  endif
#  if !defined(LZO_TEST_OVERRUN_OUTPUT)
#    define LZO_TEST_OVERRUN_OUTPUT      2
#  endif
#  if !defined(LZO_TEST_OVERRUN_LOOKBEHIND)
#    define LZO_TEST_OVERRUN_LOOKBEHIND  1
#  endif
#endif

#undef TEST_IP
#undef TEST_OP
#undef TEST_IP_AND_TEST_OP
#undef TEST_LB
#undef TEST_LBO
#undef NEED_IP
#undef NEED_OP
#undef TEST_IV
#undef TEST_OV
#undef HAVE_TEST_IP
#undef HAVE_TEST_OP
#undef HAVE_NEED_IP
#undef HAVE_NEED_OP
#undef HAVE_ANY_IP
#undef HAVE_ANY_OP

#if defined(LZO_TEST_OVERRUN_INPUT)
#  if (LZO_TEST_OVERRUN_INPUT >= 1)
#    define TEST_IP             (ip < ip_end)
#  endif
#  if (LZO_TEST_OVERRUN_INPUT >= 2)
#    define NEED_IP(x) \
            if ((lzo_uint)(ip_end - ip) < (lzo_uint)(x))  goto input_overrun
#    define TEST_IV(x)          if ((x) >  (lzo_uint)0 - (511)) goto input_overrun
#  endif
#endif

#if defined(LZO_TEST_OVERRUN_OUTPUT)
#  if (LZO_TEST_OVERRUN_OUTPUT >= 1)
#    define TEST_OP             (op <= op_end)
#  endif
#  if (LZO_TEST_OVERRUN_OUTPUT >= 2)
#    undef TEST_OP
#    define NEED_OP(x) \
            if ((lzo_uint)(op_end - op) < (lzo_uint)(x))  goto output_overrun
#    define TEST_OV(x)          if ((x) >  (lzo_uint)0 - (511)) goto output_overrun
#  endif
#endif

#if defined(LZO_TEST_OVERRUN_LOOKBEHIND)
#  define TEST_LB(m_pos)        if (PTR_LT(m_pos,out) || PTR_GE(m_pos,op)) goto lookbehind_overrun
#  define TEST_LBO(m_pos, o)     if (PTR_LT(m_pos,out) || PTR_GE(m_pos,op-(o))) goto lookbehind_overrun
#else
#  define TEST_LB(m_pos)        ((void) 0)
#  define TEST_LBO(m_pos,o)     ((void) 0)
#endif

#if !defined(LZO_EOF_CODE) && !defined(TEST_IP)
#  define TEST_IP               (ip < ip_end)
#endif

#if defined(TEST_IP)
#  define HAVE_TEST_IP 1
#else
#  define TEST_IP               1
#endif
#if defined(TEST_OP)
#  define HAVE_TEST_OP 1
#else
#  define TEST_OP               1
#endif

#if defined(HAVE_TEST_IP) && defined(HAVE_TEST_OP)
#  define TEST_IP_AND_TEST_OP   (TEST_IP && TEST_OP)
#elif defined(HAVE_TEST_IP)
#  define TEST_IP_AND_TEST_OP   TEST_IP
#elif defined(HAVE_TEST_OP)
#  define TEST_IP_AND_TEST_OP   TEST_OP
#else
#  define TEST_IP_AND_TEST_OP   1
#endif

#if defined(NEED_IP)
#  define HAVE_NEED_IP 1
#else
#  define NEED_IP(x)            ((void) 0)
#  define TEST_IV(x)            ((void) 0)
#endif
#if defined(NEED_OP)
#  define HAVE_NEED_OP 1
#else
#  define NEED_OP(x)            ((void) 0)
#  define TEST_OV(x)            ((void) 0)
#endif

#if defined(HAVE_TEST_IP) || defined(HAVE_NEED_IP)
#  define HAVE_ANY_IP 1
#endif
#if defined(HAVE_TEST_OP) || defined(HAVE_NEED_OP)
#  define HAVE_ANY_OP 1
#endif

LZO_PUBLIC(int)
lzo1x_decompress_safe(const lzo_bytep in, lzo_uint in_len,
                      lzo_bytep out, lzo_uintp out_len)
#endif
{
    lzo_bytep op;
    const lzo_bytep ip;
    lzo_uint t;
#if defined(COPY_DICT)
    lzo_uint m_off;
    const lzo_bytep dict_end;
#else
    const lzo_bytep m_pos;
#endif

    const lzo_bytep const ip_end = in + in_len;
#if defined(HAVE_ANY_OP)
    lzo_bytep const op_end = out + *out_len;
#endif
#if defined(LZO1Z)
    lzo_uint last_m_off = 0;
#endif

#if defined(COPY_DICT)
    if (dict)
    {
        if (dict_len > M4_MAX_OFFSET)
        {
            dict += dict_len - M4_MAX_OFFSET;
            dict_len = M4_MAX_OFFSET;
        }
        dict_end = dict + dict_len;
    }
    else
    {
        dict_len = 0;
        dict_end = NULL;
    }
#endif

    *out_len = 0;

    op = out;
    ip = in;

    NEED_IP(1);
    if (*ip > 17) {
        t = *ip++ - 17;
        if (t < 4)
            goto match_next;
        assert(t > 0);
        NEED_OP(t);
        NEED_IP(t + 3);
        do *op++ = *ip++; while (--t > 0);
        goto first_literal_run;
    }

    for (;;) {
        NEED_IP(3);
        t = *ip++;
        if (t >= 16)
            goto match;
        if (t == 0) {
            while (*ip == 0) {
                t += 255;
                ip++;
                TEST_IV(t);
                NEED_IP(1);
            }
            t += 15 + *ip++;
        }
        assert(t > 0);
        NEED_OP(t + 3);
        NEED_IP(t + 6);
#if (LZO_OPT_UNALIGNED64) && (LZO_OPT_UNALIGNED32)
        t += 3;
        if (t >= 8) do
        {
            UA_COPY8(op,ip);
            op += 8; ip += 8; t -= 8;
        } while (t >= 8);
        if (t >= 4)
        {
            UA_COPY4(op,ip);
            op += 4; ip += 4; t -= 4;
        }
        if (t > 0)
        {
            *op++ = *ip++;
            if (t > 1) { *op++ = *ip++; if (t > 2) { *op++ = *ip++; } }
        }
#elif (LZO_OPT_UNALIGNED32) || (LZO_ALIGNED_OK_4)
#if !(LZO_OPT_UNALIGNED32)
        if (PTR_ALIGNED2_4(op,ip))
        {
#endif
        UA_COPY4(op,ip);
        op += 4; ip += 4;
        if (--t > 0)
        {
            if (t >= 4)
            {
                do {
                    UA_COPY4(op,ip);
                    op += 4; ip += 4; t -= 4;
                } while (t >= 4);
                if (t > 0) do *op++ = *ip++; while (--t > 0);
            }
            else
                do *op++ = *ip++; while (--t > 0);
        }
#if !(LZO_OPT_UNALIGNED32)
        }
        else
#endif
#endif
#if !(LZO_OPT_UNALIGNED32)
        {
            *op++ = *ip++;
            *op++ = *ip++;
            *op++ = *ip++;
            do *op++ = *ip++; while (--t > 0);
        }
#endif

        first_literal_run:

        t = *ip++;
        if (t >= 16)
            goto match;
#if defined(COPY_DICT)
#if defined(LZO1Z)
        m_off = (1 + M2_MAX_OFFSET) + (t << 6) + (*ip++ >> 2);
        last_m_off = m_off;
#else
        m_off = (1 + M2_MAX_OFFSET) + (t >> 2) + (*ip++ << 2);
#endif
        NEED_OP(3);
        t = 3; COPY_DICT(t,m_off)
#else
#if defined(LZO1Z)
        t = (1 + M2_MAX_OFFSET) + (t << 6) + (*ip++ >> 2);
        m_pos = op - t;
        last_m_off = t;
#else
        m_pos = op - (1 + M2_MAX_OFFSET);
        m_pos -= t >> 2;
        m_pos -= *ip++ << 2;
#endif
        TEST_LB(m_pos);
        NEED_OP(3);
        *op++ = *m_pos++;
        *op++ = *m_pos++;
        *op++ = *m_pos;
#endif
        goto match_done;

        for (;;) {
            match:
            if (t >= 64) {
#if defined(COPY_DICT)
#if defined(LZO1X)
                m_off = 1 + ((t >> 2) & 7) + (*ip++ << 3);
                t = (t >> 5) - 1;
#elif defined(LZO1Y)
                m_off = 1 + ((t >> 2) & 3) + (*ip++ << 2);
                t = (t >> 4) - 3;
#elif defined(LZO1Z)
                m_off = t & 0x1f;
                if (m_off >= 0x1c)
                    m_off = last_m_off;
                else
                {
                    m_off = 1 + (m_off << 6) + (*ip++ >> 2);
                    last_m_off = m_off;
                }
                t = (t >> 5) - 1;
#endif
#else
#if defined(LZO1X)
                m_pos = op - 1;
                m_pos -= (t >> 2) & 7;
                m_pos -= *ip++ << 3;
                t = (t >> 5) - 1;
#elif defined(LZO1Y)
                m_pos = op - 1;
                m_pos -= (t >> 2) & 3;
                m_pos -= *ip++ << 2;
                t = (t >> 4) - 3;
#elif defined(LZO1Z)
                {
                    lzo_uint off = t & 0x1f;
                    m_pos = op;
                    if (off >= 0x1c)
                    {
                        assert(last_m_off > 0);
                        m_pos -= last_m_off;
                    }
                    else
                    {
                        off = 1 + (off << 6) + (*ip++ >> 2);
                        m_pos -= off;
                        last_m_off = off;
                    }
                }
                t = (t >> 5) - 1;
#endif
                TEST_LB(m_pos);
                assert(t > 0);
                NEED_OP(t + 3 - 1);
                goto copy_match;
#endif
            } else if (t >= 32) {
                t &= 31;
                if (t == 0) {
                    while (*ip == 0) {
                        t += 255;
                        ip++;
                        TEST_OV(t);
                        NEED_IP(1);
                    }
                    t += 31 + *ip++;
                    NEED_IP(2);
                }
#if defined(COPY_DICT)
#if defined(LZO1Z)
                m_off = 1 + (ip[0] << 6) + (ip[1] >> 2);
                last_m_off = m_off;
#else
                m_off = 1 + (ip[0] >> 2) + (ip[1] << 6);
#endif
#else
#if defined(LZO1Z)
                {
                    lzo_uint off = 1 + (ip[0] << 6) + (ip[1] >> 2);
                    m_pos = op - off;
                    last_m_off = off;
                }
#elif (LZO_OPT_UNALIGNED16) && (LZO_ABI_LITTLE_ENDIAN)
                m_pos = op - 1;
                m_pos -= UA_GET_LE16(ip) >> 2;
#else
                m_pos = op - 1;
                m_pos -= (ip[0] >> 2) + (ip[1] << 6);
#endif
#endif
                ip += 2;
            } else if (t >= 16) {
#if defined(COPY_DICT)
                m_off = (t & 8) << 11;
#else
                m_pos = op;
                m_pos -= (t & 8) << 11;
#endif
                t &= 7;
                if (t == 0) {
                    while (*ip == 0) {
                        t += 255;
                        ip++;
                        TEST_OV(t);
                        NEED_IP(1);
                    }
                    t += 7 + *ip++;
                    NEED_IP(2);
                }
#if defined(COPY_DICT)
#if defined(LZO1Z)
                m_off += (ip[0] << 6) + (ip[1] >> 2);
#else
                m_off += (ip[0] >> 2) + (ip[1] << 6);
#endif
                ip += 2;
                if (m_off == 0)
                    goto eof_found;
                m_off += 0x4000;
#if defined(LZO1Z)
                last_m_off = m_off;
#endif
#else
#if defined(LZO1Z)
                m_pos -= (ip[0] << 6) + (ip[1] >> 2);
#elif (LZO_OPT_UNALIGNED16) && (LZO_ABI_LITTLE_ENDIAN)
                m_pos -= UA_GET_LE16(ip) >> 2;
#else
                m_pos -= (ip[0] >> 2) + (ip[1] << 6);
#endif
                ip += 2;
                if (m_pos == op)
                    goto eof_found;
                m_pos -= 0x4000;
#if defined(LZO1Z)
                last_m_off = pd((const lzo_bytep)op, m_pos);
#endif
#endif
            } else {
#if defined(COPY_DICT)
#if defined(LZO1Z)
                m_off = 1 + (t << 6) + (*ip++ >> 2);
                last_m_off = m_off;
#else
                m_off = 1 + (t >> 2) + (*ip++ << 2);
#endif
                NEED_OP(2);
                t = 2; COPY_DICT(t,m_off)
#else
#if defined(LZO1Z)
                t = 1 + (t << 6) + (*ip++ >> 2);
                m_pos = op - t;
                last_m_off = t;
#else
                m_pos = op - 1;
                m_pos -= t >> 2;
                m_pos -= *ip++ << 2;
#endif
                TEST_LB(m_pos);
                NEED_OP(2);
                *op++ = *m_pos++;
                *op++ = *m_pos;
#endif
                goto match_done;
            }

#if defined(COPY_DICT)

            NEED_OP(t+3-1);
            t += 3-1; COPY_DICT(t,m_off)

#else

            TEST_LB(m_pos);
            assert(t > 0);
            NEED_OP(t + 3 - 1);
#if (LZO_OPT_UNALIGNED64) && (LZO_OPT_UNALIGNED32)
            if (op - m_pos >= 8)
            {
                t += (3 - 1);
                if (t >= 8) do
                {
                    UA_COPY8(op,m_pos);
                    op += 8; m_pos += 8; t -= 8;
                } while (t >= 8);
                if (t >= 4)
                {
                    UA_COPY4(op,m_pos);
                    op += 4; m_pos += 4; t -= 4;
                }
                if (t > 0)
                {
                    *op++ = m_pos[0];
                    if (t > 1) { *op++ = m_pos[1]; if (t > 2) { *op++ = m_pos[2]; } }
                }
            }
            else
#elif (LZO_OPT_UNALIGNED32) || (LZO_ALIGNED_OK_4)
#if !(LZO_OPT_UNALIGNED32)
            if (t >= 2 * 4 - (3 - 1) && PTR_ALIGNED2_4(op,m_pos))
            {
                assert((op - m_pos) >= 4);
#else
            if (t >= 2 * 4 - (3 - 1) && (op - m_pos) >= 4)
            {
#endif
                UA_COPY4(op,m_pos);
                op += 4; m_pos += 4; t -= 4 - (3 - 1);
                do {
                    UA_COPY4(op,m_pos);
                    op += 4; m_pos += 4; t -= 4;
                } while (t >= 4);
                if (t > 0) do *op++ = *m_pos++; while (--t > 0);
            }
            else
#endif
            {
                copy_match:
                *op++ = *m_pos++;
                *op++ = *m_pos++;
                do *op++ = *m_pos++; while (--t > 0);
            }

#endif

            match_done:
#if defined(LZO1Z)
            t = ip[-1] & 3;
#else
            t = ip[-2] & 3;
#endif
            if (t == 0)
                break;

            match_next:
            assert(t > 0);
            assert(t < 4);
            NEED_OP(t);
            NEED_IP(t + 3);
#if 0
            do *op++ = *ip++; while (--t > 0);
#else
            *op++ = *ip++;
            if (t > 1) {
                *op++ = *ip++;
                if (t > 2) { *op++ = *ip++; }
            }
#endif
            t = *ip++;
        }
    }

    eof_found:
    *out_len = pd(op, out);
    return (ip == ip_end ? LZO_E_OK :
            (ip < ip_end ? LZO_E_INPUT_NOT_CONSUMED : LZO_E_INPUT_OVERRUN));

#if defined(HAVE_NEED_IP)
    input_overrun:
    *out_len = pd(op, out);
    return LZO_E_INPUT_OVERRUN;
#endif

#if defined(HAVE_NEED_OP)
    output_overrun:
    *out_len = pd(op, out);
    return LZO_E_OUTPUT_OVERRUN;
#endif

#if defined(LZO_TEST_OVERRUN_LOOKBEHIND)
    lookbehind_overrun:
    *out_len = pd(op, out);
    return LZO_E_LOOKBEHIND_OVERRUN;
#endif
}
