

#ifndef __MINILZO_H_INCLUDED
#define __MINILZO_H_INCLUDED 1

#define MINILZO_VERSION         0x20a0  /* 2.10 */

#include <limits.h>
#include <stddef.h>

#include "lzodefs.h"
#include "lzoconf.h"

#if !defined(LZO_VERSION) || (LZO_VERSION != MINILZO_VERSION)
#  error "version mismatch in header files"
#endif

#ifdef __cplusplus
extern "C" {
#endif


/***********************************************************************
//
************************************************************************/

/* Memory required for the wrkmem parameter.
 * When the required size is 0, you can also pass a NULL pointer.
 */

//#define LZO1X_MEM_COMPRESS      LZO1X_1_MEM_COMPRESS
#define LZO1X_1_MEM_COMPRESS    ((lzo_uint32_t) (16384L * lzo_sizeof_dict_t))


/* compression */
LZO_EXTERN(int)
lzo1x_1_compress(const lzo_bytep src, lzo_uint src_len,
                 lzo_bytep dst, lzo_uintp dst_len,
                 lzo_voidp wrkmem);

/* decompression */
LZO_EXTERN(int)
lzo1x_decompress(const lzo_bytep src, lzo_uint src_len,
                 lzo_bytep dst, lzo_uintp dst_len);

/* safe decompression with overrun testing */
LZO_EXTERN(int)
lzo1x_decompress_safe(const lzo_bytep src, lzo_uint src_len,
                      lzo_bytep dst, lzo_uintp dst_len);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* already included */


/* vim:set ts=4 sw=4 et: */
