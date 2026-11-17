/*
 * Copyright (C) 2026, D. R. Commander.  All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of the libjpeg-turbo Project nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS",
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * This program tests EXIF orientation handling in tj3Transform() (the
 * TJXOPT_AUTOORIENT transform option.)  It verifies that:
 *
 * 1. Each of the eight EXIF orientation values produces output that is
 *    byte-identical to that of the equivalent explicit transform operation,
 *    with both little-endian and big-endian TIFF headers.
 * 2. Auto-orienting a "camera" image (an upright image that was losslessly
 *    rotated/flipped and then tagged with the corresponding EXIF orientation)
 *    reproduces the upright image exactly.
 * 3. A missing or unparseable EXIF orientation tag is a no-op.
 * 4. An invalid EXIF orientation value (not 1-8) is a fatal error.
 * 5. Combining TJXOPT_AUTOORIENT with an explicit transform operation is a
 *    fatal error.
 * 6. Unless TJXOPT_AUTOORIENT is specified, EXIF orientation tags are ignored
 *    and tj3Transform() behaves exactly as it did before.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "turbojpeg.h"

#define WIDTH  48
#define HEIGHT 32

/* The transform operation that corrects each EXIF orientation (1-8) */
static const int orientOp[9] = {
  TJXOP_NONE, TJXOP_NONE, TJXOP_HFLIP, TJXOP_ROT180, TJXOP_VFLIP,
  TJXOP_TRANSPOSE, TJXOP_ROT90, TJXOP_TRANSVERSE, TJXOP_ROT270
};

/* The inverse of orientOp[], used to simulate how a camera stores pixels */
static const int orientInvOp[9] = {
  TJXOP_NONE, TJXOP_NONE, TJXOP_HFLIP, TJXOP_ROT180, TJXOP_VFLIP,
  TJXOP_TRANSPOSE, TJXOP_ROT270, TJXOP_TRANSVERSE, TJXOP_ROT90
};

static int exitStatus = 0;

#define THROW_TJ(f) { \
  printf("FAIL (%s:%d): %s\n", f, __LINE__, tj3GetErrorStr(handle)); \
  exitStatus = 1; \
  goto bailout; \
}

#define THROW_MSG(f, m) { \
  printf("FAIL (%s:%d): %s\n", f, __LINE__, m); \
  exitStatus = 1; \
  goto bailout; \
}


/* Build an EXIF APP1 payload containing the given orientation value. */
static unsigned char *buildEXIF(int orientation, int bigEndian, int malformed,
                                size_t *payloadLen)
{
  unsigned char *p, *payload;

  *payloadLen = malformed ? 6 + 3 : 6 + 26;
  if ((payload = malloc(*payloadLen)) == NULL)
    return NULL;
  p = payload;
  memcpy(p, "Exif\0\0", 6);  p += 6;
  if (malformed) {
    /* Truncated TIFF header (unparseable) */
    *p++ = 0xDE;  *p++ = 0xAD;  *p++ = 0xBE;
  } else if (bigEndian) {
    *p++ = 'M';  *p++ = 'M';
    *p++ = 0;  *p++ = 42;                       /* TIFF magic */
    *p++ = 0;  *p++ = 0;  *p++ = 0;  *p++ = 8;  /* IFD0 offset */
    *p++ = 0;  *p++ = 1;                        /* one entry */
    *p++ = 0x01;  *p++ = 0x12;                  /* tag: Orientation */
    *p++ = 0;  *p++ = 3;                        /* type: SHORT */
    *p++ = 0;  *p++ = 0;  *p++ = 0;  *p++ = 1;  /* count */
    *p++ = 0;  *p++ = (unsigned char)orientation;
    *p++ = 0;  *p++ = 0;
    *p++ = 0;  *p++ = 0;  *p++ = 0;  *p++ = 0;  /* next IFD */
  } else {
    *p++ = 'I';  *p++ = 'I';
    *p++ = 42;  *p++ = 0;
    *p++ = 8;  *p++ = 0;  *p++ = 0;  *p++ = 0;
    *p++ = 1;  *p++ = 0;
    *p++ = 0x12;  *p++ = 0x01;
    *p++ = 3;  *p++ = 0;
    *p++ = 1;  *p++ = 0;  *p++ = 0;  *p++ = 0;
    *p++ = (unsigned char)orientation;  *p++ = 0;
    *p++ = 0;  *p++ = 0;
    *p++ = 0;  *p++ = 0;  *p++ = 0;  *p++ = 0;
  }
  return payload;
}


/* Insert an APP1 segment with the given payload immediately after the SOI
   marker of a JPEG image. */
static unsigned char *insertAPP1(const unsigned char *jpegBuf, size_t jpegSize,
                                 const unsigned char *payload,
                                 size_t payloadLen, size_t *newSize)
{
  size_t app1Len = payloadLen + 2;  /* the length field includes itself */
  unsigned char *out;

  if (jpegSize < 2 || jpegBuf[0] != 0xFF || jpegBuf[1] != 0xD8 ||
      app1Len > 0xFFFF)
    return NULL;
  *newSize = jpegSize + 2 + app1Len;
  if ((out = malloc(*newSize)) == NULL)
    return NULL;
  out[0] = 0xFF;  out[1] = 0xD8;
  out[2] = 0xFF;  out[3] = 0xE1;
  out[4] = (unsigned char)(app1Len >> 8);
  out[5] = (unsigned char)(app1Len & 0xFF);
  memcpy(out + 6, payload, payloadLen);
  memcpy(out + 6 + payloadLen, jpegBuf + 2, jpegSize - 2);
  return out;
}


/* Convenience wrapper: return a copy of jpegBuf with an EXIF orientation
   APP1 segment inserted. */
static unsigned char *addEXIF(const unsigned char *jpegBuf, size_t jpegSize,
                              int orientation, int bigEndian, int malformed,
                              size_t *newSize)
{
  unsigned char *payload, *out;
  size_t payloadLen;

  if ((payload = buildEXIF(orientation, bigEndian, malformed,
                           &payloadLen)) == NULL)
    return NULL;
  out = insertAPP1(jpegBuf, jpegSize, payload, payloadLen, newSize);
  free(payload);
  return out;
}


static int doTransform(tjhandle handle, const unsigned char *jpegBuf,
                       size_t jpegSize, int op, int options,
                       unsigned char **dstBuf, size_t *dstSize,
                       size_t *bufSize)
{
  tjtransform t;

  memset(&t, 0, sizeof(t));
  t.op = op;
  t.options = options;
  *dstBuf = NULL;
  *dstSize = 0;
  if (tj3DecompressHeader(handle, jpegBuf, jpegSize) != 0)
    return -1;
  if (bufSize != NULL)
    *bufSize = tj3TransformBufSize(handle, &t);
  return tj3Transform(handle, jpegBuf, jpegSize, 1, dstBuf, dstSize, &t);
}


static unsigned char *decompress(tjhandle handle, const unsigned char *jpegBuf,
                                 size_t jpegSize, int *w, int *h)
{
  unsigned char *buf;

  if (tj3DecompressHeader(handle, jpegBuf, jpegSize) != 0)
    return NULL;
  *w = tj3Get(handle, TJPARAM_JPEGWIDTH);
  *h = tj3Get(handle, TJPARAM_JPEGHEIGHT);
  if ((buf = malloc((size_t)(*w) * (size_t)(*h) * 3)) == NULL)
    return NULL;
  if (tj3Decompress8(handle, jpegBuf, jpegSize, buf, 0, TJPF_RGB) != 0) {
    free(buf);
    return NULL;
  }
  return buf;
}


int main(void)
{
  tjhandle handle = NULL, compHandle = NULL;
  unsigned char *srcBuf = NULL, *jpegBuf = NULL;
  size_t jpegSize = 0;
  int x, y, o, be;

  if ((handle = tj3Init(TJINIT_TRANSFORM)) == NULL) {
    printf("FAIL: tj3Init(TJINIT_TRANSFORM): %s\n", tj3GetErrorStr(NULL));
    return 1;
  }
  if ((compHandle = tj3Init(TJINIT_COMPRESS)) == NULL) {
    printf("FAIL: tj3Init(TJINIT_COMPRESS): %s\n", tj3GetErrorStr(NULL));
    exitStatus = 1;
    goto bailout;
  }

  /* Generate a test image in which every pixel is a unique function of its
     coordinates, compress it, and use it as the upright reference image. */
  if ((srcBuf = malloc(WIDTH * HEIGHT * 3)) == NULL)
    THROW_MSG("main", "Memory allocation failure");
  for (y = 0; y < HEIGHT; y++)
    for (x = 0; x < WIDTH; x++) {
      srcBuf[(y * WIDTH + x) * 3 + 0] = (unsigned char)(x * 5 + y);
      srcBuf[(y * WIDTH + x) * 3 + 1] = (unsigned char)(y * 7 + x);
      srcBuf[(y * WIDTH + x) * 3 + 2] = (unsigned char)((x * y) * 3 + 13);
    }
  if (tj3Set(compHandle, TJPARAM_QUALITY, 90) != 0 ||
      tj3Set(compHandle, TJPARAM_SUBSAMP, TJSAMP_420) != 0) {
    printf("FAIL: tj3Set(): %s\n", tj3GetErrorStr(compHandle));
    exitStatus = 1;
    goto bailout;
  }
  if (tj3Compress8(compHandle, srcBuf, WIDTH, 0, HEIGHT, TJPF_RGB, &jpegBuf,
                   &jpegSize) != 0) {
    printf("FAIL: tj3Compress8(): %s\n", tj3GetErrorStr(compHandle));
    exitStatus = 1;
    goto bailout;
  }

  /* Test 1: For each EXIF orientation value, auto-orientation must produce
     output that is byte-identical to that of the equivalent explicit
     transform operation applied to the same source image. */
  for (be = 0; be <= 1; be++) {
    for (o = 1; o <= 8; o++) {
      unsigned char *exifBuf = NULL, *autoBuf = NULL, *refBuf = NULL;
      size_t exifSize = 0, autoSize = 0, refSize = 0, bufSize = 0;

      if ((exifBuf = addEXIF(jpegBuf, jpegSize, o, be, 0, &exifSize)) == NULL)
        THROW_MSG("Test 1", "Memory allocation failure");
      if (doTransform(handle, exifBuf, exifSize, TJXOP_NONE,
                      TJXOPT_AUTOORIENT | TJXOPT_COPYNONE, &autoBuf, &autoSize,
                      &bufSize) != 0)
        THROW_TJ("Test 1");
      if (doTransform(handle, exifBuf, exifSize, orientOp[o], TJXOPT_COPYNONE,
                      &refBuf, &refSize, NULL) != 0)
        THROW_TJ("Test 1");
      if (autoSize != refSize || memcmp(autoBuf, refBuf, autoSize) != 0) {
        printf("FAIL (Test 1): EXIF orientation %d (%s-endian) output does not match explicit transform\n",
               o, be ? "big" : "little");
        exitStatus = 1;
      }
      if (bufSize < autoSize) {
        printf("FAIL (Test 1): tj3TransformBufSize() returned %lu, but %lu bytes were needed\n",
               (unsigned long)bufSize, (unsigned long)autoSize);
        exitStatus = 1;
      }
      free(exifBuf);
      tj3Free(autoBuf);
      tj3Free(refBuf);
      if (exitStatus) goto bailout;
    }
  }
  printf("Test 1 (orientation -> transform op mapping, LE & BE): OK\n");

  /* Test 2: Simulate a camera by losslessly rotating/flipping the upright
     image and tagging it with the corresponding EXIF orientation.  Auto-
     orientation must reproduce the upright image exactly. */
  for (o = 1; o <= 8; o++) {
    unsigned char *camBuf = NULL, *camExifBuf = NULL, *fixedBuf = NULL;
    unsigned char *fixedPixels = NULL, *refPixels = NULL;
    size_t camSize = 0, camExifSize = 0, fixedSize = 0;
    int fw = 0, fh = 0, rw = 0, rh = 0;

    if (doTransform(handle, jpegBuf, jpegSize, orientInvOp[o],
                    TJXOPT_COPYNONE, &camBuf, &camSize, NULL) != 0)
      THROW_TJ("Test 2");
    if ((camExifBuf = addEXIF(camBuf, camSize, o, 0, 0, &camExifSize)) == NULL)
      THROW_MSG("Test 2", "Memory allocation failure");
    if (doTransform(handle, camExifBuf, camExifSize, TJXOP_NONE,
                    TJXOPT_AUTOORIENT | TJXOPT_COPYNONE, &fixedBuf, &fixedSize,
                    NULL) != 0)
      THROW_TJ("Test 2");
    if ((fixedPixels = decompress(handle, fixedBuf, fixedSize, &fw,
                                  &fh)) == NULL)
      THROW_TJ("Test 2");
    if ((refPixels = decompress(handle, jpegBuf, jpegSize, &rw, &rh)) == NULL)
      THROW_TJ("Test 2");
    if (fw != WIDTH || fh != HEIGHT || rw != WIDTH || rh != HEIGHT ||
        memcmp(fixedPixels, refPixels, (size_t)WIDTH * HEIGHT * 3) != 0) {
      printf("FAIL (Test 2): auto-orientation of EXIF orientation %d did not reproduce the upright image\n",
             o);
      exitStatus = 1;
    }
    tj3Free(camBuf);
    free(camExifBuf);
    tj3Free(fixedBuf);
    free(fixedPixels);
    free(refPixels);
    if (exitStatus) goto bailout;
  }
  printf("Test 2 (camera simulation reproduces upright image): OK\n");

  /* Test 3: A JPEG image without an EXIF orientation tag is not transformed
     when TJXOPT_AUTOORIENT is specified. */
  {
    unsigned char *autoBuf = NULL, *refBuf = NULL;
    size_t autoSize = 0, refSize = 0;

    if (doTransform(handle, jpegBuf, jpegSize, TJXOP_NONE,
                    TJXOPT_AUTOORIENT | TJXOPT_COPYNONE, &autoBuf, &autoSize,
                    NULL) != 0)
      THROW_TJ("Test 3");
    if (doTransform(handle, jpegBuf, jpegSize, TJXOP_NONE, TJXOPT_COPYNONE,
                    &refBuf, &refSize, NULL) != 0)
      THROW_TJ("Test 3");
    if (autoSize != refSize || memcmp(autoBuf, refBuf, autoSize) != 0) {
      printf("FAIL (Test 3): TJXOPT_AUTOORIENT altered an image with no EXIF orientation\n");
      exitStatus = 1;
    }
    tj3Free(autoBuf);
    tj3Free(refBuf);
    if (exitStatus) goto bailout;
  }
  printf("Test 3 (missing EXIF orientation is a no-op): OK\n");

  /* Test 4: Unless TJXOPT_AUTOORIENT is specified, an EXIF orientation tag
     is ignored, and the output does not depend on the orientation value. */
  {
    unsigned char *refOutBuf = NULL;
    size_t refOutSize = 0;

    for (o = 1; o <= 8; o++) {
      unsigned char *exifBuf = NULL, *outBuf = NULL;
      size_t exifSize = 0, outSize = 0;

      if ((exifBuf = addEXIF(jpegBuf, jpegSize, o, 0, 0, &exifSize)) == NULL)
        THROW_MSG("Test 4", "Memory allocation failure");
      if (doTransform(handle, exifBuf, exifSize, TJXOP_NONE, TJXOPT_COPYNONE,
                      &outBuf, &outSize, NULL) != 0)
        THROW_TJ("Test 4");
      if (o == 1) {
        refOutBuf = outBuf;
        refOutSize = outSize;
      } else {
        if (outSize != refOutSize ||
            memcmp(outBuf, refOutBuf, outSize) != 0) {
          printf("FAIL (Test 4): EXIF orientation %d affected output without TJXOPT_AUTOORIENT\n",
                 o);
          exitStatus = 1;
        }
        tj3Free(outBuf);
      }
      free(exifBuf);
      if (exitStatus) goto bailout;
    }
    tj3Free(refOutBuf);
  }
  printf("Test 4 (EXIF ignored without TJXOPT_AUTOORIENT): OK\n");

  /* Test 5: An invalid EXIF orientation value is a fatal error. */
  for (o = 0; o <= 9; o += 9) {
    unsigned char *exifBuf = NULL, *dstBuf = NULL;
    size_t exifSize = 0, dstSize = 0;

    if ((exifBuf = addEXIF(jpegBuf, jpegSize, o, 0, 0, &exifSize)) == NULL)
      THROW_MSG("Test 5", "Memory allocation failure");
    if (doTransform(handle, exifBuf, exifSize, TJXOP_NONE,
                    TJXOPT_AUTOORIENT | TJXOPT_COPYNONE, &dstBuf,
                    &dstSize, NULL) == 0 ||
        tj3GetErrorCode(handle) != TJERR_FATAL ||
        strstr(tj3GetErrorStr(handle), "EXIF orientation") == NULL) {
      printf("FAIL (Test 5): invalid EXIF orientation %d not properly rejected (error: %s)\n",
             o, tj3GetErrorStr(handle));
      exitStatus = 1;
    }
    free(exifBuf);
    tj3Free(dstBuf);
    if (exitStatus) goto bailout;
  }
  printf("Test 5 (invalid EXIF orientation is a fatal error): OK\n");

  /* Test 6: TJXOPT_AUTOORIENT cannot be combined with an explicit transform
     operation. */
  {
    unsigned char *dstBuf = NULL;
    size_t dstSize = 0;

    if (doTransform(handle, jpegBuf, jpegSize, TJXOP_ROT90,
                    TJXOPT_AUTOORIENT | TJXOPT_COPYNONE, &dstBuf, &dstSize,
                    NULL) == 0 ||
        tj3GetErrorCode(handle) != TJERR_FATAL ||
        strstr(tj3GetErrorStr(handle), "AUTOORIENT") == NULL) {
      printf("FAIL (Test 6): TJXOPT_AUTOORIENT + explicit op not properly rejected (error: %s)\n",
             tj3GetErrorStr(handle));
      exitStatus = 1;
    }
    tj3Free(dstBuf);
    if (exitStatus) goto bailout;
  }
  printf("Test 6 (TJXOPT_AUTOORIENT + explicit op is a fatal error): OK\n");

  /* Test 7: An unparseable EXIF segment is treated the same as a missing
     EXIF orientation tag. */
  {
    unsigned char *exifBuf = NULL, *autoBuf = NULL, *refBuf = NULL;
    size_t exifSize = 0, autoSize = 0, refSize = 0;

    if ((exifBuf = addEXIF(jpegBuf, jpegSize, 0, 0, 1, &exifSize)) == NULL)
      THROW_MSG("Test 7", "Memory allocation failure");
    if (doTransform(handle, exifBuf, exifSize, TJXOP_NONE,
                    TJXOPT_AUTOORIENT | TJXOPT_COPYNONE, &autoBuf, &autoSize,
                    NULL) != 0)
      THROW_TJ("Test 7");
    if (doTransform(handle, exifBuf, exifSize, TJXOP_NONE, TJXOPT_COPYNONE,
                    &refBuf, &refSize, NULL) != 0)
      THROW_TJ("Test 7");
    if (autoSize != refSize || memcmp(autoBuf, refBuf, autoSize) != 0) {
      printf("FAIL (Test 7): malformed EXIF segment not treated as missing orientation\n");
      exitStatus = 1;
    }
    free(exifBuf);
    tj3Free(autoBuf);
    tj3Free(refBuf);
    if (exitStatus) goto bailout;
  }
  printf("Test 7 (malformed EXIF segment is a no-op): OK\n");

  /* Test 8: A non-EXIF APP1 segment preceding the EXIF APP1 segment does not
     interfere with orientation parsing. */
  {
    static const char xmp[] = "http://ns.adobe.com/xap/1.0/";
    unsigned char *xmpBuf = NULL, *exifBuf = NULL, *autoBuf = NULL;
    unsigned char *refBuf = NULL;
    size_t xmpSize = 0, exifSize = 0, autoSize = 0, refSize = 0;

    if ((xmpBuf = insertAPP1(jpegBuf, jpegSize, (const unsigned char *)xmp,
                             sizeof(xmp), &xmpSize)) == NULL)
      THROW_MSG("Test 8", "Memory allocation failure");
    if ((exifBuf = addEXIF(xmpBuf, xmpSize, 6, 1, 0, &exifSize)) == NULL)
      THROW_MSG("Test 8", "Memory allocation failure");
    if (doTransform(handle, exifBuf, exifSize, TJXOP_NONE,
                    TJXOPT_AUTOORIENT | TJXOPT_COPYNONE, &autoBuf, &autoSize,
                    NULL) != 0)
      THROW_TJ("Test 8");
    if (doTransform(handle, exifBuf, exifSize, TJXOP_ROT90, TJXOPT_COPYNONE,
                    &refBuf, &refSize, NULL) != 0)
      THROW_TJ("Test 8");
    if (autoSize != refSize || memcmp(autoBuf, refBuf, autoSize) != 0) {
      printf("FAIL (Test 8): non-EXIF APP1 segment interfered with orientation parsing\n");
      exitStatus = 1;
    }
    free(xmpBuf);
    free(exifBuf);
    tj3Free(autoBuf);
    tj3Free(refBuf);
    if (exitStatus) goto bailout;
  }
  printf("Test 8 (non-EXIF APP1 segment skipped): OK\n");

bailout:
  free(srcBuf);
  tj3Free(jpegBuf);
  tj3Destroy(compHandle);
  tj3Destroy(handle);

  if (exitStatus == 0)
    printf("All EXIF orientation tests passed.\n");
  return exitStatus;
}
