/***************************************************************************
*    Copyright (c) 2013-2017, Broadcom Corporation
*    All rights reserved.
*
*  Statement regarding contribution of copyrighted materials to VESA:
*
*  This code is owned by Broadcom Corporation and is contributed to VESA
*  for inclusion and use in its VESA Display Stream Compression specification.
*  Accordingly, VESA is hereby granted a worldwide, perpetual, non-exclusive
*  license to revise, modify and create derivative works to this code and
*  VESA shall own all right, title and interest in and to any derivative 
*  works authored by VESA.
*
*  Terms and Conditions
*
*  Without limiting the foregoing, you agree that your use
*  of this software program does not convey any rights to you in any of
*  Broadcom’s patent and other intellectual property, and you
*  acknowledge that your use of this software may require that
*  you separately obtain patent or other intellectual property
*  rights from Broadcom or third parties.
*
*  Except as expressly set forth in a separate written license agreement
*  between you and Broadcom, if applicable:
*
*  1. TO THE MAXIMUM EXTENT PERMITTED BY LAW, THE SOFTWARE IS PROVIDED
*  "AS IS" AND WITH ALL FAULTS AND BROADCOM MAKES NO PROMISES,
*  REPRESENTATIONS OR WARRANTIES, EITHER EXPRESS, IMPLIED, STATUTORY, OR
*  OTHERWISE, WITH RESPECT TO THE SOFTWARE.  BROADCOM SPECIFICALLY
*  DISCLAIMS ANY AND ALL IMPLIED WARRANTIES OF TITLE, MERCHANTABILITY,
*  NONINFRINGEMENT, FITNESS FOR A PARTICULAR PURPOSE, LACK OF VIRUSES,
*  ACCURACY OR COMPLETENESS, QUIET ENJOYMENT, QUIET POSSESSION OR
*  CORRESPONDENCE TO DESCRIPTION. YOU ASSUME THE ENTIRE RISK ARISING
*  OUT OF USE OR PERFORMANCE OF THE SOFTWARE.
* 
*  2. TO THE MAXIMUM EXTENT PERMITTED BY LAW, IN NO EVENT SHALL
*  BROADCOM OR ITS LICENSORS BE LIABLE FOR (i) CONSEQUENTIAL, INCIDENTAL,
*  SPECIAL, INDIRECT, OR EXEMPLARY DAMAGES WHATSOEVER ARISING OUT OF OR
*  IN ANY WAY RELATING TO YOUR USE OF OR INABILITY TO USE THE SOFTWARE EVEN
*  IF BROADCOM HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES; OR (ii)
*  ANY AMOUNT IN EXCESS OF THE AMOUNT ACTUALLY PAID FOR THE SOFTWARE ITSELF
*  OR U.S. $1, WHICHEVER IS GREATER. THESE LIMITATIONS SHALL APPLY
*  NOTWITHSTANDING ANY FAILURE OF ESSENTIAL PURPOSE OF ANY LIMITED REMEDY.
***************************************************************************/

#ifndef __DSC_UTILS_H_
#define __DSC_UTILS_H_

#include "utl.h"
#include "dsc_types.h"
//#define REDUCE_CHROMA_12BPC

void putbits(int val, int size, unsigned char *buf, int *bit_count);
int getbits(int size, unsigned char *buf, int *bit_count, int sign_extend);

void *pcreateb(int format, int color, int chroma, int w, int h, int bits);

void rgb2ycocg(pic_t *ip, pic_t *op, dsc_cfg_t *bdc_cfg);
void ycocg2rgb(pic_t *ip, pic_t *op, dsc_cfg_t* bdc_cfg);

void simple422to444(pic_t *ip, pic_t *op);
void simple444to422(pic_t *ip, pic_t *op);

void parse_pps(unsigned char *buf, dsc_cfg_t *dsc_cfg);
void write_pps(unsigned char *buf, dsc_cfg_t *dsc_cfg);
void print_pps(FILE *logfp, dsc_cfg_t *dsc_cfg);
void print_pps_v2(FILE *logfp, dsc_cfg_t *dsc_cfg);

int ceil_log2(int val);

void yuv_422_444_region(pic_t *p, dsc_cfg_t *dsc_cfg);
void yuv_444_422_region(pic_t *p, dsc_cfg_t *dsc_cfg);

#endif // __DSC_UTILS_H_
