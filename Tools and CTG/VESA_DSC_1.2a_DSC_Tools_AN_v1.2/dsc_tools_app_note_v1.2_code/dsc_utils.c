/***************************************************************************
*    Copyright (c) 2013-2017, Broadcom Ltd.
*    All rights reserved.
*    VESA CONFIDENTIAL
*
*  Statement regarding contribution of copyrighted materials to VESA:
*
*  This code is owned by Broadcom Limited and is contributed to VESA
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

#include "dsc_utils.h"
#include "dsc_types.h"
#include "logging.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#define REDUCE_CHROMA_16BPC

extern fir_t chm_444_422_11;
extern fir_t chm_422_444_11;


/*! \file dsc_utils.c
 *    DSC utility functions */

//! Calculate ceil(log2(value))
/*! \param val		 Value to process
    \return          Result of computation */
int ceil_log2(int val)
{
	int ret = 0, x;
	x = val;
	while(x) { ret++; x>>=1; }
	return(ret);
}


//! Put bits into a buffer in memory
/*! \param val		 Value to write
    \param size		 Number of bits to write
	\param buf       Pointer to buffer location
	\param bit_count Bit index into buffer (modified) */
void putbits(int val, int size, unsigned char *buf, int *bit_count)
{
	int i;
	int curbit;
	int bitcntmod8;
	int bufidx;

	if(size>32)
		printf("error: putbits supports max of 32 bits\n");
	for(i=size-1; i>=0; --i)
	{
		bitcntmod8 = (*bit_count)%8;
		bufidx = (*bit_count)>>3;
		curbit = (val >> i) & 1;
		if(bitcntmod8 == 0)
			buf[bufidx] = 0;		// Zero current byte
		if(curbit)
			buf[bufidx] |= (1<<(7-bitcntmod8));
		(*bit_count)++;
	}
}

//! Read bits from a buffer in memory
/*! \param size		 Number of bits to read
    \param buf       Pointer to compressed bits buffer
	\param bit_count Number of bits read so far (modified)
	\param sign_extend Flag indicating to do a sign extension on the result
	\return          Value from bitstream */
int getbits(int size, unsigned char *buf, int *bit_count, int sign_extend)
{
	int i;
	int outval = 0;
	int bitcntmod8;
	int bufidx;
	int bit;
	int sign = 0;

	if(size==0)
		return(0);
	for(i=0; i<size; ++i)
	{
		bitcntmod8 = (*bit_count)%8;
		bufidx = (*bit_count)>>3;
		bit = buf[bufidx]>>(7-bitcntmod8);
		bit &= 1;
		if(i==0)
			sign = bit;
		outval = (outval<<1) | bit;
		(*bit_count)++;
	}
	if(sign_extend && sign)
	{
		int mask;
		mask = (1<<size)-1;
		outval |= (~mask);
	}
	return(outval);
}


//! Create a picture buffer (pic_t) in memory
/*! \param format	 Unused (0)
	\param color     Colorspace (RGB or YUV_SD or YUV_HD)
	\param chroma    YUV_422 or YUV_444
	\param w         Picture width
	\param h         Picture height
    \param bits      Bit depth of picuter
	\return          Pointer to pic_t structure */
void *pcreateb(int format, int color, int chroma, int w, int h, int bits)
{
    pic_t *p;    
    p = pcreate( format, color, chroma, w, h );
    p->bits = bits;
    return p;
}


//! Convert a slice's worth of data from RGB to YCoCg
/*! \param ip		 Input picture
	\param op        Output picture
	\param dsc_cfg   BDC configuration structure (specifying slice characteristics) */
void rgb2ycocg(pic_t *ip, pic_t *op, dsc_cfg_t *dsc_cfg)
{
	int i, j;
	int r, g, b;
	int y, co, cg;
	int half;

	if (ip->chroma != YUV_444)
	{
		fprintf(stderr, "ERROR: rgb2yuv() Incorrect input chroma type.\n");
		exit(1);
	}

	if (ip->color != RGB)
	{
		fprintf(stderr, "ERROR: rgb2yuv() Incorrect input color type.\n");
		exit(1);
	}

	if (ip->w != op->w || ip->h != op->h)
	{
		fprintf(stderr, "ERROR: rgb2yuv() Incorrect picture size.\n");
		exit(1);
	}

	if (op->chroma != YUV_444)
	{
		fprintf(stderr, "ERROR: rgb2yuv() Incorrect output chroma type.\n");
		exit(1);
	}

	if (op->color != YUV_SD && op->color != YUV_HD)
	{
		fprintf(stderr, "ERROR: rgb2yuv() Incorrect output color type.\n");
		exit(1);
	}

	half = 1 << (ip->bits-1);

	for (i = dsc_cfg->ystart; i < dsc_cfg->ystart + dsc_cfg->slice_height; i++)
	{
		if(i >= ip->h)
			break;
		for (j = dsc_cfg->xstart; j < dsc_cfg->xstart + dsc_cfg->slice_width; j++)
		{
			int t;
			if(j >= ip->w)
				break;
			r = ip->data.rgb.r[i][j];
			g = ip->data.rgb.g[i][j];
			b = ip->data.rgb.b[i][j];

			// *MODEL NOTE* MN_ENC_CSC
			co = r - b;
			t = b + (co>>1);
			cg = g - t;
			y = t + (cg>>1);

			op->data.yuv.y[i][j] = y;
#ifdef REDUCE_CHROMA_16BPC
			if(ip->bits == 16)
			{
				op->data.yuv.u[i][j] = MIN(0xffff, ((co+1)>>1) + half);
				op->data.yuv.v[i][j] = MIN(0xffff, ((cg+1)>>1) + half);
			}
			else 
#endif
			{
				op->data.yuv.u[i][j] = co + half*2;
				op->data.yuv.v[i][j] = cg + half*2;
			}
		}
	}
}


//! Convert a slice's worth of data from YCoCg to RGB
/*! \param ip		 Input picture
	\param op        Output picture
	\param dsc_cfg   BDC configuration structure (specifying slice characteristics) */
void ycocg2rgb(pic_t *ip, pic_t *op, dsc_cfg_t* dsc_cfg)
{
	int i, j;
	int y, co, cg, r, g, b;
	int half, max;

	if (ip->chroma != YUV_444)
	{
		fprintf(stderr, "ERROR: Incorrect input chroma type.\n");
		exit(1);
	}

	if (ip->color != YUV_SD && ip->color != YUV_HD)
	{
		fprintf(stderr, "ERROR: Incorrect input color type.\n");
		exit(1);
	}

	if (ip->w != op->w || ip->h != op->h)
	{
		fprintf(stderr, "ERROR: Incorrect picture size.\n");
		exit(1);
	}

	if (op->chroma != YUV_444)
	{
		fprintf(stderr, "ERROR: Incorrect output chroma type.\n");
		exit(1);
	}

	if (op->color != RGB)
	{
		fprintf(stderr, "ERROR: Incorrect output color type.\n");
		exit(1);
	}

	half = 1 << (ip->bits-1);
	max = (1 << ip->bits) - 1;

	for (i = dsc_cfg->ystart; i < dsc_cfg->ystart + dsc_cfg->slice_height; i++)
	{
		if(i >= ip->h)
			break;
		for (j = dsc_cfg->xstart; j < dsc_cfg->xstart + dsc_cfg->slice_width; j++)
		{
			int t;

			if(j>=ip->w)
				break;

			// *MODEL NOTE* MN_DEC_CSC
			y = ip->data.yuv.y[i][j];
#ifdef REDUCE_CHROMA_16BPC
			if(ip->bits==16)
			{
				co = (ip->data.yuv.u[i][j] - half) << 1;
				cg = (ip->data.yuv.v[i][j] - half) << 1;
			}
			else
#endif
			{
				co = ip->data.yuv.u[i][j] - half*2;
				cg = ip->data.yuv.v[i][j] - half*2;
			}

			t = y - (cg>>1);
			g = cg+t;
			b = t - (co>>1);
			r = co+b;

			op->data.rgb.r[i][j] = CLAMP(r, 0, max);
			op->data.rgb.g[i][j] = CLAMP(g, 0, max);
			op->data.rgb.b[i][j] = CLAMP(b, 0, max);
		}
	}
}


//! Simple conversion from 4:2:2 to 4:4:4
/*! \param ip		 Input picture
	\param op        Output picture */
void simple422to444(pic_t *ip, pic_t *op)
{
	int i, j;

	// *MODEL NOTE* MN_SIMPLE_422_444
	if((ip->w != op->w) || (ip->h != op->h))
	{
		fprintf(stderr, "ERROR: simple422to444() expects input and output raster sizes to match\n");
		exit(1);
	}
	if((op->color == RGB) || (ip->chroma != YUV_422) || (op->chroma != YUV_444))
	{
		fprintf(stderr, "ERROR: simple422to444() expects 4:2:2 input and 4:4:4 output\n");
		exit(1);
	}

	for(i=0; i<ip->h; ++i)
	{
		for(j=0; j<ip->w; ++j)
		{
			op->data.yuv.y[i][j] = ip->data.yuv.y[i][j];
			if((j%2) && (j<ip->w-1))
			{
				op->data.yuv.u[i][j] = (ip->data.yuv.u[i][j/2] + ip->data.yuv.u[i][j/2+1]) >> 1;
				op->data.yuv.v[i][j] = (ip->data.yuv.v[i][j/2] + ip->data.yuv.v[i][j/2+1]) >> 1;
			} else {
				op->data.yuv.u[i][j] = ip->data.yuv.u[i][j/2];
				op->data.yuv.v[i][j] = ip->data.yuv.v[i][j/2];
			}
		}
	}
}


//! Simple conversion from 4:4:4 to 4:2:2 by dropping samples
/*! \param ip		 Input picture
	\param op        Output picture */
void simple444to422(pic_t *ip, pic_t *op)
{
	int i, j;

	// *MODEL NOTE* MN_SIMPLE_444_422
	if((ip->w != op->w) || (ip->h != op->h))
	{
		fprintf(stderr, "ERROR: simple444to422() expects input and output raster sizes to match\n");
		exit(1);
	}
	if((ip->color == RGB) || (op->chroma != YUV_422) || (ip->chroma != YUV_444))
	{
		fprintf(stderr, "ERROR: simple444to422() expects 4:4:4 input and 4:2:2 output\n");
		exit(1);
	}

	for(i=0; i<ip->h; ++i)
	{
		for(j=0; j<ip->w; ++j)
		{
			op->data.yuv.y[i][j] = ip->data.yuv.y[i][j];
			if((j%2)==0)
			{
				op->data.yuv.u[i][j/2] = ip->data.yuv.u[i][j];
				op->data.yuv.v[i][j/2] = ip->data.yuv.v[i][j];
			}
		}
	}
}

void yuv_422_444_region(pic_t *p, dsc_cfg_t *dsc_cfg)
{
	printf("yuv_422_444_region not supported\n");
	exit(1);
}


//
// Chroma 444 to 422 conversion.
//
void yuv_444_422_region(pic_t *p, dsc_cfg_t *dsc_cfg)
{
	printf("yuv_444_422_region not supported\n");
	exit(1);
}


/*!
 ************************************************************************
 * \brief
 *    parse_pps() - Parse picture parameter set (PPS)
 *
 * \param buf
 *    Pointer to PPS buffer
 * \param dsc_cfg
 *    Configuration structure (output)
 *
 ************************************************************************
 */
void parse_pps(unsigned char *buf, dsc_cfg_t *dsc_cfg)
{
	int nbits = 0;
	int i;

	if(getbits(4, buf, &nbits, 0) != 1)
		UErr("PPS parser: Expected dsc_version_major=1\n");
	dsc_cfg->dsc_version_minor = getbits(4, buf, &nbits, 0);
	dsc_cfg->pps_identifier = getbits(8, buf, &nbits, 0);
	getbits(8, buf, &nbits, 0);   // Reserved (ignored)	
	dsc_cfg->bits_per_component = getbits(4, buf, &nbits, 0);
	if(dsc_cfg->bits_per_component == 0)
		dsc_cfg->bits_per_component = 16;
	dsc_cfg->linebuf_depth = getbits(4, buf, &nbits, 0);
	if(dsc_cfg->linebuf_depth == 0)
		dsc_cfg->linebuf_depth = 16;
	getbits(2, buf, &nbits, 0);   // Reserved (ignored)
	dsc_cfg->block_pred_enable = getbits(1, buf, &nbits, 0);
	dsc_cfg->convert_rgb = getbits(1, buf, &nbits, 0);
	dsc_cfg->simple_422 = getbits(1, buf, &nbits, 0);  // enable_422
	dsc_cfg->vbr_enable = getbits(1, buf, &nbits, 0);
	dsc_cfg->bits_per_pixel = getbits(10, buf, &nbits, 0);
	dsc_cfg->pic_height = getbits(16, buf, &nbits, 0);
	dsc_cfg->pic_width = getbits(16, buf, &nbits, 0);
	dsc_cfg->slice_height = getbits(16, buf, &nbits, 0);
	dsc_cfg->slice_width = getbits(16, buf, &nbits, 0);
	dsc_cfg->chunk_size = getbits(16, buf, &nbits, 0);
	getbits(6, buf, &nbits, 0);   // Reserved (ignored)
	dsc_cfg->initial_xmit_delay = getbits(10, buf, &nbits, 0);
	dsc_cfg->initial_dec_delay = getbits(16, buf, &nbits, 0);
	getbits(10, buf, &nbits, 0);   // Reserved (ignored)
	dsc_cfg->initial_scale_value = getbits(6, buf, &nbits, 0);
	dsc_cfg->scale_increment_interval = getbits(16, buf, &nbits, 0);
	getbits(4, buf, &nbits, 0);   // Reserved (ignored)
	dsc_cfg->scale_decrement_interval = getbits(12, buf, &nbits, 0);
	getbits(11, buf, &nbits, 0);   // Reserved (ignored)
	dsc_cfg->first_line_bpg_ofs = getbits(5, buf, &nbits, 0);
	dsc_cfg->nfl_bpg_offset = getbits(16, buf, &nbits, 0);
	dsc_cfg->slice_bpg_offset = getbits(16, buf, &nbits, 0);
	dsc_cfg->initial_offset = getbits(16, buf, &nbits, 0);
	dsc_cfg->final_offset = getbits(16, buf, &nbits, 0);
	getbits(3, buf, &nbits, 0);   // Reserved (ignored)
	dsc_cfg->flatness_min_qp = getbits(5, buf, &nbits, 0);
	getbits(3, buf, &nbits, 0);   // Reserved (ignored)
	dsc_cfg->flatness_max_qp = getbits(5, buf, &nbits, 0);

	// RC parameter set
	dsc_cfg->rc_model_size = getbits(16, buf, &nbits, 0);
	getbits(4, buf, &nbits, 0);   // Reserved (ignored)
	dsc_cfg->rc_edge_factor = getbits(4, buf, &nbits, 0);
	getbits(3, buf, &nbits, 0);   // Reserved (ignored)
	dsc_cfg->rc_quant_incr_limit0 = getbits(5, buf, &nbits, 0);
	getbits(3, buf, &nbits, 0);   // Reserved (ignored)
	dsc_cfg->rc_quant_incr_limit1 = getbits(5, buf, &nbits, 0);
	dsc_cfg->rc_tgt_offset_hi = getbits(4, buf, &nbits, 0);
	dsc_cfg->rc_tgt_offset_lo = getbits(4, buf, &nbits, 0);
	for(i=0; i<14; ++i)
		dsc_cfg->rc_buf_thresh[i] = getbits(8, buf, &nbits, 0) * 64;
	for(i=0; i<15; ++i)
	{
		dsc_cfg->rc_range_parameters[i].range_min_qp = getbits(5, buf, &nbits, 0);
		dsc_cfg->rc_range_parameters[i].range_max_qp = getbits(5, buf, &nbits, 0);
		dsc_cfg->rc_range_parameters[i].range_bpg_offset = getbits(6, buf, &nbits, 1);
	}

	/// DSC 1.2 bits
	if(dsc_cfg->dsc_version_minor == 2)
	{
		getbits(6, buf, &nbits, 0);   // Reserved (ignored)		
		dsc_cfg->native_420 = getbits(1, buf, &nbits, 0);
		dsc_cfg->native_422 = getbits(1, buf, &nbits, 0);

		//getbits(3, buf, &nbits, 0);    // Reserved (ignored)
		//dsc_cfg->very_flat_qp = getbits(5, buf, &nbits, 0);
		//getbits(3, buf, &nbits, 0);    // Reserved (ignored)
		//dsc_cfg->somewhat_flat_qp_thresh = getbits(5, buf, &nbits, 0);

		//getbits(3, buf, &nbits, 0);    // Reserved (ignored)
		//dsc_cfg->somewhat_flat_qp_delta = getbits(5, buf, &nbits, 0);
		getbits(3, buf, &nbits, 0);   // Reserved (ignored)
		dsc_cfg->second_line_bpg_ofs = getbits(5, buf, &nbits, 0);

		dsc_cfg->nsl_bpg_offset = getbits(16, buf, &nbits, 0);

		dsc_cfg->second_line_ofs_adj = getbits(16, buf, &nbits, 0);
	} else {		// DSC 1.1 mode
		dsc_cfg->native_420 = 0;
		dsc_cfg->native_422 = 0;
		//dsc_cfg->very_flat_qp = 1+(2*(dsc_cfg->bits_per_component-8));
		//dsc_cfg->somewhat_flat_qp_delta = 4;
		//dsc_cfg->somewhat_flat_qp_thresh = 7+(2*(dsc_cfg->bits_per_component-8));
		dsc_cfg->second_line_bpg_ofs = 0;
		dsc_cfg->nsl_bpg_offset = 0;
		dsc_cfg->second_line_ofs_adj = 0;
	}
}


/*!
 ************************************************************************
 * \brief
 *    write_pps() - Construct picture parameter set (PPS)
 *
 * \param buf
 *    Pointer to PPS buffer
 * \param dsc_cfg
 *    Configuration structure
 *
 ************************************************************************
 */
void write_pps(unsigned char *buf, dsc_cfg_t *dsc_cfg)
{
	int nbits = 0;
	int i;

	putbits(1, 4, buf, &nbits);   // dsc_version_major
	putbits(dsc_cfg->dsc_version_minor, 4, buf, &nbits);   // dsc_version_minor
	putbits(dsc_cfg->pps_identifier, 8, buf, &nbits);
	putbits(0, 8, buf, &nbits);   // reserved
	putbits(dsc_cfg->bits_per_component, 4, buf, &nbits);
	putbits(dsc_cfg->linebuf_depth, 4, buf, &nbits);
	putbits(0, 2, buf, &nbits);   // reserved
	putbits(dsc_cfg->block_pred_enable, 1, buf, &nbits);
	putbits(dsc_cfg->convert_rgb, 1, buf, &nbits);
	putbits(dsc_cfg->simple_422, 1, buf, &nbits);
	putbits(dsc_cfg->vbr_enable, 1, buf, &nbits);
	putbits(dsc_cfg->bits_per_pixel, 10, buf, &nbits);
	putbits(dsc_cfg->pic_height, 16, buf, &nbits);
	putbits(dsc_cfg->pic_width, 16, buf, &nbits);
	putbits(dsc_cfg->slice_height, 16, buf, &nbits);
	putbits(dsc_cfg->slice_width, 16, buf, &nbits);
	putbits(dsc_cfg->chunk_size, 16, buf, &nbits);
	putbits(0, 6, buf, &nbits);   // reserved
	putbits(dsc_cfg->initial_xmit_delay, 10, buf, &nbits);
	putbits(dsc_cfg->initial_dec_delay, 16, buf, &nbits);
	putbits(0, 10, buf, &nbits);   // reserved
	putbits(dsc_cfg->initial_scale_value, 6, buf, &nbits);
	putbits(dsc_cfg->scale_increment_interval, 16, buf, &nbits);
	putbits(0, 4, buf, &nbits);   // reserved
	putbits(dsc_cfg->scale_decrement_interval, 12, buf, &nbits);
	putbits(0, 11, buf, &nbits);   // reserved
	putbits(dsc_cfg->first_line_bpg_ofs, 5, buf, &nbits);
	putbits(dsc_cfg->nfl_bpg_offset, 16, buf, &nbits);
	putbits(dsc_cfg->slice_bpg_offset, 16, buf, &nbits);
	putbits(dsc_cfg->initial_offset, 16, buf, &nbits);
	putbits(dsc_cfg->final_offset, 16, buf, &nbits);
	putbits(0, 3, buf, &nbits);   // reserved
	putbits(dsc_cfg->flatness_min_qp, 5, buf, &nbits);
	putbits(0, 3, buf, &nbits);   // reserved
	putbits(dsc_cfg->flatness_max_qp, 5, buf, &nbits);

	// RC parameter set
	putbits(dsc_cfg->rc_model_size, 16, buf, &nbits);
	putbits(0, 4, buf, &nbits);   // reserved
	putbits(dsc_cfg->rc_edge_factor, 4, buf, &nbits);
	putbits(0, 3, buf, &nbits);   // reserved
	putbits(dsc_cfg->rc_quant_incr_limit0, 5, buf, &nbits);
	putbits(0, 3, buf, &nbits);   // reserved
	putbits(dsc_cfg->rc_quant_incr_limit1, 5, buf, &nbits);
	putbits(dsc_cfg->rc_tgt_offset_hi, 4, buf, &nbits);
	putbits(dsc_cfg->rc_tgt_offset_lo, 4, buf, &nbits);

	for(i=0; i<14; ++i)
		putbits(dsc_cfg->rc_buf_thresh[i]>>6, 8, buf, &nbits);
	for(i=0; i<15; ++i)
	{
		putbits(dsc_cfg->rc_range_parameters[i].range_min_qp, 5, buf, &nbits);
		putbits(dsc_cfg->rc_range_parameters[i].range_max_qp, 5, buf, &nbits);
		putbits(dsc_cfg->rc_range_parameters[i].range_bpg_offset, 6, buf, &nbits);
	}
	/// DSC 1.2 bits
	if(dsc_cfg->dsc_version_minor == 2)
	{
		putbits(0, 6, buf, &nbits);   // Reserved	
		putbits(dsc_cfg->native_420, 1, buf, &nbits);
		putbits(dsc_cfg->native_422, 1, buf, &nbits);

		//putbits(0, 3, buf, &nbits);   // reserved
		//putbits(dsc_cfg->very_flat_qp, 5, buf, &nbits);
		//putbits(0, 3, buf, &nbits);   // reserved
		//putbits(dsc_cfg->somewhat_flat_qp_thresh, 5, buf, &nbits);

		//putbits(0, 3, buf, &nbits);   // Reserved (ignored)
		//putbits(dsc_cfg->somewhat_flat_qp_delta, 5, buf, &nbits);
		putbits(0, 3, buf, &nbits);   // Reserved (ignored)
		putbits(dsc_cfg->second_line_bpg_ofs, 5, buf, &nbits);

		putbits(dsc_cfg->nsl_bpg_offset, 16, buf, &nbits);

		putbits(dsc_cfg->second_line_ofs_adj, 16, buf, &nbits);
	}
}

/*!
 ************************************************************************
 * \brief
 *    print_pps() - Display the picture parameter set (PPS)
 *
 * \param logfp
 *    File pointer for log file
 * \param dsc_cfg
 *    Configuration structure
 *
 ************************************************************************
 */
void print_pps(FILE *logfp, dsc_cfg_t *dsc_cfg)
{
	int i;

	fprintf(logfp, "=========================================\n");
    fprintf(logfp, "|  PPS Set                              |\n");
    fprintf(logfp, "-----------------------------------------\n");
    fprintf(logfp, "Version:                 %d.%d\n", 1, dsc_cfg->dsc_version_minor);
    fprintf(logfp, "pps_identifier           %5d\n", dsc_cfg->pps_identifier);
    fprintf(logfp, "bits_per_component       %5d bpc\n" , ((dsc_cfg->bits_per_component)==0)?16:dsc_cfg->bits_per_component);
    fprintf(logfp, "linebuf_depth            %5d bits\n", ((dsc_cfg->linebuf_depth)     ==0)?16:dsc_cfg->linebuf_depth);
    fprintf(logfp, "block_pred_enable        %5d\n", dsc_cfg->block_pred_enable);
    fprintf(logfp, "convert_rgb              %5d\n", dsc_cfg->convert_rgb);
    fprintf(logfp, "simple_422               %5d\n", dsc_cfg->simple_422);
    fprintf(logfp, "vbr_enable               %5d\n", dsc_cfg->vbr_enable);
    fprintf(logfp, "bits_per_pixel           %5d (%f bits)\n", dsc_cfg->bits_per_pixel, (float)(dsc_cfg->bits_per_pixel)/16.0);
    fprintf(logfp, "pic_height               %5d\n"  , dsc_cfg->pic_height);
    fprintf(logfp, "pic_width                %5d\n"  , dsc_cfg->pic_width);
    fprintf(logfp, "slice_height             %5d\n"  , dsc_cfg->slice_height);
    fprintf(logfp, "slice_width              %5d\n"  , dsc_cfg->slice_width);
    fprintf(logfp, "chunk_size               %5d\n"  , dsc_cfg->chunk_size);
    fprintf(logfp, "initial_xmit_delay       %5d\n" , dsc_cfg->initial_xmit_delay);
    fprintf(logfp, "initial_dec_delay        %5d\n"  , dsc_cfg->initial_dec_delay);
    fprintf(logfp, "initial_scale_value      %5d\n", dsc_cfg->initial_scale_value);
    fprintf(logfp, "scale_increment_interval %5d\n", dsc_cfg->scale_increment_interval);
    fprintf(logfp, "scale_decrement_interval %5d\n", dsc_cfg->scale_decrement_interval);
    fprintf(logfp, "first_line_bpg_ofs       %5d\n", dsc_cfg->first_line_bpg_ofs);
    fprintf(logfp, "nfl_bpg_offset           %5d\n", dsc_cfg->nfl_bpg_offset);
    fprintf(logfp, "slice_bpg_offset         %5d\n", dsc_cfg->slice_bpg_offset);
    fprintf(logfp, "initial_offset           %5d\n", dsc_cfg->initial_offset);
    fprintf(logfp, "final_offset             %5d\n", dsc_cfg->final_offset);
    fprintf(logfp, "flatness_min_qp          %5d\n", dsc_cfg->flatness_min_qp);
    fprintf(logfp, "flatness_max_qp          %5d\n", dsc_cfg->flatness_max_qp);
    fprintf(logfp, "\n");
    fprintf(logfp, "------------------------------------------\n");
    fprintf(logfp, "|  RC parameter set                      |\n");
    fprintf(logfp, "------------------------------------------\n");
    fprintf(logfp, "rc_model_size            %5d\n", dsc_cfg->rc_model_size);
    fprintf(logfp, "rc_edge_factor           %5d\n", dsc_cfg->rc_edge_factor);
    fprintf(logfp, "rc_quant_incr_limit0     %5d\n", dsc_cfg->rc_quant_incr_limit0);
    fprintf(logfp, "rc_quant_incr_limit1     %5d\n", dsc_cfg->rc_quant_incr_limit1);
    fprintf(logfp, "rc_tgt_offset_hi         %5d\n", dsc_cfg->rc_tgt_offset_hi);
    fprintf(logfp, "rc_tgt_offset_lo         %5d\n", dsc_cfg->rc_tgt_offset_lo);

    fprintf(logfp, "\n");
    for(i=0; i<14; ++i)
		fprintf(logfp, "dsc_cfg->rc_buf_thresh[%2d]   %5d\n", i, dsc_cfg->rc_buf_thresh[i]>>6);

    fprintf(logfp, "\n");
    for(i=0; i<15; ++i)
    {
		fprintf(logfp, "rc_range_parameters[%2d].range_min_qp     %5d\n", i, dsc_cfg->rc_range_parameters[i].range_min_qp);
		fprintf(logfp, "rc_range_parameters[%2d].range_max_qp     %5d\n", i, dsc_cfg->rc_range_parameters[i].range_max_qp);
		fprintf(logfp, "rc_range_parameters[%2d].range_bpg_offset %5d\n", i, dsc_cfg->rc_range_parameters[i].range_bpg_offset);
		fprintf(logfp, "\n");
    }

	if(dsc_cfg->dsc_version_minor == 2)
	{	
		fprintf(logfp, "native_420              %5d\n", dsc_cfg->native_420);
		fprintf(logfp, "native_422              %5d\n", dsc_cfg->native_422);
		//fprintf(logfp, "very_flat_qp            %5d\n", dsc_cfg->very_flat_qp);
		//fprintf(logfp, "somewhat_flat_qp_thresh %5d\n", dsc_cfg->somewhat_flat_qp_thresh);
		//fprintf(logfp, "somewhat_flat_qp_delta  %5d\n", dsc_cfg->somewhat_flat_qp_delta);
		fprintf(logfp, "second_line_bpg_ofs     %5d\n", dsc_cfg->second_line_bpg_ofs);
		fprintf(logfp, "nsl_bpg_offset          %5d\n", dsc_cfg->nsl_bpg_offset);
		fprintf(logfp, "second_line_ofs_adj     %5d\n", dsc_cfg->second_line_ofs_adj);
	}
    fprintf(logfp, "=========================================\n");
}


/*!
************************************************************************
* \brief
*    print_pps_v2() - Display the picture parameter set (PPS)
*
* \param logfp
*    File pointer for log file
* \param dsc_cfg
*    Configuration structure
*
************************************************************************
*/
#define PRINT_PPS_VAL(name,hex,value,nl) sprintf(val1,hex,value); sprintf(val2,"%d",value); fprintf(logfp, "%-43s %-7s // %-6s %s",name,val1,val2,nl?"\n":"");
void print_pps_v2(FILE *logfp, dsc_cfg_t *dsc_cfg)
{
	int i;
	unsigned char buf[130];
	char val1[128], val2[128], name[128];

	memset((void *)buf, 0, 128);
	write_pps(buf, dsc_cfg);

	fprintf(logfp, "// Raw PPS bytes:\n");
	fprintf(logfp, "PPS[0:127] = {");
	for (i = 0; i < 128; ++i)
	{
		if ((i & 0xf) == 0)
			fprintf(logfp, "\n\t");
		fprintf(logfp, "0x%02x", buf[i]);
		if (i < 127)
			fprintf(logfp, ", ");
	}
	fprintf(logfp, "\n};\n");

	fprintf(logfp, "// =========================================\n");
	fprintf(logfp, "// |  Annotated PPS Set by field           |\n");
	fprintf(logfp, "// -----------------------------------------\n");
	PRINT_PPS_VAL("dsc_version_major", "0x%x", 1, 1);
	PRINT_PPS_VAL("dsc_version_minor", "0x%x", dsc_cfg->dsc_version_minor, 1);
	PRINT_PPS_VAL("pps_identifier", "0x%x", dsc_cfg->pps_identifier, 1);
	PRINT_PPS_VAL("bits_per_component", "0x%x", dsc_cfg->bits_per_component & 0xf, 0);
	fprintf(logfp, "=> %d bpc\n", dsc_cfg->bits_per_component);
	PRINT_PPS_VAL("linebuf_depth", "0x%x", dsc_cfg->linebuf_depth & 0xf, 0);
	fprintf(logfp, "=> %d bits\n", dsc_cfg->linebuf_depth);
	PRINT_PPS_VAL("block_pred_enable", "0x%x", dsc_cfg->block_pred_enable, 1);
	PRINT_PPS_VAL("convert_rgb", "0x%x", dsc_cfg->convert_rgb, 1);
	PRINT_PPS_VAL("simple_422", "0x%x", dsc_cfg->simple_422, 1);
	PRINT_PPS_VAL("vbr_enable", "0x%x", dsc_cfg->vbr_enable, 1);
	PRINT_PPS_VAL("bits_per_pixel", "0x%03x", dsc_cfg->bits_per_pixel, 0);
	if (dsc_cfg->native_420 || dsc_cfg->native_422)
		fprintf(logfp, "=> %f bpp\n",  (float)(dsc_cfg->bits_per_pixel) / 32.0);
	else
		fprintf(logfp, "=> %f bpp\n", (float)(dsc_cfg->bits_per_pixel) / 16.0);
	PRINT_PPS_VAL("pic_height", "0x%04x", dsc_cfg->pic_height, 1);
	PRINT_PPS_VAL("pic_width", "0x%04x", dsc_cfg->pic_width, 1);
	PRINT_PPS_VAL("slice_height", "0x%04x", dsc_cfg->slice_height, 1);
	PRINT_PPS_VAL("slice_width", "0x%04x", dsc_cfg->slice_width, 1);
	PRINT_PPS_VAL("chunk_size", "0x%04x", dsc_cfg->chunk_size, 1);
	PRINT_PPS_VAL("initial_xmit_delay", "0x%03x", dsc_cfg->initial_xmit_delay, 1);

	PRINT_PPS_VAL("initial_dec_delay", "0x%04x", dsc_cfg->initial_dec_delay, 1);
	PRINT_PPS_VAL("initial_scale_value", "0x%02x", dsc_cfg->initial_scale_value, 0);
	fprintf(logfp, "=> %f\n", dsc_cfg->initial_scale_value / 8.0);
	PRINT_PPS_VAL("scale_increment_interval", "0x%04x", dsc_cfg->scale_increment_interval, 1);
	PRINT_PPS_VAL("scale_decrement_interval", "0x%03x", dsc_cfg->scale_decrement_interval, 1);
	PRINT_PPS_VAL("first_line_bpg_ofs", "0x%02x", dsc_cfg->first_line_bpg_ofs, 1);
	PRINT_PPS_VAL("nfl_bpg_offset", "0x%04x", dsc_cfg->nfl_bpg_offset, 0);
	fprintf(logfp, "=> %f\n", dsc_cfg->nfl_bpg_offset / 2048.0);
	PRINT_PPS_VAL("slice_bpg_offset", "0x%04x", dsc_cfg->slice_bpg_offset, 0);
	fprintf(logfp, "=> %f\n", dsc_cfg->slice_bpg_offset/2048.0);
	PRINT_PPS_VAL("initial_offset", "0x%04x", dsc_cfg->initial_offset, 1);
	PRINT_PPS_VAL("final_offset", "0x%04x", dsc_cfg->final_offset, 1);
	PRINT_PPS_VAL("flatness_min_qp", "0x%02x", dsc_cfg->flatness_min_qp, 1);
	PRINT_PPS_VAL("flatness_max_qp", "0x%02x", dsc_cfg->flatness_max_qp, 1);
	fprintf(logfp, "\n");
	fprintf(logfp, "// ------------------------------------------\n");
	fprintf(logfp, "// |  RC parameter set                      |\n");
	fprintf(logfp, "// ------------------------------------------\n");
	PRINT_PPS_VAL("rc_model_size", "0x%04x", dsc_cfg->rc_model_size, 1);
	PRINT_PPS_VAL("rc_edge_factor", "0x%x", dsc_cfg->rc_edge_factor, 0);
	fprintf(logfp, "=> %f\n", dsc_cfg->rc_edge_factor/2.0);
	PRINT_PPS_VAL("rc_quant_incr_limit0", "0x%02x", dsc_cfg->rc_quant_incr_limit0, 1);
	PRINT_PPS_VAL("rc_quant_incr_limit1", "0x%02x", dsc_cfg->rc_quant_incr_limit1, 1);
	PRINT_PPS_VAL("rc_tgt_offset_hi", "0x%x", dsc_cfg->rc_tgt_offset_hi, 1);
	PRINT_PPS_VAL("rc_tgt_offset_lo", "0x%x", dsc_cfg->rc_tgt_offset_lo, 1);

	fprintf(logfp, "\n");
	for (i = 0; i < 14; ++i)
	{
		sprintf(name, "rc_buf_thresh[%d]", i);
		PRINT_PPS_VAL(name, "0x%02x", (dsc_cfg->rc_buf_thresh[i] >> 6), 0);
		fprintf(logfp, "=> %d\n",  dsc_cfg->rc_buf_thresh[i]);
	}

	fprintf(logfp, "\n");
	for (i = 0; i<15; ++i)
	{
		sprintf(name, "rc_range_parameters[%d].range_min_qp", i);
		PRINT_PPS_VAL(name, "0x%02x", dsc_cfg->rc_range_parameters[i].range_min_qp, 1);
		sprintf(name, "rc_range_parameters[%d].range_max_qp", i);
		PRINT_PPS_VAL(name, "0x%02x", dsc_cfg->rc_range_parameters[i].range_max_qp, 1);
		sprintf(name, "rc_range_parameters[%d].range_bpg_offset", i); 
		PRINT_PPS_VAL(name, "0x%02x", dsc_cfg->rc_range_parameters[i].range_bpg_offset & 0x3f, 0);
		fprintf(logfp, "=> %d\n", dsc_cfg->rc_range_parameters[i].range_bpg_offset);
		fprintf(logfp, "\n");
	}
	fprintf(logfp, "// =========================================\n");

	if (dsc_cfg->dsc_version_minor == 2)
	{
		PRINT_PPS_VAL("native_420", "0x%x", dsc_cfg->native_420, 1);
		PRINT_PPS_VAL("native_422", "0x%x", dsc_cfg->native_422, 1);
		PRINT_PPS_VAL("second_line_bpg_ofs", "0x%02x", dsc_cfg->second_line_bpg_ofs, 1);
		PRINT_PPS_VAL("nsl_bpg_offset", "0x%04x", dsc_cfg->nsl_bpg_offset, 0);
		fprintf(logfp, "=> %f\n", dsc_cfg->nsl_bpg_offset/2048.0);
		PRINT_PPS_VAL("second_line_ofs_adj", "0x%04x", dsc_cfg->second_line_ofs_adj, 1);
	}
}
