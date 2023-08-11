/***************************************************************************
*    Copyright (c) 2013-2017, Broadcom Ltd.
*    All rights reserved.
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

/*! \file codec_main.c
 *    Main codec loop
 *  \author Frederick Walls (fwalls@broadcom.com) */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "vdo.h"
#include "dsc_types.h"
#include "dsc_utils.h"
#include "dpx.h"
#include "utl.h"
#include "psnr.h"
#include "cmd_parse.h"
#include "dsc_codec.h"
#include "logging.h"
#include "rc_tables.h"

#define PATH_MAX 1024
#define MAX_OPTNAME_LEN 200
#define CFGLINE_LEN ((PATH_MAX) + MAX_OPTNAME_LEN)
#define RANGE_CHECK(s,a,b,c) { if(((a)<(b))||((a)>(c))) { UErr("%s out of range, needs to be between %d and %d\n",s,b,c); } }

static int assign_line (char* line, cmdarg_t *cmdargs);


static int      help = 0;
static char fn_i[PATH_MAX+1] = "";
static char fn_o[PATH_MAX+1] = "";
static char fn_log[PATH_MAX+1] = "";
static char filepath[PATH_MAX+1] = "";
static char option[PATH_MAX+1]   = "";
static int rcModelSize;
static float bitsPerPixel;
static int bitsPerComponent;
static int enable422;
static int simple422;
static int native422;
static int native420;
static int lineBufferBpc;
static int bpEnable;
static int initialDelay;
static int sliceWidth;
static int sliceHeight;
static int firstLineBpgOfs;
static int secondLineBpgOfs;
static int secondLineOfsAdj;
static int initialFullnessOfs;
static int useYuvInput;
static int function;
static int *rcOffset;
static int *rcMinQp;
static int *rcMaxQp;
static int *rcBufThresh;
static int tgtOffsetHi;
static int tgtOffsetLo;
static int rcEdgeFactor;
static int quantIncrLimit0;
static int quantIncrLimit1;
static int rbSwap;
static int rbSwapOut;
static int flatnessMinQp;
static int flatnessMaxQp;
static int flatnessDetThresh;
static int enableVbr;
static int fullIchErrPrecision;
static int muxWordSize;
static int picWidth;
static int picHeight;
static int dscVersionMinor;
static int dpxRForceBe;
static int dpxRDatumOrder;
static int dpxRPadEnds;
static int dpxWPadEnds;
static int dpxWDatumOrder;
static int dpxWForcePacking;
static int dpxWByteSwap;
static int ppmFileOutput;
static int dpxFileOutput;
static int yuvFileOutput;
static int yuvFileFormat;
static int printPps;
static int printPpsFormat;
static int autoSliceHeightAlgorithm;
static int generateRcParameters;

static  cmdarg_t cmd_args[] = {

	// The array arguments have to be first:
	{ IVARG, NULL,    "RC_OFFSET",            "-rcofs",  0,  15},  // RC offset values
	{ IVARG, NULL,     "RC_MINQP",             "-rcmqp",  0,  15},  // Min QP values
	{ IVARG, NULL,     "RC_MAXQP",             "-rcmxqp", 0,  15},  // Max QP values
	{ IVARG,  NULL, "RC_BUF_THRESH",       "-rcbt",   0,   14},  // RC buffer threshold

	{ PARG,  &rcModelSize,        "RC_MODEL_SIZE",        "-rms",   0,  0},  // RC model size
	{ FARG,  &bitsPerPixel,		  "BITS_PER_PIXEL",		  "-bpp",   0,  0},  // bits per pixel
	{ PARG,  &bitsPerComponent,   "BITS_PER_COMPONENT",   "-bpc",   0,  0},  // bits per component
	{ PARG,  &enable422,          "ENABLE_422",           "-e422",  0,  0},  // enable_422/simple_422
	{ PARG,  &simple422,          "SIMPLE_422",           "-s422",  0,  0},  // enable_422/simple_422
	{ PARG,  &native422,          "NATIVE_422",           "-n422",  0,  0},  // native 422
	{ PARG,  &native420,          "NATIVE_420",           "-n420",  0,  0},  // native 420
	{ PARG,  &bpEnable,           "BLOCK_PRED_ENABLE",    "-bpe",   0,  0},  // Block prediction range
	{ PARG,  &lineBufferBpc,      "LINE_BUFFER_BPC",      "-lbpc",  0,  0},  // Line buffer storage bits/component
	{ PARG,  &sliceWidth,         "SLICE_WIDTH",         "-nsh",   0,  0},  // Slice width (0=pic width)
	{ PARG,  &sliceHeight,        "SLICE_HEIGHT",         "-nsv",   0,  0},  // slice height (0=pic height)
	{ PARG,  &firstLineBpgOfs,    "FIRST_LINE_BPG_OFFSET", "-flbo", 0,  0},  // Additional bpp budget for 1st line
	{ PARG,  &secondLineBpgOfs,   "SECOND_LINE_BPG_OFFSET", "-slbo", 0,  0},  // Additional bpp budget for 2nd line (420 only)
	{ PARG,  &secondLineOfsAdj,   "SECOND_LINE_OFFSET_ADJ", "-sloa", 0,  0},  // 2nd line offset adjustment (420 only)
	{ PARG,  &initialFullnessOfs, "INITIAL_FULLNESS_OFFSET", "-ifo", 0, 0},  // Initial fullness offset
	{ PARG,  &initialDelay,       "INITIAL_DELAY",        "-id",    0,  0},  // Initial delay (in pixel time units) from encode start to xmit start
	{ PARG,  &useYuvInput,        "USE_YUV_INPUT",        "-uyi",   0,  0},   // Use YUV input (convert if necessary)
	{ PARG,  &rbSwap,             "SWAP_R_AND_B",         "-rbswp", 0,  0},  // Swap red & blue components
	{ PARG,  &rbSwapOut,          "SWAP_R_AND_B_OUT",     "-rbswpo", 0,  0},  // Swap red & blue components
	{ PARG,  &function,           "FUNCTION",             "-do",    0,  0},   // 0=encode/decode, 1=encode, 2=decode
	{ PARG,  &dpxRForceBe,        "DPXR_FORCE_BE",        "-dpxrfbe", 0, 0},  // Force big-endian DPX read
	{ PARG,  &dpxRPadEnds,        "DPXR_PAD_ENDS",        "-dpxrpad",  0, 0},  // Pad line ends for DPX input
	{ PARG,  &dpxRDatumOrder,     "DPXR_DATUM_ORDER",     "-dpxrdo",  0, 0},   // DPX input datum order
	{ PARG,  &dpxWPadEnds,        "DPXW_PAD_ENDS",        "-dpxwpad",  0, 0},  // Pad line ends for DPX writing
	{ PARG,  &dpxWDatumOrder,     "DPXW_DATUM_ORDER",     "-dpxwdo",  0, 0},   // DPX output datum order
	{ PARG,  &dpxWForcePacking,   "DPXW_FORCE_PACKING",   "-dpxwfp",  0, 0},   // DPX output force packing method
	{ PARG,  &dpxWByteSwap,       "DPXW_BYTE_SWAP",       "-dpxwbs",  0, 0},   // DPX output write in LE order
	{ PARG,  &enableVbr,          "VBR_ENABLE",           "-vbr",  0,  0},    // 1=disable stuffing bits (on/off VBR)
	{ PARG,  &picWidth,           "PIC_WIDTH",            "-pw",  0,  0},    // picture width for YUV input
	{ PARG,  &picHeight,          "PIC_HEIGHT",           "-ph",  0,  0},    // picture height for YUV input
	{ PARG,  &ppmFileOutput,      "PPM_FILE_OUTPUT",      "-ppm",  0,  0},    // output PPM files
	{ PARG,  &dpxFileOutput,      "DPX_FILE_OUTPUT",      "-dpx",  0,  0},    // output DPX files
	{ PARG,  &yuvFileOutput,      "YUV_FILE_OUTPUT",      "-yuvo", 0,  0},   // Enable YUV file output for 4:2:0
	{ PARG,  &yuvFileFormat,      "YUV_FILE_FORMAT",      "-yuvff", 0,  0},   // YUV file format (0=planar 420, 1=UYVY)

	{ PARG,  &tgtOffsetHi,		  "RC_TGT_OFFSET_HI",     "-thi",   0,  0},   // Target hi
	{ PARG,  &tgtOffsetLo,		  "RC_TGT_OFFSET_LO",     "-tlo",   0,  0},   // Target lo
	{ PARG,  &rcEdgeFactor,       "RC_EDGE_FACTOR",       "-ef",    0,  0},   // Edge factor
	{ PARG,  &quantIncrLimit0,    "RC_QUANT_INCR_LIMIT0", "-qli0",  0,  0},   // Quant limit incr 0
	{ PARG,  &quantIncrLimit1,    "RC_QUANT_INCR_LIMIT1", "-qli1",  0,  0},   // Quant limit incr 1
	{ PARG,  &flatnessMinQp,      "FLATNESS_MIN_QP",       "-fmin", 0,  0},   // Flatness min QP
	{ PARG,  &flatnessMaxQp,      "FLATNESS_MAX_QP",       "-fmax", 0,  0},   // Flatness max QP
	{ PARG,  &flatnessDetThresh,  "FLATNESS_DET_THRESH",  "-fdt",  0,  0},   // Flatness detect threshold
	{ PARG,  &dscVersionMinor,    "DSC_VERSION_MINOR", "-dvm", 0, 0},      // DSC minor version
	{ PARG,  &fullIchErrPrecision, "FULL_ICH_ERR_PRECISION", "-fiep",  0,  0},   // Use full error precision for ICH selection

	{ PARG,  &autoSliceHeightAlgorithm, "AUTO_SLICE_HEIGHT_ALGORITHM", "-asha",  0,  0 }, // Algorithm to auto-compute slice height
	{ PARG,  &printPpsFormat,     "PRINT_PPS_FORMAT",     "-ppf",    0,  0 }, // Format to print PPS in
	{ PARG,  &generateRcParameters, "GENERATE_RC_PARAMETERS", "-grc",    0,  0 }, // Generate RC parameters automatically

	 {NARG,  &printPps,           "",                     "-P"      , 0, 0}, // print out PPS
	 {NARG,  &help,               "",                     "-help"   , 0, 0}, // video format
     {SARG,   filepath,           "INCLUDE",              "-F"      , 0, 0}, // Cconfig file
     {SARG,   option,             "",                     "-O"      , 0, 0}, // key/value pair
     {SARG,   fn_i,               "SRC_LIST",              ""        , 0, 0}, // Input file name
     {SARG,   fn_o,               "OUT_DIR",              ""        , 0, 0}, // Output file name
     {SARG,   fn_log,             "LOG_FILENAME",         ""        , 0, 0}, // Log file name

     {PARG,   NULL,               "",                     "",       0, 0 }
};

/*!
 ************************************************************************
 * \brief
 *    set_defaults() - Set default configuration values
 ************************************************************************
 */
void set_defaults(void)
{
	// Default is for 8bpc/8bpp
	int default_rcofs[] = { 2, 0, 0, -2, -4, -6, -8, -8, -8, -10, -10, -12, -12, -12, -12 };
	int default_minqp[] = { 0, 0, 1, 1, 3, 3, 3, 3, 3, 3, 5, 5, 5, 7, 13 };
	int default_maxqp[] = { 4, 4, 5, 6, 7, 7, 7, 8, 9, 10, 11, 12, 13, 13, 15 };
	int default_threshold[] = { 896, 1792, 2688, 3584, 4480, 5376, 6272, 6720, 7168, 7616, 7744, 7872, 8000, 8064 };
	int i;

	rcModelSize = 8192;
	bitsPerPixel = 8.0;
	bitsPerComponent = 8;
	enable422 = 0;
	simple422 = 0;
	bpEnable = 1;
	initialFullnessOfs = 6144;
	initialDelay = 170;
	firstLineBpgOfs = -1;
	sliceWidth = 0;
	sliceHeight = 0;
	useYuvInput = 0;
	rbSwap = 0;
	rbSwapOut = 0;
	function = 0;
	lineBufferBpc = 9;
	ppmFileOutput = 0;
	yuvFileOutput = 0;
	dpxFileOutput = 1;
	yuvFileFormat = 0;

	// Standards compliant DPX settings:
	dpxRForceBe = 0;
	dpxRDatumOrder = 0;
	dpxRPadEnds = 0;
	dpxWPadEnds = 0;
	dpxWDatumOrder = 0;
	dpxWForcePacking = 1;   // Method B is non-standard per DPX spec
	enableVbr = 0;
	native420 = 0;
	native422 = 0;
	secondLineBpgOfs = -1;
	secondLineOfsAdj = 0;

	dscVersionMinor = 2;

	tgtOffsetHi = 3;
	tgtOffsetLo = 3;
	rcEdgeFactor = 6;
	quantIncrLimit0 = 11;
	quantIncrLimit1 = 11;
	flatnessMinQp = 3;
	flatnessMaxQp = 12;
	flatnessDetThresh = 2;
	muxWordSize = 0;
	fullIchErrPrecision = 0;
	picWidth = 1920;  // Default for YUV input
	picHeight = 1080;  
	printPps = 0;
	printPpsFormat = 1;
	autoSliceHeightAlgorithm = 0;
	generateRcParameters = 0;
	for (i=0; i<15; ++i)
	{
		rcOffset[i] = default_rcofs[i];
		rcMinQp[i] = default_minqp[i];
		rcMaxQp[i] = default_maxqp[i];
		if (i<14)
			rcBufThresh[i] = default_threshold[i];
	}

	strcpy(fn_o, ".");
	strcpy(fn_log, "log.txt");
}

/*!
 ************************************************************************
 * \brief
 *    parse_cfgfile() - loop over a file's lines and process
 *
 * \param fn
 *    Config file filename
 * \param cmdargs
 *    Command argument structure
 *
 ************************************************************************
 */
static int parse_cfgfile (char* fn, cmdarg_t *cmdargs)
{
	FILE *fd;
    char line[CFGLINE_LEN+1] = "";

    if (NULL == fn)
    {
        Err("%s called with NULL file name", __FUNCTION__);
    }
    if (0 == fn[0])
    {
        Err("empty configuration file name");
    }
    fd = fopen(fn, "r");
    if (NULL == fd)
    {
        PErr("cannot open file '%s'", fn);
    }
    fn[0] = '\0';
    while (NULL != fgets(line, CFGLINE_LEN, fd)) // we re-use the storage
    {
        assign_line(line, cmdargs);    
    }
    if (!feof(fd))
    {
        PErr("while reading from file '%s'", fn);
    }
    fclose(fd);
    return 0;
}


/*!
 ************************************************************************
 * \brief
 *    assign_line() - process a single line
 *
 * \param line
 *    String containing line
 * \param cmdargs
 *    Command argument structure
 *
 ************************************************************************
 */
static int assign_line (char* line, cmdarg_t *cmdargs)
{
    if (!parse_line (line, cmdargs))
    {
        UErr("unknown configuration field '%s'", line);
    }
    if ('\0' != filepath[0])   // config file
    {
        parse_cfgfile(filepath, cmdargs);
    }
    return 0;
}

/*!
 ************************************************************************
 * \brief
 *    usage() - Print the usage message
 ************************************************************************
 */
void usage(void)
{
	printf("Usage: DSC <options>\n");
	printf(" Option list:\n");
	printf("  -help => print this message\n");
	printf("  -F <cfg_file> => Specify configuration file (required)\n");
	printf("  -O\"PARAMETER <value>\" => override config file parameter PARAMETER with new value\n");
	printf("  See README.txt for a list of parameters.\n");
	exit(1);
}

/*!
 ************************************************************************
 * \brief
 *    process_args() - set defaults and process command line
 *
 * \param argc
 *    Argument count (from main)
 * \param argv
 *    Arguments (from main)
 * \param cmddargs
 *    Command arguemnts structure
 *
 ************************************************************************
 */
static int process_args(int argc, char *argv[], cmdarg_t *cmdargs)
{
    enum arg_order_e
    {
        IN_ARG, OUT_ARG, DONE
    };
    int expected_arg = IN_ARG; // track if we've seen these args
    int incr;
	int i;
	char *arg1;

    if (1 == argc) // no arguments
    {
        usage();
    }

    /* process each argument */
    for (i=1; i<argc; i+=incr)
    {
        arg1 = (i<argc-1)? argv[i+1]: NULL;
        incr = parse_cmd(argv[i], arg1, cmdargs);

        if (incr)
        {
            if (help) // help
            {
                usage();
            }
            if (0 != filepath[0])   // config file
            {
                parse_cfgfile(filepath, cmdargs);
            }
            else if (0 != option[0]) // config line
            {
                assign_line(option, cmdargs);
                option[0] = '\0';
            }
            continue;
        }
        if (argv[i][0]=='-')
        {
            fprintf(stderr, "ERROR: Unknown option '%s'!\n", argv[i]);
            usage();
        }
        incr = 1;
        switch (expected_arg)
        {
        case IN_ARG:  // input file name */
            expected_arg = OUT_ARG;
            strcpy(fn_i, argv[i]);
            break;
        case OUT_ARG:  // output file name */
            expected_arg = DONE;
            strcpy(fn_o, argv[i]);
            break;
        default:  /* input file name */
            fprintf(stderr, "ERROR: Unexpected argument %s (in=%s, out=%s)", argv[i], fn_i, fn_o);
            usage();
        }
    }

    return 0;
}


/*!
 ************************************************************************
 * \brief
 *    split_base_and_ext() - separate base file name and extension
 *
 * \param infname
 *    Filename with (optional) path and extension
 * \param base_name
 *    Allocated string to copy base filename to
 * \param extension
 *    Returns pointer to extension (uses part of base_name storage)
 *
 ************************************************************************
 */
void split_base_and_ext(char *infname, char *base_name, char **extension)
{
	int zz;
	int dot=-1, slash=-1;

	// Find last . and / (or \) in filename
	for (zz = strlen(infname)-1; zz >= 0; --zz)
	{
		if ((slash<0) && ((infname[zz] == '\\') || (infname[zz] == '/')))
			slash = zz;
		if ((dot<0) && (infname[zz] == '.'))
			dot = zz;
	}
	if ((dot < slash) || (dot < 0))
	{
		printf("ERROR: picture format unrecognized\n");
		exit(1);
	}
	strcpy(base_name, &(infname[slash+1]));
	*extension = &(base_name[dot-slash]);
	base_name[dot-slash-1] = '\0';  // terminate base_name string
}


/*!
 ************************************************************************
 * \brief
 *    write_dsc_data() - Write a .DSC formatted file
 *
 * \param bit_buffer
 *    Pointer to bitstream buffer
 * \param nbytes
 *    Number of bytes to write
 * \param fp
 *    File handle
 * \param vbr_enable
 *    VBR enable flag
 *
 ************************************************************************
 */
void write_dsc_data(unsigned char **bit_buffer, int nbytes, FILE *fp, int vbr_enable, int slices_per_line, int slice_height, int **sizes)
{
	int i;
	int slice_x, slice_y;
	int *current_idx;

	current_idx = (int *)malloc(sizeof(int) * slices_per_line);
	for (i=0; i<slices_per_line; ++i)
		current_idx[i] = 0;

	for (slice_y = 0; slice_y < slice_height; ++slice_y)
	{
		for (slice_x = 0; slice_x < slices_per_line; ++slice_x)
		{
			if (vbr_enable)
			{
				nbytes = sizes[slice_x][slice_y];
				fputc((nbytes>>8) & 0xff, fp);
				fputc(nbytes & 0xff, fp);
			}
			for (i=0; i<nbytes; ++i)
				fputc(bit_buffer[slice_x][current_idx[slice_x] + i], fp);
			current_idx[slice_x] += nbytes;
		}
	}
	free(current_idx);
}


/*!
 ************************************************************************
 * \brief
 *    read_dsc_data() - Read a .DSC formatted file
 *
 * \param bit_buffer
 *    Pointer to bitstream buffer
 * \param nbytes
 *    Number of bytes to write (if CBR mode)
 * \param fp
 *    File handle
 * \param vbr_enable
 *    VBR enable flag
 * \return
 *    Number of bytes read
 *
 ************************************************************************
 */
int read_dsc_data(unsigned char **bit_buffer, int nbytes, FILE *fp, int vbr_enable, int slices_per_line, int slice_height)
{
	int i, slice_y, slice_x;
	int *current_idx, total_bytes = 0;

	current_idx = (int *)malloc(sizeof(int) * slices_per_line);
	for (i=0; i<slices_per_line; ++i)
		current_idx[i] = 0;

	for (slice_y = 0; slice_y < slice_height; ++slice_y)
	{
		for (slice_x = 0; slice_x < slices_per_line; ++slice_x)
		{
			if (vbr_enable)
			{
				nbytes = 0;
				for(i=0; i<2; ++i)
					nbytes = (nbytes<<8) | (fgetc(fp) & 0xff);
			}
			for(i=0; i<nbytes; ++i)
				bit_buffer[slice_x][i+current_idx[slice_x]] = fgetc(fp) & 0xff;
			current_idx[slice_x] += nbytes;
			total_bytes += nbytes;
		}
	}

	free(current_idx);
	return(total_bytes);
}


/*!
 ************************************************************************
 * \brief
 *    compute_offset() - Compute offset value at a specific group
 *
 * \param dsc_cfg
 *    Pointer to DSC configuration structure
 * \param pixelsPerGroup
 *    Number of pixels per group
 * \param groupsPerLine
 *    Number of groups per line
 * \param grpcnt
 *    Group to compute offset for
 * \return
 *    Offset value for the group grpcnt
 ************************************************************************
 */
int compute_offset(dsc_cfg_t *dsc_cfg, int pixelsPerGroup, int groupsPerLine, int grpcnt)
{
	int offset = 0;
	int grpcnt_id;

	grpcnt_id = (int)ceil((float)initialDelay / pixelsPerGroup); 
	if(grpcnt <= grpcnt_id)
		offset = (int)ceil(grpcnt * pixelsPerGroup * bitsPerPixel);
	else
		offset = (int)ceil(grpcnt_id * pixelsPerGroup * bitsPerPixel) - (((grpcnt-grpcnt_id) * dsc_cfg->slice_bpg_offset)>>OFFSET_FRACTIONAL_BITS);

	if(grpcnt <= groupsPerLine)
		offset += grpcnt * firstLineBpgOfs;
	else
		offset += groupsPerLine * firstLineBpgOfs - (((grpcnt - groupsPerLine) * dsc_cfg->nfl_bpg_offset)>>OFFSET_FRACTIONAL_BITS);
	if(native420)
	{
		if(grpcnt <= groupsPerLine)
			offset -= (grpcnt * dsc_cfg->nsl_bpg_offset) >> OFFSET_FRACTIONAL_BITS;
		else if(grpcnt <= 2*groupsPerLine)
			offset += (grpcnt - groupsPerLine) * secondLineBpgOfs - ((groupsPerLine * dsc_cfg->nsl_bpg_offset)>>OFFSET_FRACTIONAL_BITS);
		else
			offset += (grpcnt - groupsPerLine) * secondLineBpgOfs - (((grpcnt - groupsPerLine) * dsc_cfg->nsl_bpg_offset)>>OFFSET_FRACTIONAL_BITS);
	}
	return(offset);
}


/*!
 ************************************************************************
 * \brief
 *    compute_rc_parameters() - Compute rate control parameters
 *
 * \param dsc_cfg
 *    Pointer to DSC configuration structure
 * \param pixelsPerGroup
 *    Number of pixels per group
 * \param groupsPerLine
 *    Number of groups per line
 * \param grpcnt
 *    Group to compute offset for
 * \return
 *    0 = success; 1 = error with configuration
 ************************************************************************
 */
int compute_rc_parameters(dsc_cfg_t *dsc_cfg, int pixelsPerGroup, int numSsps)
{
	int groupsPerLine;
	int num_extra_mux_bits;
	int sliceBits;
	int final_value;
	int final_scale;
	int invalid = 0;
	int groups_total;
	int maxOffset;
	int rbsMin;
	int hrdDelay;
	int slicew;
	int uncompressedBpgRate;

	slicew = dsc_cfg->slice_width >> (native420 || native422);  // /2 in 4:2:0 mode
	if (native422)
		uncompressedBpgRate = 3 * bitsPerComponent * 4;
	else
		uncompressedBpgRate = (3 * bitsPerComponent + (useYuvInput ? 0 : 2)) * 3;

	if (firstLineBpgOfs < 0)
	{
		if (dsc_cfg->slice_height >= 8)
			firstLineBpgOfs = 12 + ((int)(0.09 * MIN(34, dsc_cfg->slice_height - 8)));
		else
			firstLineBpgOfs = 2 * (dsc_cfg->slice_height - 1);
		firstLineBpgOfs = CLAMP(firstLineBpgOfs, 0, (int)(uncompressedBpgRate - 3 * bitsPerPixel));
	}
	if (secondLineBpgOfs < 0)
	{
		secondLineBpgOfs = native420 ? 12 : 0;
		secondLineBpgOfs = CLAMP(secondLineBpgOfs, 0, (int)(uncompressedBpgRate - 3 * bitsPerPixel));
	}
	dsc_cfg->first_line_bpg_ofs = firstLineBpgOfs;
	RANGE_CHECK("first_line_bpg_offset", dsc_cfg->first_line_bpg_ofs, 0, 31);
	dsc_cfg->second_line_bpg_ofs = secondLineBpgOfs;
	RANGE_CHECK("second_line_bpg_offset", dsc_cfg->second_line_bpg_ofs, 0, 31);

	groupsPerLine = (slicew + pixelsPerGroup-1) / pixelsPerGroup;
	dsc_cfg->chunk_size = (int)(ceil(slicew * bitsPerPixel / 8.0)); // Number of bytes per chunk
	RANGE_CHECK("chunk_size", dsc_cfg->chunk_size, 0, 65535);

	if (dsc_cfg->convert_rgb)
		num_extra_mux_bits = (numSsps*(muxWordSize + (4*bitsPerComponent+4)-2));
	else if (!dsc_cfg->native_422) // YCbCr
		num_extra_mux_bits = (numSsps*muxWordSize + (4*bitsPerComponent+4) + 2*(4*bitsPerComponent) - 2);
	else
		num_extra_mux_bits = (numSsps*muxWordSize + (4*bitsPerComponent+4) + 3*(4*bitsPerComponent) - 2);
	sliceBits = 8 * dsc_cfg->chunk_size * dsc_cfg->slice_height;
	while ((num_extra_mux_bits>0) && ((sliceBits - num_extra_mux_bits) % muxWordSize))
		num_extra_mux_bits--;

	// The following line was added in 1.57e along with RANGE_CHECK()
	dsc_cfg->initial_scale_value = 8 * dsc_cfg->rc_model_size / (dsc_cfg->rc_model_size - dsc_cfg->initial_offset);
	if (groupsPerLine < dsc_cfg->initial_scale_value - 8)
		dsc_cfg->initial_scale_value = groupsPerLine + 8;
	RANGE_CHECK("initial_scale_value", dsc_cfg->initial_scale_value, 0, 63);

	if (dsc_cfg->initial_scale_value > 8)
		dsc_cfg->scale_decrement_interval = groupsPerLine / (dsc_cfg->initial_scale_value - 8);
	else
		dsc_cfg->scale_decrement_interval = 4095;
	RANGE_CHECK("scale_decrement_interval", dsc_cfg->scale_decrement_interval, 0, 4095);
	final_value = dsc_cfg->rc_model_size - ((dsc_cfg->initial_xmit_delay * dsc_cfg->bits_per_pixel + 8)>>4) + num_extra_mux_bits;
	dsc_cfg->final_offset = final_value;
	RANGE_CHECK("final_offset", dsc_cfg->final_offset, 0, 65535);
	if (final_value >= dsc_cfg->rc_model_size)
		UErr("The final_offset must be less than the rc_model_size.  Try increasing initial_xmit_delay.\n");
	final_scale = 8 * dsc_cfg->rc_model_size / (dsc_cfg->rc_model_size - final_value);
	if (final_scale > 63)
		printf("WARNING: A final scale value > than 63/8 may have undefined behavior on some implementations.  Try increasing initial_xmit_delay.\n");
	if(dsc_cfg->slice_height > 1)
		dsc_cfg->nfl_bpg_offset = (int)ceil((double)(dsc_cfg->first_line_bpg_ofs << OFFSET_FRACTIONAL_BITS) / (dsc_cfg->slice_height - 1));
	else
		dsc_cfg->nfl_bpg_offset = 0;
	if (dsc_cfg->nfl_bpg_offset > 65535)
	{
		printf("nfl_bpg_offset is too large for this slice height\n");
		invalid = 1;
	}
	if(dsc_cfg->slice_height > 2)
		dsc_cfg->nsl_bpg_offset = (int)ceil((double)(dsc_cfg->second_line_bpg_ofs << OFFSET_FRACTIONAL_BITS) / (dsc_cfg->slice_height - 1));
	else
		dsc_cfg->nsl_bpg_offset = 0;
	if (dsc_cfg->nsl_bpg_offset > 65535)
	{
		printf("nsl_bpg_offset is too large for this slice height\n");
		invalid = 1;
	}
	groups_total = groupsPerLine * dsc_cfg->slice_height;
	dsc_cfg->slice_bpg_offset = (int)ceil((double)(1<<OFFSET_FRACTIONAL_BITS) * 
				(dsc_cfg->rc_model_size - dsc_cfg->initial_offset + num_extra_mux_bits)
				/ (groups_total));
	RANGE_CHECK("slice_bpg_offset", dsc_cfg->slice_bpg_offset, 0, 65535);

	if(dsc_cfg->slice_height == 1)
	{
		if(dsc_cfg->first_line_bpg_ofs > 0)
			UErr("For slice_height == 1, the FIRST_LINE_BPG_OFFSET must be 0\n");
	} else if(pixelsPerGroup * bitsPerPixel - 
			((double)(dsc_cfg->slice_bpg_offset + dsc_cfg->nfl_bpg_offset)/(1<<OFFSET_FRACTIONAL_BITS)) < (1.0+5.0*pixelsPerGroup))
		UErr("The bits/pixel allocation for non-first lines is too low (<5.33bpp).\nConsider decreasing FIRST_LINE_BPG_OFFSET.");

	// BEGIN scale_increment_interval fix
	if(final_scale > 9)
	{
		// Note: the following calculation assumes that the rcXformOffset crosses 0 at some point.  If the zero-crossing
		//   doesn't occur in a configuration, we recommend to reconfigure the rc_model_size and thresholds to be smaller
		//   for that configuration.
		dsc_cfg->scale_increment_interval = (int)((double)(1<<OFFSET_FRACTIONAL_BITS) * dsc_cfg->final_offset / 
					                            ((double)(final_scale - 9) * (dsc_cfg->nfl_bpg_offset + dsc_cfg->slice_bpg_offset + dsc_cfg->nsl_bpg_offset)));
		if (dsc_cfg->scale_increment_interval > 65535)
		{
			printf("scale_increment_interval is too large for this slice height.\n");
			invalid = 1;
		}
	}
	else
		dsc_cfg->scale_increment_interval = 0;
	// END scale_increment_interval fix

	if(dsc_cfg->dsc_version_minor == 2 && (dsc_cfg->native_420 || dsc_cfg->native_422))
	{
		// OPTIMIZED computation of rbsMin:
		// Compute max by sampling offset at points of inflection
		// *MODEL NOTE* MN_RBS_MIN
		maxOffset = compute_offset(dsc_cfg, pixelsPerGroup, groupsPerLine, (int)(ceil((float)initialDelay / pixelsPerGroup)));  // After initial delay
		maxOffset = MAX(maxOffset, compute_offset(dsc_cfg, pixelsPerGroup, groupsPerLine, groupsPerLine));   // After first line
		maxOffset = MAX(maxOffset, compute_offset(dsc_cfg, pixelsPerGroup, groupsPerLine, 2*groupsPerLine));
		rbsMin = dsc_cfg->rc_model_size - initialFullnessOfs + maxOffset;
	} else  // DSC 1.1 method
		rbsMin = (int)(dsc_cfg->rc_model_size - initialFullnessOfs + ((int)ceil(initialDelay * bitsPerPixel)) + groupsPerLine * firstLineBpgOfs);


	hrdDelay = (int)(ceil((double)rbsMin / bitsPerPixel));
	dsc_cfg->rcb_bits = (int)(ceil((double)hrdDelay * bitsPerPixel));
	dsc_cfg->initial_dec_delay = hrdDelay - dsc_cfg->initial_xmit_delay;
	RANGE_CHECK("initial_dec_delay", dsc_cfg->initial_dec_delay, 0, 65535);

	return (invalid);
}


extern int qlevel_luma_8bpc[];
extern int qlevel_chroma_8bpc[];
extern int qlevel_luma_10bpc[];
extern int qlevel_chroma_10bpc[];
extern int qlevel_luma_12bpc[];
extern int qlevel_chroma_12bpc[];
extern int qlevel_luma_14bpc[];
extern int qlevel_chroma_14bpc[];
extern int qlevel_luma_16bpc[];
extern int qlevel_chroma_16bpc[];

int Qp2Qlevel(dsc_cfg_t *dsc_cfg, int qp, int cpnt)
{
	int qlevel = 0;

	if (((cpnt%3) == 0) || ((dsc_cfg->native_420) && (cpnt == 1)))	
	{
		switch(dsc_cfg->bits_per_component)
		{
		case 8:    qlevel = qlevel_luma_8bpc[qp];  break;
		case 10:   qlevel = qlevel_luma_10bpc[qp]; break;
		case 12:   qlevel = qlevel_luma_12bpc[qp]; break;
		case 14:   qlevel = qlevel_luma_14bpc[qp]; break;
		case 16:   qlevel = qlevel_luma_16bpc[qp]; break;
		}
	} else {
		switch(dsc_cfg->bits_per_component)
		{
		case 8:     qlevel = qlevel_chroma_8bpc[qp];   break;
		case 10:    qlevel = qlevel_chroma_10bpc[qp];  break;
		case 12:    qlevel = qlevel_chroma_12bpc[qp];  break;
		case 14:    qlevel = qlevel_chroma_14bpc[qp];  break;
		case 16:    qlevel = qlevel_chroma_16bpc[qp];  break;
		}
		if((dsc_cfg->dsc_version_minor==2) && !dsc_cfg->convert_rgb)  // QP adjustment for YCbCr mode
			qlevel = MAX(0, qlevel-1);
	}

	return (qlevel);
}


/*!
 ************************************************************************
 * \brief
 *    check_qp_for_overflow() - Ensure max QP's are programmed to avoid overflow
 *
 * \param dsc_cfg
 *    Pointer to DSC configuration structure
 * \param pixelsPerGroup
 *    Number of pixels per group
 * \return
 *    0 = success; 1 = error with configuration
 ************************************************************************
 */
void check_qp_for_overflow(dsc_cfg_t *dsc_cfg, int pixelsPerGroup)
{
	double min_target_bpg;
	int p_mode_bits, max_bits;
	int cpnt, max_res_size;
	int cpntBitDepth[NUM_COMPONENTS];

	for (cpnt = 0; cpnt<NUM_COMPONENTS; ++cpnt)
		cpntBitDepth[cpnt] = (dsc_cfg->convert_rgb && (cpnt==1 || cpnt==2)) ? (dsc_cfg->bits_per_component + 1) : dsc_cfg->bits_per_component;

	min_target_bpg = pixelsPerGroup * bitsPerPixel - 
			((double)(dsc_cfg->slice_bpg_offset + dsc_cfg->nfl_bpg_offset)/(1<<OFFSET_FRACTIONAL_BITS));

	// MPP group when predicted size is 0
	p_mode_bits = 1;  // extra bit for luma prefix/ICH switch
	for (cpnt=0; cpnt<(dsc_cfg->native_422 ? 4 : 3); ++cpnt)
	{
		max_res_size = cpntBitDepth[cpnt] - Qp2Qlevel(dsc_cfg, dsc_cfg->rc_range_parameters[14].range_max_qp, cpnt); 
		p_mode_bits += max_res_size * 4;  // prefix + residuals
	}

	// Followed by predicted group (where predicted size is max size-1)
	max_bits = p_mode_bits;
	p_mode_bits = 1;
	for (cpnt=0; cpnt<(dsc_cfg->native_422 ? 4 : 3); ++cpnt)
	{
		max_res_size = cpntBitDepth[cpnt] - Qp2Qlevel(dsc_cfg, dsc_cfg->rc_range_parameters[14].range_max_qp, cpnt)-1; 
		p_mode_bits += 1 + max_res_size * 3;  // prefix (1bit) + residuals
	}
	max_bits += p_mode_bits;
	if ((double)max_bits > min_target_bpg*2)
		printf("WARNING: RC_MAXQP[14] will not guarantee RC model fullness will decrease\nin worst case and could cause a buffer overflow condition.\n");
}


/*!
************************************************************************
* \brief
*    generate_rc_parameters() - Generate RC parameters
*
* \param dsc_codec
*    PPS data structure
************************************************************************
*/
void generate_rc_parameters(dsc_cfg_t *dsc_codec)
{
	int qp_bpc_modifier;
	int i;
	int padding_pixels;
	int sw;

	make_qp_tables();

	qp_bpc_modifier = (bitsPerComponent - 8) * 2 - (useYuvInput && (dscVersionMinor==1));
	dsc_codec->rc_quant_incr_limit0 = 11 + qp_bpc_modifier;
	dsc_codec->rc_quant_incr_limit1 = 11 + qp_bpc_modifier;

	if (native422)
	{
		// =IF(CompressBpp >= 8, 2048, IF(CompressBpp <=  7, 5632, 5632 - ROUND((CompressBpp - 7) * (3584), 0)))
		if (bitsPerPixel >= 16.0)
			dsc_codec->initial_offset = 2048;
		else if (bitsPerPixel >= 14.0)
			dsc_codec->initial_offset = 5632 - (int)((bitsPerPixel - 14.0) * 1792 + 0.5);
		else if (bitsPerPixel >= 12.0)
			dsc_codec->initial_offset = 5632;
		else
			UErr("No auto-generated parameters available for bitsPerPixel < 6 in native 4:2:2 mode\n");
	}
	else  // 4:4:4 or simple 4:2:2 or native 4:2:0
	{
		if (bitsPerPixel >= 12.0)
			dsc_codec->initial_offset = 2048;
		else if (bitsPerPixel >= 10.0)
			dsc_codec->initial_offset = 5632 - (int)((bitsPerPixel - 10.0) * 1792 + 0.5);
		else if (bitsPerPixel >= 8.0)
			dsc_codec->initial_offset = 6144 - (int)((bitsPerPixel - 8.0) * 256 + 0.5);
		else if (bitsPerPixel >= 6.0)
			dsc_codec->initial_offset = 6144;
		else
			UErr("No auto-generated parameters available for bitsPerPixel < 6 in 4:4:4 mode (bitsPerPixel < 3 in native 4:2:0)\n");
	}
	dsc_codec->initial_xmit_delay = (int)(4096.0 / bitsPerPixel + 0.5);

	sw = ((native422 || native420) ? (dsc_codec->slice_width / 2) : dsc_codec->slice_width);
	padding_pixels = ((sw % 3) ? (3 - (sw % 3)) : 0) * (dsc_codec->initial_xmit_delay / sw);
	if (3 * bitsPerPixel >= ((dsc_codec->initial_xmit_delay + 2) / 3) * (native422 ? 4 : 3) && 
		(((dsc_codec->initial_xmit_delay + padding_pixels) % 3) == 1))
		dsc_codec->initial_xmit_delay++;

	firstLineBpgOfs = -1;   // Auto generate
	secondLineBpgOfs = -1;  // Auto generate
	initialFullnessOfs = dsc_codec->initial_offset;
	initialDelay = dsc_codec->initial_xmit_delay;
	dsc_codec->flatness_min_qp = 3 + qp_bpc_modifier;
	dsc_codec->flatness_max_qp = 12 + qp_bpc_modifier;
	dsc_codec->flatness_det_thresh = 2 << (bitsPerComponent - 8);
	// The following two lines were added in 1.57e
	if (native420)
		dsc_codec->second_line_ofs_adj = 512;

	for (i = 0; i < 15; ++i)
	{
		int idx;

		if (native420)
		{
			int ofs_und4[] = { 2, 0, 0, -2, -4, -6, -8, -8, -8, -10, -10, -12, -12, -12, -12 };
			int ofs_und5[] = { 2, 0, 0, -2, -4, -6, -8, -8, -8, -10, -10, -10, -12, -12, -12 };
			int ofs_und6[] = { 2, 0, 0, -2, -4, -6, -8, -8, -8, -10, -10, -10, -12, -12, -12 };
			int ofs_und8[] = { 10, 8, 6, 4, 2, 0, -2, -4, -6, -8, -10, -10, -12, -12, -12 };
			idx = (int)(bitsPerPixel - 8.0);
			dsc_codec->rc_range_parameters[i].range_min_qp = minqp_420[(bitsPerComponent - 8) / 2][i][idx];
			dsc_codec->rc_range_parameters[i].range_max_qp = maxqp_420[(bitsPerComponent - 8) / 2][i][idx];
			if (bitsPerPixel <= 8.0)
				dsc_codec->rc_range_parameters[i].range_bpg_offset = ofs_und4[i];
			else if (bitsPerPixel <= 10.0)
				dsc_codec->rc_range_parameters[i].range_bpg_offset = ofs_und4[i] + (int)(0.5 * (bitsPerPixel - 8.0) * (ofs_und5[i] - ofs_und4[i]) + 0.5);
			else if (bitsPerPixel <= 12.0)
				dsc_codec->rc_range_parameters[i].range_bpg_offset = ofs_und5[i] + (int)(0.5 * (bitsPerPixel - 10.0) * (ofs_und6[i] - ofs_und5[i]) + 0.5);
			else if (bitsPerPixel <= 16.0)
				dsc_codec->rc_range_parameters[i].range_bpg_offset = ofs_und6[i] + (int)(0.25 * (bitsPerPixel - 12.0) * (ofs_und8[i] - ofs_und6[i]) + 0.5);
			else
				dsc_codec->rc_range_parameters[i].range_bpg_offset = ofs_und8[i];
		}
		else if (native422)
		{
			int ofs_und6[] = { 2, 0, 0, -2, -4, -6, -8, -8, -8, -10, -10, -12, -12, -12, -12 };
			int ofs_und7[] = { 2, 0, 0, -2, -4, -6, -8, -8, -8, -10, -10, -10, -12, -12, -12 };
			int ofs_und10[] = { 10, 8, 6, 4, 2, 0, -2, -4, -6, -8, -10, -10, -12, -12, -12 };

			idx = (int)(bitsPerPixel - 12.0);
			dsc_codec->rc_range_parameters[i].range_min_qp = minqp_422[(bitsPerComponent - 8) / 2][i][idx];
			dsc_codec->rc_range_parameters[i].range_max_qp = maxqp_422[(bitsPerComponent - 8) / 2][i][idx];
			if (bitsPerPixel <= 12.0)
				dsc_codec->rc_range_parameters[i].range_bpg_offset = ofs_und6[i];
			else if(bitsPerPixel <= 14.0)
				dsc_codec->rc_range_parameters[i].range_bpg_offset = ofs_und6[i] + (int)((bitsPerPixel - 12.0) * (ofs_und7[i] - ofs_und6[i]) / 2.0 + 0.5);
			else if(bitsPerPixel <= 16.0)
				dsc_codec->rc_range_parameters[i].range_bpg_offset = ofs_und7[i];
			else if(bitsPerPixel <= 20.0)
				dsc_codec->rc_range_parameters[i].range_bpg_offset = ofs_und7[i] + (int)((bitsPerPixel - 16.0) * (ofs_und10[i] - ofs_und7[i]) / 4.0 + 0.5);
			else
				dsc_codec->rc_range_parameters[i].range_bpg_offset = ofs_und10[i];
		}
		else
		{
			int ofs_und6[] = { 0, -2, -2, -4, -6, -6, -8, -8, -8, -10, -10, -12, -12, -12, -12 };
			int ofs_und8[] = { 2, 0, 0, -2, -4,	-6,	-8,	-8, -8, -10, -10, -10, -12,	-12, -12 };
			int ofs_und12[] = { 2, 0, 0, -2, -4, -6, -8, -8, -8, -10, -10, -10, -12, -12, -12 };
			int ofs_und15[] = { 10, 8, 6, 4, 2, 0, -2, -4, -6, -8, -10, -10, -12, -12, -12 };

			idx = (int)(2 * (bitsPerPixel - 6.0));
			dsc_codec->rc_range_parameters[i].range_min_qp = MAX(0, minqp_444[(bitsPerComponent - 8) / 2][i][idx] - (useYuvInput && (dscVersionMinor==1)));
			dsc_codec->rc_range_parameters[i].range_max_qp = MAX(0, maxqp_444[(bitsPerComponent - 8) / 2][i][idx] - (useYuvInput && (dscVersionMinor==1)));
			if (bitsPerPixel <= 6.0)
				dsc_codec->rc_range_parameters[i].range_bpg_offset = ofs_und6[i];
			else if (bitsPerPixel <= 8.0)
				dsc_codec->rc_range_parameters[i].range_bpg_offset = ofs_und6[i] + (int)(0.5 * (bitsPerPixel - 6.0) * (ofs_und8[i] - ofs_und6[i]) + 0.5);
			else if (bitsPerPixel <= 12.0)
				dsc_codec->rc_range_parameters[i].range_bpg_offset = ofs_und8[i];
			else if (bitsPerPixel <= 15.0)
				dsc_codec->rc_range_parameters[i].range_bpg_offset = ofs_und12[i] + (int)((bitsPerPixel - 12.0) * (ofs_und15[i] - ofs_und12[i]) / 3.0 + 0.5);
			else
				dsc_codec->rc_range_parameters[i].range_bpg_offset = ofs_und15[i];
		}
		// The following code was added in 1.57g (parameter sanity check)
		RANGE_CHECK("range_max_qp", dsc_codec->rc_range_parameters[i].range_max_qp, 0, 15 + 2 * (bitsPerComponent - 8));
		if (dscVersionMinor == 1 && useYuvInput == 1 && dsc_codec->rc_range_parameters[i].range_max_qp > 12 + 2 * (bitsPerComponent - 8))
			UErr("ERROR: In DSC 1.1 mode with YCbCr, the max QP for range %d must be less than %d\n", i, 12 + 2 * (bitsPerComponent - 8));
	}
}


/*!
************************************************************************
* \brief
*    populate_pps() - Populate the PPS data structure
*
* \param dsc_codec
*    PPS data structure
* \param slicew
*    Slice width (output)
* \param sliceh
*    Slice height (output)
************************************************************************
*/
void populate_pps(dsc_cfg_t *dsc_codec, int *slicew, int *sliceh)
{
	int target_bpp_x16;
	int prev_min_qp, prev_max_qp, prev_thresh, prev_offset;
	int i;
	int pixelsPerGroup, numSsps;

	dsc_codec->native_420 = native420;
	dsc_codec->native_422 = native422;
	if (native422 && native420)
		UErr("ERROR: NATIVE_420 and NATIVE_422 modes cannot both be enabled at the same time\n");

	if (dscVersionMinor == 1)
	{
		secondLineOfsAdj = 0;
	}

	dsc_codec->dsc_version_minor = dscVersionMinor;
	dsc_codec->full_ich_err_precision = fullIchErrPrecision;
	if (dscVersionMinor == 1 && fullIchErrPrecision)
		UErr("ERROR: Alternate ICH selection only supported in DSC 1.2 mode\n");
	dsc_codec->second_line_ofs_adj = secondLineOfsAdj;
	if (function == 3 && (dsc_codec->pic_width == 0 || dsc_codec->pic_height == 0))
		UErr("ERROR: When using FUNCTION 3 (print PPS), the PIC_WIDTH and PIC_HEIGHT must be specified in the cfg file");
	RANGE_CHECK("pic_width", dsc_codec->pic_width, 1, 65535);
	RANGE_CHECK("pic_height", dsc_codec->pic_height, 1, 65535);
	dsc_codec->simple_422 = simple422;
	RANGE_CHECK("simple_422", dsc_codec->simple_422, 0, 1);
	dsc_codec->bits_per_component = bitsPerComponent;
	dsc_codec->rcb_bits = 0;
	if (dscVersionMinor == 1)   // DSC 1.1
	{
		dsc_codec->linebuf_depth = lineBufferBpc;
		RANGE_CHECK("linebuf_depth", dsc_codec->linebuf_depth, 8, 13);
		if (dsc_codec->bits_per_component != 8 && dsc_codec->bits_per_component != 10 && dsc_codec->bits_per_component != 12)
			UErr("bits_per_component must be either 8, 10, or 12\n");
	}
	else {					// DSC 1.2
		dsc_codec->linebuf_depth = lineBufferBpc;
		RANGE_CHECK("linebuf_depth", dsc_codec->linebuf_depth, 8, 16);
		if (dsc_codec->bits_per_component != 8 && dsc_codec->bits_per_component != 10 && dsc_codec->bits_per_component != 12 &&
			dsc_codec->bits_per_component != 14 && dsc_codec->bits_per_component != 16)
			UErr("bits_per_component must be either 8, 10, 12, 14, or 16\n");
	}

	// Removed from PPS in v1.44:
	dsc_codec->very_flat_qp = 1 + (2 * (dsc_codec->bits_per_component - 8));
	dsc_codec->somewhat_flat_qp_delta = 4;
	dsc_codec->somewhat_flat_qp_thresh = 7 + (2 * (dsc_codec->bits_per_component - 8));
	if (muxWordSize == 0)
	{
		if (dsc_codec->bits_per_component <= 10)
			muxWordSize = 48;
		else
			muxWordSize = 64;  // 16bpc needs some minor syntax modifications to ensure this works
	}
	dsc_codec->mux_word_size = muxWordSize;
	dsc_codec->convert_rgb = !useYuvInput;
	dsc_codec->rc_tgt_offset_hi = tgtOffsetHi;
	RANGE_CHECK("rc_tgt_offset_hi", dsc_codec->rc_tgt_offset_hi, 0, 15);
	dsc_codec->rc_tgt_offset_lo = tgtOffsetLo;
	RANGE_CHECK("rc_tgt_offset_lo", dsc_codec->rc_tgt_offset_lo, 0, 15);
	target_bpp_x16 = (int)(bitsPerPixel * 16 + 0.5);
	dsc_codec->bits_per_pixel = target_bpp_x16;

	RANGE_CHECK("bits_per_pixel (*16)", target_bpp_x16, 96, 1023);

	dsc_codec->rc_edge_factor = rcEdgeFactor;
	RANGE_CHECK("rc_edge_factor", dsc_codec->rc_edge_factor, 0, 15);
	if (rcEdgeFactor < 2)
		printf("WARNING: The rate control will not work as designed with rc_edge_factor < 2.\n");
	dsc_codec->rc_quant_incr_limit1 = quantIncrLimit1;
	RANGE_CHECK("rc_quant_incr_limit1", dsc_codec->rc_quant_incr_limit1, 0, 31);
	dsc_codec->rc_quant_incr_limit0 = quantIncrLimit0;
	RANGE_CHECK("rc_quant_incr_limit0", dsc_codec->rc_quant_incr_limit0, 0, 31);
	prev_min_qp = rcMinQp[0];
	prev_max_qp = rcMaxQp[0];
	prev_thresh = rcBufThresh[0];
	prev_offset = rcOffset[0];
	// The following line was added in v1.57g:
	for (i = 0; i<NUM_BUF_RANGES; ++i)
	{
		if (!generateRcParameters)
		{
			dsc_codec->rc_range_parameters[i].range_bpg_offset = rcOffset[i];
			RANGE_CHECK("range_bpg_offset", dsc_codec->rc_range_parameters[i].range_bpg_offset, -32, 31);
			if ((i>0) && (prev_offset < rcOffset[i]))
				printf("WARNING: The RC_OFFSET values should not increase as the range increases\n");
			dsc_codec->rc_range_parameters[i].range_max_qp = rcMaxQp[i];
			RANGE_CHECK("range_max_qp", dsc_codec->rc_range_parameters[i].range_max_qp, 0, 15 + 2 * (bitsPerComponent - 8));
			if (dscVersionMinor == 1 && useYuvInput == 1 && dsc_codec->rc_range_parameters[i].range_max_qp > 12 + 2 * (bitsPerComponent - 8))
				UErr("ERROR: In DSC 1.1 mode with YCbCr, the max QP for range %d must be less than %d\n", i, 12 + 2 * (bitsPerComponent - 8));
			if ((i>0) && (prev_max_qp > rcMaxQp[i]))
				printf("WARNING: The RC_MAX_QP values should not decrease as the range increases\n");
			dsc_codec->rc_range_parameters[i].range_min_qp = rcMinQp[i];
			RANGE_CHECK("range_min_qp", dsc_codec->rc_range_parameters[i].range_min_qp, 0, 15 + 2 * (bitsPerComponent - 8));
			if ((i>0) && (prev_min_qp > rcMinQp[i]))
				printf("WARNING: The RC_MIN_QP values should not decrease as the range increases\n");
		}
		if (i<NUM_BUF_RANGES - 1)
		{
			dsc_codec->rc_buf_thresh[i] = rcBufThresh[i];
			RANGE_CHECK("rc_buf_thresh", dsc_codec->rc_buf_thresh[i], 0, rcModelSize);
			if (dsc_codec->rc_buf_thresh[i] & 0x3f)
				UErr("All rc_buf_thresh must be evenly divisible by 64");
			if ((i>0) && (prev_thresh > rcBufThresh[i]))
				printf("WARNING: The RC_BUF_THRESH values should not decrease as the range increases\n");
			prev_thresh = rcBufThresh[i];
		}
		prev_min_qp = rcMinQp[i];
		prev_max_qp = rcMaxQp[i];
		prev_offset = rcOffset[i];
	}
	dsc_codec->rc_model_size = rcModelSize;
	RANGE_CHECK("rc_model_size", dsc_codec->rc_model_size, 0, 65535);
	dsc_codec->initial_xmit_delay = initialDelay;
	RANGE_CHECK("initial_xmit_delay", dsc_codec->initial_xmit_delay, 0, 1023);
	dsc_codec->block_pred_enable = bpEnable;
	RANGE_CHECK("block_pred_enable", dsc_codec->block_pred_enable, 0, 1);
	dsc_codec->initial_offset = initialFullnessOfs;
	RANGE_CHECK("initial_offset", dsc_codec->initial_offset, 0, rcModelSize);
	dsc_codec->xstart = 0;
	dsc_codec->ystart = 0;
	dsc_codec->flatness_min_qp = flatnessMinQp;
	RANGE_CHECK("flatness_min_qp", dsc_codec->flatness_min_qp, 0, 31);
	dsc_codec->flatness_max_qp = flatnessMaxQp;
	RANGE_CHECK("flatness_max_qp", dsc_codec->flatness_max_qp, 0, 31);
	dsc_codec->flatness_det_thresh = flatnessDetThresh;
	if (dsc_codec->rc_model_size <= dsc_codec->initial_offset)
		UErr("INITIAL_OFFSET must be less than RC_MODEL_SIZE\n");
	dsc_codec->initial_scale_value = 8 * dsc_codec->rc_model_size / (dsc_codec->rc_model_size - dsc_codec->initial_offset);
	RANGE_CHECK("initial_scale_value", dsc_codec->initial_scale_value, 0, 63);
	dsc_codec->vbr_enable = enableVbr;
	RANGE_CHECK("vbr_enable", dsc_codec->vbr_enable, 0, 1);

	// Compute rate buffer size for auto mode
	if (dsc_codec->native_422)
	{
		numSsps = 4;
		pixelsPerGroup = 3;
	}
	else {
		numSsps = 3;
		pixelsPerGroup = 3;
	}

	// Compute slice dimensions
	*slicew = (sliceWidth ? sliceWidth : dsc_codec->pic_width);
	*sliceh = (sliceHeight ? sliceHeight : dsc_codec->pic_height);

	dsc_codec->slice_width = *slicew;
	RANGE_CHECK("slice_width", dsc_codec->slice_width, 1, 65535);
	dsc_codec->slice_height = *sliceh;
	RANGE_CHECK("slice_height", dsc_codec->slice_height, 1, 65535);

	if (generateRcParameters)
		generate_rc_parameters(dsc_codec);

	if (!sliceHeight)  // Auto-detect optimal size height
	{
		if (autoSliceHeightAlgorithm)
		{
			int candidate_sliceh;
			int num_padding_lines;
			for (num_padding_lines = 0; num_padding_lines < dsc_codec->pic_height; num_padding_lines++)
			{
				// The following line was changed in 1.57f : i < dsc_codec->pic_height  ==>  i <= dsc_codec->pic_height
				for (i = 96; i <= dsc_codec->pic_height; i += 1 + dsc_codec->native_420)
				{
					if (autoSliceHeightAlgorithm == 1)
						candidate_sliceh = i;
					else    // Slice height algorithm = 2
					{
						candidate_sliceh = dsc_codec->pic_height + 96 - i;
						if (dsc_codec->native_420 && (dsc_codec->pic_height & 1))
							candidate_sliceh--;
					}
					if ((dsc_codec->pic_height % candidate_sliceh) == num_padding_lines)
					{
						printf("Trying slice height = %d (%d padding lines required)\n", candidate_sliceh, num_padding_lines);
						*sliceh = dsc_codec->slice_height = candidate_sliceh;
						if (!compute_rc_parameters(dsc_codec, pixelsPerGroup, numSsps))
							goto found_sliceh;
					}
				}
			}
			UErr("Could not find valid PPS for any slice height\n");
		found_sliceh:
			;
		}
		else
		{	// Original slice height determination algorithm
			while (*sliceh)
			{
				if (compute_rc_parameters(dsc_codec, pixelsPerGroup, numSsps))
				{
					printf("PPS was not valid for slice height = %d, trying slice_height = ", *sliceh);
					*sliceh = (*sliceh + 1) >> 1;
					if (native420)			// Add one if new slice height is odd
						*sliceh = *sliceh + (*sliceh & 1);
					printf("%d\n", *sliceh);
					dsc_codec->slice_height = *sliceh;
				}
				else
					break;
			}
			if (!(*sliceh))
				UErr("Could not find valid PPS for any slice height.");
		}
	}
	else
	{
		if (compute_rc_parameters(dsc_codec, pixelsPerGroup, numSsps))
			UErr("One or more PPS parameters exceeded their allowed bit depth.");
	}
	check_qp_for_overflow(dsc_codec, pixelsPerGroup);  // Add this line to check max QP for range 14
}

/*!
 ************************************************************************
 * \brief
 *    codec_main() - Codec mainline
 *
 * \param argc
 *    Argument count (from main)
 * \param argv
 *    Arguments (from main)
 ************************************************************************
 */
#ifdef WIN32
int codec_main(int argc, char *argv[])
#else
int main(int argc, char *argv[])
#endif
{
	pic_t *ip=NULL, *ip2, *ref_pic, *op_dsc;
	dsc_cfg_t dsc_codec;
	unsigned char **buf;
	char f[PATH_MAX], infname[PATH_MAX], bitsfname[PATH_MAX];
	char *extension;
	FILE *list_fp, *logfp;
	char base_name[PATH_MAX];
	int i, j;
	FILE *bits_fp = NULL;
	int bufsize;
	int xs, ys;
	int slicew, sliceh;
	pic_t **temp_pic = NULL;
	int numslices, slicecount;
	int slices_per_line;
	int num_frames;
	int **chunk_sizes;
	unsigned char pps[PPS_SIZE];
	int multiFrameYuvFile = 0;
	int frameNumber = 0;
	char frmnumstr[10];

	printf("Display Stream Compression (DSC) reference model version 1.57g\n");
	printf("Copyright 2013-2017 Broadcom Limited.  All rights reserved.\n\n");

	// Allocate variables
	cmd_args[0].var_ptr = rcOffset = (int *)malloc(sizeof(int)*15);
	cmd_args[1].var_ptr = rcMinQp = (int *)malloc(sizeof(int)*15);
	cmd_args[2].var_ptr = rcMaxQp = (int *)malloc(sizeof(int)*15);
	cmd_args[3].var_ptr = rcBufThresh = (int *)malloc(sizeof(int)*14);

	set_defaults();
	memset(&dsc_codec, 0, sizeof(dsc_cfg_t));

	/* process input arguments */
    process_args(argc, argv, cmd_args);

	if (function == 3)
	{
		dsc_codec.native_420 = native420;
		dsc_codec.native_422 = native422;
		if (native422 && native420)
			UErr("ERROR: NATIVE_420 and NATIVE_422 modes cannot both be enabled at the same time\n");
		dsc_codec.pic_width = picWidth;
		dsc_codec.pic_height = picHeight;
		populate_pps(&dsc_codec, &slicew, &sliceh);
		if (printPpsFormat == 1)
			print_pps(stdout, &dsc_codec);
		else if (printPpsFormat == 2)
			print_pps_v2(stdout, &dsc_codec);
		exit(0);
	}

	if (NULL == (list_fp=fopen(fn_i, "rt")))
	{
		fprintf(stderr, "Cannot open list file %s for input\n", fn_i);
		exit(1);
	}

	if (NULL == (logfp=fopen(fn_log, "wt")))
	{
		fprintf(stderr, "Cannot open list file log.txt for output\n");
		exit(1);
	}

	simple422 = simple422 || enable422;   // DSC 1.1 name is enable_422 and 1.2 name is simple_422
	if ((simple422 || native422 || native420) && !useYuvInput)
	{
		fprintf(stderr, "4:2:2/4:2:0 are only supported in YCbCr mode (USE_YUV_INPUT = 1)\n");
		exit(1);
	}

	infname[0] = '\0';
	if (fgets(infname, 512, list_fp) == NULL)
		UErr("Unexpected end of list file\n");

	while ((strlen(infname)>0) || !feof(list_fp) || multiFrameYuvFile)
	{
		sprintf(frmnumstr, "_%06d", frameNumber);
		memset(pps, 0, PPS_SIZE);
		ip = NULL;
		while ((strlen(infname)>0) && (infname[strlen(infname)-1] < '0'))
			infname[strlen(infname)-1] = '\0';

		if (strlen(infname)==0)  // Skip blank lines
		{
			infname[0] = '\0';
			if (fgets(infname, 512, list_fp) == NULL)
				break;   // reached end of file
			continue;
		}

		split_base_and_ext(infname, base_name, &extension);

		if (!strcmp(extension, "dpx") || !strcmp(extension, "DPX"))
		{
			if (dpx_read(infname, &ip, dpxRPadEnds, dpxRForceBe, dpxRDatumOrder, rbSwap))
			{
				if (function == 2)
					printf("Could not read original image for decode, PSNR will not be computed\n");
				else
				{
					fprintf(stderr, "Error read DPX file %s\n", infname);
					exit(1);
				}
			}
		}
		else if (!strcmp(extension, "ppm") || !strcmp(extension, "PPM"))
		{
			if (useYuvInput)
			{
				printf("PPM format is RGB only, converting source to YUV\n");
			}
			
			if (ppm_read(infname, &ip))
			{
				if (function == 2)
					printf("Could not read original image for decode, PSNR will not be computed\n");
				else
				{
					fprintf(stderr, "Error read PPM file %s\n", infname);
					exit(1);
				}
			}
		}
		else if (!strcmp(extension, "yuv") || !strcmp(extension, "YUV"))
		{
			if (!useYuvInput)
			{
				printf("Warning: YUV format is YUV only, upsampling source to 4:4:4 RGB\n");
			}

			// Right now just reads first frame of a YUV file, could be extended to read multiple frames
			if (yuv_read(infname, &ip, picWidth, picHeight, frameNumber, bitsPerComponent, &num_frames, yuvFileFormat))
			{
				if (function == 2)
					printf("Could not read original image for decode, PSNR will not be computed\n");
				else
				{
					fprintf(stderr, "Error read YUV file %s\n", infname);
					exit(1);
				}
			}
			multiFrameYuvFile = (frameNumber < num_frames-1);  // will be false for last frame of multi-frame file, which is ok since frameNumber will be nonzero
		}
		else if (strcmp(extension, "dsc") && strcmp(extension, "DSC"))
		{
			fprintf(stderr, "Unrecognized file format .%s\n", extension);
			exit(1);
		}

		if (ip)
		{
			ip->alpha = 0;

			if (bitsPerComponent > ip->bits)
			{
				printf("WARNING: Source picture bit depth is %d, converting to %d bits\n", ip->bits, bitsPerComponent);
				convertbits(ip, bitsPerComponent);
		    }
		}

		// 4:2:0 to 4:2:2 conversion
		if (ip && (ip->chroma == YUV_420) && !native420)
		{
			printf("WARNING: Source picture is 4:2:0, converting to 4:2:2\n");
			ip2 = pcreate(FRAME, ip->color, YUV_422, ip->w, ip->h);
			ip2->bits = ip->bits;
			ip2->alpha = 0;
			yuv_420_422(ip, ip2);
			pdestroy(ip);
			ip = ip2;
		}			

		// 4:2:2 to 4:4:4 coversion
		if (ip && (ip->chroma == YUV_422) && !(simple422 || native422 || native420))
		{
			printf("WARNING: Converting source picture to 4:4:4\n");
			ip2 = pcreate(FRAME, ip->color, YUV_444, ip->w, ip->h);
			ip2->bits = ip->bits;
			ip2->alpha = 0;
			yuv_422_444(ip, ip2);
			pdestroy(ip);
			ip = ip2;
		}

		// RGB to YUV conversion
		if (ip && (ip->color == RGB) && useYuvInput)
		{
			printf("WARNING: Source picture is RGB, converting to YCbCr\n");
			ip2 = pcreate(FRAME, YUV_HD, ip->chroma, ip->w, ip->h);
			ip2->bits = ip->bits;
			ip2->alpha = 0;
			rgb2yuv(ip, ip2);
			pdestroy(ip);
			ip = ip2;
		}
		else if (ip && ((ip->color == YUV_HD) || (ip->color == YUV_SD)) && !useYuvInput)      // YUV to RGB conversion
		{
			printf("WARNING: Source picture is YCbCr, converting to RGB\n");
			ip2 = pcreate(FRAME, RGB, YUV_444, ip->w, ip->h);
			ip2->bits = ip->bits;
			ip2->alpha = 0;
			yuv2rgb(ip, ip2);
			pdestroy(ip);
			ip = ip2;
		}

		if (ip && (ip->chroma == YUV_444) && (simple422 || native422 || native420))
		{
			printf("WARNING: Converting source picture to 4:2:2\n");
			ip2 = pcreate(FRAME, ip->color, YUV_422, ip->w, ip->h);
			ip2->bits = ip->bits;
			ip2->alpha = 0;
			yuv_444_422(ip, ip2);
			pdestroy(ip);
			ip = ip2;
		}

		if (ip && (ip->chroma == YUV_422) && native420)
		{
			printf("WARNING: Converting source picture to 4:2:0\n");
			ip2 = pcreate(FRAME, ip->color, YUV_420, ip->w, ip->h);
			ip2->bits = ip->bits;
			ip2->alpha = 0;
			yuv_422_420(ip, ip2);
			pdestroy(ip);
			ip = ip2;
		}

		if (ip && (bitsPerComponent < ip->bits))
		{
			printf("WARNING: Source bit depth is %d, converting to %d bits\n", ip->bits, bitsPerComponent);
			convertbits(ip, bitsPerComponent);
		}

		if (ip && (simple422 || native422 || native420) && (ip->w%2))
		{
			ip->w--;
			printf("WARNING: 4:2:2 picture width is constrained to be a multiple of 2.\nThe image %s will be cropped to %d pixels wide.\n", base_name, ip->w);
		}

		if (ip && (simple422 || native422 || native420) && (sliceWidth%2))
		{
			UErr("ERROR: 4:2:2 slice width is constrained to be a multiple of 2.\n");
		}

		if (ip && (native420) && (ip->h%2))
		{
			ip->h--;
			printf("WARNING: 4:2:0 picture height is constrained to be a multiple of 2.\nThe image %s will be cropped to %d pixels high.\n", base_name, ip->h);
		}

		if (ip && (native420) && (sliceHeight%2))
		{
			UErr("ERROR: 4:2:0 slice height is constrained to be a multiple of 2.\n");
		}

		ref_pic = ip;
		if (ip && (ip->chroma == YUV_420) && native420) 
        {
			ip2 = pcreate(FRAME, ip->color, YUV_444, ip->w/2, ip->h);
			ip2->bits = ip->bits;
			ip2->alpha = 0;

			// 4:2:0 horizontal packing
			for(i = 0; i<ip2->h ; i++) {
				for(j = 0 ; j<ip2->w ; j++) {
					ip2->data.yuv.y[i][j] = ip->data.yuv.y[i][j*2];
					ip2->data.yuv.u[i][j] = ip->data.yuv.y[i][j*2+1];

					if (i %2) 
						ip2->data.yuv.v[i][j] = ip->data.yuv.v[i/2][j];
					else
					   ip2->data.yuv.v[i][j] = ip->data.yuv.u[i/2][j];
				}
			}

			ip = ip2;
		}
		
		dsc_codec.native_420 = native420;
		dsc_codec.native_422 = native422;
		if(native422 && native420)
			UErr("ERROR: NATIVE_420 and NATIVE_422 modes cannot both be enabled at the same time\n");
		if (ip && simple422 && !dsc_codec.native_422)
		{
			ip2 = pcreate(FRAME, ip->color, YUV_444, ip->w, ip->h);
			ip2->bits = ip->bits;
			ip2->alpha = 0;
			simple422to444(ip, ip2);
			ip = ip2;
		}

		if ((function == 1) || (function == 0))
		{
			if (!ip)
			{
				fprintf(stderr, "Cannot use .dsc file as input\n");
				exit(1);
			}
			dsc_codec.pic_width = ref_pic->w;
			dsc_codec.pic_height = ip->h;
			populate_pps(&dsc_codec, &slicew, &sliceh);

			if (function==1)
			{
#ifdef WIN32
				if (frameNumber > 0 || multiFrameYuvFile)		// Output one DSC file per frame
					sprintf(bitsfname, "%s\\%s_%06d.dsc", fn_o, base_name, frameNumber);
				else
					sprintf(bitsfname, "%s\\%s.dsc", fn_o, base_name);
#else
				if (frameNumber > 0 || multiFrameYuvFile)
					sprintf(bitsfname, "%s/%s_%06d.dsc", fn_o, base_name, frameNumber);
				else
					sprintf(bitsfname, "%s/%s.dsc", fn_o, base_name);
#endif
				if ((bits_fp = fopen(bitsfname, "wb")) == NULL)
				{
					printf("Fatal error: Cannot open bitstream output file %s\n", bitsfname);
					exit(1);
				}
				//fwrite((void *)&dsc_codec, sizeof(dsc_cfg_t), 1, bits_fp);
				fputc('D', bits_fp); fputc('S', bits_fp); fputc('C', bits_fp); fputc('F', bits_fp);
				write_pps(pps, &dsc_codec);
				for (i=0; i<PPS_SIZE; ++i)
					fputc(pps[i], bits_fp);
			}
		}
		else   //  (function == 2) => decode
		{
#ifdef WIN32
			if (frameNumber > 0 || multiFrameYuvFile)		// If YUV reference has multi-frames, use DSC file per frame
				sprintf(bitsfname, "%s\\%s_%06d.dsc", fn_o, base_name, frameNumber);
			else
				sprintf(bitsfname, "%s\\%s.dsc", fn_o, base_name);
#else
			if (frameNumber > 0 || multiFrameYuvFile)
				sprintf(bitsfname, "%s/%s_%06d.dsc", fn_o, base_name, frameNumber);
			else
				sprintf(bitsfname, "%s/%s.dsc", fn_o, base_name);
#endif
			if ((bits_fp = fopen(bitsfname, "rb")) == NULL)
			{
				printf("Fatal error: Cannot open bitstream input file %s\n", bitsfname);
				exit(1);
			}
			//fread((void *)&dsc_codec, sizeof(dsc_cfg_t), 1, bits_fp);
			if ((fgetc(bits_fp) != 'D') || (fgetc(bits_fp) != 'S') || (fgetc(bits_fp) != 'C') || (fgetc(bits_fp) != 'F'))
				UErr("DSC file read error, invalid magic number\n");
			for (i=0; i<PPS_SIZE; ++i)
				pps[i] = fgetc(bits_fp);
			parse_pps(pps, &dsc_codec);

			// Removed from PPS in v1.44:
			dsc_codec.very_flat_qp = 1+(2*(dsc_codec.bits_per_component-8));
			dsc_codec.somewhat_flat_qp_delta = 4;
			dsc_codec.somewhat_flat_qp_thresh = 7+(2*(dsc_codec.bits_per_component-8));
			// Estimate rate buffer size based on delays:
			
			bitsPerPixel = (float)(dsc_codec.bits_per_pixel/16.0);
			dsc_codec.rcb_bits = (int)ceil((dsc_codec.initial_xmit_delay + dsc_codec.initial_dec_delay) * bitsPerPixel);

			sliceh = dsc_codec.slice_height;
			if (muxWordSize==0)
				muxWordSize = (dsc_codec.bits_per_component>=12) ? 64 : 48;
			dsc_codec.mux_word_size = muxWordSize;
		}
		bufsize = dsc_codec.chunk_size * sliceh;   // Total number of bytes to generate
		slices_per_line = (dsc_codec.pic_width + dsc_codec.slice_width - 1) / dsc_codec.slice_width;
		buf = (unsigned char **)malloc(sizeof(unsigned char *) * slices_per_line);
		for (i=0; i<slices_per_line; ++i)
			buf[i] = (unsigned char *)malloc(bufsize);

		if(dsc_codec.native_420)
			op_dsc = (pic_t *)pcreate(FRAME, (ip ? ip->color : YUV_HD), YUV_444, dsc_codec.pic_width/2, dsc_codec.pic_height);
		else if(dsc_codec.native_422)
			op_dsc = (pic_t *)pcreate(FRAME, dsc_codec.convert_rgb ? RGB : (ip ? ip->color : YUV_HD), YUV_422, dsc_codec.pic_width, dsc_codec.pic_height);
		else
			op_dsc = (pic_t *)pcreate(FRAME, dsc_codec.convert_rgb ? RGB : (ip ? ip->color : YUV_HD), YUV_444, dsc_codec.pic_width, dsc_codec.pic_height);
		op_dsc->bits = bitsPerComponent;
		op_dsc->alpha = 0;
		if (dsc_codec.convert_rgb)
		{
			int tpidx;

			temp_pic = (pic_t **)malloc(sizeof(pic_t*) * 2);
			for (tpidx=0; tpidx<2; ++tpidx)
			{
				// Space for converting to YCoCg
				temp_pic[tpidx] = (pic_t *)pcreate(FRAME, YUV_HD, YUV_444, dsc_codec.pic_width, dsc_codec.pic_height);
				temp_pic[tpidx]->bits = bitsPerComponent;
				temp_pic[tpidx]->alpha = 0;	
			}
		}

		slicecount = 0;
		chunk_sizes = (int **)malloc(sizeof(int *) * slices_per_line);
		for(i=0; i<slices_per_line; ++i)
			chunk_sizes[i] = (int *)malloc(sizeof(int) * sliceh);
		for (ys = 0; ys < dsc_codec.pic_height; ys+=sliceh)
		{
			if(function == 2)
				read_dsc_data(buf, dsc_codec.chunk_size, bits_fp, dsc_codec.vbr_enable, slices_per_line, dsc_codec.slice_height);
			for (xs = 0; xs < slices_per_line; xs++)
			{
				unsigned char *buf2;

				buf2 = buf[xs];
				numslices = ((dsc_codec.pic_width+dsc_codec.slice_width-1)/dsc_codec.slice_width) * ((dsc_codec.pic_height+sliceh-1)/sliceh);
				printf("Processing slice %d / %d\r", ++slicecount, numslices);
				fflush(stdout);  // For Bob.
				if(function != 2)
					memset(buf2, 0, bufsize);
				dsc_codec.xstart = xs * dsc_codec.slice_width;
				dsc_codec.ystart = ys;

				// Encoder
				if ((function==0) || (function==1))
					DSC_Encode(&dsc_codec, ip, op_dsc, buf2, temp_pic, chunk_sizes[xs]);

				// Decoder
				if ((function==0) || (function == 2))
					DSC_Decode(&dsc_codec, op_dsc, buf2, temp_pic); 
			}
			if(function == 1)
				write_dsc_data(buf, dsc_codec.chunk_size, bits_fp, dsc_codec.vbr_enable, slices_per_line, dsc_codec.slice_height, chunk_sizes);
		}
		printf("\n");
		for(i=0; i<slices_per_line; ++i)
			free(buf[i]);
		free(buf);
		for(i=0; i<slices_per_line; ++i)
			free(chunk_sizes[i]);
		free(chunk_sizes);
		if (function != 0)
			fclose(bits_fp);

		if (temp_pic)
		{
			pdestroy(temp_pic[0]);
			pdestroy(temp_pic[1]);
			free(temp_pic);
		}

		// Convert 444 to 422 if coded as 422
		if (dsc_codec.simple_422 && !dsc_codec.native_422)
		{
			ip2 = pcreate(FRAME, op_dsc->color, YUV_422, op_dsc->w, op_dsc->h);
			ip2->bits = op_dsc->bits;
			ip2->alpha = 0;
			simple444to422(op_dsc, ip2);
			pdestroy(op_dsc);
			op_dsc = ip2;
		}

        //SBR:OPTION 5
		if (dsc_codec.native_420) {
			ip2 = pcreate(FRAME, op_dsc->color, YUV_420, op_dsc->w*2, op_dsc->h);
			ip2->bits = op_dsc->bits;
			ip2->alpha = 0;

			// 4:2:0 horizontal packing
			for(i = 0; i<ip2->h ; i++) {
				for(j = 0 ; j<op_dsc->w ; j++) {
					ip2->data.yuv.y[i][j*2] = op_dsc->data.yuv.y[i][j];
					ip2->data.yuv.y[i][j*2+1] = op_dsc->data.yuv.u[i][j];

	                if (i %2) 
	                 	ip2->data.yuv.v[i/2][j]   = op_dsc->data.yuv.v[i][j];
					else
					    ip2->data.yuv.u[i/2][j]   = op_dsc->data.yuv.v[i][j];
				}
			}

			pdestroy(op_dsc);
			op_dsc = ip2;
		}

		if (function!=1)  // Don't write if encode only
		{
			if (yuvFileOutput)
			{
				strcpy(f, fn_o);
#ifdef WIN32
				strcat(f, "\\");
#else
				strcat(f, "/");
#endif
				strcat(f, base_name);
				strcat(f, ".out.yuv");
				if (yuv_write(f, op_dsc, frameNumber, yuvFileFormat))
				{
					fprintf(stderr, "Error writing YUV file %s\n", f);
					exit(1);
				}
			}
			if (dpxFileOutput)
			{
				strcpy(f, fn_o);
#ifdef WIN32
				strcat(f, "\\");
#else
				strcat(f, "/");
#endif
				strcat(f, base_name);
				if (multiFrameYuvFile || frameNumber > 0)
					strcat(f, frmnumstr);
				strcat(f, ".out.dpx");
				if (op_dsc->bits==14)
					convertbits(op_dsc, 16);
				if (dpx_write(f, op_dsc, dpxWPadEnds, dpxWDatumOrder, dpxWForcePacking, rbSwapOut, dpxWByteSwap))
				{
					fprintf(stderr, "Error writing DPX file %s\n", f);
					exit(1);
				}
			}
			if (ppmFileOutput)
			{
				strcpy(f, fn_o);
#ifdef WIN32
				strcat(f, "\\");
#else
				strcat(f, "/");
#endif
				strcat(f, base_name);
				if (multiFrameYuvFile || frameNumber > 0)
					strcat(f, frmnumstr);
				strcat(f, ".out.ppm");
				if (ppm_write(f, op_dsc))
				{
					fprintf(stderr, "Error writing PPM file %s\n", f);
					exit(1);
				}
			}
		}

		if (ip)
		{
			if (yuvFileOutput)
			{
				strcpy(f, fn_o);
#ifdef WIN32
				strcat(f, "\\");
#else
				strcat(f, "/");
#endif
				strcat(f, base_name);
				strcat(f, ".ref.yuv");
				if (yuv_write(f, ref_pic, frameNumber, yuvFileFormat))
				{
					fprintf(stderr, "Error writing YUV file %s\n", f);
					exit(1);
				}
			}
			if (dpxFileOutput)
			{
				strcpy(f, fn_o);
#ifdef WIN32
				strcat(f, "\\");
#else
				strcat(f, "/");
#endif
				strcat(f, base_name);
				if (multiFrameYuvFile || frameNumber > 0)
					strcat(f, frmnumstr);
				strcat(f, ".ref.dpx");
				if (ref_pic->bits==14)
					convertbits(ref_pic, 16);
				if (dpx_write(f, ref_pic, dpxWPadEnds, dpxWDatumOrder, dpxWForcePacking, rbSwapOut, dpxWByteSwap))
				{
					fprintf(stderr, "Error writing DPX file %s\n", f);
					exit(1);
				}
			}
			if (ppmFileOutput)
			{
				strcpy(f, fn_o);
#ifdef WIN32
				strcat(f, "\\");
#else
				strcat(f, "/");
#endif
				strcat(f, base_name);
				if (multiFrameYuvFile || frameNumber > 0)
					strcat(f, frmnumstr);
				strcat(f, ".ref.ppm");
				if (ppm_write(f, ref_pic))
				{
					fprintf(stderr, "Error writing PPM file %s\n", f);
					exit(1);
				}
			}

			fprintf(logfp, "Filename: %s.%s",  base_name, extension);
			if (multiFrameYuvFile || frameNumber>0)
				fprintf(logfp, " (frame %d)", frameNumber);
			fprintf(logfp,"\n%2.2f bits/pixel, %d bits/component,", bitsPerPixel, bitsPerComponent);
			fprintf(logfp," %s, %s,", useYuvInput ? "YUV" : "RGB", (dsc_codec.native_422 || dsc_codec.simple_422) ? "4:2:2" : ((dsc_codec.native_420) ? "4:2:0" : "4:4:4"));
			fprintf(logfp," %dx%d slices, block_pred_enable=%d\n", dsc_codec.slice_width, sliceh, bpEnable);
			compute_and_display_PSNR(ref_pic, op_dsc, ref_pic->bits, logfp);
			if (printPps)				// Print PPS (if specified on command line)
			{
				if (printPpsFormat == 1)
					print_pps(logfp, &dsc_codec);
				else if (printPpsFormat == 2)
					print_pps_v2(logfp, &dsc_codec);
			}

			if (ref_pic != ip)
				pdestroy(ref_pic);
			pdestroy(ip);
		}
		pdestroy(op_dsc);

		if (multiFrameYuvFile)
			frameNumber++;		// Increment frame counter
		else
		{
			infname[0] = '\0';
			frameNumber = 0;
			if (fgets(infname, 512, list_fp) == NULL)
				break;  // end of file
		}
	}

	fclose(list_fp);
	fclose(logfp);
	free(rcOffset);
	free(rcMinQp);
	free(rcMaxQp);
	free(rcBufThresh);
	return(0);
}
