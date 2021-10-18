/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2005
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE. 
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

/*****************************************************************************
 *
 * Filename:
 * ---------
 *   image_sensor.c
 *
 * Project:
 * --------
 *   Maui_sw
 *
 * Description:
 * ------------
 *   Image sensor driver function
 *
 * Author:
 * -------
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * 
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include "drv_comm.h"
#include "IntrCtrl.h"
#include "reg_base.h"
#include "gpio_sw.h"
#include "sccb.h"
#include "isp_if.h"
#include "image_sensor.h"
#include "camera_para.h"
#include "upll_ctrl.h"
#include "af.h"

#define	USE_48MHZ
/* Global Variables */
SensorInfo g_CCT_MainSensor = OV2640_OMNIVISION;
kal_uint8 g_CCT_FirstGrabColor = BAYER_Gr;

kal_bool gVGAmode = KAL_TRUE, MPEG4_encode_mode = KAL_FALSE;
static kal_uint8  g_iPV_PCLK_Divider = 1;
kal_uint16 extra_exposure_lines = 0;
static kal_uint16 g_iExpLines = 0;
kal_uint16 sensor_global_gain=BASEGAIN, sensor_gain_base=0x0;
static kal_uint16 g_iPreview_Column_Pixel = 0;
OV2640_OP_TYPE g_iOV2640_Mode = OV2640_MODE_NONE;

/* MAX/MIN Explosure Lines Used By AE Algorithm */
kal_uint16 MAX_EXPOSURE_LINES = 1000;
kal_uint8  MIN_EXPOSURE_LINES = 2;

/* Parameter For Engineer mode function */
kal_uint32 FAC_SENSOR_REG;

extern kal_uint32 SCCB_DELAY;//huafeizhou061016 add

#ifndef HW_SCCB

static void SCCB_send_byte(const kal_uint8 iSendByte)
{
    volatile signed char iI;
    volatile kal_uint8 iJ;

    for (iI = 7; iI >= 0; iI--) {   // data bit 7~0
        if (iSendByte & (1 << iI)) {
            SET_SCCB_DATA_HIGH;
        }else {
            SET_SCCB_DATA_LOW;
        }
        for (iJ = 0; iJ < SENSOR_I2C_DELAY; iJ++);
        SET_SCCB_CLK_HIGH;
        for (iJ = 0; iJ < SENSOR_I2C_DELAY; iJ++);
        SET_SCCB_CLK_LOW;
        for (iJ = 0; iJ < SENSOR_I2C_DELAY; iJ++);
    }

    // don't care bit, 9th bit
    SET_SCCB_DATA_LOW;
    SET_SCCB_DATA_INPUT;
    SET_SCCB_CLK_HIGH;
    for (iJ = 0; iJ < SENSOR_I2C_DELAY; iJ++);
    SET_SCCB_CLK_LOW;
    SET_SCCB_DATA_OUTPUT;
}   /* SCCB_send_byte() */

static kal_uint8 SCCB_get_byte(void)
{
    volatile signed char iI;
    volatile kal_uint8 iJ;
    kal_uint8 iGetByte = 0;

    SET_SCCB_DATA_INPUT;

    for (iJ = 0; iJ < SENSOR_I2C_DELAY; iJ++);

	for (iI = 7; iI >= 0; iI--) {    // data bit 7~0
        SET_SCCB_CLK_HIGH;
        for (iJ = 0; iJ < SENSOR_I2C_DELAY; iJ++);
        if (GET_SCCB_DATA_BIT) {
            iGetByte |= (1 << iI);
        }
        for (iJ = 0; iJ < SENSOR_I2C_DELAY; iJ++);
        SET_SCCB_CLK_LOW;
        for (iJ = 0; iJ < SENSOR_I2C_DELAY; iJ++);
    }

    // don't care bit, 9th bit
    SET_SCCB_DATA_OUTPUT;
    SET_SCCB_DATA_HIGH;
    for (iJ = 0; iJ < SENSOR_I2C_DELAY; iJ++);
    SET_SCCB_CLK_HIGH;
    for (iJ = 0; iJ < SENSOR_I2C_DELAY; iJ++);
    SET_SCCB_CLK_LOW;

	return iGetByte;
}   /* SCCB_get_byte()  */

#endif

static void write_cmos_sensor(const kal_uint32 iAddr, const kal_uint32 iPara)
{
    volatile kal_uint8 iI;

    #ifdef HW_SCCB
        SET_SCCB_DATA_LENGTH(3);
        ENABLE_SCCB;
        REG_SCCB_DATA = OV2640_WRITE_ID | SCCB_DATA_REG_ID_ADDRESS;
        REG_SCCB_DATA = iAddr;
        REG_SCCB_DATA = iPara;
        while (SCCB_IS_WRITTING);
    #else
        I2C_START_TRANSMISSION;
        for (iI = 0; iI < SENSOR_I2C_DELAY; iI++);
        SCCB_send_byte(OV2640_WRITE_ID);
        for (iI = 0; iI < SENSOR_I2C_DELAY; iI++);
        SCCB_send_byte(iAddr);
        for (iI = 0; iI < SENSOR_I2C_DELAY; iI++);
        SCCB_send_byte(iPara);
        for (iI = 0; iI < SENSOR_I2C_DELAY; iI++);
        I2C_STOP_TRANSMISSION;
    #endif  /*  HW_SCCB */
}   /*  write_cmos_sensor()  */

static kal_uint32 read_cmos_sensor(const kal_uint32 iAddr)
{
    volatile kal_uint8 iI;
    kal_uint8 iGetByte = 0;

    #ifdef HW_SCCB
        SET_SCCB_DATA_LENGTH(2);
        ENABLE_SCCB;
        REG_SCCB_DATA = OV2640_WRITE_ID | SCCB_DATA_REG_ID_ADDRESS;
        REG_SCCB_DATA = iAddr;
        while (SCCB_IS_WRITTING);
        ENABLE_SCCB;
        REG_SCCB_DATA = OV2640_READ_ID | SCCB_DATA_REG_ID_ADDRESS;
        REG_SCCB_DATA = 0;
        while (SCCB_IS_READING);
        iGetByte = REG_SCCB_READ_DATA & 0xFF;
    #else
        I2C_START_TRANSMISSION;
        for (iI = 0; iI < SENSOR_I2C_DELAY; iI++);
        SCCB_send_byte(OV2640_WRITE_ID);
        for (iI = 0; iI < SENSOR_I2C_DELAY; iI++);
        SCCB_send_byte(iAddr);
        for (iI = 0; iI < SENSOR_I2C_DELAY; iI++);
        I2C_STOP_TRANSMISSION;
        for (iI = 0; iI < SENSOR_I2C_DELAY; iI++);
        I2C_START_TRANSMISSION;
        for (iI = 0; iI < SENSOR_I2C_DELAY; iI++);
        SCCB_send_byte(OV2640_READ_ID);
        for (iI = 0; iI < SENSOR_I2C_DELAY; iI++);
        iGetByte = SCCB_get_byte();
        for (iI = 0; iI < SENSOR_I2C_DELAY; iI++);
        I2C_STOP_TRANSMISSION;
    #endif

    return iGetByte;
}   /*  read_cmos_sensor()  */

void page_write_cmos_sensor(kal_uint32 iAddr, kal_uint32 iPara)
{
    kal_uint16 iRegPage, iRegAddr;

    iRegPage = iAddr >> 8;
    iRegAddr = iAddr & 0x000000FF;

    write_cmos_sensor(PAGE_SETTING_REG, iRegPage);
    write_cmos_sensor(iRegAddr, iPara);
}

static kal_uint32 page_read_cmos_sensor(const kal_uint32 iAddr)
{
    kal_uint16 iRegPage, iRegAddr;
    kal_uint8 iValue = 0x00;

    iRegPage = iAddr >> 8;
    iRegAddr = iAddr & 0x000000FF;

    write_cmos_sensor(PAGE_SETTING_REG, iRegPage);
    iValue = read_cmos_sensor(iRegAddr);

    return iValue;
}

void write_OV2640_shutter(kal_uint16 shutter)
{
    kal_uint8 iTemp;

    if (gVGAmode) {
        if (shutter <= PV_EXPOSURE_LIMITATION) {
            extra_exposure_lines = 0;
        }else {
            extra_exposure_lines=shutter - PV_EXPOSURE_LIMITATION;
        }

        if (shutter > PV_EXPOSURE_LIMITATION) {
            shutter = PV_EXPOSURE_LIMITATION;
        }
    }else {
        if (shutter <= FULL_EXPOSURE_LIMITATION) {
            extra_exposure_lines = 0;
        }else {
            extra_exposure_lines = shutter - FULL_EXPOSURE_LIMITATION;
        }

        if (shutter > FULL_EXPOSURE_LIMITATION) {
            shutter = FULL_EXPOSURE_LIMITATION;
        }
    }

    write_cmos_sensor(PAGE_SETTING_REG, 0x01);
    write_cmos_sensor(0x2D, extra_exposure_lines & 0xFF);             // ADVFL(LSB of extra exposure lines)
    write_cmos_sensor(0x2E, (extra_exposure_lines & 0xFF00) >> 8);      // ADVFH(MSB of extra exposure lines)

    iTemp = read_cmos_sensor(0x04);
    write_cmos_sensor(0x04, ((iTemp & 0xFC) | (shutter & 0x3)));	// AEC[b1~b0]
    write_cmos_sensor(0x10, ((shutter & 0x3FC) >> 2));						// AEC[b9~b2]
    write_cmos_sensor(0x45, ((shutter & 0xFC00) >> 10));						// AEC[b10]/AEC[b15~b10]
}   /* write_OV2640_shutter */

static kal_uint16 Reg2Gain(const kal_uint8 iReg)
{
    kal_uint8 iI;
    kal_uint16 iGain = BASEGAIN;    // 1x-gain base

    // Range: 1x to 32x
    // Gain = (GAIN[7] + 1) * (GAIN[6] + 1) * (GAIN[5] + 1) * (GAIN[4] + 1) * (1 + GAIN[3:0] / 16)
    for (iI = 7; iI >= 4; iI--) {
        iGain *= (((iReg >> iI) & 0x01) + 1);
    }

    return iGain +  iGain * (iReg & 0x0F) / 16;
}

static kal_uint8 Gain2Reg(const kal_uint16 iGain)
{
    kal_uint8 iReg = 0x00;

    if (iGain < 2 * BASEGAIN) {
        // Gain = 1 + GAIN[3:0](0x00) / 16
        iReg = 16 * (iGain - BASEGAIN) / BASEGAIN;
    }else if (iGain < 4 * BASEGAIN) {
        // Gain = 2 * (1 + GAIN[3:0](0x00) / 16)
        iReg |= 0x10;
        iReg |= 8 * (iGain - 2 * BASEGAIN) / BASEGAIN;
    }else if (iGain < 8 * BASEGAIN) {
        // Gain = 4 * (1 + GAIN[3:0](0x00) / 16)
        iReg |= 0x30;
        iReg |= 4 * (iGain - 4 * BASEGAIN) / BASEGAIN;
    }else if (iGain < 16 * BASEGAIN) {
        // Gain = 8 * (1 + GAIN[3:0](0x00) / 16)
        iReg |= 0x70;
        iReg |= 2 * (iGain - 8 * BASEGAIN) / BASEGAIN;
    }else if (iGain < 32 * BASEGAIN) {
        // Gain = 16 * (1 + GAIN[3:0](0x00) / 16)
        iReg |= 0xF0;
        iReg |= (iGain - 16 * BASEGAIN) / BASEGAIN;
    }else {
        ASSERT(0);
    }

    return iReg;
}

static void OV2640_SetDummy(const kal_uint16 iPixels, const kal_uint16 iLines)
{
    ASSERT(iPixels < 4096);
    write_cmos_sensor(PAGE_SETTING_REG, 0x01);
    write_cmos_sensor(0x2A, (iPixels & 0x0F00) >> 4);
    write_cmos_sensor(0x2B, iPixels & 0x00FF);
    write_cmos_sensor(0x46, iLines & 0x00FF);
    write_cmos_sensor(0x47, iLines >> 8);
}   /*  OV2640_SetDummy */

static void OV2640_InitialSetting(void)
{
#ifdef OV_PROCESSING_RAW
    write_cmos_sensor(PAGE_SETTING_REG, 0x00);
    write_cmos_sensor(0x2C, 0xFF);
    write_cmos_sensor(0x2E, 0xDF);
    write_cmos_sensor(PAGE_SETTING_REG, 0x01);
    write_cmos_sensor(0x3C, 0x32);
    //
    write_cmos_sensor(0x11, 0x00); // clk divider
    write_cmos_sensor(0x09, 0x02);
    write_cmos_sensor(0x04, 0x28);
    write_cmos_sensor(0x13, 0xE0); // AEC/AGC off
    write_cmos_sensor(0x14, 0x48);
    write_cmos_sensor(0x2C, 0x0C); // reserved
    write_cmos_sensor(0x33, 0x78); // reserved
    write_cmos_sensor(0x3A, 0x33); // reserved
    write_cmos_sensor(0x3B, 0xFB); // reserved
    write_cmos_sensor(0x3E, 0x00); // reserved
    write_cmos_sensor(0x43, 0x11); // reserved
    write_cmos_sensor(0x16, 0x10); // reserved
    //
    write_cmos_sensor(0x39, 0x02); // reserved
    //
    write_cmos_sensor(0x35, 0xDA); // reserved
    write_cmos_sensor(0x22, 0x1A); // reserved
    write_cmos_sensor(0x37, 0xC3); // reserved
    write_cmos_sensor(0x23, 0x00); // reserved
    write_cmos_sensor(0x34, 0xC0); // reserved
    write_cmos_sensor(0x36, 0x1A); // reserved
    write_cmos_sensor(0x06, 0x88); // reserved
    write_cmos_sensor(0x07, 0xC0); // reserved
    write_cmos_sensor(0x0D, 0x87);
    write_cmos_sensor(0x0E, 0x41); // reserved
    write_cmos_sensor(0x4C, 0x00); // reserved
    //
    write_cmos_sensor(0x4A, 0x81); // reserved
    write_cmos_sensor(0x21, 0x99); // reserved
    //
    write_cmos_sensor(0x24, 0x40);
    write_cmos_sensor(0x25, 0x38);
    write_cmos_sensor(0x26, 0x82);
    write_cmos_sensor(0x5C, 0x00);
    write_cmos_sensor(0x63, 0x00);
    //
    write_cmos_sensor(0x61, 0x70);
    write_cmos_sensor(0x62, 0x80);
    write_cmos_sensor(0x7C, 0x05); // reserved
    //
    write_cmos_sensor(0x20, 0x80); // reserved
    write_cmos_sensor(0x28, 0x30); // reserved
    write_cmos_sensor(0x6C, 0x00); // reserved
    write_cmos_sensor(0x6E, 0x00); // reserved
    write_cmos_sensor(0x70, 0x02); // reserved
    write_cmos_sensor(0x71, 0x94); // reserved
    write_cmos_sensor(0x73, 0xc1); // reserved
    //
    //write_cmos_sensor(0x3D, 0x34);
    write_cmos_sensor(0x5A, 0x57);
    write_cmos_sensor(0x4F, 0xBB);
    write_cmos_sensor(0x50, 0x9C);
    //
    //
    write_cmos_sensor(PAGE_SETTING_REG, 0x00);
    write_cmos_sensor(0xE5, 0x7F); // Bypass DSP
    write_cmos_sensor(0xF9, 0xC0); // Bypass DSP
    write_cmos_sensor(0x41, 0x24);
    write_cmos_sensor(0xE0, 0x14);
    write_cmos_sensor(0x76, 0xFF); // Bypass DSP
    write_cmos_sensor(0x33, 0xA0);
    write_cmos_sensor(0x42, 0x20);
    write_cmos_sensor(0x43, 0x18);
    write_cmos_sensor(0x4C, 0x00);
    //write_cmos_sensor(0x87, 0xD0);
    write_cmos_sensor(0x88, 0x3F);
    write_cmos_sensor(0xD7, 0x03);
    write_cmos_sensor(0xD9, 0x10);
    write_cmos_sensor(0xD3, 0x82);
    //
    write_cmos_sensor(0xC8, 0x08);
    write_cmos_sensor(0xC9, 0x80);
    //
    write_cmos_sensor(0x7C, 0x00);// SDE command
    write_cmos_sensor(0x7D, 0x00);
    write_cmos_sensor(0x7C, 0x03);
    write_cmos_sensor(0x7D, 0x48);
    write_cmos_sensor(0x7D, 0x48);
    write_cmos_sensor(0x7C, 0x08);
    write_cmos_sensor(0x7D, 0x20);
    write_cmos_sensor(0x7D, 0x10);
    write_cmos_sensor(0x7D, 0x0E);
    //
    write_cmos_sensor(0x92, 0x00);
    write_cmos_sensor(0x93, 0x06);
    write_cmos_sensor(0x93, 0xE4);
    write_cmos_sensor(0x93, 0x05);
    write_cmos_sensor(0x93, 0x05);
    write_cmos_sensor(0x93, 0x00);
    write_cmos_sensor(0x93, 0x04);
    write_cmos_sensor(0x93, 0x00);
    write_cmos_sensor(0x93, 0x00);
    write_cmos_sensor(0x93, 0x00);
    write_cmos_sensor(0x93, 0x00);
    write_cmos_sensor(0x93, 0x00);
    write_cmos_sensor(0x93, 0x00);
    write_cmos_sensor(0x93, 0x00);
    //
    write_cmos_sensor(0xC3, 0xED);
    write_cmos_sensor(0xA4, 0x00);
    write_cmos_sensor(0xA8, 0x00);
    write_cmos_sensor(0xC5, 0x11);
    write_cmos_sensor(0xC6, 0x51);
    write_cmos_sensor(0xBF, 0x80);
    write_cmos_sensor(0xC7, 0x10);
    write_cmos_sensor(0xB6, 0x66);
    write_cmos_sensor(0xB8, 0xA5);
    write_cmos_sensor(0xB7, 0x64);
    write_cmos_sensor(0xB9, 0x7C);
    write_cmos_sensor(0xB3, 0xAF);
    write_cmos_sensor(0xB4, 0x97);
    write_cmos_sensor(0xB5, 0xFF);
    write_cmos_sensor(0xB0, 0xC5);
    write_cmos_sensor(0xB1, 0x94);
    write_cmos_sensor(0xB2, 0x0F);
    write_cmos_sensor(0xB4, 0x5C);
    //
    write_cmos_sensor(0xC0, 0xC8);
    write_cmos_sensor(0xC1, 0x96);
    write_cmos_sensor(0x86, 0x1D);
    write_cmos_sensor(0x50, 0x00);
    write_cmos_sensor(0x51, 0x90);
    write_cmos_sensor(0x52, 0x18);
    write_cmos_sensor(0x53, 0x00);
    write_cmos_sensor(0x54, 0x00);
    write_cmos_sensor(0x55, 0x88);
    write_cmos_sensor(0x57, 0x00);
    write_cmos_sensor(0x5A, 0x90);
    write_cmos_sensor(0x5B, 0x18);
    write_cmos_sensor(0x5C, 0x05);
    //
    write_cmos_sensor(0xC3, 0xED);
    write_cmos_sensor(0x7F, 0x00);
    //
    write_cmos_sensor(0xDA, 0x04);
    //
    write_cmos_sensor(0xE5, 0x1F);
    write_cmos_sensor(0xE1, 0x67);
    write_cmos_sensor(0xE0, 0x00);
    write_cmos_sensor(0xDD, 0x7F);
    write_cmos_sensor(0x05, 0x00);

    write_cmos_sensor(PAGE_SETTING_REG, 0x01);

    write_cmos_sensor(0x2A, 0x30);
    write_cmos_sensor(0x2B, 0x00);

	  write_cmos_sensor(0x11, 0x00);
	  write_cmos_sensor(0x12, 0x40); // SVGA mode

    // setup windowing
    write_cmos_sensor(0x17, 0x11);
    write_cmos_sensor(0x18, 0x43);
    write_cmos_sensor(0x19, 0x00);
    write_cmos_sensor(0x1A, 0x4B);
    write_cmos_sensor(0x32, 0x09);

    write_cmos_sensor(0x03, 0x04);
    write_cmos_sensor(0x3D, 0x38);
    write_cmos_sensor(0x39, 0x12);
    write_cmos_sensor(0x35, 0xDA);
    write_cmos_sensor(0x22, 0x1A);
    write_cmos_sensor(0x37, 0xC3);
    write_cmos_sensor(0x23, 0x00);
    write_cmos_sensor(0x34, 0xA0);
    write_cmos_sensor(0x36, 0x1A);
    write_cmos_sensor(0x06, 0x88);
    write_cmos_sensor(0x07, 0xC0);
    write_cmos_sensor(0x0D, 0x87);
    write_cmos_sensor(0x0E, 0x41);
    write_cmos_sensor(0x4C, 0x00);
    //
    write_cmos_sensor(0x63, 0x20);// CIP RAW
    //
    write_cmos_sensor(PAGE_SETTING_REG, 0x00);
    write_cmos_sensor(0xC0, 0x66);
    write_cmos_sensor(0xC1, 0x4C);
    write_cmos_sensor(0x8C, 0x06);
    write_cmos_sensor(0x86, 0x35);
    write_cmos_sensor(0x50, 0x00);
    write_cmos_sensor(0x51, 0xCC);
    write_cmos_sensor(0x52, 0x99);
    write_cmos_sensor(0x53, 0x00);
    write_cmos_sensor(0x54, 0x00);
    write_cmos_sensor(0x55, 0x00);
    write_cmos_sensor(0x5A, 0xCC);
    write_cmos_sensor(0x5B, 0x99);

    write_cmos_sensor(0x5C, 0x00);
    write_cmos_sensor(0xD3, 0x82);
    //
    write_cmos_sensor(0xDA, 0x04);
    //
    write_cmos_sensor(0xE5, 0x1F);
    write_cmos_sensor(0xE1, 0x67);
    write_cmos_sensor(0xE0, 0x00);
    write_cmos_sensor(0xDD, 0x7F);
    write_cmos_sensor(0x05, 0x00);
    //
    write_cmos_sensor(0x87, OV_WPC_BPC_CTRL);// Pixel correction on
    write_cmos_sensor(0xC3, 0x81);
    write_cmos_sensor(0xC2, 0x01);// raw

    write_cmos_sensor(0x92, 0x71);
    write_cmos_sensor(0x93, 0x00);
    write_cmos_sensor(0x92, 0x00);
    write_cmos_sensor(0x93, 0x00);

    write_cmos_sensor(0x4F, 0xCA); // 50Hz banding
    write_cmos_sensor(0x50, 0xA8); // 60Hz banding


    write_cmos_sensor(PAGE_SETTING_REG, 0x00);

#else
    write_cmos_sensor(PAGE_SETTING_REG, 0x00);
    write_cmos_sensor(0x2C, 0xFF);
    write_cmos_sensor(0x2E, 0xDF);
    write_cmos_sensor(PAGE_SETTING_REG, 0x00);
    write_cmos_sensor(0xE5, 0x7F); // Bypass DSP
    write_cmos_sensor(0xF9, 0xC0); // Bypass DSP
    write_cmos_sensor(0x05, 0x01); // Bypass DSP
    write_cmos_sensor(0x88, 0x00); // Bypass DSP
    write_cmos_sensor(0x89, 0x00); // Bypass DSP
    write_cmos_sensor(0x76, 0x00); // Bypass DSP
    write_cmos_sensor(0x85, 0x1F); // Bypass DSP
    write_cmos_sensor(0x7F, 0x0F); // Bypass DSP

    write_cmos_sensor(PAGE_SETTING_REG, 0x01);
    write_cmos_sensor(0x3C, 0x32);  // reserved

    write_cmos_sensor(0x2A, 0x30);
    write_cmos_sensor(0x2B, 0x00);

    write_cmos_sensor(0x11, 0x00); // clk divider
    write_cmos_sensor(0x09, 0x02);
    write_cmos_sensor(0x04, 0x28);
    write_cmos_sensor(0x13, 0xE0); // AEC/AGC off
    write_cmos_sensor(0x14, 0x48);
    write_cmos_sensor(0x2C, 0x0C); // reserved
    write_cmos_sensor(0x33, 0x78); // reserved
    write_cmos_sensor(0x3A, 0x33); // reserved
    write_cmos_sensor(0x3B, 0xFB); // reserved

    write_cmos_sensor(0x3E, 0x00); // reserved
    write_cmos_sensor(0x43, 0x11); // reserved
    write_cmos_sensor(0x16, 0x10); // reserved

	write_cmos_sensor(0x12, 0x40); // SVGA mode

    // setup windowing
    write_cmos_sensor(0x17, 0x11);
    write_cmos_sensor(0x18, 0x43);
    write_cmos_sensor(0x19, 0x00);
    write_cmos_sensor(0x1A, 0x4B);
    write_cmos_sensor(0x32, 0x09);
    write_cmos_sensor(0x4F, 0xCA); // 50Hz banding
    write_cmos_sensor(0x50, 0xA8); // 60Hz banding
    write_cmos_sensor(0x5A, 0x23); // reserved
    write_cmos_sensor(0x6D, 0x00); // reserved
    write_cmos_sensor(0x3D, 0x38); // reserved

    write_cmos_sensor(0x39, 0x12); // reserved
    write_cmos_sensor(0x35, 0xDA); // reserved
    write_cmos_sensor(0x22, 0x1A); // reserved
    write_cmos_sensor(0x37, 0xC3); // reserved
    write_cmos_sensor(0x23, 0x00); // reserved
    write_cmos_sensor(0x34, 0xC0); // reserved
    write_cmos_sensor(0x36, 0x1A); // reserved
    write_cmos_sensor(0x06, 0x88); // reserved
    write_cmos_sensor(0x07, 0xC0); // reserved
    write_cmos_sensor(0x0D, 0x87);
    write_cmos_sensor(0x0E, 0x41); // reserved
    write_cmos_sensor(0x4C, 0x00); // reserved

    write_cmos_sensor(0x4A, 0x81); // reserved
    write_cmos_sensor(0x21, 0x99); // reserved

    write_cmos_sensor(0x5C, 0x00); // reserved
    write_cmos_sensor(0x63, 0x00); // reserved
    write_cmos_sensor(0x61, 0x70); // histogram low
    write_cmos_sensor(0x62, 0x80); // histogram high

    write_cmos_sensor(0x7C, 0x05); // reserved
    write_cmos_sensor(0x20, 0x80); // reserved
    write_cmos_sensor(0x28, 0x30); // reserved
    write_cmos_sensor(0x6C, 0x00); // reserved
    write_cmos_sensor(0x6E, 0x00); // reserved
    write_cmos_sensor(0x70, 0x02); // reserved
    write_cmos_sensor(0x71, 0x94); // reserved
    write_cmos_sensor(0x73, 0xc1); // reserved

    write_cmos_sensor(PAGE_SETTING_REG, 0x00);
    write_cmos_sensor(0x05, 0x01); // bypass DSP, sensor output directly

#endif
    write_cmos_sensor(PAGE_SETTING_REG, 0x01);
    write_cmos_sensor(0x0F, 0x43); // reserved
    write_cmos_sensor(0x2D, 0x00); // VSYNC pulse width
    write_cmos_sensor(0x2E, 0x00); // VSYNC pulse width

    write_cmos_sensor(0x11,0x01); // clock divider
    write_cmos_sensor(0x12,0x40); // SVGA mode

    // setup windowing
    write_cmos_sensor(0x17, 0x10);
    write_cmos_sensor(0x18, 0x43);
    write_cmos_sensor(0x19, 0x00);
    write_cmos_sensor(0x1A, 0x4d);

    write_cmos_sensor(PAGE_SETTING_REG, 0x01);
    write_cmos_sensor(0x11, 0x01); // clock divider
    write_cmos_sensor(0x3D, 0x38); // reserved
    write_cmos_sensor(0x13, 0xC0); // turn off AGC/AEC
    write_cmos_sensor(0x00, 0x00); // global gain
    write_cmos_sensor(0x04, 0x28);
    write_cmos_sensor(0x10, 0x33); // exposure line
    write_cmos_sensor(0x45, 0x00); // AGC/AEC
} 
/*************************************************************************
* FUNCTION
*	OV2640_Init
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor and ISP control register.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
kal_int8 OV2640_Init(void)
{
    volatile kal_uint32 i;
    kal_uint16 iSensor_ID;

    SET_TG_OUTPUT_CLK_DIVIDER(1);
    SET_CMOS_RISING_EDGE(0);
    SET_CMOS_FALLING_EDGE(1);
#ifdef USE_48MHZ
    ENABLE_CAMERA_TG_CLK_48M;
    UPLL_Enable(UPLL_OWNER_ISP);
#endif
    SET_CMOS_CLOCK_POLARITY_LOW;
#ifdef OV_PROCESSING_RAW
    SET_VSYNC_POLARITY_HIGH;
#else
    SET_VSYNC_POLARITY_LOW;
#endif
    SET_HSYNC_POLARITY_LOW;
    ENABLE_CAMERA_PIXEL_CLKIN_ENABLE;
    SET_FIRST_GRAB_COLOR(BAYER_Gr);

    cis_module_power_on(KAL_TRUE);      // Power On CIS Power
    kal_sleep_task(2);					// To wait for Stable Power
#if (defined(MT6228)||defined(MT6229))
    kal_sleep_task(2);
#endif

    SET_RESET_CMOS_SENSOR_HIGH; //Sensor RESET pin high//huafeizhou061016 del
    for (i=0;i<0x8000;i++);
#if (defined(MT6228)||defined(MT6229))
    for (i=0;i<0x8000;i++);
#endif	
    SET_RESET_CMOS_SENSOR_LOW; //Sensor RESET pin low
    for (i=0;i<40000;i++);//about 1ms huafeizhou061016
#if (defined(MT6228)||defined(MT6229))
    for (i=0;i<40000;i++);//about 1ms huafeizhou061016
#endif
    SET_RESET_CMOS_SENSOR_HIGH; //Sensor RESET pin high

    kal_sleep_task(4);
#if (defined(MT6228)||defined(MT6229))
    kal_sleep_task(4);
#endif

    set_isp_driving_current(camera_para.SENSOR.reg[CMMCLK_CURRENT_INDEX].para); //8MA

    // Reset Sensor
    write_cmos_sensor(PAGE_SETTING_REG, 0x01);
    write_cmos_sensor(0x12, 0x80);
    kal_sleep_task(2);

#if (defined(MT6228)||defined(MT6229))
    kal_sleep_task(2);
#endif	

    iSensor_ID = (read_cmos_sensor(0x0A) << 8) | read_cmos_sensor(0x0B);

    if ((iSensor_ID != OV2640_SENSOR_ID_2C) && (iSensor_ID != OV2640_SENSOR_ID_2B)) {
        return -1;
    }

    set_isp_interrupt_trigger_delay_lines(1);

    // Initail Sequence Write In.
    OV2640_InitialSetting();
    camera_para_to_sensor();

    //set sensor driving capacity 0x0==>1x  0x1==>3x  0x2==>2x  0x03==>4x 
    write_cmos_sensor(PAGE_SETTING_REG, 0x01);
	write_cmos_sensor(0x09, 0x03); //PCLK Driving current

    write_cmos_sensor(PAGE_SETTING_REG, 0x01);
//    sensor_gain_base=read_OV2640_gain();

    return 1;
}   /* OV2640_Init  */

/*************************************************************************
* FUNCTION
*	OV2640_PowerOff
*
* DESCRIPTION
*	This function is to turn off sensor module power.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void OV2640_PowerOff(void)
{
    cis_module_power_on(KAL_FALSE); // turn off sensor power

#ifdef USE_48MHZ		
    UPLL_Disable(UPLL_OWNER_ISP);	
#endif	
    SET_SCCB_CLK_LOW;
    SET_SCCB_DATA_LOW;
}   /*  OV2640_PowerOff */

/*************************************************************************
* FUNCTION
*	OV2640_GetID
*
* DESCRIPTION
*	This function return the sensor read/write id of SCCB interface.
*
* PARAMETERS
*	*pWriteID : address pointer of sensor write id
*   *pReadID  : address pointer of sensor read id
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void OV2640_GetID(kal_uint8 *pWriteID, kal_uint8 *pReadID)
{
    *pWriteID = OV2640_WRITE_ID;
    *pReadID = OV2640_READ_ID;
}   /*  OV2640_GetID    */

/*************************************************************************
* FUNCTION
*	OV2640_GetSize
*
* DESCRIPTION
*	This function return the image width and height of image sensor.
*
* PARAMETERS
*	*pWidth : address pointer of horizontal effect pixels of image sensor
*  *pHeight : address pointer of vertical effect pixels of image sensor
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void OV2640_GetSize(kal_uint16 *pWidth, kal_uint16 *pHeight)
{
    *pWidth = IMAGE_SENSOR_FULL_WIDTH;      // effect pixel number in one line
    *pHeight = IMAGE_SENSOR_FULL_HEIGHT;    // effective line number in one frame
}   /*  OV2640_GetSize  */

/*************************************************************************
* FUNCTION
*	get_OV2640_period
*
* DESCRIPTION
*	This function return the image width and height of image sensor.
*
* PARAMETERS
*	*pixel_number : address pointer of pixel numbers in one period of HSYNC
*  *line_number : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void OV2640_GetPeriod(kal_uint16 *pPixels, kal_uint16 *pLines)
{
    *pPixels = PV_PERIOD_PIXEL_NUMS;    // pixel numbers in one period of HSYNC
    *pLines = PV_PERIOD_LINE_NUMS;      // line numbers in one period of VSYNC
}   /*  OV2640_GetPeriod    */

void OV2640_Preview(image_sensor_exposure_window_struct *pImageWindow, image_sensor_config_struct *pSensorConfigData)
{
    kal_uint8 iTemp;
    kal_uint16 iDummyPixels, iDummyLines, iStartX, iStartY;

    write_cmos_sensor(PAGE_SETTING_REG, 0x01);
    write_cmos_sensor(0x12, 0x40);  // change to SVGA(800x600) mode

#ifdef OV_PROCESSING_RAW
    // setup sensor output ROI
    write_cmos_sensor(0x17, 0x11);
    write_cmos_sensor(0x18, 0x44);
    write_cmos_sensor(0x32, 0x09);
    write_cmos_sensor(0x19, 0x00);
    write_cmos_sensor(0x1A, 0x4D);
    write_cmos_sensor(0x03, 0x04);

    write_cmos_sensor(0x4F, 0xCA);  // 50Hz banding AEC 8 LSBs
    write_cmos_sensor(0x50, 0xA8);  // 60Hz banding AEC 8 LSBs

    write_cmos_sensor(0x6D, 0x00);  // reserved //????
    write_cmos_sensor(0x3D, 0x38);  // PLL/divider setting
    write_cmos_sensor(0x39, 0x12);  // PWCOM1, reserved
    write_cmos_sensor(0x35, 0xDA);  // reserved
    iTemp = read_cmos_sensor(0x22); // ANCOM3
    write_cmos_sensor(0x22, iTemp | 0x10);
    write_cmos_sensor(0x37, 0xC3);  // reserved
    write_cmos_sensor(0x23, 0x00);  // reserved
    write_cmos_sensor(0x34, 0xA0);  // ARCOM2, reserved
    write_cmos_sensor(0x36, 0x1A);  // reserved
    write_cmos_sensor(0x06, 0x88);  // reserved
    write_cmos_sensor(0x07, 0xC0);  // reserved
    write_cmos_sensor(0x0D, 0x87);  // reserved
    write_cmos_sensor(0x0E, 0x41);  // reserved
    write_cmos_sensor(0x4C, 0x00);  // reserved

    write_cmos_sensor(PAGE_SETTING_REG, 0x00);
    write_cmos_sensor(0xC0, 0x66);
    write_cmos_sensor(0xC1, 0x4C);
    write_cmos_sensor(0x8C, 0x06);
    write_cmos_sensor(0x86, 0x35);
    write_cmos_sensor(0x50, 0x00);

    write_cmos_sensor(0x51, 0xCC);
    write_cmos_sensor(0x52, 0x99);

    write_cmos_sensor(0x53, 0x00);
    write_cmos_sensor(0x54, 0x00);
    write_cmos_sensor(0x55, 0x00);


    write_cmos_sensor(0x5A, 0xCC);
    write_cmos_sensor(0x5B, 0x99);
    write_cmos_sensor(0x5C, 0x00);
    write_cmos_sensor(0xD3, 0x82);

    write_cmos_sensor(0xE5, 0x1F);
    write_cmos_sensor(0xE1, 0x67);
    write_cmos_sensor(0xE0, 0x00);
    write_cmos_sensor(0xDD, 0x7F);
    write_cmos_sensor(0x05, 0x00);

    //write_cmos_sensor(0x87, 0xD0);
    write_cmos_sensor(0xC3, 0x81);
    write_cmos_sensor(0xC2, 0x01);
    write_cmos_sensor(0x92, 0x01);
    write_cmos_sensor(0x93, 0x00);
    write_cmos_sensor(0x92, 0x00);
    write_cmos_sensor(0x93, 0x00);
#else
    // setup sensor output ROI
    write_cmos_sensor(0x17, 0x10);
    write_cmos_sensor(0x18, 0x43);
    write_cmos_sensor(0x32, 0x36);
    write_cmos_sensor(0x19, 0x00);
    write_cmos_sensor(0x1A, 0x4D);
    write_cmos_sensor(0x03, 0x00);

    write_cmos_sensor(0x4F, 0xCA);  // 50Hz banding AEC 8 LSBs
    write_cmos_sensor(0x50, 0xA8);  // 60Hz banding AEC 8 LSBs
    write_cmos_sensor(0x5A, 0x23);  // 50/60Hz banding AEC maximum steps
    write_cmos_sensor(0x6D, 0x00);  // reserved
    write_cmos_sensor(0x3D, 0x38);  // PLL/divider setting
    write_cmos_sensor(0x39, 0x12);  // PWCOM1, reserved
    write_cmos_sensor(0x35, 0xDA);  // reserved
    iTemp = read_cmos_sensor(0x22); // ANCOM3
    write_cmos_sensor(0x22, iTemp | 0x10);
    write_cmos_sensor(0x37, 0xC3);  // reserved
    write_cmos_sensor(0x23, 0x00);  // reserved
    write_cmos_sensor(0x34, 0xC0);  // ARCOM2, reserved
    write_cmos_sensor(0x36, 0x1A);  // reserved
    write_cmos_sensor(0x06, 0x88);  // reserved
    write_cmos_sensor(0x07, 0xC0);  // reserved
    write_cmos_sensor(0x0D, 0x87);  // reserved
    write_cmos_sensor(0x0E, 0x41);  // reserved
    write_cmos_sensor(0x4C, 0x00);  // reserved
#endif

    gVGAmode = KAL_TRUE;

    if (pSensorConfigData->frame_rate == 0x0F) {   // MPEG4 Encode Mode
        MPEG4_encode_mode = KAL_TRUE;

#ifdef OV2640_MPEG4_QCIF_VIDEO_30FPS
        if (pImageWindow->image_target_width == 176 &&
            pImageWindow->image_target_height == 144) {
            // MT6228 supports QCIF MPEG4 up to 30fps 
            g_iOV2640_Mode = OV2640_MODE_QCIF_VIDEO;

            SET_TG_OUTPUT_CLK_DIVIDER(1);   // MCLK = 24MHz
            SET_CMOS_RISING_EDGE(0);
            SET_CMOS_FALLING_EDGE(1);
            SET_TG_PIXEL_CLK_DIVIDER(1);    // PCLK = 24MHz
            SET_CMOS_DATA_LATCH(1);

            //*******************************************
            // OV2640 clock calculation:
            // Fclk = (64 - 0x3D[5:0]) x MCLK / M, where
            //      Fclk: PLL output clock
            //      M = 2 if 0x3D[7:6] = 00
            //      M = 3 if 0x3D[7:6] = 01
            //      M = 4 if 0x3D[7:6] = 10
            //      M = 6 if 0x3D[7:6] = 11
            // Fint = Fclk / (2 x (0x11[5:0] + 1)), where
            //      Fint: internal clock
            // PCLK = Fint / 2
            //*******************************************
            write_cmos_sensor(PAGE_SETTING_REG, 0x01);
            write_cmos_sensor(0x3D, 0x38);
            write_cmos_sensor(0x11, 0x00);

//            iStartX = 4;
//            iStartY = 4;
            iDummyPixels = OV2640_VIDEO__QCIF_DUMMY_PIXELS;
            iDummyLines = 17; 
        }else {
#endif
            // MT6228 supports CIF MPEG4 up to 15fps 
            g_iOV2640_Mode = OV2640_MODE_CIF_VIDEO;

            /* config TG of ISP to match the setting of image sensor*/
            SET_TG_OUTPUT_CLK_DIVIDER(1);   // MCLK = 24MHz
            SET_CMOS_RISING_EDGE(0);
            SET_CMOS_FALLING_EDGE(1);
            SET_TG_PIXEL_CLK_DIVIDER(3);    // PCLK = 12MHz
            SET_CMOS_DATA_LATCH(1);

            //*******************************************
            // OV2640 clock calculation:
            // Fclk = (64 - 0x3D[5:0]) x MCLK / M, where
            //      Fclk: PLL output clock
            //      M = 2 if 0x3D[7:6] = 00
            //      M = 3 if 0x3D[7:6] = 01
            //      M = 4 if 0x3D[7:6] = 10
            //      M = 6 if 0x3D[7:6] = 11
            // Fint = Fclk / (2 x (0x11[5:0] + 1)), where
            //      Fint: internal clock
            // PCLK = Fint / 2
            //*******************************************
            write_cmos_sensor(PAGE_SETTING_REG, 0x01);
            write_cmos_sensor(0x3D, 0x38);
            write_cmos_sensor(0x11, 0x01);

//            iStartX = 4;
//            iStartY = 4;
            iDummyPixels = OV2640_VIDEO__CIF_DUMMY_PIXELS;
            iDummyLines = 0;
#ifdef OV2640_MPEG4_QCIF_VIDEO_30FPS
        }
#endif
    }else {
        MPEG4_encode_mode = KAL_FALSE;

        SET_TG_OUTPUT_CLK_DIVIDER(1);   // MCLK = 24MHz
        SET_CMOS_RISING_EDGE(0);
        SET_CMOS_FALLING_EDGE(1);
        SET_TG_PIXEL_CLK_DIVIDER(1);    // PCLK = 24MHz
        SET_CMOS_DATA_LATCH(1);

        //*******************************************
        // OV2640 clock calculation:
        // Fclk = (64 - 0x3D[5:0]) x MCLK / M, where
        //      Fclk: PLL output clock
        //      M = 2 if 0x3D[7:6] = 00
        //      M = 3 if 0x3D[7:6] = 01
        //      M = 4 if 0x3D[7:6] = 10
        //      M = 6 if 0x3D[7:6] = 11
        // Fint = Fclk / (2 x (0x11[5:0] + 1)), where
        //      Fint: internal clock
        // PCLK = Fint / 2
        //*******************************************
        write_cmos_sensor(PAGE_SETTING_REG, 0x01);
        write_cmos_sensor(0x3D, 0x38);
        write_cmos_sensor(0x11, 0x00);
        g_iPV_PCLK_Divider = ((DRV_Reg32(ISP_TG_PHASE_COUNTER_REG) >> 4) & 0x0000000F) + 1;

//        iStartX = 4;
//        iStartY = 4+2;//remove top dummy line
        iDummyPixels = OV2640_PV_DUMMY_PIXELS;
        iDummyLines = 0; 

        g_iOV2640_Mode = OV2640_MODE_PREVIEW;
    }

    write_cmos_sensor(PAGE_SETTING_REG, 0x01);
    iTemp = read_cmos_sensor(0x04) & 0x2F;

    switch (pSensorConfigData->image_mirror) {
    case IMAGE_NORMAL:
        iStartX = 4;
        iStartY = 6;
        // do nothing
        break;

    case IMAGE_HV_MIRROR:
        iStartX = 4;
#ifdef OV_PROCESSING_RAW
        iStartY = 5;
        iTemp |= 0xC0;
#else
        iStartY = 4;
        iTemp |= 0xD0;
#endif
        break;

    default:
        ASSERT(0);
    }
    write_cmos_sensor(0x04, iTemp);

    // compensate OV2640 ABLC for every frame
    write_cmos_sensor(PAGE_SETTING_REG, 0x01);
    write_cmos_sensor(0x71, 0x94);  // reserved register

    OV2640_SetDummy(iDummyPixels, iDummyLines);

    g_iPreview_Column_Pixel = PV_PERIOD_PIXEL_NUMS + iDummyPixels;

    pImageWindow->grab_start_x = iStartX;
    pImageWindow->grab_start_y = iStartY;
    pImageWindow->exposure_window_width = IMAGE_SENSOR_PV_WIDTH;
    pImageWindow->exposure_window_height = IMAGE_SENSOR_PV_HEIGHT;

    write_OV2640_shutter(g_iExpLines);
}   /*  OV2640_Preview  */

void OV2640_Capture(image_sensor_exposure_window_struct *pImageWindow, image_sensor_config_struct *pSensorConfigData)
{
    kal_uint16 iCapture_Column_Pixel = 0, iShutter = g_iExpLines;
    kal_uint16 iDummyPixels = 0, iDummyLines = 0, iStartX, iStartY, iGrabWidth, iGrabHeight;
    kal_uint8 iTemp, iCP_PCLK_Div = 1;
    const kal_bool bMetaMode = pSensorConfigData->meta_mode != CAPTURE_MODE_NORMAL;

    if (pSensorConfigData->enable_shutter_tansfer == KAL_TRUE) {
        iShutter = pSensorConfigData->capture_shutter;
    }

    if ((pImageWindow->image_target_width <= IMAGE_SENSOR_PV_WIDTH) &&
        (pImageWindow->image_target_height <= IMAGE_SENSOR_PV_HEIGHT)) {
        gVGAmode = KAL_TRUE;

        if (pSensorConfigData->frame_rate == 0xF0) {    //  WEBCAM mode
            SET_TG_PIXEL_CLK_DIVIDER(1);    // PCLK = 24MHz
            SET_CMOS_DATA_LATCH(1);

            write_cmos_sensor(PAGE_SETTING_REG, 0x01);
            write_cmos_sensor(0x11, 0x00);
            iDummyPixels = 600; // to make line period same as preview
            iDummyLines = 0;

//            iStartX = 4;
//            iStartY = 4;
            iGrabWidth = IMAGE_SENSOR_PV_WIDTH;
            iGrabHeight = IMAGE_SENSOR_PV_HEIGHT - 20;
        }else {
            if (pImageWindow->digital_zoom_factor >= (ISP_DIGITAL_ZOOM_INTERVAL << 1) || bMetaMode == KAL_TRUE) {
                SET_TG_PIXEL_CLK_DIVIDER(7);    // PCLK = 6MHz
                SET_CMOS_DATA_LATCH(4);
                write_cmos_sensor(PAGE_SETTING_REG, 0x01);
                write_cmos_sensor(0x11, 0x03);
                iDummyPixels = 70;
                iDummyLines = 0;
            }else { // capture < 2x digital zoom
                SET_TG_PIXEL_CLK_DIVIDER(3);    // PCLK = 12MHz
                SET_CMOS_DATA_LATCH(2);
                write_cmos_sensor(PAGE_SETTING_REG, 0x01);
                write_cmos_sensor(0x11, 0x01);
                iDummyPixels = 0;
                iDummyLines = 0;
            }
//            iStartX = 4;
//            iStartY = 4;
            iGrabWidth = IMAGE_SENSOR_PV_WIDTH;
            iGrabHeight = IMAGE_SENSOR_PV_HEIGHT;
        }

        switch (pSensorConfigData->image_mirror) {
        case IMAGE_NORMAL:
            iStartX = 4;
#ifdef OV_PROCESSING_RAW
            iStartY = 6;
#else
            iStartY = 4;
#endif
            break;

        case IMAGE_HV_MIRROR:
            iStartX = 4;
#ifdef OV_PROCESSING_RAW
            iStartY = 5;
#else
            iStartY = 4;
#endif
        break;

        default:
            ASSERT(0);
        }

        iCapture_Column_Pixel = PV_PERIOD_PIXEL_NUMS + iDummyPixels;
        iCP_PCLK_Div = ((DRV_Reg32(ISP_TG_PHASE_COUNTER_REG) >> 4) & 0x0000000F) + 1;
        iShutter = iShutter * g_iPreview_Column_Pixel / iCapture_Column_Pixel;
        iShutter = iShutter * g_iPV_PCLK_Divider / iCP_PCLK_Div;
    }else { // 2M UXGA Mode
        gVGAmode = KAL_FALSE;

#ifdef OV_PROCESSING_RAW
        /*Switch mode to 1610x1210*/
        write_cmos_sensor(PAGE_SETTING_REG,0x01);
        write_cmos_sensor(0x12, 0x00);  // switch to UXGA(1600x1200) mode

        // setup sensor output ROI
        write_cmos_sensor(0x17, 0x11);
        write_cmos_sensor(0x18, 0x76);
        write_cmos_sensor(0x32, 0x24);
        write_cmos_sensor(0x19, 0x01);
        write_cmos_sensor(0x1A, 0x99);
        write_cmos_sensor(0x03, 0x0E);

        write_cmos_sensor(0x4F, 0xBB);  // 50Hz banding AEC 8 LSBs
        write_cmos_sensor(0x50, 0x9C);  // 60Hz banding AEC 8 LSBs
        write_cmos_sensor(0x5A, 0x57);  // 50/60Hz banding AEC maximum steps
        write_cmos_sensor(0x6D, 0x80);  // reserved
//        write_cmos_sensor(0x3D, 0x34);  // PLL/divider setting
        write_cmos_sensor(0x3D, 0x38);  // PLL/divider setting

        write_cmos_sensor(0x39, 0x02);  // PWCOM1, reserved
        write_cmos_sensor(0x35, 0x88);  // reserved
        iTemp = read_cmos_sensor(0x22); // ANCOM3
        write_cmos_sensor(0x22, iTemp & 0xEF);

        write_cmos_sensor(0x37, 0x40);  // reserved
        write_cmos_sensor(0x23, 0x00);  // reserved
        write_cmos_sensor(0x34, 0xA0);  // ARCOM2, reserved
        write_cmos_sensor(0x36, 0x1A);  // reserved
        write_cmos_sensor(0x06, 0x02);  // reserved
        write_cmos_sensor(0x07, 0xC0);  // reserved
        write_cmos_sensor(0x0D, 0xB7);  // reserved
        write_cmos_sensor(0x0E, 0x01);  // reserved
        write_cmos_sensor(0x4C, 0x00);  // reserved

        write_cmos_sensor(PAGE_SETTING_REG, 0x00);
        write_cmos_sensor(0xC0, 0xCA);
        write_cmos_sensor(0xC1, 0x98);
        write_cmos_sensor(0x8C, 0x02);
        write_cmos_sensor(0x86, 0x35);
        write_cmos_sensor(0x50, 0x00);

        write_cmos_sensor(0x51, 0x94);
        write_cmos_sensor(0x52, 0x30);

        write_cmos_sensor(0x53, 0x00);
        write_cmos_sensor(0x54, 0x00);
        write_cmos_sensor(0x55, 0x88);
        write_cmos_sensor(0x57, 0x00);

        write_cmos_sensor(0x5A, 0x94);
        write_cmos_sensor(0x5B, 0x30);
        write_cmos_sensor(0x5C, 0x05);
        write_cmos_sensor(0xD3, 0x82);

        write_cmos_sensor(0xE5, 0x1F);
        write_cmos_sensor(0xE1, 0x67);
        write_cmos_sensor(0xE0, 0x00);
        write_cmos_sensor(0xDD, 0x7F);
        write_cmos_sensor(0x05, 0x00);

        write_cmos_sensor(0x87, OV_WPC_BPC_CTRL);

        write_cmos_sensor(0xC3, 0x01);
        write_cmos_sensor(0xC2, 0x81);
        write_cmos_sensor(0x92, 0x01);
        write_cmos_sensor(0x93, 0x00);
        write_cmos_sensor(0x92, 0x00);
        write_cmos_sensor(0x93, 0x00);
#else
        /*Switch mode to 1610x1210*/
        write_cmos_sensor(PAGE_SETTING_REG,0x01);
        write_cmos_sensor(0x12, 0x00);  // switch to UXGA(1600x1200) mode

        // setup sensor output ROI
        write_cmos_sensor(0x17, 0x11);
        write_cmos_sensor(0x18, 0x75);
        write_cmos_sensor(0x32, 0x3A);
        write_cmos_sensor(0x19, 0x01);
        write_cmos_sensor(0x1A, 0x98);
        write_cmos_sensor(0x03, 0x84);

        write_cmos_sensor(0x4F, 0xBB);  // 50Hz banding AEC 8 LSBs
        write_cmos_sensor(0x50, 0x9C);  // 60Hz banding AEC 8 LSBs
        write_cmos_sensor(0x5A, 0x57);  // 50/60Hz banding AEC maximum steps
        write_cmos_sensor(0x6D, 0x80);  // reserved
//        write_cmos_sensor(0x3D, 0x34);  // PLL/divider setting
        write_cmos_sensor(0x3D, 0x38);  // PLL/divider setting

        write_cmos_sensor(0x39, 0x02);  // PWCOM1, reserved
        write_cmos_sensor(0x35, 0x88);  // reserved
        iTemp = read_cmos_sensor(0x22); // ANCOM3
        write_cmos_sensor(0x22, iTemp & 0xEF);

        write_cmos_sensor(0x37, 0x40);  // reserved
        write_cmos_sensor(0x23, 0x00);  // reserved
        write_cmos_sensor(0x34, 0xA0);  // ARCOM2, reserved
        write_cmos_sensor(0x36, 0x1A);  // reserved
        write_cmos_sensor(0x06, 0x02);  // reserved
        write_cmos_sensor(0x07, 0xC0);  // reserved
        write_cmos_sensor(0x0D, 0xB7);  // reserved
        write_cmos_sensor(0x0E, 0x01);  // reserved
        write_cmos_sensor(0x4C, 0x00);  // reserved

#endif
        if ((pImageWindow->image_target_width <= IMAGE_SENSOR_FULL_WIDTH) &&
            (pImageWindow->image_target_height <= IMAGE_SENSOR_FULL_WIDTH)) {
            if (pImageWindow->digital_zoom_factor >= (ISP_DIGITAL_ZOOM_INTERVAL << 1) || bMetaMode == KAL_TRUE) {
                // capture >= 2x zoom
                if (bMetaMode == KAL_TRUE) {
                    SET_TG_PIXEL_CLK_DIVIDER(7);    // PCLK = 6MHz
                    SET_CMOS_DATA_LATCH(4);
                    write_cmos_sensor(PAGE_SETTING_REG, 0x01);
                    write_cmos_sensor(0x11, 0x03);
                    iDummyPixels = 400;
                    iDummyLines = 0;
                }else {
                    SET_TG_PIXEL_CLK_DIVIDER(15);    // PCLK = 6MHz
                    SET_CMOS_DATA_LATCH(8);
                    write_cmos_sensor(PAGE_SETTING_REG, 0x01);
                    write_cmos_sensor(0x11, 0x07);
                    iDummyPixels = 100;
                    iDummyLines = 0;
                }
            }else if (pImageWindow->digital_zoom_factor == ISP_DIGITAL_ZOOM_INTERVAL) {
                // capture w/o zoom
                SET_TG_PIXEL_CLK_DIVIDER(3);    // PCLK = 12MHz
                SET_CMOS_DATA_LATCH(2);
                write_cmos_sensor(PAGE_SETTING_REG, 0x01);
                write_cmos_sensor(0x11, 0x01);
                iDummyPixels = 115;
                iDummyLines = 0;
            }else { // 1x < zoom factor < 2x
                SET_TG_PIXEL_CLK_DIVIDER(3);    // PCLK = 12MHz
                SET_CMOS_DATA_LATCH(2);
                write_cmos_sensor(PAGE_SETTING_REG, 0x01);
                write_cmos_sensor(0x11, 0x01);
                iDummyPixels = 1725;
                iDummyLines = 0;
            }
        }else { // 3M capture mode
            if (pImageWindow->digital_zoom_factor >= (ISP_DIGITAL_ZOOM_INTERVAL << 1) || bMetaMode == KAL_TRUE) {
                SET_TG_PIXEL_CLK_DIVIDER(7);    // PCLK = 6MHz
                SET_CMOS_DATA_LATCH(4);
                write_cmos_sensor(PAGE_SETTING_REG, 0x01);
                write_cmos_sensor(0x11, 0x03);
                iDummyPixels = 70;
                iDummyLines = 0;
            }else { // capture < 2x digital zoom
                SET_TG_PIXEL_CLK_DIVIDER(3);    // PCLK = 12MHz
                SET_CMOS_DATA_LATCH(2);
                write_cmos_sensor(PAGE_SETTING_REG, 0x01);
                write_cmos_sensor(0x11, 0x01);
                iDummyPixels = 0;
                iDummyLines = 0;
            }
        }

        switch (pSensorConfigData->image_mirror) {
        case IMAGE_NORMAL:
            iStartX = 4;
#ifdef OV_PROCESSING_RAW
            iStartY = 6;
#else
            iStartY = 4;
#endif
            break;

        case IMAGE_HV_MIRROR:
            iStartX = 4;
#ifdef OV_PROCESSING_RAW
            iStartY = 5;
#else
            iStartY = 4;
#endif
        break;

        default:
            ASSERT(0);
        }

        iGrabWidth = IMAGE_SENSOR_FULL_WIDTH;
        iGrabHeight = IMAGE_SENSOR_FULL_HEIGHT;

        iCapture_Column_Pixel = FULL_PERIOD_PIXEL_NUMS + iDummyPixels;
        iCP_PCLK_Div = ((DRV_Reg32(ISP_TG_PHASE_COUNTER_REG) >> 4) & 0x0000000F) + 1;
        iShutter = iShutter * g_iPreview_Column_Pixel / iCapture_Column_Pixel;
        iShutter = iShutter * 2 * g_iPV_PCLK_Divider / iCP_PCLK_Div;    // x2 for SVGA -> UXGA switch
    }

    // 1. To force OV2640 re-calibrate ABLC by chaning global gain
    // 2. Skip at least one frame after chaing global gain
    iTemp = read_cmos_sensor(0x00);
    if (bMetaMode == KAL_FALSE) {   // normal capture mode
        write_cmos_sensor(0x00, iTemp % 2 ? --iTemp : ++iTemp);
    }else { // CCT mode
        // 1. CCT mode needs linearity test, gain must be kept constant
        // 2. CCT mode doesn't care capture speed
        write_cmos_sensor(0x00, iTemp % 2 ? --iTemp : ++iTemp);
        kal_sleep_task(250);    // wait at least one frame
        iTemp = read_cmos_sensor(0x00);
        kal_sleep_task(250);    // wait at least one frame
        write_cmos_sensor(0x00, iTemp % 2 ? --iTemp : ++iTemp);
    }

    //compensate OV2640 ABLC for every frame
    write_cmos_sensor(PAGE_SETTING_REG, 0x01);
    write_cmos_sensor(0x71, 0x96);

    OV2640_SetDummy(iDummyPixels, iDummyLines);
    write_OV2640_shutter(iShutter);

    pImageWindow->grab_start_x = iStartX;
    pImageWindow->grab_start_y = iStartY;
    pImageWindow->exposure_window_width = iGrabWidth;
    pImageWindow->exposure_window_height = iGrabHeight;
}   /*  OV2640_Capture  */

/*************************************************************************
* FUNCTION
*	write_OV2640_reg
*
* DESCRIPTION
*	This function set the register of OV2640.
*
* PARAMETERS
*	addr : the register index of OV2640
*  para : setting parameter of the specified register of OV2640
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void write_OV2640_reg(kal_uint32 iAddr, kal_uint32 iPara)
{
    kal_sleep_task(1);
    page_write_cmos_sensor(iAddr, iPara);
}   /*  write_OV2640_reg()  */

/*************************************************************************
* FUNCTION
*	read_cmos_sensor
*
* DESCRIPTION
*	This function read parameter of specified register from OV2640.
*
* PARAMETERS
*	addr : the register index of OV2640
*
* RETURNS
*	the data that read from OV2640
*
* GLOBALS AFFECTED
*
*************************************************************************/
kal_uint32 read_OV2640_reg(kal_uint32 iAddr)
{
    kal_sleep_task(1);
    return page_read_cmos_sensor(iAddr);
}   /*  read_OV2640_reg()   */

/*************************************************************************
* FUNCTION
*	OV2640_SetShutter
*
* DESCRIPTION
*	This function set e-shutter of OV2640 to change exposure time.
*
* PARAMETERS
*	shutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void OV2640_SetShutter(kal_uint16 iShutter)
{
    g_iExpLines = iShutter;
    write_OV2640_shutter(iShutter);
}   /*  OV2640_SetShutter   */

/*************************************************************************
* FUNCTION
*	OV2640_SetGain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	gain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
kal_uint16 OV2640_SetGain(kal_uint16 iGain)
{
    const kal_uint16 iBaseGain = Reg2Gain(camera_para.SENSOR.cct[GLOBAL_GAIN_INDEX].para);
    const kal_uint16 iGain2Set = iGain * iBaseGain / BASEGAIN;
    kal_uint8 iReg = Gain2Reg(iGain2Set);

    write_cmos_sensor(PAGE_SETTING_REG, 0x01);
    write_cmos_sensor(0x00, iReg);

    return Reg2Gain(iReg) * BASEGAIN / iBaseGain;

#if 0
	sensor_global_gain=(iGain * sensor_gain_base) / BASEGAIN;
	write_OV2640_gain(sensor_global_gain);
	sensor_global_gain=(sensor_global_gain*BASEGAIN)/sensor_gain_base;

	return sensor_global_gain;
#endif
}   /*  OV2640_SetGain  */

/*************************************************************************
* FUNCTION
*	OV2640_NightMode
*
* DESCRIPTION
*	This function enable/disable night mode of OV2640.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void OV2640_NightMode(kal_bool bEnable)
{
    // Night mode and normal mode switch effect is switched by different AE tables now
}   /*  OV2640_NightMode    */

void OV2640_Set_Flashlight(kal_bool bEnable)
{
    flashlight_power_on(bEnable);
}   /*  OV2640_Set_Flashlight   */

/*************************************************************************
* FUNCTION
*	image_sensor_func_OV2640
*
* DESCRIPTION
*	OV2640 Image Sensor functions struct.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
image_sensor_func_struct image_sensor_func_OV2640=
{
	OV2640_Init,
	OV2640_GetID,
	OV2640_GetSize,
	OV2640_GetPeriod,
	OV2640_Preview,
	OV2640_Capture,
	write_OV2640_reg,
	read_OV2640_reg,
	OV2640_SetShutter,
	OV2640_NightMode,
	OV2640_PowerOff,
	OV2640_SetGain,
	OV2640_Set_Flashlight
};  /*  image_sensor_func_OV2640    */

/*************************************************************************
* FUNCTION
*	image_sensor_func_config
*
* DESCRIPTION
*	This function maps the camera module function API structure.
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void image_sensor_func_config(void)
{
    image_sensor_func = &image_sensor_func_OV2640;
}   /*  image_sensor_func_config    */

// write camera_para to sensor register
void camera_para_to_sensor(void)
{
	kal_uint32	i;
	
	for(i=0; 0xFFFFFFFF!=camera_para.SENSOR.reg[i].addr; i++)
	{
		write_OV2640_reg(camera_para.SENSOR.reg[i].addr, camera_para.SENSOR.reg[i].para);
	}
	for(i=FACTORY_START_ADDR; 0xFFFFFFFF!=camera_para.SENSOR.reg[i].addr; i++)
	{
		write_OV2640_reg(camera_para.SENSOR.reg[i].addr, camera_para.SENSOR.reg[i].para);
	}
	for(i=0; i<CCT_END_ADDR; i++)
	{
		write_OV2640_reg(camera_para.SENSOR.cct[i].addr, camera_para.SENSOR.cct[i].para);
	}
}

// update camera_para from sensor register
void sensor_to_camera_para(void)
{
	kal_uint32	i;
	
	for(i=0; 0xFFFFFFFF!=camera_para.SENSOR.reg[i].addr; i++)
	{
		camera_para.SENSOR.reg[i].para = read_OV2640_reg(camera_para.SENSOR.reg[i].addr);
	}
	for(i=FACTORY_START_ADDR; 0xFFFFFFFF!=camera_para.SENSOR.reg[i].addr; i++)
	{
		camera_para.SENSOR.reg[i].para = read_OV2640_reg(camera_para.SENSOR.reg[i].addr);
	}
	
}

//------------------------Engineer mode---------------------------------

void  get_sensor_group_count(kal_int32* sensor_count_ptr)
{
   *sensor_count_ptr=GROUP_TOTAL_NUMS;
   
	return;
}

void get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
{
   switch (group_idx)
   {
      	case PRE_GAIN:
			sprintf(group_name_ptr, "CCT");
			*item_count_ptr = 3;
		break;
		default:
		   ASSERT(0);
	}
}

void get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, ENG_sensor_info* info_ptr)
{
	kal_uint8 temp_reg;
	write_cmos_sensor(PAGE_SETTING_REG,0x01);
	switch (group_idx)
	{
		
		case PRE_GAIN:
			switch (item_idx)
			{
				case 0:
				  sprintf(info_ptr->item_name_ptr,"Pregain-R");
				  
				  temp_reg = read_OV2640_reg(camera_para.SENSOR.cct[PRE_GAIN_RB_INDEX].addr);
				  temp_reg &= 0x0C;
				  temp_reg >>= 2; 
				  
				  if(temp_reg==0)
				      info_ptr->item_value=1000;
				  else if(temp_reg==1)
				      info_ptr->item_value=1250;
				  else if(temp_reg==2)
				      info_ptr->item_value=1500;
				  else if(temp_reg==3)
				      info_ptr->item_value=1750;
				  
				  info_ptr->is_true_false=KAL_FALSE;
				  info_ptr->is_read_only=KAL_FALSE;
				  info_ptr->is_need_restart=KAL_FALSE;
				  info_ptr->min=1000;
				  info_ptr->max=1875;
				  
				break; 
				case 1:
				  sprintf(info_ptr->item_name_ptr,"Pregain-Gr");
				  
				  
				  temp_reg = read_OV2640_reg(camera_para.SENSOR.cct[PRE_GAIN_G_INDEX].addr);
				  temp_reg &= 0x03;

				  
				  if(temp_reg==0)
				      info_ptr->item_value=1000;
				  else if(temp_reg==1)
				      info_ptr->item_value=1250;
				  else if(temp_reg==2)
				      info_ptr->item_value=1500;
				  else if(temp_reg==3)
				      info_ptr->item_value=1750;
				      
				  info_ptr->is_true_false=KAL_FALSE;
				  info_ptr->is_read_only=KAL_FALSE;
				  info_ptr->is_need_restart=KAL_FALSE;
				  info_ptr->min=1000;
				  info_ptr->max=1875;
				break;
				case 2:
				  sprintf(info_ptr->item_name_ptr,"SENSOR_BASEGAIN");
				  
//				  temp_reg = read_OV2640_gain();
                  temp_reg = Reg2Gain(camera_para.SENSOR.cct[GLOBAL_GAIN_INDEX].para);
				  
				  info_ptr->item_value=(temp_reg*1000)/BASEGAIN;
				  info_ptr->is_true_false=KAL_FALSE;
				  info_ptr->is_read_only=KAL_FALSE;
				  info_ptr->is_need_restart=KAL_FALSE;
				  info_ptr->min=1000;
				  info_ptr->max=16000;
				break;
				default:
				   ASSERT(0);		
			}
		break;
		
		default:
			ASSERT(0); 
	}
}

kal_bool set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 item_value)
{
   kal_uint8 temp_reg;
   kal_uint16 temp_gain;
   
   write_cmos_sensor(PAGE_SETTING_REG,0x01);
   
   switch (group_idx)
	{
		case PRE_GAIN:
			switch (item_idx)
			{
				case 0:
				  temp_reg = read_OV2640_reg(camera_para.SENSOR.cct[PRE_GAIN_RB_INDEX].addr);
				  temp_reg &= ~0x0C;
				  
				  if(item_value>=1000 && item_value<=1125)
				      temp_reg |= 0x00;
				  else if(item_value>1125 && item_value<=1375)
				      temp_reg |= 0x04;
				  else if(item_value>1375 && item_value<=1625)
				      temp_reg |= 0x08;
				  else if(item_value>1625 && item_value<=1875)
				      temp_reg |= 0x0C;
				  else
				  		return KAL_FALSE;
				  		
				  camera_para.SENSOR.cct[PRE_GAIN_RB_INDEX].para = temp_reg;
				  write_OV2640_reg(camera_para.SENSOR.cct[PRE_GAIN_RB_INDEX].addr,temp_reg);
				break;
				case 1:
				  temp_reg = read_OV2640_reg(camera_para.SENSOR.cct[PRE_GAIN_G_INDEX].addr);
				  temp_reg &= ~0x03;
				  
				  if(item_value>=1000 && item_value<=1125)
				      temp_reg |= 0x00;
				  else if(item_value>1125 && item_value<=1375)
				      temp_reg |= 0x01;
				  else if(item_value>1375 && item_value<=1625)
				      temp_reg |= 0x02;
				  else if(item_value>1625 && item_value<=1875)
				      temp_reg |= 0x03;
				  else
				  		return KAL_FALSE;
				  
				  camera_para.SENSOR.cct[PRE_GAIN_G_INDEX].para = temp_reg;
				  write_OV2640_reg(camera_para.SENSOR.cct[PRE_GAIN_G_INDEX].addr,temp_reg);
				break;
				case 2:
				  temp_gain = (item_value * BASEGAIN) / 1000;
                  temp_reg = Gain2Reg(temp_gain);				

				  camera_para.SENSOR.cct[GLOBAL_GAIN_INDEX].para = temp_reg;
				  write_OV2640_reg(camera_para.SENSOR.cct[GLOBAL_GAIN_INDEX].addr,temp_reg);
				  break;
				default:
				   ASSERT(0);	
			}
		break;
		
		default:
		   ASSERT(0);
	}
	
	return KAL_TRUE;
}
