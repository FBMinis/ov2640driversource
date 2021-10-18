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
 *   image_sensor.h
 *
 * Project:
 * --------
 *   MT6219
 *
 * Description:
 * ------------
 *   CMOS sensor header file
 *
 * Author:
 * -------
 *		Ian Cheng (mtk00827) //This is OV2641
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
#ifndef _IMAGE_SENSOR_H
#define _IMAGE_SENSOR_H

    #include "isp_if.h"

    /*****************************************************************************/
    #define OV_PROCESSING_RAW
    #define OV_WPC_BPC_CTRL 0x50 //OV 0x87 control register

    #define OV2640_QVGA_LCM     // define this option when LCM size >= QVGA resolution
//    #define MAX_ZOOM_FACTOR_2X  // zoom factor <= 2x. If this option is defined, the driver
                                // supports up to 2x zoom factor. If not defined, it supports
                                // up to 3x zoom factor

    #if defined(MT6228) || defined(MT6229) || defined(MT6230)
        #define OV2640_MPEG4_QCIF_VIDEO_30FPS   // this option enables 30fps QCIF MPEG4 video mode
    #endif
    /*****************************************************************************/
    //------------------------Engineer mode---------------------------------

    #define FACTORY_START_ADDR 5

    typedef enum group_enum {
        PRE_GAIN,
        GROUP_TOTAL_NUMS
    } FACTORY_GROUP_ENUM;

    typedef enum register_index {
        CMMCLK_CURRENT_INDEX = FACTORY_START_ADDR,
        FACTORY_END_ADDR
    } FACTORY_REGISTER_INDEX;

    typedef enum cct_register_index {
        GLOBAL_GAIN_INDEX = 0,
        PRE_GAIN_RB_INDEX,
        PRE_GAIN_G_INDEX,
        CCT_END_ADDR
    } CCT_REGISTER_INDEX;

    typedef struct {
        kal_uint8 item_name_ptr[50];    // item name
        kal_int32 item_value;           // item value
        kal_bool is_true_false;         // is this item for enable/disable functions
        kal_bool is_read_only;          // is this item read only
        kal_bool is_need_restart;       // after set this item need restart
        kal_int32 min;                  // min value of item value	
        kal_int32 max;                  // max value of item value	
    } ENG_sensor_info; 

    // API FOR ENGINEER FACTORY MODE
    void get_sensor_group_count(kal_int32* sensor_count_ptr);
    void get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr);
    void get_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, ENG_sensor_info* info_ptr);
    kal_bool set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 item_value);

	//------------------------Engineer mode---------------------------------

    typedef struct {
        kal_uint32 addr;
        kal_uint32 para;
    } sensor_reg_struct;

    typedef struct {
        sensor_reg_struct reg[FACTORY_END_ADDR];
        sensor_reg_struct cct[CCT_END_ADDR];
    } sensor_data_struct;

    typedef struct {
        kal_uint16 width;
        kal_uint16 height;
    } sensor_resolution_struct;
	
    extern sensor_resolution_struct resolution_info;

    // write camera_para to sensor register 
    void camera_para_to_sensor(void);
    // update camera_para from sensor register 
    void sensor_to_camera_para(void);
    // config sensor callback function 
    void image_sensor_func_config(void);
    // Compact Image Sensor Module Power ON/OFF
    void cis_module_power_on(kal_bool on);
	
    // HW PRODUCE I2C SIGNAL TO CONTROL SENSOR REGISTER
    //#define HW_SCCB

    typedef enum _SENSOR_TYPE {
        CMOS_SENSOR = 0,
        CCD_SENSOR
    } SENSOR_TYPE;

    typedef struct {
        kal_uint16 id;
        SENSOR_TYPE type;
    } SensorInfo;

    /* MAXIMUM EXPLOSURE LINES USED BY AE */
    extern kal_uint16 MAX_EXPOSURE_LINES;
    extern kal_uint8  MIN_EXPOSURE_LINES;

    /* DEFINITION USED BY CCT */
    extern SensorInfo	g_CCT_MainSensor;
    extern kal_uint8	g_CCT_FirstGrabColor;

    typedef enum _OV2640_OP_TYPE_ {
        OV2640_MODE_NONE,
        OV2640_MODE_PREVIEW,
        OV2640_MODE_CAPTURE,
        OV2640_MODE_QCIF_VIDEO,
        OV2640_MODE_CIF_VIDEO
    } OV2640_OP_TYPE;

    extern OV2640_OP_TYPE g_iOV2640_Mode;

    /* MAX/MIN FRAME RATE (FRAMES PER SEC.) */
    #define MAX_FRAME_RATE  (15)    // Limitation for MPEG4 Encode Only
    #define MIN_FRAME_RATE  (12)    // Limitation for Camera Preiview

    /* SENSOR PIXEL/LINE NUMBERS IN ONE PERIOD */
    #define FULL_PERIOD_PIXEL_NUMS  (1922)  // default pixel#(w/o dummy pixels) in UXGA mode
    #define FULL_PERIOD_LINE_NUMS   (1248)  // default line#(w/o dummy lines) in UXGA mode
    #define PV_PERIOD_PIXEL_NUMS    (1190)  // default pixel#(w/o dummy pixels) in SVGA mode
    #define PV_PERIOD_LINE_NUMS     (672)   // default line#(w/o dummy lines) in SVGA mode

    /* SENSOR EXPOSURE LINE LIMITATION */
    #define FULL_EXPOSURE_LIMITATION    (1248)
    #define PV_EXPOSURE_LIMITATION      (672)

    // SENSOR UXGA SIZE
    #define IMAGE_SENSOR_FULL_WIDTH     (1600)
    #define IMAGE_SENSOR_FULL_HEIGHT    (1200)

    // SENSOR VGA SIZE
    #define IMAGE_SENSOR_PV_WIDTH   (800)
    #define IMAGE_SENSOR_PV_HEIGHT  (600)

    // SETUP TIME NEED TO BE INSERTED
    #define IMAGE_SENSOR_PV_INSERTED_PIXELS (390)
    #define IMAGE_SENSOR_PV_INSERTED_LINES  (9 - 6)

    #define IMAGE_SENSOR_FULL_INSERTED_PIXELS   (248)
    #define IMAGE_SENSOR_FULL_INSERTED_LINES    (11 - 2)

    #if defined(MT6228) || defined(MT6229) || defined(MT6230)
        #ifdef OV2640_QVGA_LCM
            #ifdef MAX_ZOOM_FACTOR_2X
                #define OV2640_PV_DUMMY_PIXELS          (400)
                #define OV2640_VIDEO__CIF_DUMMY_PIXELS  (0)
                #define OV2640_VIDEO__QCIF_DUMMY_PIXELS (0)
            #else   // #ifdef MAX_ZOOM_FACTOR_2X
                #define OV2640_PV_DUMMY_PIXELS          (600)
                #define OV2640_VIDEO__CIF_DUMMY_PIXELS  (100)
                #define OV2640_VIDEO__QCIF_DUMMY_PIXELS (0)
            #endif
        #else   // #ifdef OV2640_QVGA_LCM
            #ifdef MAX_ZOOM_FACTOR_2X
                #define OV2640_PV_DUMMY_PIXELS          (400)
                #define OV2640_VIDEO__CIF_DUMMY_PIXELS  (0)
                #define OV2640_VIDEO__QCIF_DUMMY_PIXELS (0)
            #else   // #ifdef MAX_ZOOM_FACTOR_2X
                #define OV2640_PV_DUMMY_PIXELS          (600)
                #define OV2640_VIDEO__CIF_DUMMY_PIXELS  (100)
                #define OV2640_VIDEO__QCIF_DUMMY_PIXELS (0)
            #endif
        #endif
    #else   // #if defined(MT6228) || defined(MT6229) || defined(MT6230)
        #ifdef OV2640_QVGA_LCM
            #ifdef MAX_ZOOM_FACTOR_2X
                #define OV2640_PV_DUMMY_PIXELS          (400)
                #define OV2640_VIDEO__CIF_DUMMY_PIXELS  (0)
                #define OV2640_VIDEO__QCIF_DUMMY_PIXELS (0)
            #else   // #ifdef MAX_ZOOM_FACTOR_2X
                #define OV2640_PV_DUMMY_PIXELS          (600)
                #define OV2640_VIDEO__CIF_DUMMY_PIXELS  (100)
                #define OV2640_VIDEO__QCIF_DUMMY_PIXELS (0)
            #endif
        #else   // #ifdef OV2640_QVGA_LCM
            #ifdef MAX_ZOOM_FACTOR_2X
                #define OV2640_PV_DUMMY_PIXELS          (400)
                #define OV2640_VIDEO__CIF_DUMMY_PIXELS  (0)
                #define OV2640_VIDEO__QCIF_DUMMY_PIXELS (0)
            #else   // #ifdef MAX_ZOOM_FACTOR_2X
                #define OV2640_PV_DUMMY_PIXELS          (600)
                #define OV2640_VIDEO__CIF_DUMMY_PIXELS  (100)
                #define OV2640_VIDEO__QCIF_DUMMY_PIXELS (0)
            #endif
        #endif
    #endif  // #if defined(MT6228) || defined(MT6229) || defined(MT6230)

    // SENSOR READ/WRITE ID
    #define OV2640_WRITE_ID (0x60)
    #define OV2640_READ_ID  (0x61)

    // SENSOR CHIP VERSION
    #define OV2640_SENSOR_ID_2C (0x2642)
    #define OV2640_SENSOR_ID_2B (0x2641)

    #define PAGE_SETTING_REG    (0xFF)

    #define SET_RESET_CMOS_SENSOR_HIGH  (REG_ISP_CMOS_SENSOR_MODE_CONFIG |= REG_CMOS_SENSOR_RESET_BIT)
    #define SET_RESET_CMOS_SENSOR_LOW   (REG_ISP_CMOS_SENSOR_MODE_CONFIG &= ~REG_CMOS_SENSOR_RESET_BIT)
	
    #define SENSOR_I2C_DELAY    (0x20)

    #define I2C_START_TRANSMISSION \
    { \
        volatile kal_uint8 iJ; \
        SET_SCCB_CLK_OUTPUT; \
        SET_SCCB_DATA_OUTPUT; \
        SET_SCCB_CLK_HIGH; \
        SET_SCCB_DATA_HIGH; \
        for (iJ = 0; iJ < SENSOR_I2C_DELAY; iJ++); \
        SET_SCCB_DATA_LOW; \
        for (iJ = 0; iJ < SENSOR_I2C_DELAY; iJ++); \
        SET_SCCB_CLK_LOW; \
    }

    #define I2C_STOP_TRANSMISSION \
    { \
        volatile kal_uint8 iJ; \
        SET_SCCB_CLK_OUTPUT; \
        SET_SCCB_DATA_OUTPUT; \
        SET_SCCB_CLK_LOW; \
        SET_SCCB_DATA_LOW; \
        for (iJ = 0; iJ < SENSOR_I2C_DELAY; iJ++); \
        SET_SCCB_CLK_HIGH; \
        for (iJ = 0; iJ < SENSOR_I2C_DELAY; iJ++); \
        SET_SCCB_DATA_HIGH; \
    }

#endif /* _IMAGE_SENSOR_H */