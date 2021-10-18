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
 *   camera_para.c
 *
 * Project:
 * --------
 *   MT6219
 *
 * Description:
 * ------------
 *   Camera Parameter for PixArt VGA Sensor (OV9640)
 *
 ****************************************************************************/
#if !defined(ISP_SUPPORT)
	// DO NOT delete this section!!! 
	// When ISP_SUPPORT is not defined, NVRAM still need the dummy structure 
	// and default value to initialize NVRAM_EF_CAMERA_PARA_LID.             
	#include "camera_para.h"
	const nvram_camera_para_struct CAMERA_PARA_DEFAULT_VALUE={0};
#else
#include "drv_comm.h"
#include "stdio.h"
#include "isp_if.h"
#include "image_sensor.h"
#include "sccb.h"
#include "ae_awb.h"
#include "af.h"
#include "camera_para.h"
#include "med_api.h"

extern kal_uint32	eShutter;
//extern kal_bool MPEG4_30fps_mode;
nvram_camera_para_struct	camera_para;
nvram_camera_defect_struct	camera_defect;
static kal_uint16 Rgain_max_Local,Ggain_max_Local,Bgain_max_Local;
static kal_uint16 Rgain_PreGain,Ggain_PreGain,Bgain_PreGain;
static float Rgain_PreGain_Ratio,Bgain_PreGain_Ratio;
static kal_uint16 bParaAwb = KAL_TRUE;
static kal_uint16 bParaAwbBitFlag = 0x0000;
static kal_uint16 bParaAwbBitFlagPre = 0x0000;
#define LIGHT_SOURCE_BIT_NO4		0x10
#define LIGHT_SOURCE_BIT_NO1		0x02
#define LIGHT_SOURCE_BIT_NO1_H	0x20

#define AWB_PARA_OUTDOOR_AE_IDX_D65_H  	  90  //LV 13.5
#define AWB_PARA_OUTDOOR_AE_IDX_D65  	     69  //LV 11.5
#define AWB_PARA_OUTDOOR_AE_IDX_CWF 		  54  //LV 10
#define AWB_PARA_OUTDOOR_AE_IDX_CWF_LOW  	  44  //LV 9
#define AWB_PARA_OUTDOOR_THRES_I  	(float)(0.15)
#define AWB_PARA_OUTDOOR_THRES_I_H  	(float)(0.3)//(float)(0.3)
#define AWB_PARA_OUTDOOR_BLIMIT_I      (float)(0.2)//(float)(0.35)
#define AWB_PARA_OUTDOOR_BLIMIT_I_H  (float)(0.1)//(float)(0.2)
#define AWB_PARA_OUTDOOR_THRES_II  	(float)(0.0)
#define LIGHT_SOURCE_BIT_NO4_H (float)(0.6)
#define LIGHT_SOURCE_BIT_NO4_L  (float)(0.2)
// _Camera Parameter Structure START_
const nvram_camera_defect_struct CAMERA_DEFECT_DEFAULT_VALUE={0xFFFFFFFF};
const nvram_camera_para_struct CAMERA_PARA_DEFAULT_VALUE=
{	
	/* STRUCT: ISP */
	{
		/* ARRAY: ISP.reg[132] */
		{
			       0xA3020074, 0x0FFF0FFF, 0x006302E6, 0x00310212, 
			0x00000001, 0x00000000, 0x00000040, 0x0000000F, 
				0x00000000, 0x00000000, 0x00000000, 0x00000000, 
			0x80001100, 0x00B70080, 0x00B70080, 0x03070BF8, 
			0xFF000000, 0x00618940, 0x00000000, 0x00000000,
			0x00000000, 0x00000000, 0x161B1015, 0x1A270912,
			0x000D121B, 0x0D1A121B, 0x1A27121B, 0x0524051A,
			0x00000001, 0x0A041910, 0x00000014, 0x061FC805,
			0x41181802, 0x00000002, 0x24050F0F, 0x061F8224,
			0x003F272E, 0x84181604, 0x23170200, 0x0046A383, 
			0x008E4496, 0x008C8E3A, 0x00202020, 0x00000008, 
			0x00808080, 0x00000000, 0x00408030, 0x20200000, 
			0x00FF91B8, 0x00018080, 0x213C5578, 0x91A8C0D7, 
				0xE1EBF500, 0x00000000, 0x00000000, 0x00000000, 
				0x00000000, 0x00000000, 0x00000000, 0x00000000, 
				0x00000000, 0x00000000, 0x00000000, 0x00000000, 
				0x00000000, 0x00000000, 0x00000000, 0x00000000, 
			0x00000000, 0x00000000, 0x00000000, 0x00000000,
			0x245C5C24, 0x245C5C24, 0x12580258, 0x13200320,
				0x00325065, 0x7694AEC5, 0xDAE4EDF7, 0x32506576, 
				0x94AEC5DA, 0xE4EDF700, 0x32506576, 0x94AEC5DA, 
			0xE4EDF700, 0x00040000, 0x4001F900, 0x00000000, 
			0x4001FD00, 0x00550062, 0x11111111, 0x11111111,
				0x11111111, 0x11111111, 0x11111111, 0x11111111, 
			0x11111111, 0x11111110, 0x00000300, 0xF8000200, 
			0x96960000, 0x00000000, 0x00000000, 0x00000000, 
				0x60708090, 0xA0B0C0D0, 0xE0F00000, 0x00000000, 
				0x00000000, 0x00000000, 0x00000000, 0x00000000, 
				0x00000000, 0x00000000, 0x00000000, 0x00000000, 
			0x00000000, 0x123B72FF, 0xB4928900, 0x03204163, 
			0x869AA9B7, 0xC6D4E3F1, 0x20416386, 0x96A5B4C4, 
			0xD3E2F100, 0x20406285, 0x95A4B4C3, 0xD2E2F100, 
				0x00000000, 0x00000000, 0x00000000, 0x00000000
		}
	}, 
	/* STRUCT: PREDGAMMA */
	{
		/* PREDGAMMA.gamma_select */ 0x00, 		/* ARRAY: PREDGAMMA.gamma[9][11] */
		{
			/* ARRAY: PREDGAMMA.gamma[0][11] */
			{
				0x28, 0x4E, 0x67, 0x78, 0x91, 0xA8, 0xC0, 0xD7, 
				0xE1, 0xEB, 0xF5
			},

			/* ARRAY: PREDGAMMA.gamma[1][11] */
	{
				0x20, 0x3C, 0x55, 0x69, 0x8D, 0xA8, 0xC0, 0xD7, 
				0xE1, 0xEB, 0xF5
			},

			/* ARRAY: PREDGAMMA.gamma[2][11] */
			{
				0x14, 0x28, 0x46, 0x64, 0x91, 0xB4, 0xCE, 0xE0, 
				0xE8, 0xF0, 0xF8
			},

			/* ARRAY: PREDGAMMA.gamma[3][11] */
			{
				0x49, 0x64, 0x78, 0x88, 0xA4, 0xBA, 0xCE, 0xE0, 
				0xE8, 0xF0, 0xF8
			},

			/* ARRAY: PREDGAMMA.gamma[4][11] */
			{
				0x29, 0x59, 0x75, 0x88, 0xA4, 0xBA, 0xCE, 0xE0, 
				0xE8, 0xF0, 0xF8
			},

			/* ARRAY: PREDGAMMA.gamma[5][11] */
			{
				0x14, 0x30, 0x50, 0x69, 0x87, 0xA5, 0xBE, 0xD7, 
				0xE1, 0xEB, 0xF5
			},

			/* ARRAY: PREDGAMMA.gamma[6][11] */
			{
				0x17, 0x30, 0x53, 0x74, 0x99, 0xAB, 0xBE, 0xD7, 
				0xE1, 0xEB, 0xF5
			},

			/* ARRAY: PREDGAMMA.gamma[7][11] */
			{
				0x30, 0x49, 0x5D, 0x6F, 0x8D, 0xA8, 0xC0, 0xD7, 
				0xE1, 0xEB, 0xF5
			},

			/* ARRAY: PREDGAMMA.gamma[8][11] */
			{
				0x10, 0x20, 0x30, 0x40, 0x60, 0x80, 0xA0, 0xC0, 
				0xD0, 0xE0, 0xF0
			}
		}
	},

	/* STRUCT: Comp */
	{
		/* STRUCT: shading_cap */
		{
			0x1472660A, 0xB9928500, 0x03204163, 0x869AA9B7, 
			0xC6D4E3F1, 0x20416386, 0x96A5B4C4, 0xD3E2F100, 
			0x20406283, 0x93A3B2C2, 0xD1E1F000
		},
		/* STRUCT: autodefect_pre_low */		
		{
				0xFC000000, 0x14142020
		},
		/* STRUCT: autodefect_cap_nor */		
		{
				0xFC000000, 0xA0A02020
		},
		/* STRUCT: autodefect_cap_low */		
		{
				0xFC000000, 0x14142020
		},
		/* STRUCT: shading_spare_1 */
		{
				0x00000000, 0x00000000, 0x00204060, 0x8090A0B0,
				0xC0D0E0F0, 0x20406080, 0x90A0B0C0, 0xD0E0F000,
				0x20406080, 0x90A0B0C0, 0xD0E0F000
		},
		/* STRUCT: shading_spare_2 */
		{
				0x00000000, 0x00000000, 0x00204060, 0x8090A0B0,
				0xC0D0E0F0, 0x20406080, 0x90A0B0C0, 0xD0E0F000,
				0x20406080, 0x90A0B0C0, 0xD0E0F000
		}		
	},

	/* STRUCT: AE */
	{
		/* AE.iniShutter */ 0x0118, /* AE.TargetLum */ 0x69, /* AE.StepperEV */ 0x02, /* AE.iniExpoIdx */ 0x28
	},

	/* STRUCT: AWB */
	{
		/* ARRAY: AWB.LightSource[5][5] */
		{
			/* ARRAY: AWB.LightSource[0][5] */
			{
				0x1D4C, 0x00E0, 0x0080, 0x0087, 0x0080
			},

			/* ARRAY: AWB.LightSource[1][5] */
			{
				0x1964, 0x00E0, 0x0080, 0x0087, 0x0080
			},

			/* ARRAY: AWB.LightSource[2][5] */
			{
				0x1130, 0x00AA, 0x0080, 0x00C0, 0x0080
			},

			/* ARRAY: AWB.LightSource[3][5] */
			{
				0x0ED8, 0x009E, 0x0080, 0x00BD, 0x0080
			},

			/* ARRAY: AWB.LightSource[4][5] */
			{
				0x0AF0, 0x009B, 0x0080, 0x00D5, 0x0080
		   },
                       /* Manual AWB gain */
			{
				0x1130, 0x0087, 0x0080, 0x00D0, 0x0080
		   }
		},

/* AWB.AWB_rgain_max */ 0x009D, /* AWB.AWB_ggain_max */ 0x00B2, /* AWB.AWB_bgain_max */ 0x0120
	},
	/* STRUCT: SENSOR */
	{
		/* ARRAY: SENSOR.reg[34] */
		{
			 { 0xFFFFFFFF, 0x00 } ,{ 0xFFFFFFFF, 0x00 } ,{ 0xFFFFFFFF, 0x00 } 
			,{ 0xFFFFFFFF, 0x00 } ,{ 0xFFFFFFFF, 0x00 } 
			,{ 0xFFFFFFFF, ISP_DRIVING_8MA }
			//------------------------Engineer mode---------------------------------
		},
		//------------------------CCT mode---------------------------------
		{
		   { 0x00, 0x00 } ,{ 0x69, 0x40 } ,{ 0x69, 0x90 }
		}
		//------------------------CCT mode---------------------------------
	}
};
// _Camera_Parameter_Structure_END_

void init_camera_operation_para(camera_operation_para_struct *oper_data)
{
	oper_data->ae_mode= QUALITY_PRIORITY;					/* QUALITY_PRIORITY, FRAME_RATE_PRIORITY */
	oper_data->pregain_mode=ISP_SENSOR_BOTH;							/* ISP_ONLY, SENSOR_ONLY, ISP_SENSOR_BOTH */
	oper_data->gain_priority=SENSOR_GAIN_PRIORITY;				/* ISP_GAIN_PRIORITY, SENSOR_GAIN_PRIORITY */
	oper_data->enable_cap_shutter_compensate=KAL_FALSE;		/* KAL_TRUE, KAL_FALSE */
		oper_data->shutter_compensate_max=2*BASEGAIN;

	oper_data->isp_pregain_max=2*BASEGAIN;
	oper_data->sensor_pregain_max=4*BASEGAIN;               
	oper_data->pregain_compensate_max=2*BASEGAIN;

	oper_data->preview_display_wait_frame=2;

	oper_data->ae_smooth_upper_bound=130;
	oper_data->ae_smooth_lower_bound=60;

	oper_data->ae_awb_cal_period=4;       
	oper_data->ae_setting_gain_delay_frame=3;
	oper_data->ae_setting_sensor_gain_delay_frame=2; 
	oper_data->ae_setting_shut_delay_frame=1;            
	oper_data->ae_setting_cal_delay_frame=0;
	
   oper_data->capture_delay_frame=3;			/* Switch Preview to Capture */
   oper_data->preview_delay_frame=3;			/* Switch Capture to Preview */
	
	oper_data->ae_lowlight_threshold=10;				/* low light threshold of luminance for camera */
	oper_data->ae_lowlight_off_threshold=15;			/* low light turn off threshold of luminance for camera */
	oper_data->ae_video_lowlight_threshold=10;		/* low light threshold of luminance for video */
	oper_data->ae_video_lowlight_off_threshold=15;	/* low light turn off threshold of luminance for video */
	
	oper_data->ae_high_banding_target_enable = KAL_FALSE;	/* Disable banding taeget */
	oper_data->extreme_CT_fixWB_enable = 0;/*enable/disable extreme low and high color temperature to limit to A and D75*/	
	oper_data->outdoorIdx = 120;	/*AE index for outdoor condition,120 means 12ev*/ 
	oper_data->outdoor_fixWB_enable = 0;      /*enable/disable outdoor fix WB*/ 
	oper_data->out_Rgain = 131; 	/*outdoor Rgain*/
	oper_data->out_GRgain = 128; 	/*outdoor GRgain*/
	oper_data->out_Bgain =129; 		/*outdoor Bgain*/
	oper_data->out_GBgain = 128; 	/*outdoor GBgain*/
	
	/* AE smooth setting for preview & video */
	oper_data->AE_Smooth_Enable = KAL_TRUE;
	oper_data->Smooth_Filter_Para_Preview.AE_Smooth_Median_Filter_Tape = 3;//Tape no of median filter
	oper_data->Smooth_Filter_Para_Preview.AE_Smooth_Normal_Last_Weight = 100;//Weighting for last set
	oper_data->Smooth_Filter_Para_Preview.AE_Smooth_Fast_Check_Count = 1;//Fast mode check count
	oper_data->Smooth_Filter_Para_Preview.AE_Smooth_Fast_Last_Weight = 5;//Weighting for last set
	oper_data->Smooth_Filter_Para_Preview.AE_Smooth_Speed_Check_Count = 4;//Fast mode check count
	oper_data->Smooth_Filter_Para_Preview.AE_Smooth_Speed_Last_Weight = 48;//Weighting for last set
	oper_data->Smooth_Filter_Para_Preview.AE_Smooth_Ramp_Check_Count = 8;//Fast mode check count
	oper_data->Smooth_Filter_Para_Preview.AE_Smooth_Ramp_Last_Weight = 48;//Weighting for last set
	oper_data->Smooth_Filter_Para_Video.AE_Smooth_Median_Filter_Tape = 3;//Tape no of median filter
	oper_data->Smooth_Filter_Para_Video.AE_Smooth_Normal_Last_Weight = 110;//Weighting for last set
	oper_data->Smooth_Filter_Para_Video.AE_Smooth_Fast_Check_Count = 2;//Fast mode check count
	oper_data->Smooth_Filter_Para_Video.AE_Smooth_Fast_Last_Weight = 5;//Weighting for last set
	oper_data->Smooth_Filter_Para_Video.AE_Smooth_Speed_Check_Count = 4;//Fast mode check count
	oper_data->Smooth_Filter_Para_Video.AE_Smooth_Speed_Last_Weight = 96;//Weighting for last set
	oper_data->Smooth_Filter_Para_Video.AE_Smooth_Ramp_Check_Count = 8;//Fast mode check count
	oper_data->Smooth_Filter_Para_Video.AE_Smooth_Ramp_Last_Weight = 96;//Weighting for last set
	
#if (defined(HORIZONTAL_CAMERA))
	camera_horizontal_flag=1;
#else
	camera_horizontal_flag=0;
#endif	

    oper_data->flashlight_mode = FLASHLIGHT_LED_PEAK;    
    oper_data->flashlight_delta_main_lum = 192;    // 3*64(3X)        
}

#ifdef AF_SUPPORT
void init_af_operation_para(af_operation_para_struct *oper_data)
{
	oper_data->manual_focus_step				= 5;
	oper_data->af_auto_range_start_idx		= 15;//10;
	oper_data->af_auto_range_end_idx			= 1;
	oper_data->af_normal_range_start_idx	= 10;
	oper_data->af_normal_range_end_idx		= 4;
	oper_data->af_macro_range_start_idx		= 15;
	oper_data->af_macro_range_end_idx		= 1;
	oper_data->af_infinite_range_start_idx	= 3;
	oper_data->af_infinite_range_end_idx	= 0;

		oper_data->isp_pregain_max=1.1*BASEGAIN;				  
	oper_data->sensor_pregain_max=4*BASEGAIN;               
		oper_data->pregain_compensate_max=2*BASEGAIN;				

	oper_data->preview_display_wait_frame=4;       

	oper_data->ae_smooth_upper_bound=140;            
		oper_data->ae_smooth_lower_bound=60;			

	oper_data->ae_awb_cal_period=4;       
	oper_data->ae_setting_gain_delay_frame=3;
	oper_data->ae_setting_sensor_gain_delay_frame=2;
	oper_data->ae_setting_shut_delay_frame=1;            
	oper_data->ae_setting_cal_delay_frame=0;
	
	oper_data->capture_delay_frame=2;			/* Switch Preview to Capture */
	oper_data->preview_delay_frame=3;			/* Switch Capture to Preview */
	
	oper_data->ae_lowlight_threshold=10;				/* low light threshold of luminance for camera */
	oper_data->ae_lowlight_off_threshold=15;			/* low light turn off threshold of luminance for camera */
	oper_data->ae_video_lowlight_threshold=10;		/* low light threshold of luminance for video */
	oper_data->ae_video_lowlight_off_threshold=15;	/* low light turn off threshold of luminance for video */
	
	oper_data->ae_high_banding_target_enable = KAL_FALSE;	/* Disable banding taeget */
}
#endif

void set_camera_mode_para(kal_uint8 mode)
{
	kal_uint16 sensor_width,sensor_height;	
	kal_uint16 max_expo_width,max_expo_line;	
	image_sensor_func->get_sensor_size(&sensor_width,&sensor_height);	
	image_sensor_func->get_sensor_period(&max_expo_width, &max_expo_line);
	switch(mode)
	{
		case CAMERA_PARA_PREVIEW_MODE:
			/* isp preview parameter */
			SET_PREPROCESS_PIXEL_LIMIT(0xA0);
			SET_INTERPOLATION_THRE_SM(0x0A);
			DISABLE_AUTO_DEFECT_ALL;
			DISABLE_Y_LPF;//6228 can not enable LPF in ckl/2
			DISABLE_C_LPF;			
			ENABLE_RGB_EDGE_GAIN;
			 DISABLE_Y_EDGE;
		break;
		case CAMERA_PARA_CAPTURE_MODE:
			/* isp capture parameter */
			SET_PREPROCESS_PIXEL_LIMIT(0xA0);			
			/* Check Night Mode */
			if((isp_preview_config_data.night_mode==KAL_TRUE)||
				(eShutter>=max_expo_width))
			{
				SET_INTERPOLATION_THRE_SM(0x10);								
				DISABLE_AUTO_DEFECT_ALL;
				ENABLE_RGB_EDGE_GAIN;										
				ENABLE_Y_LPF;
				ENABLE_C_LPF;	
				 DISABLE_Y_EDGE;
			}
			else
			{
				/* Check small size capture */
				if ((exposure_window.image_target_width<(IMAGE_SENSOR_PV_WIDTH>>1))&&
					 (exposure_window.image_target_height<(IMAGE_SENSOR_PV_HEIGHT>>1))) 
				{
					SET_INTERPOLATION_THRE_SM(0x10);							
					DISABLE_RGB_EDGE_GAIN;																			
					DISABLE_Y_LPF;
					DISABLE_C_LPF;
				}
				else
				{
					SET_INTERPOLATION_THRE_SM(0x0A);						
					ENABLE_RGB_EDGE_GAIN;										
					DISABLE_Y_LPF;
					DISABLE_C_LPF;									
				}					
				DISABLE_AUTO_DEFECT_ALL;
			}							
		break;
		case CAMERA_PARA_NIGHT_MODE:
			/* isp night mode parameter */
			SET_INTERPOLATION_THRE_SM(0x0F);					
			DISABLE_AUTO_DEFECT_ALL;
			DISABLE_RGB_EDGE_GAIN;
//            if(MPEG4_30fps_mode==KAL_FALSE)
//            {
//    			ENABLE_Y_LPF;
//	    		ENABLE_C_LPF;
//            }
			DISABLE_VSUP;											
		break;
		case CAMERA_PARA_AUTO_LOWLIGHT_MODE:
			/* isp auto lowlight parameter */
			SET_INTERPOLATION_THRE_SM(0x0F);							
			DISABLE_AUTO_DEFECT_ALL;
			ENABLE_RGB_EDGE_GAIN;
			DISABLE_Y_LPF;
			DISABLE_C_LPF;				
		break;
		case CAMERA_PARA_VIDEO_MODE:
			/* parameter for video */
			SET_INTERPOLATION_THRE_SM(0x0F);							
			DISABLE_AUTO_DEFECT_ALL;
			DISABLE_RGB_EDGE_GAIN;
			DISABLE_Y_LPF;			
			DISABLE_C_LPF;									
			DISABLE_VSUP;																		
		break;
		case CAMERA_PARA_AF_NORMAL_MODE:
			SET_PREPROCESS_PIXEL_LIMIT(0xFF);				
			DISABLE_AUTO_DEFECT_ALL;
			ENABLE_RGB_EDGE_GAIN;
			DISABLE_Y_LPF;
			DISABLE_C_LPF;				
		break;
		case CAMERA_PARA_AF_LOWLIGHT_MODE:
			SET_PREPROCESS_PIXEL_LIMIT(0xFF);				
			DISABLE_AUTO_DEFECT_ALL;
			ENABLE_RGB_EDGE_GAIN;
			DISABLE_Y_LPF;
			DISABLE_C_LPF;				
		break;
	}
}

void reduce_color_matrix(kal_uint32 sat_factor)
{
      kal_uint32 cm_table[3],m[9];
      kal_uint32 i,k,j;


      /*******************************
      	change to read nvram data
      ********************************/
      cm_table[0]=camera_para.ISP.reg[39];//REG_ISP_COLOR_MATRIX1;
      cm_table[1]=camera_para.ISP.reg[40];
      cm_table[2]=camera_para.ISP.reg[41];

      for(i=0;i<9;i++){
      	k=i/3;
      	j=(2-(i%3))*8;
      	m[i]=(cm_table[k]>>j)&0xff;
      }



#if 0
/* under construction !*/
/* under construction !*/
/* under construction !*/
/* under construction !*/
/* under construction !*/
/* under construction !*/
/* under construction !*/
/* under construction !*/
/* under construction !*/
/* under construction !*/
/* under construction !*/
/* under construction !*/
/* under construction !*/
/* under construction !*/
/* under construction !*/
/* under construction !*/
#endif

      for(i=0;i<9;i++){
     	if(i%4==0){
     	   m[i]=(m[i]-32)*sat_factor/32+32;
     	}
     	else{
     	   if(m[i]>=128){
     	   	m[i]=(m[i]-128)*sat_factor/32+128;
     	   }
     	   else{
     	   	m[i]=m[i]*sat_factor/32;
     	   }

	}
      }


	SET_COLOR_MATRIX1(m[0],m[1],m[2]);
	SET_COLOR_MATRIX2(m[3],m[4],m[5]);
	SET_COLOR_MATRIX3(m[6],m[7],m[8]);

}
static void CamParaSetAwbSet(kal_uint8 aeidx)

{
   kal_uint16 usTmpAvg,usTmpRangeH,usTmpRangeL;
   double usTmpRatio,usTmpRatioH,usTmpRatioL;
    if(bParaAwb)
    {
         Rgain_max_Local = Rgain_max;//test
         Ggain_max_Local = Rgain_max;
         Bgain_max_Local = Bgain_max;
         Rgain_PreGain = camera_para.SENSOR.cct[1].para;//camera_para.SENSOR.cct[PRE_GAIN_RB_INDEX].para;
         Ggain_PreGain = camera_para.SENSOR.cct[2].para;//camera_para.SENSOR.cct[PRE_GAIN_G_INDEX].para;
         Bgain_PreGain = camera_para.SENSOR.cct[1].para;//camera_para.SENSOR.cct[PRE_GAIN_RB_INDEX].para;
         if(Rgain_PreGain == 0)
	         Rgain_PreGain = 1;
         if(Ggain_PreGain == 0)
	         Ggain_PreGain = 1;
         if(Bgain_PreGain == 0)
	         Bgain_PreGain = 1;
         Rgain_PreGain_Ratio = ((float)Rgain_PreGain/(float)Ggain_PreGain);
         Bgain_PreGain_Ratio = ((float)Bgain_PreGain/(float)Ggain_PreGain);
         bParaAwb = KAL_FALSE;
}
       bParaAwbBitFlagPre = bParaAwbBitFlag;
       if(aeidx>AWB_PARA_OUTDOOR_AE_IDX_D65_H)
       {
           bParaAwbBitFlag |=LIGHT_SOURCE_BIT_NO1;

           usTmpAvg =  camera_para.AWB.LightSource[2][1];
           usTmpRangeH =  (camera_para.AWB.LightSource[1][1] - camera_para.AWB.LightSource[2][1])*(AWB_PARA_OUTDOOR_THRES_I_H/Rgain_PreGain_Ratio);

           if(AWB_Rgain>(kal_uint16)(usTmpAvg+usTmpRangeH))
           {
	           bParaAwbBitFlag |=LIGHT_SOURCE_BIT_NO1_H;
           }
       }
       else if(aeidx>AWB_PARA_OUTDOOR_AE_IDX_D65)
      	{
           usTmpAvg =  camera_para.AWB.LightSource[2][1];
           usTmpRangeH =  (camera_para.AWB.LightSource[1][1] - camera_para.AWB.LightSource[2][1])*(AWB_PARA_OUTDOOR_THRES_I/Rgain_PreGain_Ratio);

           if(AWB_Rgain>(kal_uint16)(usTmpAvg+usTmpRangeH))
           {
               bParaAwbBitFlag |=LIGHT_SOURCE_BIT_NO1;
           }
      	}
       if(aeidx<AWB_PARA_OUTDOOR_AE_IDX_CWF_LOW)
      	{
               bParaAwbBitFlag &=~(LIGHT_SOURCE_BIT_NO1|LIGHT_SOURCE_BIT_NO1_H);
      	}
       else if(aeidx<AWB_PARA_OUTDOOR_AE_IDX_CWF)
       {
           usTmpAvg =  camera_para.AWB.LightSource[2][1];
           usTmpRangeL =  (camera_para.AWB.LightSource[1][1] - camera_para.AWB.LightSource[2][1])*AWB_PARA_OUTDOOR_THRES_II;
           if(AWB_Rgain<=(kal_uint16)((kal_uint16)(usTmpAvg-usTmpRangeL)))
           {
               bParaAwbBitFlag &=~(LIGHT_SOURCE_BIT_NO1|LIGHT_SOURCE_BIT_NO1_H);
           }
       }

        usTmpRatioH =  (double)camera_para.AWB.LightSource[4][3] /(double)camera_para.AWB.LightSource[4][1] ;
        usTmpRatio =  (double)camera_para.AWB.LightSource[3][3] /(double)camera_para.AWB.LightSource[3][1] ;
        usTmpRatioH = usTmpRatio + (usTmpRatioH-usTmpRatio)*LIGHT_SOURCE_BIT_NO4_H;//0.6;
        usTmpRatioL = usTmpRatio + (usTmpRatioH-usTmpRatio)*LIGHT_SOURCE_BIT_NO4_L;//0.2;

       if( ( (double)AWB_Bgain/(double)AWB_Rgain)>usTmpRatioH )
      	{
      	      bParaAwbBitFlag |= LIGHT_SOURCE_BIT_NO4   ;
      	}
       else if( ( (double)AWB_Bgain/(double)AWB_Rgain)<usTmpRatioL )
       {
      	      bParaAwbBitFlag &= ~LIGHT_SOURCE_BIT_NO4   ;
       }

       if(bParaAwbBitFlag&LIGHT_SOURCE_BIT_NO1)
       {

	        usTmpRatioH =  (double)camera_para.AWB.LightSource[2][3]  ;
	        usTmpRatio =  (double)camera_para.AWB.LightSource[1][3];
		 if(bParaAwbBitFlag&LIGHT_SOURCE_BIT_NO1_H)
		        usTmpRatioH = usTmpRatio + (usTmpRatioH-usTmpRatio)*(AWB_PARA_OUTDOOR_BLIMIT_I_H/Bgain_PreGain_Ratio);
		 else
	        usTmpRatioH = usTmpRatio + (usTmpRatioH-usTmpRatio)*(AWB_PARA_OUTDOOR_BLIMIT_I/Bgain_PreGain_Ratio); //180
	   	 Bgain_max =(kal_uint16)(usTmpRatioH);
       }
      	else
       {
      	     	    Bgain_max = Bgain_max_Local;
       }
}
void set_anti_low_light_para(kal_uint8 aeidx)
{

		CamParaSetAwbSet(aeidx);

	if(aeidx>=20){
		//normal
		reduce_color_matrix(0x20);
		SET_INTERPOLATION_THRE_SM(0x0F);
		apply_camera_autodefect_to_reg(CAMERA_COMP_PREVIEW_NORMAL_SET);
		REG_ISP_EDGE_GAIN2=0x001F001F;

	}else if(aeidx >=15){
		reduce_color_matrix(0x18);
		SET_INTERPOLATION_THRE_SM(0x18);
		apply_camera_autodefect_to_reg(CAMERA_COMP_PREVIEW_LOWLIGHT_SET);
		REG_ISP_EDGE_GAIN2=0x000F000F;

	}else if(aeidx>=10){
		reduce_color_matrix(0x10);
		SET_INTERPOLATION_THRE_SM(0x1f);
		apply_camera_autodefect_to_reg(CAMERA_COMP_PREVIEW_LOWLIGHT_SET);
		REG_ISP_EDGE_GAIN2=0x00080008;

	}else if(aeidx>=8){
		reduce_color_matrix(0x08);
		SET_INTERPOLATION_THRE_SM(0x1F);
		apply_camera_autodefect_to_reg(CAMERA_COMP_PREVIEW_LOWLIGHT_SET);
		REG_ISP_EDGE_GAIN2=0x00000000;
	}else{

		reduce_color_matrix(0x00);
		SET_INTERPOLATION_THRE_SM(0x1F);
		apply_camera_autodefect_to_reg(CAMERA_COMP_PREVIEW_LOWLIGHT_SET);\
		REG_ISP_EDGE_GAIN2=0x00000000;
}

}



#if 0
/* under construction !*/
/* under construction !*/
/* under construction !*/
/* under construction !*/
/* under construction !*/
/* under construction !*/
/* under construction !*/
/* under construction !*/
/* under construction !*/
/* under construction !*/
/* under construction !*/
/* under construction !*/
/* under construction !*/
/* under construction !*/
/* under construction !*/
/* under construction !*/
/* under construction !*/
/* under construction !*/
/* under construction !*/
#endif
#endif