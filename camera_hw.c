/***************************************************************************
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
 *   camera_hw.c
 *
 * Project:
 * --------
 *   Maui_Software
 *
 * Description:
 * ------------
 *   Camera HW control API
 *
 * Author:
 * -------
 * -------
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * removed!
 *
 * removed!
 * removed!
 * removed!
 *
 * removed!
 * removed!
 * removed!
 *
 * removed!
 * removed!
 * removed!
 *
 * removed!
 * removed!
 * removed!
 *
 * removed!
 * removed!
 * removed!
 *
 * removed!
 * removed!
 * removed!
 *
 * removed!
 * removed!
 * removed!
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#if defined(ISP_SUPPORT)
#include "drv_comm.h"
#include "stdio.h"
#include "isp_if.h"
#include "image_sensor.h"
#include "sccb.h"
#include "ae_awb.h"
#include "camera_para.h"
#include "med_api.h"
#include "alerter_sw.h"
#include "alerter_hw.h"
#include "gpio_hw.h"

#ifdef __CUST_NEW__
   extern const char gpio_camera_power_en_pin;
   extern const char gpio_camera_reset_pin;
   extern const char gpio_camera_cmpdn_pin;
   #define MODULE_POWER_PIN					      gpio_camera_power_en_pin
   #define MODULE_RESET_PIN				      	gpio_camera_reset_pin
   #define MODULE_CMPDN_PIN					      gpio_camera_cmpdn_pin
#else /* __CUST_NEW__ */
// Power PIN Assignment
#define MODULE_POWER_PIN					      2 //lvjie 070927 1  // GPIO NO.
#define MODULE_RESET_PIN				      	12  // GPIO NO.
#define MODULE_CMPDN_PIN					      13  // GPIO NO.
#endif /* __CUST_NEW__ */
// Compact Image Sensor Module Power ON/OFF
void cis_module_power_on(kal_bool on)
{
   if(on==KAL_TRUE)
   {
	sccb_setDelay(0);		
	sccb_config(SCCB_SW_8BIT, OV2640_WRITE_ID, OV2640_READ_ID, NULL);//lvjie 070927 sccb_config(SCCB_HW_16BIT, 0xBA, 0xBB, NULL);	// Default 300KHz	

      GPIO_InitIO(1, MODULE_POWER_PIN);
      GPIO_ModeSetup(MODULE_POWER_PIN, 0);

      GPIO_WriteIO(1, MODULE_POWER_PIN);
#if 1//lvjie 070927
//SCCB
	GPIO_ModeSetup(SCCB_SERIAL_CLK_PIN,0);
	GPIO_ModeSetup(SCCB_SERIAL_DATA_PIN,0);		
       GPIO_WriteIO(1, SCCB_SERIAL_CLK_PIN);		
       GPIO_WriteIO(1, SCCB_SERIAL_DATA_PIN);		      
       GPIO_InitIO(1, SCCB_SERIAL_CLK_PIN);
       GPIO_InitIO(1, SCCB_SERIAL_DATA_PIN);		      

       //POWER DOWN	  
       GPIO_InitIO(1, MODULE_CMPDN_PIN);	  
       GPIO_ModeSetup(MODULE_CMPDN_PIN, 0);	  
       GPIO_WriteIO(0, MODULE_CMPDN_PIN);   
#endif
   }
   else
   {
   	// Sensor Module Power

      GPIO_InitIO(1, MODULE_POWER_PIN);
      GPIO_ModeSetup(MODULE_POWER_PIN, 0);

      GPIO_WriteIO(0, MODULE_POWER_PIN);

   	// Sensor Power, CMOS Sensor Power Down Signal Output      
    //lvjie 070927  GPIO_ModeSetup(MODULE_CMPDN_PIN, 0);   	
      GPIO_ModeSetup(MODULE_RESET_PIN, 0);                  
		GPIO_InitIO(1, MODULE_RESET_PIN);//lvjie 070927
      GPIO_WriteIO(0, MODULE_RESET_PIN);		                  
#if 1//lvjie 070927
      GPIO_ModeSetup(MODULE_CMPDN_PIN, 0);
	GPIO_WriteIO(1, MODULE_CMPDN_PIN);    //shenchensi new mod 20070517 --1-POWER DOWN MODE 0-NORMAL
	GPIO_InitIO(0, MODULE_CMPDN_PIN);

			// SCCB Low
			GPIO_ModeSetup(SCCB_SERIAL_CLK_PIN,0);
			GPIO_ModeSetup(SCCB_SERIAL_DATA_PIN,0);		
      GPIO_WriteIO(1, SCCB_SERIAL_CLK_PIN);		
      GPIO_WriteIO(1, SCCB_SERIAL_DATA_PIN);		      
       GPIO_InitIO(0, SCCB_SERIAL_CLK_PIN);
       GPIO_InitIO(0, SCCB_SERIAL_DATA_PIN);
#else
      GPIO_InitIO(1, MODULE_CMPDN_PIN);
      GPIO_InitIO(1, MODULE_RESET_PIN);

      // SCCB Low
		GPIO_ModeSetup(SCCB_SERIAL_CLK_PIN,0);
		GPIO_ModeSetup(SCCB_SERIAL_DATA_PIN,0);		
      GPIO_WriteIO(0, SCCB_SERIAL_CLK_PIN);		
      GPIO_WriteIO(0, SCCB_SERIAL_DATA_PIN);		      
      GPIO_InitIO(1, SCCB_SERIAL_CLK_PIN);		
      GPIO_InitIO(1, SCCB_SERIAL_DATA_PIN);
#endif		      
   }
}

void flashlight_power_on(kal_bool on)
{
}

#endif
