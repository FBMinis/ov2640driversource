//Generated by CDT_DLL  2006-11-14 16:40:33  
#if defined(ISP_SUPPORT)     
#include "drv_comm.h"      
#include "stdio.h"         
#include "isp_if.h"        
#include "image_sensor.h"  
#include "sccb.h"          
#include "ae_awb.h"        
#include "camera_para.h"   
#include "med_api.h"       
dsc_info_struct dsc_support_info ={                                     
  {1, 0, 0, 0, 0, 1, 0, 0, 0},   /*dscmode: 9 AE Mode*/        
  {0, 0},                               /*dsccomp: flash/af*/         
  {0, 0, 0, 0},                       /*flashlight: 4 flash mode*/  
  {0, 0, 0, 0}};                      /*af: 4 af mode*/             

device_info_struct device_support_info = {                            
/*ae_info: step num/step/minEV/maxEV/No.Iris*/                        
{140, 100, 3100, 17000, 0},                                                 
/*flash_info: pol/lumIdx/duty/offset/shutter/minShut/maxShut/R/G/B/sensor_gain/isp_gain*/ 
/*  Target Distance 50 cm                                          */ 
{0, 11, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};                            

kal_bool const APERTURE_PRI_TABLE[AV_NO] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

kal_bool const SHUTTER_PRI_60HZ_TABLE[TV_NO]={  
   0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0};

kal_bool const SHUTTER_PRI_50HZ_TABLE[TV_NO]={  
   0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0};

kal_bool const ISO_PRI_TABLE[ISO_NO] = { 0, 0, 0};
kal_uint16 const ISO_INFO_TABLE[ISO_NO] = { 0, 0, 0};

const exposure_lut_struct AE_AUTO_60HZ[140]= {
{672, 256, 128}, /*0*/
{672, 256, 128}, /*1*/
{672, 256, 128}, /*2*/
{672, 256, 128}, /*3*/
{672, 256, 128}, /*4*/
{672, 256, 128}, /*5*/
{672, 256, 128}, /*6*/
{672, 256, 128}, /*7*/
{672, 256, 128}, /*8*/
{672, 256, 128}, /*9*/
{672, 256, 128}, /*10*/
{672, 256, 128}, /*11*/
{672, 256, 127}, /*12*/
{672, 256, 118}, /*13*/
{672, 256, 111}, /*14*/
{672, 256, 103}, /*15*/
{672, 256, 96}, /*16*/
{672, 256, 90}, /*17*/
{672, 256, 84}, /*18*/
{672, 256, 78}, /*19*/
{672, 256, 73}, /*20*/
{672, 256, 68}, /*21*/
{672, 252, 65}, /*22*/
{672, 236, 64}, /*23*/
{672, 220, 64}, /*24*/
{672, 204, 65}, /*25*/
{672, 192, 64}, /*26*/
{672, 176, 65}, /*27*/
{672, 164, 65}, /*28*/
{672, 156, 64}, /*29*/
{672, 144, 65}, /*30*/
{672, 136, 64}, /*31*/
{669, 124, 66}, /*32*/
{669, 116, 66}, /*33*/
{669, 108, 66}, /*34*/
{557, 124, 64}, /*35*/
{557, 116, 64}, /*36*/
{557, 108, 64}, /*37*/
{446, 124, 65}, /*38*/
{446, 116, 65}, /*39*/
{446, 108, 65}, /*40*/
{446, 100, 66}, /*41*/
{335, 124, 66}, /*42*/
{335, 116, 66}, /*43*/
{335, 108, 66}, /*44*/
{335, 100, 66}, /*45*/
{335, 96, 64}, /*46*/
{335, 88, 66}, /*47*/
{335, 84, 64}, /*48*/
{335, 76, 66}, /*49*/
{335, 72, 65}, /*50*/
{335, 68, 64}, /*51*/
{224, 92, 66}, /*52*/
{224, 88, 65}, /*53*/
{224, 80, 66}, /*54*/
{224, 76, 65}, /*55*/
{224, 72, 64}, /*56*/
{224, 64, 67}, /*57*/
{112, 124, 65}, /*58*/
{112, 116, 65}, /*59*/
{112, 108, 65}, /*60*/
{112, 100, 65}, /*61*/
{112, 92, 66}, /*62*/
{112, 88, 65}, /*63*/
{112, 80, 66}, /*64*/
{112, 76, 65}, /*65*/
{112, 72, 64}, /*66*/
{112, 64, 67}, /*67*/
{110, 64, 64}, /*68*/
{102, 64, 64}, /*69*/
{95, 64, 65}, /*70*/
{89, 64, 64}, /*71*/
{83, 64, 64}, /*72*/
{77, 64, 65}, /*73*/
{72, 64, 65}, /*74*/
{67, 64, 65}, /*75*/
{63, 64, 64}, /*76*/
{58, 64, 65}, /*77*/
{55, 64, 64}, /*78*/
{51, 64, 64}, /*79*/
{47, 64, 65}, /*80*/
{44, 64, 65}, /*81*/
{41, 64, 65}, /*82*/
{38, 64, 65}, /*83*/
{36, 64, 65}, /*84*/
{33, 64, 66}, /*85*/
{31, 64, 65}, /*86*/
{29, 64, 65}, /*87*/
{27, 64, 65}, /*88*/
{25, 64, 66}, /*89*/
{23, 64, 67}, /*90*/
{22, 64, 65}, /*91*/
{20, 64, 67}, /*92*/
{19, 64, 65}, /*93*/
{18, 64, 65}, /*94*/
{16, 64, 68}, /*95*/
{15, 64, 67}, /*96*/
{14, 64, 67}, /*97*/
{13, 64, 68}, /*98*/
{12, 68, 64}, /*99*/
{11, 68, 66}, /*100*/
{11, 64, 65}, /*101*/
{10, 64, 67}, /*102*/
{9, 68, 65}, /*103*/
{9, 64, 65}, /*104*/
{8, 64, 68}, /*105*/
{7, 72, 64}, /*106*/
{7, 64, 67}, /*107*/
{6, 72, 65}, /*108*/
{6, 68, 64}, /*109*/
{5, 76, 65}, /*110*/
{5, 68, 67}, /*111*/
{5, 64, 67}, /*112*/
{4, 76, 65}, /*113*/
{4, 72, 65}, /*114*/
{4, 64, 68}, /*115*/
{4, 64, 64}, /*116*/
{4, 64, 64}, /*117*/
{4, 64, 64}, /*118*/
{4, 64, 64}, /*119*/
{4, 64, 64}, /*120*/
{4, 64, 64}, /*121*/
{4, 64, 64}, /*122*/
{4, 64, 64}, /*123*/
{4, 64, 64}, /*124*/
{4, 64, 64}, /*125*/
{4, 64, 64}, /*126*/
{4, 64, 64}, /*127*/
{4, 64, 64}, /*128*/
{4, 64, 64}, /*129*/
{4, 64, 64}, /*130*/
{4, 64, 64}, /*131*/
{4, 64, 64}, /*132*/
{4, 64, 64}, /*133*/
{4, 64, 64}, /*134*/
{4, 64, 64}, /*135*/
{4, 64, 64}, /*136*/
{4, 64, 64}, /*137*/
{4, 64, 64}, /*138*/
{4, 64, 64}
}; 
const exposure_lut_struct AE_AUTO_50HZ[140]= {
{672, 256, 128}, /*0*/
{672, 256, 128}, /*1*/
{672, 256, 128}, /*2*/
{672, 256, 128}, /*3*/
{672, 256, 128}, /*4*/
{672, 256, 128}, /*5*/
{672, 256, 128}, /*6*/
{672, 256, 128}, /*7*/
{672, 256, 128}, /*8*/
{672, 256, 128}, /*9*/
{672, 256, 128}, /*10*/
{672, 256, 128}, /*11*/
{672, 256, 127}, /*12*/
{672, 256, 118}, /*13*/
{672, 256, 111}, /*14*/
{672, 256, 103}, /*15*/
{672, 256, 96}, /*16*/
{672, 256, 90}, /*17*/
{672, 256, 84}, /*18*/
{672, 256, 78}, /*19*/
{672, 256, 73}, /*20*/
{672, 256, 68}, /*21*/
{672, 252, 65}, /*22*/
{672, 236, 64}, /*23*/
{672, 220, 64}, /*24*/
{672, 204, 65}, /*25*/
{672, 192, 64}, /*26*/
{672, 176, 65}, /*27*/
{672, 164, 65}, /*28*/
{672, 156, 64}, /*29*/
{672, 144, 65}, /*30*/
{672, 136, 64}, /*31*/
{668, 124, 66}, /*32*/
{668, 116, 66}, /*33*/
{668, 108, 66}, /*34*/
{668, 100, 66}, /*35*/
{535, 120, 64}, /*36*/
{535, 112, 64}, /*37*/
{535, 104, 65}, /*38*/
{535, 96, 65}, /*39*/
{401, 120, 65}, /*40*/
{401, 112, 65}, /*41*/
{401, 104, 66}, /*42*/
{401, 96, 66}, /*43*/
{401, 92, 64}, /*44*/
{401, 84, 66}, /*45*/
{401, 80, 65}, /*46*/
{401, 72, 67}, /*47*/
{401, 68, 66}, /*48*/
{401, 64, 66}, /*49*/
{269, 88, 66}, /*50*/
{269, 84, 65}, /*51*/
{269, 76, 67}, /*52*/
{269, 72, 66}, /*53*/
{269, 68, 65}, /*54*/
{269, 64, 64}, /*55*/
{134, 120, 64}, /*56*/
{134, 112, 64}, /*57*/
{134, 104, 65}, /*58*/
{134, 96, 65}, /*59*/
{134, 88, 67}, /*60*/
{134, 84, 65}, /*61*/
{134, 76, 67}, /*62*/
{134, 72, 66}, /*63*/
{134, 68, 65}, /*64*/
{134, 64, 65}, /*65*/
{126, 64, 64}, /*66*/
{117, 64, 65}, /*67*/
{110, 64, 64}, /*68*/
{102, 64, 64}, /*69*/
{95, 64, 65}, /*70*/
{89, 64, 64}, /*71*/
{83, 64, 64}, /*72*/
{77, 64, 65}, /*73*/
{72, 64, 65}, /*74*/
{67, 64, 65}, /*75*/
{63, 64, 64}, /*76*/
{58, 64, 65}, /*77*/
{55, 64, 64}, /*78*/
{51, 64, 64}, /*79*/
{47, 64, 65}, /*80*/
{44, 64, 65}, /*81*/
{41, 64, 65}, /*82*/
{38, 64, 65}, /*83*/
{36, 64, 65}, /*84*/
{33, 64, 66}, /*85*/
{31, 64, 65}, /*86*/
{29, 64, 65}, /*87*/
{27, 64, 65}, /*88*/
{25, 64, 66}, /*89*/
{23, 64, 67}, /*90*/
{22, 64, 65}, /*91*/
{20, 64, 67}, /*92*/
{19, 64, 65}, /*93*/
{18, 64, 65}, /*94*/
{16, 64, 68}, /*95*/
{15, 64, 67}, /*96*/
{14, 64, 67}, /*97*/
{13, 64, 68}, /*98*/
{12, 68, 64}, /*99*/
{11, 68, 66}, /*100*/
{11, 64, 65}, /*101*/
{10, 64, 67}, /*102*/
{9, 68, 65}, /*103*/
{9, 64, 65}, /*104*/
{8, 64, 68}, /*105*/
{7, 72, 64}, /*106*/
{7, 64, 67}, /*107*/
{6, 72, 65}, /*108*/
{6, 68, 64}, /*109*/
{5, 76, 65}, /*110*/
{5, 68, 67}, /*111*/
{5, 64, 67}, /*112*/
{4, 76, 65}, /*113*/
{4, 72, 65}, /*114*/
{4, 64, 68}, /*115*/
{4, 64, 64}, /*116*/
{4, 64, 64}, /*117*/
{4, 64, 64}, /*118*/
{4, 64, 64}, /*119*/
{4, 64, 64}, /*120*/
{4, 64, 64}, /*121*/
{4, 64, 64}, /*122*/
{4, 64, 64}, /*123*/
{4, 64, 64}, /*124*/
{4, 64, 64}, /*125*/
{4, 64, 64}, /*126*/
{4, 64, 64}, /*127*/
{4, 64, 64}, /*128*/
{4, 64, 64}, /*129*/
{4, 64, 64}, /*130*/
{4, 64, 64}, /*131*/
{4, 64, 64}, /*132*/
{4, 64, 64}, /*133*/
{4, 64, 64}, /*134*/
{4, 64, 64}, /*135*/
{4, 64, 64}, /*136*/
{4, 64, 64}, /*137*/
{4, 64, 64}, /*138*/
{4, 64, 64}
}; 
#define AE_NIGHT_60HZ AE_AUTO_60HZ

#define AE_NIGHT_50HZ AE_AUTO_50HZ

#define AE_AUTO_60HZ_VIDEO AE_AUTO_60HZ

#define AE_AUTO_50HZ_VIDEO AE_AUTO_50HZ

#define AE_NIGHT_60HZ_VIDEO AE_AUTO_60HZ

#define AE_NIGHT_50HZ_VIDEO AE_AUTO_50HZ

const kal_uint8 IRIS_AUTO_LUT[140]= {
28, /*0*/
28, /*1*/
28, /*2*/
28, /*3*/
28, /*4*/
28, /*5*/
28, /*6*/
28, /*7*/
28, /*8*/
28, /*9*/
28, /*10*/
28, /*11*/
28, /*12*/
28, /*13*/
28, /*14*/
28, /*15*/
28, /*16*/
28, /*17*/
28, /*18*/
28, /*19*/
28, /*20*/
28, /*21*/
28, /*22*/
28, /*23*/
28, /*24*/
28, /*25*/
28, /*26*/
28, /*27*/
28, /*28*/
28, /*29*/
28, /*30*/
28, /*31*/
28, /*32*/
28, /*33*/
28, /*34*/
28, /*35*/
28, /*36*/
28, /*37*/
28, /*38*/
28, /*39*/
28, /*40*/
28, /*41*/
28, /*42*/
28, /*43*/
28, /*44*/
28, /*45*/
28, /*46*/
28, /*47*/
28, /*48*/
28, /*49*/
28, /*50*/
28, /*51*/
28, /*52*/
28, /*53*/
28, /*54*/
28, /*55*/
28, /*56*/
28, /*57*/
28, /*58*/
28, /*59*/
28, /*60*/
28, /*61*/
28, /*62*/
28, /*63*/
28, /*64*/
28, /*65*/
28, /*66*/
28, /*67*/
28, /*68*/
28, /*69*/
28, /*70*/
28, /*71*/
28, /*72*/
28, /*73*/
28, /*74*/
28, /*75*/
28, /*76*/
28, /*77*/
28, /*78*/
28, /*79*/
28, /*80*/
28, /*81*/
28, /*82*/
28, /*83*/
28, /*84*/
28, /*85*/
28, /*86*/
28, /*87*/
28, /*88*/
28, /*89*/
28, /*90*/
28, /*91*/
28, /*92*/
28, /*93*/
28, /*94*/
28, /*95*/
28, /*96*/
28, /*97*/
28, /*98*/
28, /*99*/
28, /*100*/
28, /*101*/
28, /*102*/
28, /*103*/
28, /*104*/
28, /*105*/
28, /*106*/
28, /*107*/
28, /*108*/
28, /*109*/
28, /*110*/
28, /*111*/
28, /*112*/
28, /*113*/
28, /*114*/
28, /*115*/
28, /*116*/
28, /*117*/
28, /*118*/
28, /*119*/
28, /*120*/
28, /*121*/
28, /*122*/
28, /*123*/
28, /*124*/
28, /*125*/
28, /*126*/
28, /*127*/
28, /*128*/
28, /*129*/
28, /*130*/
28, /*131*/
28, /*132*/
28, /*133*/
28, /*134*/
28, /*135*/
28, /*136*/
28, /*137*/
28, /*138*/
28
}; 
const nvram_camera_lens_struct CAMERA_LENS_DEFAULT_VALUE={  
  /*af_table_num, af_home_idx, af_macro_idx, af_infinity_idx, ae_hyper_pos, 
   af_me_home_pos, af_me_macro_pos, af_calibration_offset */ 
  {0, 0, 0, 0, 0, 0, 0, 0}

,
  {  
  }

};

//#define AE_AUTO_50HZ NULL
//#define AE_AUTO_60HZ NULL
//#define AE_AUTO_50HZ_VIDEO NULL
//#define AE_AUTO_60HZ_VIDEO NULL
#define AE_PORTRAIT_50HZ NULL
#define AE_PORTRAIT_60HZ NULL
#define AE_PORTRAIT_50HZ_VIDEO NULL
#define AE_PORTRAIT_60HZ_VIDEO NULL
#define AE_LANDSCAPE_50HZ NULL
#define AE_LANDSCAPE_60HZ NULL
#define AE_LANDSCAPE_50HZ_VIDEO NULL
#define AE_LANDSCAPE_60HZ_VIDEO NULL
#define AE_SPORT_50HZ NULL
#define AE_SPORT_60HZ NULL
#define AE_SPORT_50HZ_VIDEO NULL
#define AE_SPORT_60HZ_VIDEO NULL
#define AE_FLOWER_50HZ NULL
#define AE_FLOWER_60HZ NULL
#define AE_FLOWER_50HZ_VIDEO NULL
#define AE_FLOWER_60HZ_VIDEO NULL
//#define AE_NIGHT_50HZ NULL
//#define AE_NIGHT_60HZ NULL
//#define AE_NIGHT_50HZ_VIDEO NULL
//#define AE_NIGHT_60HZ_VIDEO NULL
#define AE_SHUTTER_50HZ NULL
#define AE_SHUTTER_60HZ NULL
#define AE_SHUTTER_50HZ_VIDEO NULL
#define AE_SHUTTER_60HZ_VIDEO NULL
#define AE_APERTURE_50HZ NULL
#define AE_APERTURE_60HZ NULL
#define AE_APERTURE_50HZ_VIDEO NULL
#define AE_APERTURE_60HZ_VIDEO NULL
#define AE_ISO_50HZ NULL
#define AE_ISO_60HZ NULL
#define AE_ISO_50HZ_VIDEO NULL
#define AE_ISO_60HZ_VIDEO NULL
nvram_camera_lens_struct	camera_lens; 
static const exposure_lut_struct *AE_LUTs[9][2][2]={ /*[Scene][video as 1][60Hz as 1]*/  
AE_AUTO_50HZ,      AE_AUTO_60HZ,     AE_AUTO_50HZ_VIDEO,       AE_AUTO_60HZ_VIDEO,       
AE_PORTRAIT_50HZ,  AE_PORTRAIT_60HZ, AE_PORTRAIT_50HZ_VIDEO,   AE_PORTRAIT_60HZ_VIDEO,   
AE_LANDSCAPE_50HZ, AE_LANDSCAPE_60HZ,AE_LANDSCAPE_50HZ_VIDEO,  AE_LANDSCAPE_60HZ_VIDEO,  
AE_SPORT_50HZ,     AE_SPORT_60HZ,    AE_SPORT_50HZ_VIDEO,      AE_SPORT_60HZ_VIDEO,      
AE_FLOWER_50HZ,    AE_FLOWER_60HZ,   AE_FLOWER_50HZ_VIDEO,     AE_FLOWER_60HZ_VIDEO,     
AE_NIGHT_50HZ,     AE_NIGHT_60HZ,    AE_NIGHT_50HZ_VIDEO,      AE_NIGHT_60HZ_VIDEO,      
AE_SHUTTER_50HZ,   AE_SHUTTER_60HZ,  AE_SHUTTER_50HZ_VIDEO,    AE_SHUTTER_60HZ_VIDEO,    
AE_APERTURE_50HZ,  AE_APERTURE_60HZ, AE_APERTURE_50HZ_VIDEO,   AE_APERTURE_60HZ_VIDEO,   
AE_ISO_50HZ,       AE_ISO_60HZ,      AE_ISO_50HZ_VIDEO,        AE_ISO_60HZ_VIDEO         
}; 

const exposure_lut_struct *get_ae_lut(ae_lut_info_struct info)
{                                                             
	kal_uint32 idxHz = 0;                                       
	kal_uint32 idxVideo = 0;                                    
	const exposure_lut_struct *pAeLut = NULL;                   
	                                                            
	if (info.band==CAM_BANDING_60HZ) idxHz = 1;                 
	if (info.videomode==KAL_TRUE) idxVideo = 1;                 
	                                                            
	pAeLut = AE_LUTs[info.dscmode][idxVideo][idxHz];            
	ASSERT(pAeLut != NULL);                                     
	return pAeLut;                                              
}                                                             
                                                              
const kal_uint8 *get_iris_lut(ae_lut_info_struct info)        
{                                                             
	switch(info.dscmode)                                        
	{                                                           
		case CAM_AUTO_DSC :                                       
				return IRIS_AUTO_LUT;                                 
		break;                                                    
	}                                                           
	return IRIS_AUTO_LUT;	/* default */                         
}                                                             
                                                              
#endif                                                        
