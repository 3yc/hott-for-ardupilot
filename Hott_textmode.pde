

#ifdef HOTT_TELEMETRY
#ifdef HOTT_SIM_TEXTMODE

//Textmode key codes
#define HOTT_TEXT_MODE_ESC		0x07
#define HOTT_TEXT_MODE_INC		0x0D
#define HOTT_TEXT_MODE_DEC		0x0B
#define HOTT_TEXT_MODE_ENTER	        0x0E
//Inc+Dec -> Set button on transmittier
#define HOTT_TEXT_MODE_INC_DEC	        0x09
#define HOTT_TEXT_MODE_NIL              0x0F







//////////////////////////////////////////////////////////////////////////////////////////////////
// Defines structure of Parameter_list
//////////////////////////////////////////////////////////////////////////////////////////////////
/// for reference see Parameters.pde
/// Or CLI --->>> SETUP --->>> SHOW
/// https://code.google.com/p/arducopter/wiki/AC2_attitude_PID?wl=en
/// https://code.google.com/p/arducopter/wiki/AC2_Standard_Parameters?wl=en

typedef struct PROGMEM {
  //char *label;
  const char *param_name;      
  bool editable;
  float increment; 
} HOTTMENU;

//////////////////////////////////////////////////////////////////////////////////////////////////

char const _LOW_VOLT[] PROGMEM         = "LOW_VOLT"; 
char const _BATT_CAPACITY[] PROGMEM    = "BATT_CAPACITY"; 
char const _TUNE[] PROGMEM             = "TUNE"; 
char const _TUNE_LOW[] PROGMEM         = "TUNE_LOW"; 
char const _TUNE_HIGH[] PROGMEM        = "TUNE_HIGH"; 
char const _LED_MODE[] PROGMEM         = "LED_MODE"; 

char const _RATE_RLL_P[] PROGMEM       = "RATE_RLL_P"; 
char const _RATE_RLL_I[] PROGMEM       = "RATE_RLL_I"; 
char const _RATE_RLL_D[] PROGMEM       = "RATE_RLL_D"; 
char const _RATE_RLL_IMAX[] PROGMEM    = "RATE_RLL_IMAX"; 
char const _RATE_PIT_P[] PROGMEM       = "RATE_PIT_P"; 
char const _RATE_PIT_I[] PROGMEM       = "RATE_PIT_I"; 

char const _RATE_PIT_D[] PROGMEM       = "RATE_PIT_D"; 
char const _RATE_PIT_IMAX[] PROGMEM    = "RATE_PIT_IMAX"; 
char const _RATE_YAW_P[] PROGMEM       = "RATE_YAW_P"; 
char const _RATE_YAW_I[] PROGMEM       = "RATE_YAW_I"; 
char const _RATE_YAW_D[] PROGMEM       = "RATE_YAW_D"; 
char const _RATE_YAW_IMAX[] PROGMEM    = "RATE_YAW_IMAX"; 


// rest up to you ////////////////

//char const _STB_RLL_P[] PROGMEM        = "STB_RLL_P"; 
//char const _STB_RLL_IMAX[] PROGMEM     = "STB_RLL_IMAX"; 
//char const _STB_PIT_P[] PROGMEM        = "STB_PIT_P"; 
//char const _STB_PIT_IMAX[] PROGMEM     = "STB_PIT_IMAX"; 
//char const _STB_YAW_P[] PROGMEM        = "STB_YAW_P"; 
//char const _STB_YAW_IMAX[] PROGMEM     = "STB_YAW_IMAX"; 



const HOTTMENU settings[] PROGMEM = { 
        // APM_Parameter  ,editiable,   increment
        
        {_LOW_VOLT,           true,     0.1 }, 
	{_BATT_CAPACITY,      true,     100},
	{_TUNE,               true,     1},       
	{_TUNE_LOW,           true,     100},
	{_TUNE_HIGH,          true,     100},
        {_LED_MODE,           true,      1},
  
	{_RATE_RLL_P,         true,      0.01},
        {_RATE_RLL_I,         true,      0.001},
        {_RATE_RLL_D,         true,      0.0001},
        {_RATE_RLL_IMAX,      true,      10},
        {_RATE_PIT_P,         true,      0.01},
        {_RATE_PIT_I,         true,      0.001},
       
        {_RATE_PIT_D,         true,      0.0001},
        {_RATE_PIT_IMAX,      true,      10},
        {_RATE_YAW_P,         true,      0.01},
        {_RATE_YAW_I,         true,      0.001},
        {_RATE_YAW_D,         true,      0.0001},
        {_RATE_YAW_IMAX,      true,      10},
        
        
//        {_STB_RLL_P,          true,      0.1},
//        {_STB_RLL_IMAX,       true,      10},
//        {_STB_PIT_P,          true,      0.1},
//        {_STB_PIT_IMAX,       true,      10},
//        {_STB_YAW_P,          true,      0.1},
//        {_STB_YAW_IMAX,       true,      10},
             

        

}; 


#define HOTT_PARAMS_PER_PAGE	6

const int num_parameters  =  sizeof(settings)/sizeof(settings[0]);
const int max_pages       =  ceil( ( (float)num_parameters / HOTT_PARAMS_PER_PAGE) ) + 1;


//////////////////////////////////////////////////////////////////////////////////////////////////
// CH6 Tuning 
// CH6 Tuning see http://code.google.com/p/arducopter/wiki/AC2_Tweaks
//////////////////////////////////////////////////////////////////////////////////////////////////

// 8 signs only!

const char hott_tuneparam[31][9] PROGMEM = 
{
   "CH6_NONE", 
   "STAB_KP ",    "STAB_KI ",    "YAW_KP  ",   "RATE_KP ",  "RATE_KI ",   "YAW_R_KP",   "THROT_KP",    "TO_BO_RA",   "RELAY   ",   "TRAV_SPE",   
   "NAV_KP  ",    "LOIT_KP ",    "HE_EX_GY",   "TH_HO_KP",  "Z_GAIN  ",   "DAMP    ",   "OPTFL_KP",    "OPTFL_KI",   "OPTFL_KD",   "NAV_I   ", 
   "RATE_KD ",    "LOI_R_KP",    "LOI_R_KD",   "YAW_KI  ",  "ACRO_KP ",   "YAW_R_KD",   "LOIT_KI ",    "LOI_R_KI",   "STAB_KD ",   "AHR_Y_KP"
 
 };

//////////////////////////////////////////////////////////////////////////////////////////////////
/// Order menue
/// Order CH6 params Like Tuning-list see http://code.google.com/p/arducopter/wiki/AC2_Tweaks

const uint8_t ch6_settings_menu[31]  = { 
  0,
  //Attitude 
  1, 2, 29, 3, 24,
  //Rate
  25, 4, 5, 21, 6, 26,
  //Altitude rate controller 
  7,
  // Extras
  8, 9, 
  // Navigation
  10, 11, 12, 27,
  // heli
  13, 
  //altitude controller 
  14, 15, 16, 
  //optical flow controller 
  17, 18, 19, 
  20, 22, 28, 23 , 30
  
};

const uint8_t num_ch6_parameters  =  sizeof(ch6_settings_menu) / sizeof(uint8_t);

uint8_t getposition_ch6_menu(uint8_t value) {
  
    uint8_t pos;
    for (int i = 0; i < num_ch6_parameters ; i++)
    {
       if (  ch6_settings_menu[i]  == value)  pos = i;
    }
    
    return pos;
}


//////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//  clears the text block

void HOTT_Clear_Text_Screen() {
   memset(hott_txt_msg.msg_txt, 0x20, sizeof(hott_txt_msg.msg_txt));
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// Printing to Textframe

void  HOTT_PrintWord(uint8_t pos,  char *w, bool inverted) {
  for (uint8_t index = 0; ; index++) {
    if (w[index] == 0x0) {
      break;
    } else { 
      hott_txt_msg.msg_txt[index+pos] = inverted ? w[index] +128: w[index];
    }
  }

}

void  HOTT_PrintWord_P(uint8_t pos,   prog_char_t *w, bool inverted) {
  for (uint8_t index = 0; ; index++) {
    if (pgm_read_byte(&w[index]) == 0x0) {
      break;
    } else { 
      hott_txt_msg.msg_txt[index+pos] = inverted ? pgm_read_byte(&w[index]) +128: pgm_read_byte(&w[index]);
    }
  }

}



//////////////////////////////////////////////////////////////////////////////////////////////////
// Get Parameter from APM and format it as Char to print it
// See AP_Common / AP_Param
//////////////////////////////////////////////////////////////////////////////////////////////////


char *  HOTT_GetParamAsChar( const char *key) {

    static char _hott_str_convert_buffer[10];	//not the best solution...
           
    AP_Param      *vp;
    enum ap_var_type        var_type;
    vp = AP_Param::find(key, &var_type);
        
    switch (var_type) {
    case AP_PARAM_INT8:
	{
	    	int8_t t = ((AP_Int8 *)vp)->get();
	    	
                if (strncmp (key, "TUNE", 16) == 0 ) {
                  //CH6 Tuning Int Value to text convertion
                  strcpy_P (_hott_str_convert_buffer, hott_tuneparam[t]);
                } else {
                  sprintf(_hott_str_convert_buffer,"%d",t);
                }
                
	    	break;
    	}
    case AP_PARAM_INT16:
	{
	    	int16_t t = ((AP_Int16 *)vp)->get();
	    	sprintf(_hott_str_convert_buffer,"%d",t);
		break;
    	}
    case AP_PARAM_INT32:
	 {
	    	int32_t t = ((AP_Int32 *)vp)->get();
	    	sprintf(_hott_str_convert_buffer,"%d",t);
	    	break;
	}
    case AP_PARAM_FLOAT:
	 {
	    	float t = ((AP_Float *)vp)->get();
	    	dtostrf(t,1,6,_hott_str_convert_buffer);
	    	break;
    	  }
    }
    
    return _hott_str_convert_buffer;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// prints tempvalue in editmode
//////////////////////////////////////////////////////////////////////////////////////////////////


char *  HOTT_TempValueAsChar( const char *key, float param_value_temp) {
    
    static char _hott_str_convert_buffer[10];	//not the best solution...
   
    AP_Param      *vp;
    enum ap_var_type        var_type;
    vp = AP_Param::find(key, &var_type);
        
    switch (var_type) {
    case AP_PARAM_INT8:
	{
	    	int8_t t = param_value_temp;
                
                if (strncmp (key, "TUNE", 16) == 0 ) {
                 //CH6 Tuning Int Value to text convertion
                  strcpy_P (_hott_str_convert_buffer, hott_tuneparam[t]);
                } else {
	    	  sprintf(_hott_str_convert_buffer,"%d",t);
                }
	    	break;
        }
    case AP_PARAM_INT16:
	{
	    	int16_t t = param_value_temp;
	    	sprintf(_hott_str_convert_buffer,"%d",t);
			break;
    	}
    case AP_PARAM_INT32:
	{
	    	int32_t t = param_value_temp;
	    	sprintf(_hott_str_convert_buffer,"%d",t);
	    	break;
	}
    case AP_PARAM_FLOAT:
	{
	    	float t = param_value_temp;
	    	dtostrf(t,1,6,_hott_str_convert_buffer);
	    	break;
    	}
    }

	return _hott_str_convert_buffer;
}





//////////////////////////////////////////////////////////////////////////////////////////////////
// Get Parameter and read as float to change it 
// See AP_Common / AP_Param
//////////////////////////////////////////////////////////////////////////////////////////////////


float  HOTT_GetParamAsFloat(const char *key) {
  
    AP_Param      *vp;
    enum ap_var_type        var_type;
    vp = AP_Param::find(key, &var_type);
        
    switch (var_type) {
    case AP_PARAM_INT8:
      return (float)((AP_Int8 *)vp)->get(); 
    case AP_PARAM_INT16:
      return (float)((AP_Int16 *)vp)->get();
    case AP_PARAM_INT32:
      return (float)((AP_Int32 *)vp)->get();
    case AP_PARAM_FLOAT:
      return (float)((AP_Float *)vp)->get();
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// Set Parameter via hott 
// See AP_Common / AP_Param
// gsc-mavlink
//////////////////////////////////////////////////////////////////////////////////////////////////


void  HOTT_Set_Param(const char *key, float value) {
 
    AP_Param      *vp;
    enum ap_var_type        var_type;
    vp = AP_Param::find(key, &var_type);
      
    switch (var_type) {
    case AP_PARAM_INT8:
      value = constrain(value, -128, 127);
      if (strncmp (key, "TUNE", 16) == 0 ) value = constrain(value, 0, 30);
      ((AP_Int8 *)vp)->set_and_save(value);
      break;
    case AP_PARAM_INT16:
      value = constrain(value, -32768, 32767);
      ((AP_Int16 *)vp)->set_and_save(value);
      break;
    case AP_PARAM_INT32:
      value = constrain(value, -2147483648.0, 2147483647.0);
      ((AP_Int32 *)vp)->set_and_save(value);
      break;
    case AP_PARAM_FLOAT:
      ((AP_Float *)vp)->set_and_save(value);
      break;
    }
    
   
}




//////////////////////////////////////////////////////////////////////////////////////////////////
// Parameter List
//////////////////////////////////////////////////////////////////////////////////////////////////


void  HOTT_ParamSettings(uint8_t page, int8_t selectedRow, int8_t editRow, float param_value_temp ) {
  uint8_t lineOffset = 1;
  
 char param_name[AP_MAX_NAME_SIZE+1]; 
  
  for (uint8_t index = 0; index <  HOTT_PARAMS_PER_PAGE; index++) {
       
      
      uint8_t rowpos    = 21*(index+lineOffset);
      uint8_t p_index   = ((page-1)*HOTT_PARAMS_PER_PAGE)+index;
      
      
      //if (p_index < sizeof(settings)-1) {
      if (p_index <  num_parameters ) {
               
                    const char *param_name_F = (const char *)pgm_read_word(&settings[p_index].param_name);
                    strcpy_P (param_name,   param_name_F    );
                    
                    ////// Label
                    HOTT_PrintWord(rowpos, (char *)param_name, 0);
                    
                    //////// Cursor
                    uint16_t sign = 0; 
                    if((index) == selectedRow) {
                      sign = '*';
                      if(pgm_read_byte(&settings[p_index].editable)) sign = '>';
                    } else {
                      sign = ':';  
                    }
                              
                    HOTT_PrintWord(rowpos + 12, (char *)&sign ,0);
                    
                    //////// VALUE 
                    bool inverted = ((index == editRow) && pgm_read_byte(&settings[p_index].editable) ) ? 1 : 0; 
                    
                    if (inverted) {
                      HOTT_PrintWord(rowpos + 13, HOTT_TempValueAsChar(param_name, param_value_temp) , 1 );
                    } else {
                      HOTT_PrintWord(rowpos + 13, HOTT_GetParamAsChar(param_name) , 0 );
                    }
                   
    
        
      }
      
  }
 
}







//////////////////////////////////////////////////////////////////////////////////////////////////
// TExtmode hande keys and funktions
//////////////////////////////////////////////////////////////////////////////////////////////////


void  HOTT_HandleTextMode(uint8_t addr) {

    uint8_t sensor   = (addr >> 4);   // 0xE0
    uint8_t key      = (addr & 0x0f);
    hott_txt_msg.fill1 = sensor;
    HOTT_Clear_Text_Screen();
    

  //if (!f.ARMED) {    
    static uint8_t     row = 0;  // equal to Parameter list == first
    static uint8_t     page = 1;
    
    static bool        editmode = false;  // Set button (INC + DEC)
    static int8_t      edit_row = -1;      // -1 == no line selected
    
    static float       param_value_temp;   // Tempvalue usefull for editmode
    
    char param_name[AP_MAX_NAME_SIZE+1]; // needs to copy the name from Progmem (16 characters)
    
    int8_t param_id;
    
    param_id = row + ((page-1)*6);

    //////// title
    HOTT_PrintWord_P(0, PSTR("APM_HOTT"),0);
    
    
    //////// armed    
    
    if (motors.armed()) {
      HOTT_PrintWord_P(9, PSTR("armed"),1 );
    } else {
      HOTT_PrintWord_P(9, PSTR("off  "),0 );
    }

    //////// Pagination
    
    uint16_t sign = 0; 
    
    if(page < max_pages) {
      sign = '>';
    } else {
      sign = ' ';
    }
    HOTT_PrintWord(20, (char *)&sign ,0);  
    
    sign = '<';
    HOTT_PrintWord(19, (char *)&sign ,0);  


    uint16_t t = page + 0x30; //works up to 9 max!
    HOTT_PrintWord(15, (char *)&t ,0 );
    HOTT_PrintWord_P(16, PSTR("/"),0 );
    t = max_pages +0x30;	//works up to 9 max!
    HOTT_PrintWord(17,(char *)&t ,0 );


    
       
   
    
    /// Page content  
   
    if(page < max_pages) {
          // Parameter Pages
          
          // editrownumber usefull for secutity stuff  
          edit_row = (editmode == true && !motors.armed()) ? row : -1; 
        
          HOTT_ParamSettings(page, row, edit_row, param_value_temp );
          
          const char *parmeter_name_F = (const char *)pgm_read_word(&settings[param_id].param_name);
          strcpy_P (param_name,   parmeter_name_F   );
          
      
    } else {
          // CREDITS
          HOTT_PrintWord_P((21*1), PSTR(THISFIRMWARE),0 );
          HOTT_PrintWord_P((21*3), PSTR("------ CREDTIS ------") ,0 );
          HOTT_PrintWord_P((21*3), PSTR("  by Adam Majerczyk  ") ,0 );
          HOTT_PrintWord_P((21*4), PSTR("     adam@3yc.de     ") ,0 );
          HOTT_PrintWord_P((21*5), PSTR("  Textmode by Michi  ") ,0 );
          HOTT_PrintWord_P((21*6), PSTR("mamaretti32@gmail.com") ,0 );
    }

      
      
      
          
      if( editmode ) {
            
                     switch (key) {
                      
                      case HOTT_TEXT_MODE_NIL:
                           //HOTT_PrintWord(21*7, "NIL",0 );
                      break;
                      
                      case HOTT_TEXT_MODE_DEC:
                           // change and copy value to temp
                           
                          if (strncmp (param_name, "TUNE", 16) == 0 ) {  
                            
                            if ( param_value_temp <= 0 ) {
                              param_value_temp =  ch6_settings_menu[ num_ch6_parameters-1 ];
                            } else {
                              param_value_temp =  ch6_settings_menu[ getposition_ch6_menu(param_value_temp) - 1  ];
                            }
                            
                          }else{
                            param_value_temp =  param_value_temp - pgm_read_float(&settings[param_id].increment);
                          }
                      break;
                      
                      case HOTT_TEXT_MODE_INC:
                           // change and copy value to temp
                           
                           if (strncmp (param_name, "TUNE", 16) == 0 ) {    
                             
                             if ( getposition_ch6_menu(param_value_temp) > num_ch6_parameters-1 ) {
                                param_value_temp =  ch6_settings_menu[ 0 ];
                             }else { 
                                param_value_temp =  ch6_settings_menu[ getposition_ch6_menu(param_value_temp) + 1  ];
                             }
                          }else{
                            param_value_temp =  param_value_temp + pgm_read_float(&settings[param_id].increment);
                          }
                           
                      break;
                      
                      case HOTT_TEXT_MODE_INC_DEC: {
                           // set new value to Parameter  BE CAREFULL!
                           HOTT_Set_Param( param_name , param_value_temp  );
                           // leave editmode 
                           editmode = false; 
                      }
                      break;
                      
                      case HOTT_TEXT_MODE_ENTER:
                           editmode = false; 
                      break;
                      
                      case HOTT_TEXT_MODE_ESC:
                           editmode = false; 
                      break;
                    }
       
      } else {
        
      // Paging mode
                   switch (key) {
                      case HOTT_TEXT_MODE_NIL:
                           // NOTHING TO DO FOR YOU ;)
                      break;
                      
                      case HOTT_TEXT_MODE_DEC:
                           // prev row
                           if (row > 0 ) row--;
                      break;
                      
                      case HOTT_TEXT_MODE_INC:
                           // next row!
                           if(row < 5) row++;
                      break;
                      
                      case HOTT_TEXT_MODE_INC_DEC: {
                           // enter editmode 
                           if ( pgm_read_byte(&settings[param_id].editable) && !motors.armed() ) {
                             editmode = true;
                             // read current param value
                             param_value_temp = (float)HOTT_GetParamAsFloat(param_name);
                           }
                      }
                      break;
                      
                      case HOTT_TEXT_MODE_ENTER:
                           // nextpage
                           if(page < max_pages) {
                             page++;
                             row = 0; 
                           }
                      break;
                      
                      case HOTT_TEXT_MODE_ESC:
                           // exit mode
                           if (page > 0) {
                             page--;
                             row = 0; 
                           }
                      break;
                    }
      }
    
    
    
    
    
    //+++++++++++++++++++++++++++++++++++++++++++++++++++HOTT_TEXT_MODE_DEBUG++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
    
    
    
    
    #ifdef HOTT_TEXT_MODE_DEBUG
      
      
      /// Display Ram usage   
      extern unsigned __brkval;
      extern unsigned char __heap_start;
      
      char mem_buf[21];
      snprintf(mem_buf, 21, "F.RAM: %u/%u", (uint16_t)(SP - __brkval), (uint16_t)(SP - (uint16_t)&__heap_start));
      HOTT_PrintWord((21*7)+0, mem_buf , 0 );
          

      /// Display Satcount  
      char sat_buf[2];
      sprintf(sat_buf,"s:%d",(int8_t)g_gps->num_sats);
      if(g_gps->status() == GPS::GPS_OK_FIX_3D) {
        HOTT_PrintWord((21*7)+17, sat_buf , 0 );
      } else {
        HOTT_PrintWord((21*7)+17, sat_buf , 1 );
      }
 
    
    #endif
    
    
    //+++++++++++++++++++++++++++++++++++++++++++++++++++ END HOTT_TEXT_MODE_DEBUG++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
    
    

    if (page < 1) {
      hott_txt_msg.fill1 = 0x01;
      page = 1;
    } else {
      hott_txt_msg.fill1 = sensor;
    }

}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ .... end textmode +++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//

#endif // textmode
#endif // hott

