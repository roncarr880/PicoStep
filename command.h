
// onstep command and response


#define CMD_DEBUG 0                  // echo on serial
#define REV_DEBUG 1                  // echo on serial1

void process_command( uint8_t check_flag );
void reply_command( char st[] );
void reply_frame( char msg[] );
void command_in0( char c );
void command_in1( char c );
void get_RA();
void get_DEC();
char* tostring2( int val );
int match( const char st[] );
int matchn( const char st[], int n );
void get_AZ();
void get_ALT();
void unknown();
void get_latitude();
void get_longitude();
void get_status();
void success();
void failure();
void Sr();
void Sd();
void ext_goto();
extern void que_goto( struct POINTING *p );
void set_date();
void set_time();
void sidereal_time();
void make_time_string( int hr, int mn, int sec );
void float2sdd( float val );
void get_date();
void get_time();
void fail_silently();

/**************************************************************************/

char command[44];
int serial_interface;
char sequence;

void command_in1( char c ){
static int in;
static char cmd[44];
static uint8_t use_check;

   if( CMD_DEBUG ) Serial.write(c);

   if( c == ':' && in == 0 ){   // can also have in the input string, not just the start character
      use_check = 0;
      return;                   // strip leading :
   }
   if( c == ';' && in == 0 ){
      use_check = 1;
      return;
   }

   if( c == '#' ){
      cmd[in] = 0;
      strcpy( command, cmd );
      serial_interface = 1;
      process_command( use_check );
      in = 0;
      return;
   }

   if( c < ' ' ) return;        // strip cr lf if any
   cmd[in++] = c;
   if( in >= 39 ) in = 0;    
   cmd[in] = 0;
  
}

void command_in0( char c ){
static int in;
static char cmd[44];
static int special;
static uint8_t use_check;

   if( REV_DEBUG ) if( isprint(c) ) Serial1.write(c);
   
   // special <ACK>
   if( c == 6 ){
      special = 1;
      return;
   }
   if( special ){
      serial_interface = 0;
      if( c == 0 ) reply_frame(( char *) "0CK_FAIL" );
      else{
         cmd[0] = c;   cmd[1] = '#';  cmd[2] = 0;  reply_command( cmd );
      }
      in = 0;  special = 0;
   }


   if( c == ':' && in == 0 ){   // can also have in the input string, not just the start character
      use_check = 0;
      return;                   // strip leading :
   }
   if( c == ';' && in == 0 ){
      use_check = 1;
      return;
   }

   if( c == '#' ){
      cmd[in] = 0;
      strcpy( command, cmd );
      serial_interface = 0;
      process_command( use_check );
      in = 0;
      return;
   }

   if( c < ' ' ) return;
   cmd[in++] = c;
   if( in >= 39 ) in = 0;    
   cmd[in] = 0;
  
}



void process_command( uint8_t check_flag ){
static uint8_t A1;
static uint8_t CS;
char temp[33];

  //pre-process checksum sequence
  if( check_flag ){
     int a = strlen( command );
     if( a < 4 ) return;             // short string, send an error?
     sequence = command[a - 1];
     // verify checksum here...
     command[ a - 3 ] = 0;           // chop checksum and sequence
  }
  else sequence = 0;                 // doubles as flag for adding checksum to response
  

  if( match( "GU" )) get_status();
  if( match( "GVN" )) reply_frame((char *)"3.16o");      //10.16a");
  if( match( "GVP" )) reply_frame((char *)"On-Step");
  if( match( "GVD" )) reply_frame((char *)"04 01 24");
  if( match( "GVT" )) reply_frame((char *)"10:20:30");
  if( match( "GR") || match("GRa") || match("GRH") ) get_RA();
  if( match( "GD") || match("GDe") || match("GDH") ) get_DEC();
  if( match( "GT" )){
     if( tracking == STAR ) reply_frame((char *)"01.00274");
     else if( tracking == MOON ) reply_frame((char *)"00.97900");  
     else if( tracking == SUN ) reply_frame((char *)"01.00000");
     //else reply_frame((char *)"00.00000");
     else reply_frame((char *)"0");
  }
  if( match( "GZ" ) || match("GZH") ) get_AZ();
  if( match( "GA" ) || match("GAH") ) get_ALT();
  if( match( "GX9A" )) unknown();                    // wx?
  if( match( "GX9B" )) unknown();
  if( match( "GX9C" )) unknown();
  if( match( "GX9E" )) unknown();
  if( match( "GXEE")) reply_frame((char *)"1" );   // #axis-1 i think, GXEE with checksum and sequence letter 131B
  if( match( "GXE6")) reply_frame((char *)"15.000001");   // steps per sidereal second, PEC commands, format convert.cpp sprintF
  //if( match( "GXE6" )) fail_silently();                 // 3600/240
  if( match( "GX98" )) reply_frame((char *)"N");      // GX98 rotator - none
  if( match( "GXE9" )) reply_frame((char *)"4" );     // past meridian east, limits commands
  if( match( "GXEA" )) reply_frame((char *)"4" );     // past meridian west in minutes, 4 is 1 deg
  if( match("GX92") || match("GX93")) reply_frame((char *)"250");   // us per step? !!! teensy says...81.28?
  if( match("FA")) failure();                         // focus present no
  if( match( "Gt" )) get_latitude(),CS = 1;
  if( match( "Gg" )) get_longitude(), CS = 2;
  if( match( "GG" ) ) reply_frame((char *)"+00");     // we use UTC time rather than local
  if( match( "GS" )) sidereal_time();
  if( match( "Gm" )){
       if( telescope.side == SIDE_EAST ) reply_frame((char *)"E");
       else if( telescope.side == SIDE_WEST ) reply_frame((char *)"W");
       else reply_frame((char *)"N");
  }
  if( match( "Gh" )) float2sdd( horizon_limit );    // these seem to be scaled
  if( match( "Go" )) float2sdd( overhead_limit );
  if( match( "GC" )) get_date();
  if( match( "GL" )) get_time();
  
     // reply[0] = '0' + ALIGN_MAX_NUM_STARS;
     // reply[1] = '0' + alignState.currentStar;
     // reply[2] = '0' + alignState.lastStar;
     // reply[3] = 0;
  if( match( "A1" )) A1 = 1, success();
  if( match( "A?" )){
    if( A1 == 1 ) reply_frame( (char *)"411");
    else if( A1 == 2 ) reply_frame((char *)"421");
    else reply_frame( (char *)"401");   // alignment status under goto, stars progress ? 401 to 411?
  }
  if( matchn( "Sr", 2 )) Sr();    //  goto RA 
  if( matchn( "Sd", 2 )) Sd();    //  goto DEC
  if( match( "MS" ) ) ext_goto(), CS = 3; // start slew, 0 is success, 1<toolow>#,2<toohigh># exceeds limits
  if( matchn( "SC", 2 )) set_date();
  if( matchn( "SL", 2 )) set_time();
  if( match( "TL" )) tracking = MOON;
  if( match( "TS" )) tracking = SUN;
  if( match( "TQ" )) tracking = STAR;
  if( match( "Td" )) tracking = OFF, success();
  if( match( "Te" )) tracking = STAR, success();
  if( match( "Me" )) slew_east = 1, slew_time = millis();     // actual slew control in main - PicoStep
  if( match( "Mw" )) slew_west = 1, slew_time = millis();
  if( match( "Mn" )) slew_north = 1, slew_time = millis();
  if( match( "Ms" )) slew_south = 1, slew_time = millis();
  if( match( "Qe" )) slew_east = 0, CS = 4;  
  if( match( "Qw" )) slew_west = 0, CS = 4;  
  if( match( "Qn" )) slew_north = 0, CS = 4;  
  if( match( "Qs" )) slew_south = 0, CS = 4;
  if( match( "Q" )) slew_east = slew_west = slew_north = slew_south = 0, CS = 4;
  if( match( "hC" )) go_home();  // go home
  if( match( "hF" )) at_home();  // at home
  // var state CS 0-?, 1-got RA, 2-got DEC, 3-goto, 4-slew .   CS at state 2, need to back figure the offset, state 4 use &telescope
  if( match( "CS" ) || match( "CM" )){
      if( CS == 2 ) add_sync( &target2 );
      if( CS == 4 ) add_point(&telescope);  // CS == 3 would be goto was directly on target, should we change anything?
      CS = 0;
      if( match( "CM" )) reply_frame((char*)"N/A");                   // is this used? or just CS    
  }
  if( match( "A+" )) add_point(&telescope), success(), A1=2;         // add 1st and only align point, rest will be sync here
  if( match( "%BR" ) || match("%BD") ) reply_frame((char *) "0" );   // no backlash
  if( matchn( "SG",2 )) success();                                   // ignore setting time offset, we know what it is - zero.
  if( matchn( "$B",2 )) success();                                   // ignore setting backlash
  if( matchn( "Sh",2 )) success();                                   // ignore set horizon limit
  if( matchn( "So",2 )) success();                                   // ignore set overhead limit
  if( matchn( "SXE9",4 )) success();                                 // ignore past meridian set
  if( matchn( "SXEA",4 )) success();
  
}


void fail_silently(){

   if( sequence ) reply_frame((char *)"0");    // send empty string for frame, else do nothing
}

int arg1(){
  return atoi(&command[2]);
}
int arg2( char m ){
char *p;

  p = strchr(command,m);
  if( p ){
     ++p;
     return atoi( p );
  }
  return 0;
}
int arg3(char m){
char *p;

   p = strchr(command,m);
   if( p ){
     ++p;
     p = strchr(p,m);
     if( p ){
      ++p;
      return atoi( p );
     }
   }
   return 0;
}


// goto - fill out tstar info, call calc_ext_object( &target2 ), call que_goto( &target2 )
   //   dec = to_degrees_dec( tstar.dd, tstar.dm, tstar.ds );
   //   ra  = to_degrees_ha( tstar.hr, tstar.mn, tstar.sc );
void Sr(){

  tstar.hr = arg1();  tstar.mn = arg2(':');  tstar.sc = arg3(':');
  success();      
}

void Sd(){

  tstar.dd = arg1();  tstar.dm = arg2('*');  tstar.ds = arg2(':');
  success();  
}

void ext_goto(){

   calc_ext_object( &target2 );    // in pointing.h , can test limits now, overhead and horizon - 2toohigh# 1toolow#
   if( target2.ALT > overhead_limit ){
      reply_frame((char *)"2TooHigh");
      return;
   }
   if( target2.ALT < horizon_limit ){
      reply_frame((char *)"1TooLow");
      return;
   }
   que_goto( &target2 );  // in PicoStep
   failure();             // return 0 if ok, failure is success in this case.
}

void success(){

  if( sequence ) reply_frame((char *)"1");
  else reply_command((char *)"1");
}
void failure(){

  if( sequence ) reply_frame((char *)"0");
  else reply_command((char *)"0");
}

void get_longitude(){  // sDDD*MM#     sign reversed, east negative 
float lg;
int d,m,a;
char rsp[15];
char s;

   s = '+';
   lg = my_longitude;
   lg = -lg;
   if( lg < 0.0 ) s = '-', lg = -lg;
   d = lg;
   lg -= d;
   m = lg * 60.0;
   a = d/100;
   d -= a * 100;

   rsp[0] = s;
   rsp[1] = a + '0';
   rsp[2] = 0;
   strcat( rsp, tostring2( d ) );
   strcat( rsp, "*" );
   strcat( rsp, tostring2( m ) );
   //strcat( rsp, "#" );
   reply_frame(rsp);
}

void get_latitude(){   //  sDD*MM#
float lat;
int d,m;
char s;
char rsp[15];

   s = '+';
   lat = my_latitude;
   if( lat < 0.0 ) s = '-', lat = -lat;
   d = lat;
   lat -= d;
   m = lat * 60;
   rsp[0] = s;
   rsp[1] = 0;
   strcat( rsp, tostring2( d ) );
   strcat( rsp, "*" );
   strcat( rsp, tostring2( m ) );
   //strcat( rsp, "#" );
   reply_frame( rsp );

}

void unknown(){
  reply_frame( (char *)"020.0");     // wx info
}

void get_AZ(){    //DDD*MM'SS# 
float az;
int d,m,s,a;
char rsp[15];

   az = telescope.AZ;
   d = az;
   az -= d;
   m = az * 60.0;
   az -= (float)m / 60.0;
   s = az * 3600.0;

   a = d / 100;
   d -= (float) a * 100.0;
   rsp[0] = a + '0';  rsp[1] = 0;
   strcat( rsp, tostring2( d ) );
   strcat( rsp,"*" );
   strcat( rsp, tostring2( m ) );
   strcat( rsp,"'" );
   strcat( rsp, tostring2( s ) );
   //strcat( rsp,"#" );
   reply_frame( rsp );

}

void get_ALT(){     // Reply: sDD*MM'SS#    // !!! same as dec
char rsp[15];
float dec;
int dec_deg, dec_min, dec_sec;
char sn;

  sn = '+';
  dec = telescope.ALT;
  if( dec < 0 ) sn = '-', dec = -dec;
  dec_deg = dec;
  dec -= dec_deg;
  dec_min = dec * 60.0;
  dec -= (float)dec_min/60.0;
  dec_sec = dec * 3600;

  rsp[0] = sn;   rsp[1] = 0;
  strcat( rsp, tostring2( dec_deg ) );
  strcat( rsp, "*" );
  strcat( rsp, tostring2( dec_min) );
  strcat( rsp, "'" );
  strcat( rsp, tostring2( dec_sec ) );
  //strcat( rsp, "#" );
  reply_frame( rsp );

}

void get_DEC(){     // sDD*MM'SS#
char rsp[15];
float dec;
int dec_deg, dec_min, dec_sec;
char sn;

  sn = '+';
  dec = telescope.DEC_;
  if( dec < 0 ) sn = '-', dec = -dec;
  dec_deg = dec;
  dec -= dec_deg;
  dec_min = dec * 60.0;
  dec -= (float)dec_min/60.0;
  dec_sec = dec * 3600;

  rsp[0] = sn;   rsp[1] = 0;
  strcat( rsp, tostring2( dec_deg ) );
  strcat( rsp, "*" );
  strcat( rsp, tostring2( dec_min) );
  strcat( rsp, "'" );
  strcat( rsp, tostring2( dec_sec ) );
  //strcat( rsp, "#" );
  reply_frame( rsp );

}

void get_RA(){
char rsp[15];
int ra_hr, ra_min, ra_sec;
//int dec_deg, dec_min;
float sid,ra;  //ha,dec;

  sid = to_degrees_ha( sid_hr, sid_mn, sid_sec );
  ra = sid - telescope.HA;
  if( ra < 0 ) ra += 360;
  if( ra >= 360 ) ra -= 360;
  ra =  ra / 15.0;
  ra_hr = ra;
  ra -= ra_hr;
  ra_min = ra * 60.0;
  ra -= (float)ra_min/60.0;
  ra_sec = ra * 3600;

  make_time_string( ra_hr, ra_min, ra_sec );
  
}


void get_status(){
char reply[40];

 //reply_frame((char *)"NHp#");
 //return;
 
      int i = 0;
      if (tracking == POWER_FAIL || tracking == HOME || tracking == OFF || tracking ==  RSET ) 
                                               reply[i++]='n';                     // [n]ot tracking
      if (!(/*finding ||*/ ext_finding))           reply[i++]='N';                     // [N]o goto, using finding disables quit slew
      if ( 1 )                                 reply[i++]='p';                     // Not [p]arked
      //if (park.state == PS_PARKING)            reply[i++]='I'; else                // Parking [I]n-progress
      //if (park.state == PS_PARKED)             reply[i++]='P'; else                // [P]arked
      //if (park.state == PS_PARK_FAILED)        reply[i++]='F';                     // Park [F]ailed
      //if (mount.syncFromOnStepToEncoders)      reply[i++]='e';                     // Sync to [e]ncoders only
      if (tracking == RSET )                   reply[i++]='H';                     // At [H]ome
      if (tracking == HOME )                   reply[i++]='h';                     // Slewing [h]ome
      //#if TIME_LOCATION_PPS_SENSE != OFF
      //  if (pps.synced)                        reply[i++]='S';                     // PPS [S]ync
      //#endif
      //if (guide.activePulseGuide())            reply[i++]='G';                     // Pulse [G]uide active
      //if (guide.active())                      reply[i++]='g';                     // [g]uide active

      //if (mount.settings.rc == RC_REFRACTION) { reply[i++]='r'; reply[i++]='s'; }  // [r]efr enabled [s]ingle axis
      //if (mount.settings.rc == RC_REFRACTION_DUAL) { reply[i++]='r'; }             // [r]efr enabled
      //if (mount.settings.rc == RC_MODEL)      { reply[i++]='t'; reply[i++]='s'; }  // On[t]rack enabled [s]ingle axis
      //if (mount.settings.rc == RC_MODEL_DUAL) { reply[i++]='t'; }                  // On[t]rack enabled
      //if (mount.settings.rc == RC_NONE) {
      //  float r = siderealToHz(mount.trackingRate);
      //  if (fequal(r, 57.900F))                reply[i++]='('; else                // Lunar rate selected
      //  if (fequal(r, 60.000F))                reply[i++]='O'; else                // SOlar rate selected
      //  if (fequal(r, 60.136F))                reply[i++]='k';                     // King rate selected
      //}
      if( tracking == SUN )                    reply[i++]='O';
      if( tracking == MOON )                   reply[i++]='(';

      //if (goTo.isHomePaused())                 reply[i++]='w';                     // [w]aiting at home 
      //if (goTo.isHomePauseEnabled())           reply[i++]='u';                     // Pa[u]se at home enabled?
      //if (sound.enabled)                       reply[i++]='z';                     // Bu[z]zer enabled?
      //if (goTo.isAutoFlipEnabled())            reply[i++]='a';                     // [a]uto meridian flip
      //#if AXIS1_PEC == ON
      //  if (pec.settings.recorded)             reply[i++]='R';                     // PEC data has been [R]ecorded
      //  if (transform.mountType != ALTAZM)
      //    reply[i++]="/,~;^"[(int)pec.settings.state];                             // PEC State (/)gnore, ready (,)lay, (~)laying, ready (;)ecord, (^)ecording
      //#endif
      if (mount_type == GEM)           reply[i++]='E'; else                // GEM
      if (mount_type  == FORK || mount_type == ALTALT)         reply[i++]='K'; else                // FORK
      if (mount_type == ALTAZ)         reply[i++]='A';                     // ALTAZM

      //Coordinate current = mount.getMountPosition(CR_MOUNT);
      //if (current.pierSide == PIER_SIDE_NONE)  reply[i++]='o'; else                // Pier side n[o]ne
      if (telescope.side == SIDE_EAST)         reply[i++]='T'; else                // Pier side eas[T]
      if (telescope.side == SIDE_WEST)         reply[i++]='W'; else                // Pier side [W]est
                                               reply[i++]='o';
      //reply[i++]='0' + guide.settings.pulseRateSelect;                             // Provide pulse-guide rate
      //reply[i++]='0' + guide.settings.axis1RateSelect;                             // Provide guide rate

      //reply[i++]='0' + limits.errorCode();                                         // Provide general error code
      reply[i++] = '0';
      
      //reply[i++] = '#';
      reply[i++]=0;

      reply_frame( reply );
}

void set_date(){      // :SC03/16/24#   RTC
int m,d,y;
int m2,d2,y2;
  
  today = RTClib::now();
  m = today.month();   d = today.day();  y = today.year();
  y -= 2000;
  m2 = arg1();
  d2 = arg2( '/' );
  y2 = arg3( '/' );

 //if( m != m2 ) RTC.setMonth( m2 );   // RTC keeps time and date, uncomment for battery change or 1st use of RTC 
 // if( d != d2 ) RTC.setDate( d2 );
 // if( y != y2 ) RTC.setYear( y2 ); 

  success();
}

void set_time(){     //:SL23:51:01#
int h,m,s;
int h2, m2, s2;


   today = RTClib::now();
   h = today.hour();   m = today.minute();  s = today.second();
   h2 = arg1();
   m2 = arg2(':');
   s2 = arg3(':');
  // if( h != h2 ) RTC.setHour( h2 );        // avoid changing RTC to avoid bogus data, uncomment for 1st use
  // if( m != m2 ) RTC.setMinute( m2 );
   if( s != s2 ) RTC.setSecond( s2 );        // allow seconds correction
   
   success();
}

void make_time_string( int hr, int mn, int sec ){
char tm[15];


   strcpy( tm, tostring2( hr ));
   strcat( tm, ":" );
   strcat( tm, tostring2( mn ));
   strcat( tm, ":" );
   strcat( tm, tostring2( sec ));
   //strcat( tm, "#" );
  
   reply_frame( tm );  
}

void sidereal_time(){      //:GS#  Returns: HH:MM:SS#

   make_time_string( sid_hr, sid_mn, sid_sec );
}

void reply_command( char st[] ){

   if( serial_interface == 1 ){
      Serial1.print( st );
      if( CMD_DEBUG ){ 
          Serial.print( "   " );              // echo for debug
          Serial.println( st );
      }
   }
   else{
      Serial.print( st );   
      if( REV_DEBUG ){ 
          Serial1.print( "   " );              // echo for debug
          Serial1.println( st );
      }
   }
}


void reply_frame( char msg[] ){
char st[44];
char ck[33];
uint8_t check;
int i;
char *p;

  strcpy( st, msg );
  if( sequence ){      // add checksum
     check = 0;  p = msg;
     while( *p ) check += *p++;
     itoa( check, ck, 16 );
     if( check < 16 ) strcat( st, "0" );
     strcat( st, ck );
     strncat(st, &sequence, 1);
  }

  strcat( st, "#" );
  reply_command( st );
}

int match( const char st[] ){

  if( strcmp( command, st ) == 0 ) return 1;
  return 0;
  
}

int matchn( const char st[], int n ){
char temp[44];

    temp[0] = 0;
    strcpy( temp,command);  
    temp[n] = 0;  
    if( strcmp( temp,st) == 0 ) return 1;
    return 0;
}

char * tostring2( int val ){
static char str[5];

   val = constrain( val, 0, 99 );
   //if( val < 10 ) str[0] = '0'; else
   str[0] = val/10 + '0';
   val = val % 10;
   str[1] = val + '0';
   str[2] = 0;
   return str;
}


void float2sdd( float val ){
char str[11];

   if( val < 0.0 ) str[0] = '-', val = -val;
   else str[0] = '+';
   str[1] = 0;
   strcat( str, tostring2( (int)val ));
   strcat( str, "0" );                   // values wrong, maybe it is sddd format?
   reply_frame( str );
}

// MM/DD/YY#
void get_date(){
int m,d,y;
char dt[15];

    today = RTClib::now();
    m = today.month();   d = today.day();  y = today.year(); y -= 2000;
    // make date string
   strcpy( dt, tostring2( m ));
   strcat( dt, "/" );
   strcat( dt, tostring2( d ));
   strcat( dt, "/" );
   strcat( dt, tostring2( y ));
   reply_frame( dt );  
}

void get_time(){
int h,m,s;

     today = RTClib::now();
     h = today.hour();   m = today.minute();  s = today.second();
     make_time_string( h, m, s );
}
