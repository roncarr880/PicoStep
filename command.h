
// onstep command and response

#define CMD_DEBUG 1                   // echo 

void process_command();
void reply_command( char st[] );
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

/**************************************************************************/

char command[44];
int serial_interface;

void command_in1( char c ){
static int in;
static char cmd[44];

   if( CMD_DEBUG ) Serial.write(c);

   if( c == ':' ){   // can also have in the input string, not just the start character
      if( in == 0 )  return;    // strip leading :
   }

   if( c == '#' ){
      cmd[in] = 0;
      strcpy( command, cmd );
      serial_interface = 1;
      process_command();
      in = 0;
      return;
   }

   cmd[in++] = c;
   if( in >= 39 ) in = 0;    
   cmd[in] = 0;
  
}

void command_in0( char c ){
static int in;
static char cmd[44];

   if( c == ':' ){   // can also have in the input string, not just the start character
      if( in == 0 )  return;    // strip leading :
   }


   if( c == '#' ){
      cmd[in] = 0;
      strcpy( command, cmd );
      serial_interface = 0;
      process_command();
      in = 0;
      return;
   }

   cmd[in++] = c;
   if( in >= 39 ) in = 0;    
   cmd[in] = 0;
  
}



void process_command(){

  if( match( "GU" )) get_status();
  if( match( "GVN" )) reply_command((char *)"3.16o#");
  if( match( "GR" )) get_RA();
  if( match( "GD" )) get_DEC();
  if( match( "GT" )){
     if( tracking == STAR ) reply_command((char *)"01.00274#");      // zero unless STAR !!!
     else if( tracking == MOON ) reply_command((char *)"00.94000#");
     else if( tracking == SUN ) reply_command((char *)"01.00000#");
     //else reply_command((char *)"00.00000#");
     else reply_command((char *)"0#");
  }
  if( match( "GZ" )) get_AZ();
  if( match( "GA" )) get_ALT();
  if( match( "GX9A" )) unknown();
  if( match( "GX9B" )) unknown();
  if( match( "GX9C" )) unknown();
  if( match( "GX9E" )) unknown();
  if( match( "GXEE" )) reply_command((char *)"0#");
  if( match( "Gt" )) get_latitude();
  if( match( "Gg" )) get_longitude();
  if( match( "A?" )){
    reply_command( (char *)"411#");   // alighnment status under goto, stars progress
  }
  if( matchn( "Sr", 2 )) Sr();    //  goto RA 
  if( matchn( "Sd", 2 )) Sd();    //  goto DEC
  if( match( "MS" ) ) ext_goto(); // start slew, 0 is success, 1<toolow>#,2<toohigh># exceeds limits
  if( match( "GG" ) ) reply_command((char *)"+00#");     // we use UTC time rather than local
  if( matchn( "SC", 2 )) set_date();
  if( matchn( "SL", 2 )) set_time();
  if( match( "GS" )) sidereal_time();
  if( match( "TL" )) tracking = MOON;
  if( match( "TS" )) tracking = SUN;
  if( match( "TQ" )) tracking = STAR;
  if( match( "Td" )) tracking = OFF, success();
  if( match( "Te" )) tracking = STAR, success();
  
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

   calc_ext_object( &target2 );    // !!! can test limits now, overhead and horizon - 1toohigh# 1toolow#
   if( target2.ALT > overhead_limit ){
      reply_command((char *)"1TooHigh#");
      return;
   }
   if( target2.ALT < horizon_limit ){
      reply_command((char *)"1TooLow#");
      return;
   }
   que_goto( &target2 );
   failure();             // return 0 if ok
}

void success(){
  reply_command((char *)"1");
}
void failure(){
  reply_command((char *)"0");
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
   strcat( rsp, "#" );
   reply_command(rsp);
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
   strcat( rsp, "#" );
   reply_command( rsp );

}

void unknown(){
  reply_command( (char *)"020.0#");     // wx info
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
   strcat( rsp,"#" );
   reply_command( rsp );

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
  strcat( rsp, "#" );
  reply_command( rsp );

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
  strcat( rsp, "#" );
  reply_command( rsp );

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

 //reply_command((char *)"NHp#");
 //return;
 
      int i = 0;
      if (tracking == POWER_FAIL || tracking == HOME || tracking == OFF || tracking ==  RSET ) 
                                               reply[i++]='n';                     // [n]ot tracking
      if (!(finding || ext_finding))           reply[i++]='N';                     // [N]o goto
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
      
      reply[i++] = '#';
      reply[i++]=0;

      reply_command( reply );
}

void set_date(){      // :SC03/16/24#   RTC
int m,d,y;
int m2,d2,y2;
  
  today = RTClib::now();
  m = today.month();   d = today.day();  y = today.year();
  y -= 2000;
 //Serial.print( m );  Serial.print( d ); Serial.print( y ); Serial.write(' ');
  m2 = arg1();
  d2 = arg2( '/' );
  y2 = arg3( '/' );
 //Serial.print( m2 );  Serial.print( d2 ); Serial.print( y2 ); Serial.println();
  if( m != m2 ) RTC.setMonth( m2 );
  if( d != d2 ) RTC.setDate( d2 );
  if( y != y2 ) RTC.setYear( y2 );

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
   if( h != h2 ) RTC.setHour( h2 );
   if( m != m2 ) RTC.setMinute( m2 );
   if( s != s2 ) RTC.setSecond( s2 );
   
   success();
}

void make_time_string( int hr, int mn, int sec ){
char tm[15];


   strcpy( tm, tostring2( hr ));
   strcat( tm, ":" );
   strcat( tm, tostring2( mn ));
   strcat( tm, ":" );
   strcat( tm, tostring2( sec ));
   strcat( tm, "#" );
  
   reply_command( tm );  
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
   else Serial.print( st );   
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
   if( val < 10 ) str[0] = '0';
   else str[0] = val/10 + '0';
   val = val % 10;
   str[1] = val + '0';
   str[2] = 0;
   return str;
}
