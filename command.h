
// onstep command and response
// !!! convert to work with both serial and serial1 as the source and destination !!!

void process_command();
void reply_command( char st[] );
void command_in( char c );
void get_RA();
void get_DEC();
char * tostring2( int val );
int match( const char st[] );
void get_AZ();
void get_ALT();
void unknown();
void get_latitude();
void get_longitude();

char command[40];

void command_in( char c ){
static int in;

   if( c == ':' ){
      in = 0;
      return;
   }

   if( c == '#' ){
      command[in] = 0;
      process_command();
      in = 0;
      return;
   }

   command[in++] = c;
   if( in >= 39 ) in = 0;    
   command[in] = 0;
  
}

void process_command(){

  if( match( "GU" )) reply_command((char *)"NHp#");      // status, quite involved
  if( match( "GVN" )) reply_command((char *)"3.16o#");
  if( match( "GR" )) get_RA();
  if( match( "GD" )) get_DEC();
  if( match( "GT" )) reply_command((char *)"01.00274#");
  if( match( "GZ" )) get_AZ();
  if( match( "GA" )) get_ALT();
  if( match( "GX9A" )) unknown();
  if( match( "GX9B" )) unknown();
  if( match( "GX9C" )) unknown();
  if( match( "GX9E" )) unknown();
  if( match( "Gt" )) get_latitude();
  if( match( "Gg" )) get_longitude();
  if( match( "A?" )){
    reply_command( (char *)"400#");   // alighnment status under goto, stars progress
  }
  
  Serial.println();    // !!! echo commands for debug
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
  reply_command( (char *)"0#");
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

  strcpy( rsp, tostring2( ra_hr ) );
  strcat( rsp, ":" );
  strcat( rsp, tostring2( ra_min) );
  strcat( rsp, ":" );
  strcat( rsp, tostring2( ra_sec ) );
  strcat( rsp, "#" );
  reply_command( rsp );
  
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


void reply_command( char st[] ){
   
   Serial1.print( st );
   Serial.print( "   " );              // !!! echo for debug
   Serial.print( st );
}

int match( const char st[] ){

  if( strcmp( command, st ) == 0 ) return 1;
  return 0;
  
}
