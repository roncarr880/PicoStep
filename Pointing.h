
// Telescope pointing model for 4 or 5 scopes
//   dummy scope where one second of HA is one step of the dummy motor
//   Fork mount - could just be same as the dummy unless want to alter under pole behavior
//   GEM
//   Alt Az
//   Alt Alt or Lazy Yoke

#define SERIAL_DEBUG 1    // turn serial printing on or off

#define R2D 57.2958       // degrees in a radian

extern void get_GMT_base(int dbg);                                 // why need to prototype in this .h?
float to_degrees_dec( int8_t dg, int8_t mn, int8_t sc );
float to_degrees_ha( int8_t hr, int8_t mn, int8_t sc );
void serial_display_pointing(struct POINTING *p);

int longseek;

struct POINTING {
   float DEC_;          // where pointing from HAstep stepper steps
   float HA;
   int side;            // GEM east or west, fork under over?
   float ALT;           // Alt Az mount
   float AZ;
   float ALT_rate;      // rate is constantly changing as telescope is tracking an object
   float AZ_rate;
   float DECp;          // Alt Alt mount
   float HAp;
   float DECp_rate;
   float HAp_rate;
};

struct POINTING telescope;
struct POINTING target;

// update the pointing struct, from actual position, or position of a database object
// actual position from the dummy stepper HAstep steps
// arg obj -1 for actual position
void calc_pointing( int obj, struct POINTING *p ){   
long ha_p, dc_p;
float ha, dec, ra, sid;

  // first update the sidereal time globals, can display clock also
   get_GMT_base( SERIAL_DEBUG );
   
   if( obj < 0 ){                        // from dummy telescope, reflects actual when not seeking,
      noInterrupts();                    // improvement would be to convert alt az back to ha,dec if 
      ha_p =  HAstep.currentPosition();  // using an alt az mount
      //dec = p->DEC_;
      dc_p = DCstep.currentPosition();
      interrupts();
      //if( RAreverse ) ha_p = -ha_p;     // don't think will be reversing the dummy scope
      ha = 15.0 * (float)ha_p / 3600.0;
      dec = (float)dc_p / (15.0 * 60.0);  // same resolution as 1 second of ha, 4 degree seconds, twice seeing
      p->HA = ha;
      p->DEC_ = dec;
   }
   else{                                 // get position from database object
      dec = to_degrees_dec( bstar[obj].dd, bstar[obj].dm, 1 );
      ra  = to_degrees_ha( bstar[obj].hr, bstar[obj].mn, 1 );     // !!! need seconds in bstar
      sid = to_degrees_ha( sid_hr, sid_mn, sid_sec );
      ha = sid - ra;                     // ra_hr = sid_hr - ha_hr;
      p->HA = ha;
      p->DEC_ = dec;
   }

   // have ha and dec, calc all scopes
   
   // alt az scope
    //sin(ALT) = sin(DEC)*sin(LAT)+cos(DEC)*cos(LAT)*cos(HA)
    // ALT = asin(ALT) 
    //               sin(DEC) - sin(ALT)*sin(LAT)
    // cos(A)   =   ---------------------------------
    //               cos(ALT)*cos(LAT)
    // A = acos(A)

    // If sin(HA) is negative, then AZ = A, otherwise
    // AZ = 360 - A
    // azimuth rate = tan(ALT)*cos(A)*cos(LAT) - sin(LAT)
    // altitude rate = -sin(A)*cos(LAT)
    // field rotation = atan( sin(HA) / ( cos(DEC)*tan(LAT) - sin(DEC)*cos(HA) )
float m_lat;
float ALT, A, AZ;
float AZrate, ALTrate;

    ha /= R2D;   dec /= R2D;  m_lat = my_latitude/R2D;
    ALT = sin(dec) * sin(m_lat) + cos(dec) * cos(m_lat) * cos(ha);
    ALT = constrain(ALT,-1.0,1.0);
    ALT = asin(ALT);
    A = (sin(dec) - sin(ALT) * sin(m_lat)) / ( cos(ALT) * cos(m_lat));
    A = constrain(A,-1.0,1.0);
    AZ = acos(A);
    
    float AZp = AZ;
    float ALTp = ALT;

    AZ *= R2D;  ALT *= R2D;             // values stored as degrees 
    if( sin(ha) >= 0 ) AZ = 360 - AZ;
    
    AZrate = tan(ALTp)*cos(AZp)*cos(m_lat) - sin(m_lat);
    AZrate = -AZrate;
    ALTrate = -sin(AZp)*cos(m_lat);
    
    p->ALT = ALT;
    p->AZ  = AZ;
    p->ALT_rate = ALTrate;
    p->AZ_rate = AZrate;

    // alt alt lazy yoke mount
float TH, PHi, x, y, z, xp, yp, zp;
 
    // change to (rectangular) spherical coord
    //TH == dec;  PHi == ha;             // angles theta and phi
    TH = 90.0/R2D - dec;  PHi = -ha;
    
    x = sin(TH) * cos(PHi);
    y = sin(TH) * sin(PHi);
    z = cos(TH);
    // rotate by latitude about y axis
    xp = x * cos( m_lat ) + z * sin( m_lat );
    yp = y;
    zp = z * cos( m_lat ) - x * sin( m_lat );
    // change back to polar coord
    PHi = atan2(yp, xp);
    TH = acos( zp );
    p->DECp = 90 - R2D * TH;
    p->HAp = -R2D * PHi;
    // calc rates of change ( empirical derived )
    p->DECp_rate = sin(-PHi) * sin( m_lat);
    p->HAp_rate = cos( m_lat) - tan( 90/R2D - TH) * sin(m_lat) * cos( PHi);

    if( SERIAL_DEBUG ) serial_display_pointing( p );
}

// set speeds if tracking, from telescope 
void set_speeds(){    // remember to disable interrupts, mount type specific
  
}


// mount type specific, do a double goto if takes a while
// goto from the star database
void goto_star( int star ){

  
  
}
//OR
// update HA after the first seek and seek again
void goto_target( struct POINTING *p ){
static long tm;
static struct POINTING *p2;
float more_ha;

   if( longseek == 0 ){
       p2 = p;            // a hack to allow to seek twice
       tm = millis();
   }
   else{
       tm = (millis() - tm)/1000;
       if( tm > 240 ){              // over 4 minutes old
          longseek  = 0;
          return;
       }
       more_ha = to_degrees_ha( 0, 0, tm );
       p2->HA += more_ha;
       longseek = 0;
   }

long ha,ra,dc,dec;

   ha = p2->HA * 3600.0/15.0;
   ra = p2->HA * 3600.0/15.0;               // times some factor for stepper gearing
   dc = 15.0 * 60.0 * p2->DEC_;
   dec = 5.0 * 60.0 * p2->DEC_;        // !!! times something other than 5
    
      noInterrupts();
      HAstep.moveTo( ha );       
      RAstep.moveTo( ra );        // hour angle, west positive here
      DECstep.moveTo( dec );           // !!! fake steps per minute
      DCstep.moveTo( dc );   
      finding = 1;               // important to set the moving flag
      interrupts();

}

void display_pointing(struct POINTING *p){   // oled display if correct screen mode
int ha_hr, ha_min;
int ra_hr, ra_min;
int dec_deg, dec_min;
float sid,ra,ha,dec;
char sn = ' ';

  sid = to_degrees_ha( sid_hr, sid_mn, sid_sec );
  ra = sid - p->HA;
  if( ra < 0 ) ra += 360;
  ra =  ra / 15.0;
  ra_hr = ra;
  ra -= ra_hr;
  ra_min = ra * 60.0;
 
  ha = p->HA / 15.0;
  if( ha < 0 ) sn = '-', ha = -ha;
  ha_hr = ha;
  ha -= ha_hr;
  ha_min = ha * 60.0;  

  LCD.print((char *)"Alt : ", 0, ROW3 );
  LCD.printNumF( p->ALT, 2, 7*6, ROW3, '.', 5 );
  LCD.print((char *)"Az  : ", 0, ROW4 );
  LCD.printNumF( p->AZ, 2, 7*6, ROW4, '.', 5 );
  

  LCD.print((char *)"HA  : ", 0, ROW5 );
  LCD.putch( sn );
  LCD.printNumI( ha_hr, 7*6, ROW5, 2, '0');
  LCD.putch(' '); LCD.putch(':'); LCD.putch(' ');
  LCD.printNumI( ha_min, 12*6, ROW5, 2, '0');

  LCD.print((char *)"RA  : ", 0, ROW6 );
  LCD.printNumI( ra_hr, 7*6, ROW6, 2, '0');
  LCD.putch(' '); LCD.putch(':'); LCD.putch(' ');
  LCD.printNumI( ra_min, 12*6, ROW6, 2, '0');

  sn = ' ';
  dec = p->DEC_;
  if( dec < 0 ) sn = '-', dec = -dec;
  dec_deg = dec;
  dec -= dec_deg;
  dec_min = dec * 60.0;

  LCD.print((char *)"DEC : ",0,ROW7 );  LCD.putch(sn);
  LCD.printNumI( dec_deg, 7*6, ROW7, 2, ' ');
  LCD.print((char *)" deg ",9*6,ROW7 );
  LCD.printNumI( dec_min, 15*6,ROW7,2,' ' );
  LCD.putch('\'');
  
}

void serial_display_pointing(struct POINTING *p){    // debug

   Serial.print("DEC  ");  Serial.print( p->DEC_);
   Serial.print("  HA  "); Serial.print( p->HA );
   Serial.print("  Side  ");  Serial.println( p->side );
   Serial.print("Alt  "); Serial.print( p->ALT );
   Serial.print("  Az "); Serial.print( p->AZ );
   Serial.print("  AltRate "); Serial.print( p->ALT_rate,5 );
   Serial.print("  AzRate "); Serial.println( p->AZ_rate,5 );
   Serial.print("DEC' "); Serial.print( p->DECp );
   Serial.print("  HA' "); Serial.print( p->HAp );
   Serial.print("  DEC'rate "); Serial.print( p->DECp_rate,5 );
   Serial.print("  HA'rate "); Serial.println( p->HAp_rate,5 );
   Serial.println();
}


// conversions
float to_degrees_ha( int8_t hr, int8_t mn, int8_t sc ){
float deg;

   deg = (float)hr + (float)mn/60.0 + (float)sc/3600.0;
   return 15.0 * deg;
}

float to_degrees_dec( int8_t dg, int8_t mn, int8_t sc ){
float deg;
float sign;

   sign = 1.0;
   if( dg < 0 ) sign = -1.0, dg = -dg;
   deg = (float)dg + (float)mn/60.0 + (float)sc/3600.0;
   return sign * deg;
  
}
