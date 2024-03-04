
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
void goto_target( struct POINTING *p );

//int longseek;
int flipping;

struct POINTING {
   float DEC_;          // where pointing from HAstep stepper steps
   float HA;
   float side;          // GEM east or west, fork under over?
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
struct POINTING SBO_object;
                                 
// update a pointing struct
void calc_pointing( struct POINTING *p ){   
float ha, dec;   

   ha = p->HA;
   dec = p->DEC_;
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
    if( AZ < 180.0 ) ALTrate = -ALTrate;    // fixup,  correct for all cases?
    
    p->ALT = ALT;
    p->AZ  = AZ;
    p->ALT_rate = ALTrate;
    p->AZ_rate = AZrate;

    // alt alt lazy yoke mount
float TH, PHi, x, y, z, xp, yp, zp;
 
    // change to rectangular coord
    //TH == dec;  PHi == ha;             // angles theta and phi
    TH = 90.0/R2D - dec;  PHi = -ha;
    
    x = sin(TH) * cos(PHi);
    y = sin(TH) * sin(PHi);
    z = cos(TH);
    // rotate by latitude about y axis
    xp = x * cos( m_lat ) + z * sin( m_lat );
    yp = y;
    zp = z * cos( m_lat ) - x * sin( m_lat );
    // change back to spherical coord
    PHi = atan2(yp, xp);
    TH = acos( zp );
    p->DECp = 90 - R2D * TH;
    p->HAp = -R2D * PHi;
    // calc rates of change ( empirical derived )
    p->DECp_rate = sin(-PHi) * sin( m_lat);
    p->HAp_rate = cos( m_lat) - tan( 90/R2D - TH) * sin(m_lat) * cos( PHi);

    if( SERIAL_DEBUG ) serial_display_pointing( p );
}

void calc_telescope( struct POINTING *p ){
long ha_p, dc_p;
float ha, dec;

    // first update the sidereal time globals, which displays clock also
   get_GMT_base( SERIAL_DEBUG );
   
   // from dummy telescope, reflects actual when not seeking,
      noInterrupts(); 
      ha_p =  HAstep.currentPosition();
      dc_p = DCstep.currentPosition();
      interrupts();
      ha = (float)ha_p / (float)HA_STEPS_PER_DEGREE;
      dec = (float)dc_p / (float)DC_STEPS_PER_DEGREE; 
      p->HA = ha;
      p->DEC_ = dec;


   calc_pointing( p );
   
}

void calc_SBO_object( int obj, struct POINTING *p ){
float dec, ra, ha, sid;

   // first update the sidereal time globals, which displays clock also
   get_GMT_base( SERIAL_DEBUG );
   
            // get position from database object
      dec = to_degrees_dec( bstar[obj].dd, bstar[obj].dm, 1 );
      ra  = to_degrees_ha( bstar[obj].hr, bstar[obj].mn, 1 );     // !!! need seconds in bstar
      sid = to_degrees_ha( sid_hr, sid_mn, sid_sec );
      ha = sid - ra;                     // ra_hr = sid_hr - ha_hr;
      if( ha > 12.0 ) ha -= 24.0;
      if( ha <= -12.0 ) ha += 24.0; 
      p->HA = ha;
      p->DEC_ = dec;

   calc_pointing( p );
}

// set speeds if tracking, from telescope 
void set_speeds(){    // remember to disable interrupts, mount type specific
float sps;

   sps = (float)HA_STEPS_PER_DEGREE / 240.0;           // steps per second, 240 is seconds in 4 minutes or 1 degree
   
   switch( tracking ){
     case POWER_FAIL:  case HOME:  case OFF:
         HAspeed = RAspeed = DCspeed = DECspeed = 0.0;
      break;
      case STAR:
         HAspeed = sps * SIDEREAL;
         DCspeed = 0.0;
      break;
      case SUN:
         HAspeed = sps * SOLAR;
         DCspeed = 0.0;
      break;
      case MOON:
         HAspeed = sps * LUNAR;
         DCspeed = 0.0;
      break;
   }
   
   switch( mount_type ){
      case FORK:
      case GEM:
        RAspeed = HAspeed * (float)RA_STEPS_PER_DEGREE / (float)HA_STEPS_PER_DEGREE;
        DECspeed = 0.0;
      break;
      case ALTAZ:
        RAspeed = HAspeed * telescope.AZ_rate * (float)RA_STEPS_PER_DEGREE / (float)HA_STEPS_PER_DEGREE;
        DECspeed = HAspeed * telescope.ALT_rate * (float)DEC_STEPS_PER_DEGREE / (float)DC_STEPS_PER_DEGREE;
      break;
      case ALTALT:
        RAspeed = HAspeed * telescope.HAp_rate * (float)RA_STEPS_PER_DEGREE / (float)HA_STEPS_PER_DEGREE;
        DECspeed = HAspeed * telescope.DECp_rate * (float)DEC_STEPS_PER_DEGREE / (float)DC_STEPS_PER_DEGREE;
      break;       
   }

   // !! use an error PID for alt az, alt alt ?  Is there a tracking problem to be fixed?

   noInterrupts();
      RAstep.setSpeed( RAspeed );   
      HAstep.setSpeed( HAspeed );
      DECstep.setSpeed( DECspeed );
      //DCstep.setSpeed( DCspeed );      // always zero I think
   interrupts();     
     
}

#ifdef NOWAY
// test and impliment meridian flip 
int meridian_flip_old( int state, struct POINTING *p ){
static struct POINTING *p2;
static float save_ha, save_dec;
uint32_t tm;
float more_ha;

   // state 0, test, 1 - 1st move, 2 - 2nd move 3 - flip and goto_target the saved pointer
   // !!! should state be an argument or just a global, will need a global anyway
   // !!! how to add to ha, same as below ?
   // !!! maybe should just impliment a goto home here

   switch( state ){
      case 0:            // test if need a flip
        if( p->HA >= 0.0 && telescope.side == SIDE_EAST ) return 0;
        if( p->HA < 0.0 && telescope.side == SIDE_WEST && p->DEC_ > 0.0 ) return 0;    // want side east for low southeast
        if( p->HA < 0.0 && telescope.side == SIDE_EAST && p->DEC_ <= 0.0 ) return 0;
        // need to flip
        flipping = 1;
        p2 = p;
        save_ha = p2->HA;   save_dec = p2->DEC_;
        tm = millis();
        return flipping;
      break;
      case 1:            // move in ra away from mount before moving dec
        if( telescope.side == SIDE_EAST && telescope.HA < 20.0 ) p2->HA = 20.0 * telescope.side;
        else if( telescope.side == SIDE_WEST && telescope.HA > -20.0 ) p2->HA = 20.0 * telescope.side;
        else p2->HA = telescope.HA;        // no movement
        p2->DEC_ = telescope.DEC_;
        ++flipping;
      break;
      case 2:            // move to home position
        p2->HA = 90.0 * telescope.side;
        //p2->HA = 0.0;
        p2->DEC_ = 90.0;
        ++flipping;
      break;
      case 3:            // flip and goto
        p2->HA = save_ha;   p2->DEC_ = save_dec;
        telescope.side = ( telescope.side == SIDE_EAST ) ? SIDE_WEST : SIDE_EAST;
        noInterrupts();
        RAstep.setCurrentPosition( telescope.side * RA_STEPS_PER_DEGREE * 90L );
        HAstep.setCurrentPosition( telescope.side * HA_STEPS_PER_DEGREE * 90L );
        interrupts();
        flipping = 0;
      break;
   }

   if( flipping == 0 ){             // add missing time
       tm = (millis() - tm)/1000;
       more_ha = to_degrees_ha( 0, 0, tm );
       p2->HA += more_ha;
       // re-calc the other positions, alt az etc
       //calc_pointing( p2 );
   }

   goto_target( p2 );
   return flipping;
}
#endif


void goto_target( struct POINTING *p2 ){
long ha,ra,dc,dec;

//  mount specific for alt az, GEM, alt alt
   ha = p2->HA * (float)HA_STEPS_PER_DEGREE;
   dc = p2->DEC_ * (float)DC_STEPS_PER_DEGREE;

   switch(mount_type){
      case FORK:
        ra = p2->HA * (float)RA_STEPS_PER_DEGREE; 
        dec = p2->DEC_ * (float)DEC_STEPS_PER_DEGREE;
      break;
      case GEM:
        float th;
        th = 90.0 - p2->DEC_;                                     // dec mirrored about 90 deg
        ra = p2->HA * (float)RA_STEPS_PER_DEGREE;
        dec = th * (float)DEC_STEPS_PER_DEGREE * telescope.side; 
        //add/sub the west/east offset for ra
        ra += telescope.side * 90.0 * (float)RA_STEPS_PER_DEGREE;
      break;
      case ALTAZ:
        ra = p2->AZ * (float)RA_STEPS_PER_DEGREE;
        dec = p2->ALT * (float)DEC_STEPS_PER_DEGREE;
      break;
      case ALTALT:
        ra = p2->HAp * (float)RA_STEPS_PER_DEGREE;
        dec = p2->DECp * (float)DEC_STEPS_PER_DEGREE;
      break;  
   }
    
      noInterrupts();
      HAstep.moveTo( ha );       
      RAstep.moveTo( ra );        // hour angle, west positive here
      DECstep.moveTo( dec );
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
static int holdoff;      // display meridian star longer than 5 seconds

  if( holdoff ){
     --holdoff;
     if( holdoff == 0 && p == &telescope ){
         LCD.clrRow(2);
         LCD.print((char *)"---Telescope---",CENTER,ROW2);
     }
     else if( p == &telescope ) return;
  }

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

  if( p != &telescope ) holdoff = 4;      // display meridian cross a bit longer

}

void serial_display_pointing(struct POINTING *p){    // debug
long hc,rc,dc,dec;

   Serial.print("DEC  ");  Serial.print( p->DEC_);
   Serial.print("  HA  "); Serial.print( p->HA );
   Serial.print("  Side_");  
   if( p->side == SIDE_EAST ) Serial.print( "East" );
   else if( p->side == SIDE_WEST ) Serial.print( "West" );
   else Serial.print( "   " );
   Serial.print("  Find  "); Serial.print(finding);
//   Serial.print("  Long  "); Serial.print(longseek);
   Serial.print("  Flip  "); Serial.println( flipping );
   Serial.print("Alt  "); Serial.print( p->ALT );
   Serial.print("  Az "); Serial.print( p->AZ );
   Serial.print("  AltRate "); Serial.print( p->ALT_rate,5 );
   Serial.print("  AzRate "); Serial.println( p->AZ_rate,5 );
   Serial.print("DEC' "); Serial.print( p->DECp );
   Serial.print("  HA' "); Serial.print( p->HAp );
   Serial.print("  DEC'rate "); Serial.print( p->DECp_rate,5 );
   Serial.print("  HA'rate "); Serial.println( p->HAp_rate,5 );
   noInterrupts();
   hc = HAstep.currentPosition();
   rc = RAstep.currentPosition();
   dc = DCstep.currentPosition();
   dec = DECstep.currentPosition();
   interrupts();
   Serial.print("HC "); Serial.print(hc);
   Serial.print("  DC "); Serial.print(dc);
   Serial.print("   RA "); Serial.print(rc);
   Serial.print("  DEC "); Serial.println(dec);
   Serial.println();
}

void at_home(){

}


void go_home(){
  
}

// GEM, two, two, two mounts in one
void go_home_GEM(){    // considering GEM home as a disconnected state in HA,RA ( ha is zero, ra is -90 or 90 )
                       // dec axis is mirrored about 90 degrees and is zero step position when at 90
      noInterrupts();
      HAstep.moveTo( 0 );                                 // dummy scope( HA ) at zero
      RAstep.moveTo( 0 );                                 // RA at 90 or -90 depending upon side, side is sort of in
      DECstep.moveTo( 0 );                                //    limbo in this position
      DCstep.moveTo( 90.0 * DC_STEPS_PER_DEGREE  );       // the dummy scope is at 90
      finding = 1;
      interrupts();
  
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
