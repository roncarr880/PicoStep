//  Pi Pico telescope goto and clock drive


// Notes:
// A minute of longitude vs a minute of declination
//   A minute of declination is 1/60 of a degree.   0.016666666 deg/min
//   A minute of longitude, an hour or 60 minutes is 15 deg.  0.25 deg/min
// each second add 1.002737907 to sidereal time, or ( total delta seconds / 365.2425) * 366.2425
// from OAT  #define SIDEREAL_SECONDS_PER_DAY 86164.0905f

#include <Arduino.h>
#include "PinMap.h"
#include "BrightStars.h"
#include <time.h>
#include <Wire.h>
#include <OLED1306_Basic.h>
#include <DS3231.h>
#include <RPi_Pico_TimerInterrupt.h>
#include <AccelStepper.h>

#define ROW0 0
#define ROW1 8
#define ROW2  16
#define ROW3  24
#define ROW4  32
#define ROW5  40
#define ROW6  48
#define ROW7  56

  OLED1306 LCD;
  DS3231 RTC;
  RPI_PICO_Timer ITimer0(0);
  AccelStepper RAstep(AccelStepper::DRIVER, RA_STEP, RA_DIR);    // pins step, direction
  AccelStepper DECstep(AccelStepper::DRIVER, DEC_STEP, DEC_DIR);
  AccelStepper HAstep(AccelStepper::DRIVER, HA_STEP, HA_DIR);    // dummy motor

extern unsigned char SmallFont[];
extern unsigned char MediumNumbers[];
extern unsigned char BigNumbers[];


#define SFACTOR0 0.002737907           // sidereal  fast by this amount
#define SFACTOR1 1.002737907

// status display, tracking mode
#define POWER_FAIL 0
#define HOME 1
#define OFF  2
#define STAR 3          // > off is moving
#define SUN  4
#define MOON 5


DateTime GMT_eq;
DateTime today;


uint8_t sid_hr, sid_mn, sid_sec;
volatile uint8_t finding;            // goto started

// jobs run on timers
uint32_t vtest_count;                // test 12 volt power, turn all off if falls below 9 volts
uint32_t sidereal_count;             // update sidereal time and display

uint8_t power_fail;                  // latched error condition
uint8_t tracking = STAR;             // !!! probably should start at HOME

#define U_6STAR 0                // user modes, idle, display 6 database objects as they cross the meridian
#define U_POINT 1                // pointing model on 3 lines, ra, ha, dec; stars on 3 lines
#define U_RA    2                // alter RA with encoder, move scope
#define U_DEC   3                // alter DEC with encoder, move scope
#define U_GOTO  4                // select an object and goto with encoder
uint8_t u_mode = U_POINT;        // !!! testing here with U_POINT as init value

int new_target, old_target;          // testing vars currently



void setup() {

  delay(200);
  i2init();
  Serial.begin(9600);

  LCD.InitLCD();
  LCD.setFont(SmallFont);
  LCD.clrScr();
  LCD.print((char *)"Hello",LEFT,ROW0);
  delay(1000);
  LCD.clrRow(0);  LCD.clrRow(1);
    
  pinMode( 25, OUTPUT );              // LED pin

  RAstep.setAcceleration( 20.0 );
  RAstep.setMinPulseWidth( 3 );
  //RAstep.setEnablePin( DRV_ENABLE );        // just control this pin without using the stepper library
  RAstep.setPinsInverted( RAreverse,0,0);     // direction, step , enable low
  DECstep.setAcceleration( 20.0 );
  DECstep.setMinPulseWidth( 3 );
  DECstep.setPinsInverted( DECreverse,0,0);
  HAstep.setAcceleration( 20.0 );            // fake motor, do not enable output(s). using accelstepper to keep time.
  HAstep.setMinPulseWidth( 3 );

  DECstep.enableOutputs();
  RAstep.enableOutputs();
  pinMode( DRV_ENABLE, OUTPUT );
  digitalWrite( DRV_ENABLE, LOW );    // enable low

  // timer running at 1ms. Max speed will be timer limited at 1000.
  // step mode 16, 200 step motor  18.75 rpm max or 3.2 seconds for one rotation
  // doubled to 2000, timer at 0.5ms
  RAstep.setMaxSpeed( 2010.0 );    // need to be last in order to work at all?
  DECstep.setMaxSpeed( 2010.0 );
  HAstep.setMaxSpeed( 2010.0 );    // !!! revisit

  // moveTo( position relative to zero starting position ) for goto's
  // move( relative to the current position );
  // setCurrentPosition( for fixups )
  //RAstep.moveTo( -1 );                 // checking if alive
  //DECstep.moveTo( 1 );                 // !!! should init to home position rather than zero
  DECstep.setCurrentPosition( 5L * 60L * 90L );  // !!! init using the fake rate
  finding = 1;                         // flag goto in progress, turns on sidereal rate

  ITimer0.attachInterruptInterval(500, TimerHandler0);   // time in us, 1000 == 1ms, 500 = 0.5ms

  disp_status(STAR);                   // !!! starts with clock drive enabled, is this ok?
  get_GMT_base();
  vtest_count = sidereal_count = millis();  

}


// keep the steppers running
bool TimerHandler0(struct repeating_timer *t){
  (void) t;
  static uint8_t toggle;

  //DECstep.run();   // !!! will need to use finding and setSpeed if using AltAz mount

  // a goto clears the constant speed setting, need to set it again after a goto
  // distanceToGo uses the old target, can't call it to see if need to call run or
  // runSpeed.  If run is called when not goto'ing it will try to move to the old target.
  // So need a flag ( finding ) to control what function to call.
  // !!! what about time lost during a goto, need to catch up. 2nd find?, update finding target during find?
  if( finding ){
     RAstep.run();
     DECstep.run();
     HAstep.run();
     if( RAstep.distanceToGo() == 0 && DECstep.distanceToGo() == 0 && HAstep.distanceToGo() == 0 ){
        RAstep.setSpeed(  SFACTOR1 /** 2.345*/);     // !!! need alt az conditional for RA DEC speed
        HAstep.setSpeed(  SFACTOR1 );      // off,sidereal,solar,lunar
        DECstep.setSpeed( 0.0 );
        finding = 0;
     }
  } 
  else{
    if( HAstep.runSpeed() ){
       digitalWrite( 25, toggle );
       toggle ^= 1;
    }
    RAstep.runSpeed();
    DECstep.runSpeed();
  }

  return true;
}

void loop() {
uint32_t t;

 // dispatch jobs from timer counters or millis counters
   // calc and display sidereal time, 3 objects or double wide ra,ha,dec, 
   //     display pointing model - ra, ha, dec.  Idle loop I think 5 sec
   // measure 12 volt power is ok, turn off stepper drivers if not ok, 1 sec
   // encoder and switch reading - 1 ms
   // serial and serial1, command processor - when available, not timer I think
   // local goto from encoder, use moveTo or move?
   // local goto from SBO database
   // limits

   t = millis();

   if( t - sidereal_count >= 5000 ) sidereal_count = t, get_GMT_base();
   if( power_fail == 0 && t - vtest_count >= 971 ) vtest_count = t, vtest();
 
   // t = encoder();
   // t = buttons();

// testing
   noInterrupts();
   long ha = RAstep.currentPosition();
   long dec = DECstep.currentPosition();
   interrupts();
   //Serial.write('d'); Serial.print( dec );  Serial.write(' ');
   //Serial.write('h'); Serial.write('a'); Serial.print( ha ); Serial.write(' ');

static int loops = 75;   // !!! test specific values of declination, test loops
   // fake goto the current target
   if( new_target != old_target ){
      old_target = new_target;

      // abs macro bugs.....?
      long stp = (long)bstar[new_target].dd;
      if( stp < 0 ) stp = -stp;
      stp *= 60L;
      stp += (long)bstar[new_target].dm;
      // long stp = abs( (long)bstar[new_target].dd ) * 60 + (long)bstar[new_target].dm;
      if( bstar[new_target].dd < 0 ) stp = -stp;
  if( loops > -30 ){    
     stp = loops * 60;
     loops -= 15;
  }    
      stp *= 5;                  // fake steps per 1 minute of dec
      
      noInterrupts();
      HAstep.moveTo( 0 * 60 * 60 );        // h * 60 * 60
      RAstep.moveTo( 0 * 60 * 60 );        // hour angle of test star, west positive here
      DECstep.moveTo( stp );
      finding = 1;               // important to set the moving flag
      interrupts();
      Serial.println( stp );
   }
   
}


// check power supply, if voltage sags the current is not controlled, fatal error
// let rest of program run, steppers should be stopped with no current
void vtest(){
int v;

   v = analogRead( A2 );
   if( v < 251 || v > 561 ){                // 9 to 20 volt ok range
      // RAstep.disableOutputs();
      // DECstep.disableOutputs();
      digitalWrite( DRV_ENABLE, HIGH );     // disable stepper drivers
      power_fail = 1;                       // latched, only recovery is to power cycle.
      disp_status( POWER_FAIL );
   }
  
}


const char stat[6][5] = {
  "PWR ", "HOME", "OFF ", "STAR", "SUN ", "MOON" 
};

// write to status area of the screen, top lines, left side
void disp_status( uint8_t msg ){

   //LCD.clrRow(0,0,29);    // always print 4 characters?
   //LCD.clrRow(1,0,29);

   LCD.print( stat[msg],0,ROW0 );

   // 2nd line if needed
   switch( msg ){
      case POWER_FAIL:
         LCD.print((char *)"FAIL",0,ROW1);
      break;
   }
  
}

void disp_pointing(){      // where is the scope pointing !!! probably want to seperate out figuring and displaying
long ra_p, dec_p, ha_p;    // or have an argument if want to display in small or medium numbers
long dec_deg, dec_min;
long ha_hr, ha_min, ha_sec;
long ra_hr, ra_min;
float ha_degrees, dec_degrees;

   ha_p = RAstep.currentPosition();
   dec_p = DECstep.currentPosition();

   if( RAreverse ) ha_p = -ha_p;      // or does it report reversed when reversed?
   if( DECreverse) dec_p = -dec_p;

  // uint8_t sid_hr, sid_mn, sid_sec; current sidereal time
  // factors needed 
  // dec is hardcoded 5 * minutes for testing !!!
  // ra/ha - with sidereal rate with no factor, just divide by 60 to get minute rate

  dec_p /= 5;  
  char sn = ' ';  if( dec_p < 0 ) sn = '-', dec_p = -dec_p;
  dec_deg = dec_p/60;  dec_min = dec_p % 60;
  //dec_degrees = (float)dec_deg + (float)dec_min / 60.0;
  dec_degrees = to_degrees_dec( dec_deg, dec_min, 0 );
  //ha_p /= 60;   ha_hr = ha_p/60;   ha_min = ha_p % 60;
  ha_hr = ha_p/3600;   ha_p -= ha_hr * 3600;
  ha_min = ha_p / 60;  ha_p -= ha_min * 60;  ha_sec = ha_p;
  //ha_degrees = (float)ha_hr + (float)ha_min / 60.0 + (float)ha_sec / 3600.0 ;
  //ha_degrees *= 15.0;
  ha_degrees = to_degrees_ha( ha_hr, ha_min, ha_sec );
  
  ra_hr = sid_hr - ha_hr;   ra_min = sid_mn - ha_min;
  //ra_hr = ha_hr - sid_hr;   ra_min = ha_min - sid_mn;
  
  if( ra_min < 0 ) ra_min += 60, --ra_hr;
  if( ra_hr < 0 ) ra_hr += 24;
  if( ra_min > 59 ) ra_min -= 60, ++ra_hr;
  if( ra_hr > 23 ) ra_hr -= 24;
  // or ra_hr = ra_hr % 24;

  LCD.print((char *)"HA  : ", 0, ROW5 );
  LCD.printNumI( ha_hr, 7*6, ROW5, 2, '0');
  LCD.putch(' '); LCD.putch(':'); LCD.putch(' ');
  LCD.printNumI( ha_min, 12*6, ROW5, 2, '0');

  LCD.print((char *)"RA  : ", 0, ROW6 );
  LCD.printNumI( ra_hr, 7*6, ROW6, 2, '0');
  LCD.putch(' '); LCD.putch(':'); LCD.putch(' ');
  LCD.printNumI( ra_min, 12*6, ROW6, 2, '0');

  LCD.print((char *)"DEC : ",0,ROW7 );  LCD.putch(sn);
  LCD.printNumI( dec_deg, 7*6, ROW7, 2, ' ');
  LCD.print((char *)" deg ",9*6,ROW7 );
  LCD.printNumI( dec_min, 15*6,ROW7,2,' ' );
  LCD.putch('\'');
  
  if( sn == '-' ) dec_degrees = -dec_degrees;
  find_alt_az( ha_degrees, dec_degrees );
}


void find_alt_az( float ha, float dec ){
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

    Serial.print("DEC  ");  Serial.print( dec );
    Serial.print("  HA  "); Serial.print( ha );
    Serial.print("  Lat "); Serial.println(my_latitude);
static float oldha;
    float diffha = ha - oldha;
    oldha = ha;
    
float m_lat;
float ALT, A, AZ;   

    ha /= 57.2958;   dec /= 57.2958;  m_lat = my_latitude/57.2958;
    ALT = sin(dec) * sin(m_lat) + cos(dec) * cos(m_lat) * cos(ha);
    ALT = constrain(ALT,-1.0,1.0);
    ALT = asin(ALT);
    A = (sin(dec) - sin(ALT) * sin(m_lat)) / ( cos(ALT) * cos(m_lat));
    A = constrain(A,-1.0,1.0);
    AZ = acos(A);
    float AZp = AZ;

    AZ *= 57.2958;  ALT *= 57.2958;
    if( sin(ha) >= 0 ) AZ = 360 - AZ;
    //if( A == NAN ){
    //   dec *= 57.2958;
    //   A = ( dec < my_latitude ) ? 180 : 360;
    //}

   float AZrate, ALTrate;

    AZrate = tan(ALT/57.2958)*cos(AZp)*cos(m_lat) - sin(m_lat);
    AZrate = -AZrate;                                          // !!! ok for all cases?
    ALTrate = -sin(AZp)*cos(m_lat);
    
    Serial.print( "Alt  " ); Serial.print( ALT ); 
    Serial.print( "  Az "); Serial.print( AZ );
    Serial.print("  AltRate "); Serial.print( ALTrate,5 );
    Serial.print("  AZrate ");  Serial.println( AZrate,5 );

    // lazy yoke calculation
    // ha dec m_lat in radians

    float TH, PHi, x, y, z, xp, yp, zp;
 

    // change to rectangular coord
    //TH = dec;  PHi = ha;
    TH = 90.0/57.2958 - dec;  PHi = -ha;
    
    x = sin(TH) * cos(PHi);
    y = sin(TH) * sin(PHi);
    z = cos(TH);
    // rotate by latitude
    xp = x * cos( m_lat ) + z * sin( m_lat );
    yp = y;
    zp = z * cos( m_lat ) - x * sin( m_lat );
    // change back to polar coord
    PHi = atan2(yp, xp);
    TH = acos( zp );

    float DECp_rate =  sin(-PHi) * sin( m_lat);

    Serial.print("DEC' "); Serial.print( 90.0 - TH * 57.2958 );     // TH * 57.2958 - 90
    Serial.print("  HA' "); Serial.print( -PHi * 57.2958 );
    Serial.print("  DEC' rate ");  Serial.print( DECp_rate,5 );

 float newPhi = -57.2958 * PHi; 
 static  float oldPHi;
 static float avrate;
 float diffPHi = newPhi - oldPHi;
     oldPHi = newPhi;
 float oldavrate = avrate; 
 
    if( diffha != 0.0 ){
        //avrate = 1.0 * avrate + diffPHi/diffha;
        //avrate /= 2.0;
        avrate = diffPHi/diffha;
    }
    //avrate = constrain(avrate,-3.0,3.0);
    Serial.print("  HaRate ");  Serial.print( avrate, 5 );

    //float HAcalc = tan(90.0/57.2958 - TH )*cos(-PHi)*cos(m_lat) - sin(m_lat);   // -PHi ?
    //float HAcalc = tan(dec)*cos(PHi)*cos(m_lat) - sin(m_lat);   // -PHi ?
    //float HAcalc = tan(dec)*cos(-PHi)*cos(m_lat); // - sin(m_lat);   // -PHi ?
    // float HAcalc = tan(90.0/57.2958 - TH)*cos(m_lat) - sin(m_lat);   //
    // float HAcalc = tan(90.0/57.2958 - TH);  
    // float HAcalc = sin(TH) * cos(m_lat) + sin(m_lat);
    // float HAcalc = sin(TH) - sin(m_lat);
    // float HAcalc = cos( 90/57.2958 - TH ) - cos(m_lat);
    //COS(LAT) - tan(DEC')*sin(LAT)
    float HAcalc = cos( m_lat) - tan( 90/57.2958 - TH) * sin(m_lat) * cos( PHi);  // cos(phi) guess
    Serial.print("  Calc ");  Serial.println( HAcalc,5);
     //Serial.println();
   Serial.print( TH * 57.2958 );   Serial.write(' ');
   Serial.print( 90 - TH * 57.2958 );   Serial.write(' ');
   Serial.print( tan(TH) );   Serial.write(' ');
   Serial.print( tan( 90/57.2958- TH) );   Serial.write(' ');
   Serial.print( sin( m_lat) );   Serial.write(' ');
   Serial.print( cos( m_lat) );   Serial.write(' ');
   Serial.print( cos( TH ) );   Serial.write(' ');            // same1
   Serial.print( cos( 90/57.2958 - TH) );   Serial.write(' ');   // same2
   Serial.print( sin(TH) );   Serial.write(' ');              // same2
   Serial.print( sin( 90/57.2958 - TH) );   Serial.write(' ');    // same1


    Serial.println();
 
    //Serial.println( cos(90/57.2958 - TH ) - sin(m_lat));
    
    // 4 * 60 = 240 seconds in 1 degree
   // float newDEC = 90.0 - TH * 57.2958;
   // static float oldDEC;

   // float dec_rate = 4 * 60 * (newDEC - oldDEC);
   // oldDEC = newDEC;
   // Serial.print("   DEC rate "); Serial.print( dec_rate/5.0, 5);
   // Serial.print("  sin(ha')sin(lat) ");  Serial.println( sin(-PHi) * sin( m_lat),5);

    Serial.println();

}


// start from date 2035/01/01 and known gmt sidereal time, find difference to todays date,
// scale diff to sidereal seconds length, add/sub offset for longitude,
// convert to a data/time and save the hours, minutes, seconds
void get_GMT_base(){
int64_t t1,t2,diff;
uint8_t h,m,s;

   GMT_eq = DateTime( 2035, 1, 1, 0, 0, 0 );
       
   // get the today info from the RTC which is using UTC time
   today = RTClib::now();
   h = today.hour();   m = today.minute();  s = today.second();
   if( h < 10 ) Serial.write('0');
   Serial.print( h ); Serial.write(':');
   if( m < 10 ) Serial.write('0');
   Serial.print( m ); Serial.write(':');
   if( s < 10 ) Serial.write('0');
   Serial.print( s ); Serial.print("  ");


   t1 = today.unixtime();   t2 = GMT_eq.unixtime();     // use 64 bit signed 
   diff =  t1 - t2;   
   
   diff += (float)diff * SFACTOR0;
   Serial.print("Base 2035/01/01 ");
   Serial.print( diff );   Serial.write(' ');   

   GMT_eq = DateTime( 2035, 1, 1, 6, 41, 56 );  // gmt sidereal time 2035/1/1 at zero hours
   t2 = GMT_eq.unixtime();
    
   t2 += diff;        // add sidereal adjustment

  // longitude adjustment
   t2 += my_longitude * 240.0;     // 86400/360;
   
   DateTime new_time = DateTime( (uint32_t)t2 );
   sid_hr = new_time.hour();   sid_mn = new_time.minute();  sid_sec = new_time.second();

   Serial.print( new_time.year() );   Serial.write('/');
   Serial.print( new_time.month() );  Serial.write('/');
   Serial.print( new_time.day() );    Serial.write(' ');

   LCD.setFont(MediumNumbers);
   if( sid_hr < 10 ) Serial.write('0');
   Serial.print( sid_hr );  Serial.write(':');
   LCD.printNumI(sid_hr,35,0,2,'0');
   if( sid_mn < 10 ) Serial.write('0');
   Serial.print( sid_mn );  Serial.write(':');
   LCD.printNumI(sid_mn,65,0,2,'0');
   if( sid_sec < 10 ) Serial.write('0');
   Serial.println( sid_sec );
   LCD.printNumI(sid_sec,95,0,2,'0');
   LCD.setFont(SmallFont); 

   if( u_mode < U_RA ) display_stars();
   if( u_mode == U_POINT ) disp_pointing();
}



void display_stars( ){
static int indx = 2555;
int i;
int hr, mn;

   hr = sid_hr - 0;  mn = sid_mn;     // west negative here 
   if( hr > 23 ) hr -= 24;
   if( hr < 0 ) hr += 24;
   // first time
   if( indx == 2555 ){
      for( i = 0; i < NUMSTAR; ++i ){
         if( bstar[i].hr > hr ) break;
         if( bstar[i].hr == hr && bstar[i].mn > mn ) break;
         display_stars2( i );
         delay(10);
      }
      indx = i;
      // if( indx < 0 ) indx = NUMSTAR - 1;
      if( indx >= NUMSTAR ) indx = 0;
      new_target = indx;    // !!! testing, goto on startup
   }

   // check if reached the next star
   if( hr == bstar[indx].hr && mn == bstar[indx].mn ){
      display_stars2( indx );
      new_target = indx;
      ++indx;
      if( indx >= NUMSTAR ) indx = 0;
   }
 
}


void display_stars2( int p ){
static char line2[25];
static char line3[25];
static char line4[25];
static char line5[25];
static char line6[25];
static char line7[25];

   strcpy( line7, line6 );
   strcpy( line6, line5 );
   strcpy( line5, line4 );
   strcpy( line4, line3 );
   strcpy( line3, line2 );

   strcpy( line2,bstar[p].con );
   strcat( line2, " " );
   strncat( line2, bstar[p].sname,15 );
   //strcat( line2, bstar[p].sname );
   line2[24] = 0;
   
   LCD.print( line2, 0, ROW2 );  LCD.clrRow(2,strlen(line2)*6 );
   LCD.print( line3, 0, ROW3 );  LCD.clrRow(3,strlen(line3)*6 );
   LCD.print( line4, 0, ROW4 );  LCD.clrRow(4,strlen(line4)*6 );
   if( u_mode == U_6STAR ){
      LCD.print( line5, 0, ROW5 );  LCD.clrRow(5,strlen(line5)*6 );
      LCD.print( line6, 0, ROW6 );  LCD.clrRow(6,strlen(line6)*6 );
      LCD.print( line7, 0, ROW7 );  LCD.clrRow(7,strlen(line7)*6 );
   }
  
}


// conversions
float to_degrees_ha( int8_t hr, int8_t mn, int8_t sc ){
float deg;

   deg = (float)hr + (float)mn/60.0 + (float)sc/3600.0;
   return 15.0 * deg;
}

float to_degrees_dec( int8_t dg, int8_t mn, int8_t sc ){
float deg;

   deg = (float)dg + (float)mn/60.0 + (float)sc/3600.0;
   return deg;
  
}

/*****  extern I2C  functions needed by the OLED library  ******/
uint8_t i2byte_count;       // avoid overfilling i2c buffer ( 32 size for pi pico ?, it is 256 it seems )
uint8_t i2adr_saved;

void i2init(){
  Wire.begin();
  Wire.setClock(400000);
}

void i2start( uint8_t adr ){
  Wire.beginTransmission(adr);
  i2byte_count = 1;
  i2adr_saved = adr;
}


void i2send( unsigned int data ){
  if( ++i2byte_count == 253 ){
     i2stop();
     i2start( i2adr_saved ); 
  }
  Wire.write(data);
}
  

void i2stop( ){
  Wire.endTransmission();
  i2byte_count = 0;
}



//********************************************saved old code **********************************************


#ifdef NOWAY

void display_stars2old( uint8_t p ){
static char snames[20];
static char cnames[20];
static char temp[45];

 // Serial.print( bstar[p].sname );  Serial.write(' ');

  strncpy( temp,bstar[p].sname,6 );  temp[6] = 0;
  strcat( temp, " " );
  strcat( temp, snames );
  strncpy( snames, temp, 18 );  snames[19] = 0;
  LCD.print( snames, 0, ROW2 );

  strncpy( temp, bstar[p].con,3 ); temp[3] = 0;
  strcat( temp, " ");
  strcat( temp, cnames );
  strncpy( cnames, temp, 18);  cnames[19] = 0;
  LCD.print( cnames, 0, ROW3 );
  
}


#endif
