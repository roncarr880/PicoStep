//  Pi Pico
//  telescope goto and clock drive


// Notes:
// A minute of longitude vs a minute of declination
//   A minute of declination is 1/60 of a degree.   0.016666666 deg/min
//   A minute of longitude, an hour or 60 minutes is 15 deg.  0.25 deg/min
// each second add 1.0027379 to sidereal time, or ( total delta seconds / 365.2425) * 366.2425
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

extern unsigned char SmallFont[];
extern unsigned char MediumNumbers[];
extern unsigned char BigNumbers[];


#define SFACTOR0 0.0027379           // sidereal  fast by this amount
#define SFACTOR1 1.0027379

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

  DECstep.enableOutputs();
  RAstep.enableOutputs();
  pinMode( DRV_ENABLE, OUTPUT );
  digitalWrite( DRV_ENABLE, LOW );    // enable low

  // timer running at 1ms. Max speed will be timer limited at 1000.
  // step mode 16, 200 step motor  18.75 rpm max or 3.2 seconds for one rotation
  // doubled to 2000, timer at 0.5ms
  RAstep.setMaxSpeed( 2010.0 );    // need to be last in order to work at all?
  DECstep.setMaxSpeed( 2010.0 );

  // moveTo( position relative to zero starting position ) for goto's
  // move( relative to the current position );
  // setCurrentPosition( for fixups )
  RAstep.moveTo( -1 );                 // checking if alive
  DECstep.moveTo( 1 );                 // !!! should init to home position rather than zero
  finding = 1;                         // flag goto in progress, need to reset sidereal speed when done

  ITimer0.attachInterruptInterval(500, TimerHandler0);   // time in us, 1000 == 1ms, 500 = 0.5ms

  disp_status(STAR);                   // !!! starts with clock drive enabled, is this ok?
  get_GMT_base();
  vtest_count = sidereal_count = millis();  

}


// keep the steppers running
bool TimerHandler0(struct repeating_timer *t){
  (void) t;
  static uint8_t toggle;

  DECstep.run();   // !!! will need to use finding and setSpeed if using AltAz mount

  // a goto clears the constant speed setting, need to set it again after a goto
  // distanceToGo uses the old target, can't call it to see if need to call run or
  // runSpeed.  If run is called when not goto'ing it will try to move to the old target.
  // So need a flag ( finding ) to control what function to call.
  if( finding ){
     RAstep.run();
     if( RAstep.distanceToGo() == 0 ) RAstep.setSpeed(  0.71 ), finding = 0;
  } 
  else if( RAstep.runSpeed() ){
    digitalWrite( 25, toggle );
    toggle ^= 1;
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
   Serial.write('d'); Serial.print( dec );  Serial.write(' ');
   Serial.write('h'); Serial.write('a'); Serial.print( ha ); Serial.write(' ');

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
      stp *= 5;                  // fake steps per 1 minute of dec
      
      noInterrupts();
      RAstep.moveTo( 0 );        // just crossed the meridian
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

   display_stars();
}



void display_stars( ){
static int indx = 2555;
int i;
int hr, mn;

   hr = sid_hr;  mn = sid_mn;
   if( hr > 23 ) hr -= 24;
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
   LCD.print( line5, 0, ROW5 );  LCD.clrRow(5,strlen(line5)*6 );
   LCD.print( line6, 0, ROW6 );  LCD.clrRow(6,strlen(line6)*6 );
   LCD.print( line7, 0, ROW7 );  LCD.clrRow(7,strlen(line7)*6 );
  
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
