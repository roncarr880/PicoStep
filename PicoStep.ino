//  Pi Pico telescope goto and clock drive


// Notes:
// A minute of longitude vs a minute of declination
//   A minute of declination is 1/60 of a degree.   0.016666666 deg/min
//   A minute of longitude, (an hour or 60 minutes is 15 deg).  0.25 deg/min, factor of 15
// each second add 1.002737907 to sidereal time, ( total delta seconds / 365.2425) * 366.2425


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
  AccelStepper DCstep(AccelStepper::DRIVER, DC_STEP, DC_DIR);    // dummy motor

extern unsigned char SmallFont[];
extern unsigned char MediumNumbers[];
extern unsigned char BigNumbers[];


#define SFACTOR0 0.002737907           // sidereal  fast by this amount
#define SFACTOR1 1.002737907

// status display, tracking mode
#define POWER_FAIL 0
#define HOME 1          // go home
#define OFF  2
#define STAR 3          // > off is moving
#define SUN  4
#define MOON 5
#define RSET 6          // reset, at home


DateTime GMT_eq;
DateTime today;


uint8_t sid_hr, sid_mn, sid_sec;     // local sidereal time
volatile uint8_t finding;            // goto started
uint8_t flipping;                    // GEM meridian flip in progress
uint8_t gem_ignore_limits;           // GEM intermediate movements, meridian flip


// jobs run on timers
uint32_t vtest_count;                // test 12 volt power, turn all off if falls below 9 volts
uint32_t sidereal_count;             // update sidereal time and display
uint32_t finding_count;
uint32_t limit_count;

uint8_t power_fail;                  // latched error condition
uint8_t track_selection = RSET;
uint8_t tracking = RSET;


#define U_POINT 0                // telescope pointing display, select tracking mode
#define U_GOTO  1                // select a database object with encoder and goto on long press
#define U_RA    2                // alter RA with encoder, move scope
#define U_DEC   3                // alter DEC with encoder, move scope
uint8_t u_mode = U_POINT;        // user interface mode1

//int new_target, old_target=400;      // testing vars currently
int meridian_star;               // database star/object crossing meridian
int target_star;

// speeds
#define SIDEREAL SFACTOR1
#define SOLAR 1.0
#define LUNAR 0.979

float HAspeed, RAspeed, DCspeed, DECspeed;

#include "Pointing.h"            // more program code
#include "Encoder.h"



void setup() {

  delay(200);
  i2init();
  Serial.begin(9600);

  LCD.InitLCD();
  LCD.setFont(SmallFont);
  LCD.clrScr();
  LCD.print((char *)"Hello",LEFT,ROW0);
  display_help();
  delay(6000);
  //LCD.clrRow(0);  LCD.clrRow(1);
  LCD.clrScr();

  encoder_setup();
    
  pinMode( 25, OUTPUT );              // LED pin

  RAstep.setAcceleration( 500.0 );
  RAstep.setMinPulseWidth( 3 );
  //RAstep.setEnablePin( DRV_ENABLE );        // just control this pin without using the stepper library
  RAstep.setPinsInverted( RAreverse,0,0);     // direction, step , enable low
  DECstep.setAcceleration( 500.0 );
  DECstep.setMinPulseWidth( 3 );
  DECstep.setPinsInverted( DECreverse,0,0);
  HAstep.setAcceleration( 1000.0 );            // fake motor, do not enable output(s). using accelstepper to provide tracking rates
  HAstep.setMinPulseWidth( 1 );
  DCstep.setAcceleration( 1000.0 );            // fake motor, do not enable output(s)
  DCstep.setMinPulseWidth( 1 );
  
  DECstep.enableOutputs();
  RAstep.enableOutputs();
  pinMode( DRV_ENABLE, OUTPUT );
  digitalWrite( DRV_ENABLE, LOW );    // enable low

  // timer running at 1ms. Max speed will be timer limited at 1000.
  // step mode 16, 200 step motor  18.75 rpm max or 3.2 seconds for one rotation
  // doubled to 2000, timer at 0.5ms
  // double again to 0.25 ms, max 4000 steps/sec
  RAstep.setMaxSpeed( 5010.0 );    // need to be last in order to work at all?
  DECstep.setMaxSpeed( 5010.0 );
  HAstep.setMaxSpeed( 5010.0 );    // !!! revisit
  DCstep.setMaxSpeed( 5010 ); 

  // moveTo( position relative to zero starting position ) for goto's
  // move( relative to the current position );
  // setCurrentPosition( for fixups )
  //RAstep.moveTo( -1 );                 // checking if alive
  //DECstep.moveTo( 1 );                 // !!! should init to home position rather than zero
    // !!! init using the fake rate
  //telescope.DEC_ = 90.0;
  // !!! should be calling the at home function here !!!
  //DCstep.setCurrentPosition( DC_STEPS_PER_DEGREE * 90L );       // at 90 degree
  //if( mount_type == GEM ){
  //   telescope.side = SIDE_EAST;
     ////RAstep.setCurrentPosition( RA_STEPS_PER_DEGREE * 90L );
     ////HAstep.setCurrentPosition( HA_STEPS_PER_DEGREE * 90L );
     //// start at zero HA and mount disconnected from RA
     //// mount points 90 deg in dec and that is the zero step position
  //}
  //else DECstep.setCurrentPosition( DEC_STEPS_PER_DEGREE * 90L );

  at_home();
  calc_telescope( &telescope);
  
  //finding = 1;                         // !!! ?? flag goto in progress, turns on sidereal rate

  ITimer0.attachInterruptInterval(250, TimerHandler0);   // time in us, 1000 == 1ms, 500 = 0.5ms

  disp_status(tracking);
  get_GMT_base(SERIAL_DEBUG);
  init_meridian_star();
  display_stars2( meridian_star );
  calc_SBO_object( meridian_star, &SBO_object );
  display_pointing( &SBO_object );
//new_target = meridian_star;               // !!! just for testing, does a goto
   
  limit_count = finding_count = vtest_count = sidereal_count = millis();

}


// keep the steppers running
bool TimerHandler0(struct repeating_timer *t){
  (void) t;
  static uint8_t toggle;
  static uint8_t count;

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
     DCstep.run();
     if( RAstep.distanceToGo() == 0 && DECstep.distanceToGo() == 0 && HAstep.distanceToGo() == 0  \
       && DCstep.distanceToGo() == 0 ){
        RAstep.setSpeed( RAspeed );  
        HAstep.setSpeed( HAspeed );
        DECstep.setSpeed( DECspeed );
        //DCstep.setSpeed( DCspeed );    zero
        finding = 0;
     }
  } 
  else{
    if( HAstep.runSpeed() && ++count == 15){
       digitalWrite( 25, toggle );
       toggle ^= 1;
       count = 0;
    }
    RAstep.runSpeed();
    DECstep.runSpeed();
   // DCstep.runSpeed();     // should always be zero so not really needed I think
  }

  return true;
}

void loop() {
uint32_t t;
int8_t t2;

 // dispatch jobs from timer counters or millis counters
   // calc and display sidereal time, 3 objects or double wide ra,ha,dec, 
   //     display pointing model - ra, ha, dec.  Idle loop I think 5 sec
   // measure 12 volt power is ok, turn off stepper drivers if not ok, 1 sec
   // encoder and switch reading - 1 ms
   // serial and serial1, command processor - when available, not timer I think
   // local movement from encoder, use moveTo or move?
   // local goto from SBO database
   // limits

   t = millis();

   if( t - sidereal_count >= 5000 ){         // update sidereal time each 5 seconds
     sidereal_count = t;
     calc_telescope( &telescope);
     if( finding == 0 ) set_speeds( ); 
     if( next_meridian_star() == 0 ){        // keep track of meridian in star database whatever the U_mode is
        if( u_mode == U_POINT ) display_pointing( &telescope );
     }                          
   }

   if( t - finding_count >= 900 ){              // !!! pick an interval, this is slow for printing
      finding_count = t;
      que_goto( 0 );
      if( flipping ) meridian_flip( 0.0,0.0 );
   }

   if( power_fail == 0 && t - vtest_count >= 971 ) vtest_count = t, vtest();
   if( t - limit_count >= 1003 ){
      limit_count = t;
      check_limits();
   }
 
   t2 = encoder();
   if( t2 ) menu( t2 );
   t2 = switches();
   if( t2 >= TAP ) menu_action(t2), sstate[0] = DONE;

   
}



// status display, tracking mode
//#define POWER_FAIL 0
//#define HOME 1
//#define OFF  2
//#define STAR 3          // > off is moving
//#define SUN  4
//#define MOON 5 
// RSET 6      
void menu( int8_t v ){
  
  switch( u_mode ){
     case U_POINT:
        track_selection += v;
        if( track_selection < HOME ) track_selection = RSET;
        if( track_selection > RSET ) track_selection = HOME;
        disp_status( track_selection );           // showing selection, but not selected. select is long press
     break;
     case U_GOTO:
        target_star -= v;                          // clockwise to the west, reverse from database of stars
        if( target_star < 0 ) target_star = NUMSTAR -1;
        if( target_star >= NUMSTAR ) target_star = 0;
        display_stars2( target_star );
        calc_SBO_object( target_star, &SBO_object );
        display_pointing( &SBO_object );                
     break;
     case U_RA:
     break;
     case U_DEC:
     break;
     // add a UTC time adjust mode ?
  }
}

void menu_action( int8_t sw ){          // act on switch press
  // when leaving U_POINT, display_status tracking or have a timeout and display actual status

  if( sw == TAP ){
     if( ++u_mode > U_DEC ) u_mode = U_DEC;       // or have tap go back to U_RA
  }
  else if( sw == DTAP ){
     if( --u_mode < U_POINT ) u_mode = U_POINT;
  }
  else if( sw == LONGP ){
     switch( u_mode ){
       case U_POINT:
          tracking = track_selection;
          if( tracking == HOME ) go_home();
          if( tracking == RSET ) at_home();
          //disp_status( tracking );
       break;
       case U_GOTO:
          calc_SBO_object( target_star, &target );
          que_goto( &target );
          u_mode = U_POINT;                        // revert to telescope pointing display
          //disp_status( tracking );
       break;
       case U_RA:               // sync here
       case U_DEC:
       break;
     }
  }

  if( sw == DTAP || sw == TAP ){     // initilize entry to new user mode
     //disp_status( tracking );
     if( u_mode == U_GOTO ){
        target_star = meridian_star;
        display_stars2( target_star );
        calc_SBO_object( target_star, &SBO_object );
        display_pointing( &SBO_object );        
     }
     // U_RA, U_DEC here !!!
  }

  disp_status( tracking );    // !!! looks like is common to all selections
  
}



int meridian_flip( float dest_ha, float dest_dec ){
static uint8_t state;
float dest;
//float dest2;

    switch( state ){ 
       case 0:  
         if( dest_ha >= -0.01 && telescope.side == SIDE_EAST ) ;   // ok
         //if( dest_ha >= 0.0 && telescope.side == SIDE_EAST ) ;   // has extra flip sometimes dest = -0.0
       #ifdef MYSCOPE         
         // my mount needs to use  EAST side when looking southeast, also will swing through the meridian while tracking
         else if( dest_ha < 0.0 && dest_dec > 0.0 && telescope.side == SIDE_WEST ) ;  //ok
         else if( dest_ha < 0.0 && dest_dec <= 0.0 && telescope.side == SIDE_EAST ) ;  // ok
       #else
         else if( dest_ha < 0.0 && telescope.side == SIDE_WEST ) ;     // normal GEM ok
       #endif  
         else ++state, flipping = 1;
       break;
   // do we want to move the RA away from the meridian here to avoid scope hitting the tripod?
       case 1:                  // may need to move away from the mount to avoid hitting
          if( telescope.DEC_ < 60.0 && \
            ( (telescope.side == SIDE_EAST && telescope.HA < 30.0) || \
            (telescope.side == SIDE_WEST && telescope.HA > -30.0) ) ){
             dest = ( telescope.side == SIDE_EAST ) ? 30.0 : -30.0 ;
             //dest2 = dest;
             dest *=  (float)RA_STEPS_PER_DEGREE;
             //dest2 *= (float)HA_STEPS_PER_DEGREE;
             dest += telescope.side * 90.0 * (float)RA_STEPS_PER_DEGREE;
             // dest2 += telescope.side * 90.0 * (float)HA_STEPS_PER_DEGREE;  not offset!
             noInterrupts();
             RAstep.moveTo( dest );
             //HAstep.moveTo( dest2 );    // attempt at overhead limit fix, but HAstep RAstep not in sync during find.
             finding = 1;
             interrupts();
             state = 2; 
             }
          else state = 3;   
       break;
       case 2:
         if( finding == 0 ) ++state, Serial.print("Here...");         // !!! remove the serial print
       break;     
       case 3:
         go_home_GEM();   // in pointing.h
         ++state;
       break;
       case 4:
         if( finding == 0 ) ++state;
       break;
       case 5:
         telescope.side = ( telescope.side == SIDE_EAST ) ? SIDE_WEST : SIDE_EAST;
         flipping = 0;
         state = 0;
       break;
    }

    return state;
}

// a separate function for GEM mount.  Due to design of dummy scope as driver of all types the overhead limit is often
// triggered when the physical scope is not really pointing where the dummy scope is.( due to different speeds of dummy stepper
// and the physical RA stepper.) 
// also provide safe dec movement with the scope tilted away from the tripod, single axis movements only, slower but safe
void gem_que_goto( struct POINTING *p ){
static uint8_t state;
static struct POINTING *p2;
static uint32_t tm;
float more_ha;
static float save_ha, save_dec;
float dest2;

   if( state == 0 && p == 0 ) return;     // idle state
   
   switch( state ){
      case 0:
        if( p ){
          state =  1;
          p2 = p;
          tm = millis();
          gem_ignore_limits = 1;
        }
      break;
      case 1:
        state = ( meridian_flip( p2->HA, p2->DEC_ ) == 0 ) ? 3 : 2;
      break;
      case 2:
        if( flipping == 0 ) ++state;
      break;
      case 3:
        save_ha = p2->HA; save_dec = p2->DEC_;
        p2->DEC_ = telescope.DEC_;
        p2->HA = ( telescope.side == SIDE_EAST ) ? 30.0 : -30.0 ;   // move mount away from the tripod, dec stays the same
        goto_target( p2 ), ++state;
      break;
      case 4:
        if( finding == 0 ) ++state;
      break;
      case 5:                                // move dec when safe away from the mount, RA stays the same
        p2->DEC_ = save_dec;
        goto_target( p2 ), ++state;
      break;
      case 6:
        if( finding == 0 ) ++state;
      break;
      case 7:                                // move HA with RA and DEC staying the same, test if RA would be over limit if moved
        gem_ignore_limits = 0;
        dest2 = save_ha;
        dest2 *= (float)HA_STEPS_PER_DEGREE;
        noInterrupts();
        HAstep.moveTo( dest2 ); 
        finding = 1;
        interrupts();
        ++state;
      break;
      case 8:
        if( finding == 0 ) ++state;
      break;
      case 9:                                // should be just final move of RA axis in toward the mount, drivers off if over limit
        p2->HA = save_ha;  p2->DEC_ = save_dec;
        calc_pointing( p2 );
        goto_target( p2 ), ++state;
      break;
      case 10:                            // !!! skips here ????
        if( finding == 0 ) ++state;
      case 11:
         tm = (millis() - tm)/1000;
         int mn = 0;
         while ( tm >= 60 ) mn++, tm -= 60;
         more_ha = to_degrees_ha( 0, mn, tm );
         p2->HA += more_ha;
         calc_pointing( p2 );  // re-calc the other positions, alt az etc,(reports side blank in serial log)
         goto_target( p2 );
         state = 0;
         tracking = STAR;      // should we enable tracking after a goto?
         disp_status(tracking);
      break;                
   }

  if( SERIAL_DEBUG ){
    Serial.print(state);
    Serial.write(' ');
  }

}

void que_goto( struct POINTING *p ){
static uint8_t state;
static struct POINTING *p2;
static uint32_t tm;
float more_ha;


   if( mount_type == GEM ){              // separate function for GEM
      gem_que_goto( p );
      return;                                       
   }

   if( state == 0 && p == 0 ) return;     // idle state
   //if( p != 0 ) state = 0;                // re-queue when goto is active ? or comment this out
                                          // re-queue hangs in meridian flip? re-entrant issue maybe 
                                            
   switch( state ){        // !!! can remove states 1 and 2 now as GEM is separate
      case 0:              // que the goto
        if( p ){
          state = ( mount_type == GEM ) ? 1 : 3;
          p2 = p;
          tm = millis();
        }
      break;
      case 1:              // meridian flip needed?
         state = ( meridian_flip( p2->HA, p2->DEC_ ) == 0 ) ? 3 : 2;
      break;
      case 2:              // wait for flip complete
          if( flipping == 0 ) ++state;
      break;
      case 3:              // do goto
        goto_target( p2 ), ++state;
      break;
      case 4:              // wait for done
        if( finding == 0 ) ++state;
      break;
      case 5:              // 2nd find for HA delay, meridian flip delay, 
         tm = (millis() - tm)/1000;
         int mn = 0;
         while ( tm >= 60 ) mn++, tm -= 60;
         more_ha = to_degrees_ha( 0, mn, tm );
         p2->HA += more_ha;
         calc_pointing( p2 );  // re-calc the other positions, alt az etc,(reports side blank in serial log)
         goto_target( p2 );
         state = 0;
         tracking = STAR;      // should we enable tracking after a goto?
         disp_status(tracking);
      break;                
   }

  if( SERIAL_DEBUG ){
    Serial.print(state);
    Serial.write(' ');
  }
}


void check_limits(){
uint8_t ok;


  if( gem_ignore_limits ) return;                    // during goto and flip
  
  // check if moved past meridian - GEM only - meridian flip
  if( mount_type == GEM && finding == 0 && flipping == 0 ){
     if( telescope.side == SIDE_WEST && telescope.HA > 1.00 ){        // goto same should flip scope
        target.HA = telescope.HA;
        target.DEC_ = telescope.DEC_;
        que_goto( &target );
        return;
     }
  }

  ok = 1;

  // horizon limits / overhead limits from ALT
  // ALT is calculated from HAstep and not RAstep so not valid during find or flip, flip GEM has movement away from overhead before flip
  // to prevent collision with tripod.  Disable check when finding.  Overhead check only useful for GEM?
   if( (telescope.ALT < horizon_limit || telescope.ALT > overhead_limit) ) ok = 0;
   
  // FORK, GEM, ALT-ALT RA axis limits
  if( mount_type == GEM && ( telescope.HA < ha_east_limit || telescope.HA > ha_west_limit ) ) ok = 0;
  if( mount_type == FORK && ( telescope.HA < ha_east_limit || telescope.HA > ha_west_limit ) ) ok = 0;
  if( mount_type == ALTALT && ( telescope.HAp < ha_east_limit || telescope.HAp > ha_west_limit ) ) ok = 0;

  // ALTALT mount specific dec limits, yoke limits
  if( mount_type == ALTALT && ( telescope.DECp > alt_north_limit || telescope.DECp < alt_south_limit ) ) ok = 0;

  if( ok ) return;
  
  // action for exceeding limits - disable drivers, let functions think scope is moving , display message  OVR LIM
//  noInterrupts();
//  finding = 0;
//  interrupts();
//  tracking = OFF;
//  set_speeds();                           // turns off stepper movement
  digitalWrite( DRV_ENABLE, HIGH );
  LCD.print( (char *)"OVR ",0,ROW0 );
  LCD.print( (char *)"LMT ",0,ROW1 );

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


const char stat[7][5] = {
  "PWR ", "HOME", "OFF ", "STAR", "SUN ", "MOON", "RSET" 
};
const char stat2[4][5] = {
  "TRK ", "GOTO", "HA  ", "DEC "
};

// write to status area of the screen, top lines, left side
void disp_status( uint8_t msg ){

   //if( msg != POWER_FAIL && msg == tracking ) LCD.invertText( 1 );
   LCD.print( stat[msg],0,ROW0 );
   //LCD.invertText( 0 );
   if( msg == POWER_FAIL ) LCD.print((char *)"FAIL",0,ROW1);
   else LCD.print( stat2[u_mode],0,ROW1);

   if( msg != POWER_FAIL && msg == tracking && u_mode == U_POINT){
      // LCD.invertText( 1 );
       LCD.print( (char *)"*",3*6,ROW1 );
      // LCD.invertText( 0 );
   }
}



// start from date 2035/01/01 and known gmt sidereal time, find difference to todays date,
// scale diff to sidereal seconds length, add/sub offset for longitude,
// convert to a data/time and save the hours, minutes, seconds
void get_GMT_base(int dbg){
int64_t t1,t2,diff;
uint8_t h,m,s;
static uint8_t last_disp;

   GMT_eq = DateTime( 2035, 1, 1, 0, 0, 0 );
       
   // get the today info from the RTC which is using UTC time
   today = RTClib::now();
   h = today.hour();   m = today.minute();  s = today.second();

   t1 = today.unixtime();   t2 = GMT_eq.unixtime();     // use 64 bit signed 
   diff =  t1 - t2;      
   diff += (float)diff * SFACTOR0;

   GMT_eq = DateTime( 2035, 1, 1, 6, 41, 56 );  // gmt sidereal time 2035/1/1 at zero hours
   t2 = GMT_eq.unixtime();
   t2 += diff;                                  // add sidereal adjustment

  // longitude adjustment
   t2 += my_longitude * 240.0;     // 86400/360;
   
   DateTime new_time = DateTime( (uint32_t)t2 );
   sid_hr = new_time.hour();   sid_mn = new_time.minute();  sid_sec = new_time.second();

   if( dbg ){
      if( h < 10 ) Serial.write('0');
      Serial.print( h ); Serial.write(':');
      if( m < 10 ) Serial.write('0');
      Serial.print( m ); Serial.write(':');
      if( s < 10 ) Serial.write('0');
      Serial.print( s ); Serial.print("  ");
      Serial.print("Base 2035/01/01 ");
      Serial.print( diff );   Serial.write(' ');   
      Serial.print( new_time.year() );   Serial.write('/');
      Serial.print( new_time.month() );  Serial.write('/');
      Serial.print( new_time.day() );    Serial.write(' ');
      if( sid_hr < 10 ) Serial.write('0');
      Serial.print( sid_hr );  Serial.write(':');
      if( sid_mn < 10 ) Serial.write('0');
      Serial.print( sid_mn );  Serial.write(':');
      if( sid_sec < 10 ) Serial.write('0');
      Serial.println( sid_sec );
   }

   if( last_disp != sid_sec ){             // display if new time
      LCD.setFont(MediumNumbers);
      LCD.printNumI(sid_hr,35,0,2,'0');
      LCD.printNumI(sid_mn,65,0,2,'0');
      LCD.printNumI(sid_sec,95,0,2,'0');
      LCD.setFont(SmallFont);
      last_disp = sid_sec;
   }
}


void init_meridian_star(){    // find where we are when starting or starting a new goto search
int i;

    for( i = 0; i < NUMSTAR; ++i ){
       if( bstar[i].hr > sid_hr ) break;
       if( bstar[i].hr == sid_hr && bstar[i].mn >= sid_mn ) break;
    }
    if( i >= NUMSTAR ) i = 0;
    meridian_star = i;
}

// keep track of objects on meridian and display info if in idle mode
int next_meridian_star(){
int next;

   next = meridian_star + 1;
   if( next >= NUMSTAR ) next = 0;
   
   if( sid_hr == bstar[next].hr && sid_mn == bstar[next].mn ){
      meridian_star = next;
  //  new_target = next;                          // !!! testing   
      if( u_mode == U_POINT ){
        display_stars2( next );
        calc_SBO_object( next, &SBO_object );          //!!! diff object for this?
        display_pointing( &SBO_object );
        return 1;
      }         
   }
   return 0;
}


void display_stars2( int p ){
static char line2[25];

   strcpy( line2,bstar[p].con );
   strcat( line2, " " );
   strncat( line2, bstar[p].sname,15 );
   line2[24] = 0;
   
   LCD.print( line2, 0, ROW2 );  LCD.clrRow(2,strlen(line2)*6 );  
}


void display_help(){

    LCD.print( (char *)"TAP  - move forward", 0, ROW3 );
    LCD.print( (char *)"DTAP - move backward", 0, ROW4 );
    LCD.print( (char *)"LONG - Select item", 0, ROW5 );    
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
//static char line3[25];
//static char line4[25];
//static char line5[25];
//static char line6[25];
//static char line7[25];

//   strcpy( line7, line6 );
//   strcpy( line6, line5 );
//   strcpy( line5, line4 );
//   strcpy( line4, line3 );
//   strcpy( line3, line2 );

   strcpy( line2,bstar[p].con );
   strcat( line2, " " );
   strncat( line2, bstar[p].sname,15 );
   //strcat( line2, bstar[p].sname );
   line2[24] = 0;
   
   LCD.print( line2, 0, ROW2 );  LCD.clrRow(2,strlen(line2)*6 );
 //  LCD.print( line3, 0, ROW3 );  LCD.clrRow(3,strlen(line3)*6 );
 //  LCD.print( line4, 0, ROW4 );  LCD.clrRow(4,strlen(line4)*6 );
 //  if( u_mode == U_6STAR ){
 //     LCD.print( line5, 0, ROW5 );  LCD.clrRow(5,strlen(line5)*6 );
 //     LCD.print( line6, 0, ROW6 );  LCD.clrRow(6,strlen(line6)*6 );
 //     LCD.print( line7, 0, ROW7 );  LCD.clrRow(7,strlen(line7)*6 );
 //  }
  
}


void disp_pointing(){      // where is the scope pointing 
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
     /*
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
   */

    //Serial.println();
 
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


       case 101:
          dest = 0.0;    // !!! rewrite as one if then goto else state = 3 or could just breeze through state 2
          // !!! not needed if dec > 60 or there abouts
          // !!! this might belong with go_home_GEM but dropping finding == 1 will be an issue, finding == 2 ? no work
          if( telescope.side == SIDE_EAST && telescope.HA < 20.0 ) dest = 20.0;
          if( telescope.side == SIDE_WEST && telescope.HA > -20.0 ) dest = -20.0;
          if( dest == 0.0 ) state = 3;
          else{
             state = 2;
             dest *=  (float)RA_STEPS_PER_DEGREE;
             dest += telescope.side * 90.0 * (float)RA_STEPS_PER_DEGREE;
             noInterrupts();
             RAstep.moveTo( dest );
             finding = 1;
             interrupts();  
          }
       break;

#endif
