
// Telescope pointing model for 5 scopes
//   dummy scope where one second of HA is one step of the dummy motor
//   Fork mount - could just be same as the dummy unless want to alter under pole behavior
//   GEM
//   Alt Az
//   Alt Alt or Lazy Yoke

#define SERIAL_DEBUG 1    // turn serial printing on or off

#define R2D 57.2958       // degrees in a radian

extern void get_GMT_base(int dbg);

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

// update the pointing struct, from actual position, or position of a database object
// actual position from the dummy stepper HAstep steps
// arg obj -1 for actual position
void calc_pointing( int obj ){   

  // first update the sidereal time, can display clock also
   get_GMT_base( SERIAL_DEBUG );
  
}

// set speeds if tracking - not finding and obj = -1
void set_speeds(){    // remember to disable interrupts, mount type specific
  
}

void display_pointing(){    // oled display if correct screen mode, display alt az and dummy scope
  
}

void serial_display_pointing(){    // debug
  
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
