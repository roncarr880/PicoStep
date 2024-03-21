
// wiring to CNCv3 board
// Hardware used:  Pi Pico, Arduino Protoboard, 128x64 I2C OLED, DS3231 RTC, CNCv3 Stepper driver shield, 
//   Encoder with switch, Bluetooth serial, LM7805 5 volt regulator, headers, wire.

// GP 4 and 5 are I2C pins, in use.
// GP 16,17,18,19 are SPI pins, saved.

// Pi Pico is on an arduino UNO proto board.  CNCv3 plugs directly in to the headers.
// On CNCv3 the Vin pin is not connected to anything, connect it to +12 volts after the fuse with a wire.
// On proto board have 12v from CNCv3 on Vin to 5 volt regulator in, out 5 volts to PiPico Vsys.
// PiPico 3.3 out to 3.3 power and to the I2C RTC and OLED screen.  SDA and SCL are wired as appropriate.
// connect 3.3v to 5v on proto board to use 3.3 VDD on CNCv3
//  ! revisit wiring if use TMC2130, power up issue will need isolation of 3.3 power
//  ! in this case 3.3 to CNCv3 5v pin should come from 5 volt regulator only via diode drops
//  ! and Vsys should be fed with a diode from 5 volt regulator to isolate from Vbus power.
// The stepper drivers and motors draw a lot of current if the Vmotor drops as when power supply is current limiting
// and do not recover if the current limit is increased. Add a 12 volt monitor via Analog input and
// turn off the motor drivers if the 12 volts is not correct.
// Wire 12v - 10k - 1k - GND voltage divider, junction of 10k/1k to A2

//   Pi Pico GP Pin           Wire to arduino Proto Board Pin for CNCv3 connection

#define RA_STEP 6            // D2
#define RA_DIR  7            // D5
#define DEC_STEP 8           // D3
#define DEC_DIR  9           // D6
#define FOCUS_STEP 2         // D4  wire if used
#define FOCUS_DIR  3         // D7  wire if used

#define DRV_ENABLE 10        // D8

 // dummy motor for pointing model, set to sidereal, solar or lunar rate
#define HA_STEP  11           // not wired anywhere
#define HA_DIR   11
#define DC_STEP  11
#define DC_DIR   11

#define ENC_A    13           // wire to encoder, not to CNCv3
#define ENC_B    12
#define ENC_SW   14

 // wire serial1 pins
 //   pico pin 0 to cncv3 0
 //   pico pin 1 to cncv3 1


// mount types 
#define FORK 0
#define GEM  1
#define ALTAZ 2
#define ALTALT 3
uint8_t mount_type = GEM;

#define SIDE_EAST -1.0
#define SIDE_WEST  1.0

// constants
#define RAreverse 0
#define DECreverse 0
#define HA_STEPS_PER_DEGREE 3600L       // 3600  dummy stepper 1 sec of resolution
#define DC_STEPS_PER_DEGREE 1000L       // !!! 3600 just using 1000 to read angle from step count
#define DEC_STEPS_PER_DEGREE 1000L      // !!! made up numbers for now
#define RA_STEPS_PER_DEGREE 1000L      // steps * micro steps * gearing / 360

// location
float my_longitude = -69.524125;
float my_latitude  = 44.447;
//float my_latitude = 30.0;

// limits
float horizon_limit =  1.0;
float overhead_limit = 80.0;           // less < 90 for scope that interfers with tripod legs
float ha_east_limit = -100.0;          // -90 keeps scope on top
float ha_west_limit = 100.0;           // more than 90 allows under pole for GEM
float alt_north_limit = 80.0;          // alt alt mount only, yoke limits if mounted in a yoke
float alt_south_limit = -80.0;         // alt alt mount only

// special
#define MYSCOPE                      // my GEM needs to use side east when pointing southeast.  Comment out for normal
                                     // use. Or this may be useful to cross meridian without a flip when pointed south.

                                     
