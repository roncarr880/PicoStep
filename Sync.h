
// somewhat simplistic alignment - if it works ok then ok

float distance( float h1, float d1, float h2, float d2 );
void print_all();


struct SYNC {
    float ha;
    float dc;
    long  ra_off;
    long  dc_off;
};

#define NUM_SYNC 16
struct SYNC sync[NUM_SYNC] = {
    { -7.5, 65.0, 0, 0 },    // GEM west side, east sky, negative HA
    { -7.5, 45.0, 0, 0 },
    { -7.5, 25.0, 0, 0 }, 
    { -7.5, 0.0, 0, 0 }, 
    { -22.5, 65.0, 0, 0 },
    { -22.5, 45.0, 0, 0 },
    { -22.5, 25.0, 0, 0 },
    { -22.5, 0.0, 0, 0 },
    {  7.5, 65.0, 0, 0 },    // GEM east side, west sky, positive HA
    {  7.5, 45.0, 0, 0 },
    {  7.5, 25.0, 0, 0 }, 
    {  7.5, 0.0, 0, 0 },
    {  22.5, 65.0, 0, 0 },
    {  22.5, 45.0, 0, 0 },
    {  22.5, 25.0, 0, 0 },
    {  22.5, 0.0, 0, 0 }
};

// offsets from telescope, replace the closest point, add_point, sync_here
void add_point( struct POINTING *p ){
static uint8_t first, first_east, first_west;
int i;
float d[NUM_SYNC];
int k;
float mn;

  if( first == 0 ){
     for( i = 0; i < NUM_SYNC; ++i ){            // global offsets on first alignment point
        sync[i].ra_off = p->RAoffset;
        sync[i].dc_off = p->DECoffset; 
     }
     first = 1;
  }
  if( first_east == 0 && p->HA >= 0.0){
     for( i = NUM_SYNC/2; i < NUM_SYNC; ++i){
        sync[i].ra_off = p->RAoffset;
        sync[i].dc_off = p->DECoffset;       
     }
    first_east = 1;
  }
  if( first_west == 0 && p->HA < 0.0){
     for(i = 0; i < NUM_SYNC/2; ++i ){
        sync[i].ra_off = p->RAoffset;
        sync[i].dc_off = p->DECoffset;             
     }
     first_west = 1;
  }
 
  // distances
   for( i = 0; i < NUM_SYNC; ++i ){
      d[i] = distance( p->HA, p->DEC_, sync[i].ha, sync[i].dc );
   }

   // replace offsets closest point
   k = 0;   mn = d[0];
   for( i = 1; i < NUM_SYNC; ++i ){
      if( d[i] < mn ) k = i, mn = d[i];
   }
   sync[k].ra_off = p->RAoffset;
   sync[k].dc_off = p->DECoffset;

   print_all();    // !!! debug
}


void print_all(){
int i;

   for( i = 0; i < NUM_SYNC; ++i ){
      Serial.print( sync[i].ha );  Serial.write(' ');
      Serial.print( sync[i].dc );  Serial.write(' ');
      Serial.print( sync[i].ra_off );  Serial.write(' ');
      Serial.println( sync[i].dc_off );
   }
   Serial.println();
  
}


float distance( float h1, float d1, float h2, float d2 ){
float hd, dd;

    hd = h1 - h2;   dd = d1 - d2;
    hd = hd*hd + dd*dd;
    return sqrtf( hd );
}

// telescope + offsets has where we think we are, target2 w/ NO offsets has where we actually are, need stepper offset  
// calc all scopes, get diff for correct type scope * steps per degree. put in offsets  for *p, add point.
void add_sync( struct POINTING *p ){
float ra, dec;
float ra2, dec2;
long ra_off, dec_off;

  calc_ext_object( p );
  switch( mount_type ){
      case GEM: case FORK:  ra = p->HA;  dec = p->DEC_; ra2 = telescope.HA;  dec2 = telescope.DEC_;  break;   
      case ALTAZ:  ra = p->AZ;  dec = p->ALT;  ra2 = telescope.AZ;  dec2 = telescope.ALT; break;
      case ALTALT: ra = p->HAp;  dec = p->DECp; ra2 = telescope.HAp;  dec2 = telescope.DECp;  break;
  }

  ra_off = (ra2 - ra ) * (float)RA_STEPS_PER_DEGREE;  ra_off += telescope.RAoffset;
  dec_off = ( dec2 - dec ) * (float)DEC_STEPS_PER_DEGREE;  dec_off += telescope.DECoffset;
  p->RAoffset = ra_off;
  p->DECoffset = dec_off;
  add_point( p );
  
}

// get some offsets and write to the pointing structure
void apply_offsets( struct POINTING *p ){
int i;
float d[NUM_SYNC];
int k;
float mn;

   // find the closest point
   // distances
   for( i = 0; i < NUM_SYNC; ++i ){
      d[i] = distance( p->HA, p->DEC_, sync[i].ha, sync[i].dc );
   }

   // get offsets closest point
   k = 0;   mn = d[0];
   for( i = 1; i < NUM_SYNC; ++i ){
      if( d[i] < mn ) k = i, mn = d[i];
   }
   p->RAoffset = sync[k].ra_off;
   p->DECoffset = sync[k].dc_off;

}
