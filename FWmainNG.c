/* Flatworld II V1.0 main test program.  Doe not use GLUT/OpenGL for sim loop & visualization. 
*  Created 17 March 2009 by T. Caudell
*  Updated 20 Sept 2009, tpc
*  Updated 14 Nov 2009, tpc
*
*  Copyright University of New Mexico 2009
*/

#include <stdio.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "Distributions_Funcs.h"
#include "FlatworldIICore.h"

#define TRUE 1
#define FALSE 0
#define PI2 6.283185307179586
#define PI 3.141592653589793


/* Global pointer to current Flatworld */
WORLD_TYPE *Flatworld ;
int simtime = 0 ;
int nlifetimes = 0, maxnlifetimes = 20 ;
int runflag = 1 ;
float avelifetime = 0.0 ;

#include "Distributions_Funcs.c"
#include "FlatworldIICore.c"
#include "Controller.c"

AGENT_TYPE *current_agent ;

/* Main Loop ---------------------------------------------------------------------------------------*/
int main(int argc, char** argv)
{
	AGENT_TYPE *agent ;
	GEOMETRIC_SHAPE_TYPE *agentshape  ;
  ACOUSTIC_SHAPE_TYPE *sound ;
	int nsoundreceptors, nsoundbands ;
  int t ;
  float angle_locations0[31] = {-15.,-14.,-13.,-12.,-11.,-10.,-9.,-8.,-7.,-6.,-5.,-4.,-3.,-2.,-1.,0.,1.,2.,3.,4.,5.,6.,7.,8.,9.,10.,11.,12.,13.,14.,15} ;
  float directions0[31] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ;
  time_t now ;
  struct tm *date ;
  char timestamp[30] ;

  /* create and initialize the world */
  printf("main- making and loading world.\n") ;	
	Flatworld = make_world( 0, 1, 600, 100.0, -100.0, 100.0, -100.0, 0 ) ;
	read_object_spec_file( Flatworld, "WorldObjects.dat" ) ;	
	
  /* Creat and initialize the Agent */
  printf("main- making and atrributing agent.\n") ;
  agent = make_agent( 1, 0.0, 0.0, 0.0, 0.5, 1.0 ) ; 
	agentshape = read_geometric_shape_file( "geoshapeAgent.dat", 0 ) ;
  sound = read_acoustic_shape_file( "soundshapeAgent.dat" ) ;  
  nsoundreceptors = sound->nfrequencies ; 
  nsoundbands = sound->nbands ;
  add_physical_shape_to_agent( agent, agentshape ) ;
  add_sound_shape_to_agent( agent, sound ) ;
	add_visual_sensor_to_agent( agent, 31, 3, 0.0, angle_locations0, directions0 ) ;
  add_acoustic_sensor_to_agent( agent, nsoundreceptors, nsoundbands, 0.0, 90.0 ) ;
  add_acoustic_sensor_to_agent( agent, nsoundreceptors, nsoundbands, 0.0, -90.0 ) ;
  add_cargo_manifest_type_to_agent( agent, 0 ) ;
  add_soma_sensor_to_agent( agent, 1, 0.0, agentshape ) ;
  add_actuators_to_agent( agent ) ;
  set_max_translation_delta_agent( agent, 0.1 ) ;
  set_agent_head_angle( agent, 0.0 ) ;
  set_metabolic_burn_rate_agent(agent, 2.0e-4 ) ;
	set_movement_burn_rate_agent( agent, 5.0e-4 ) ;
  
	add_agent_to_world( Flatworld, agent ) ; 
  
  current_agent = agent ;

  /* Initialize the world and wall clock times. */
  now = time(NULL) ;
  date = localtime( &now ) ;
  strftime(timestamp, 30, "%y/%m/%d H: %H M: %M S: %S",date) ;
  printf("main- Start time: %s\n",timestamp) ;

  for( t=0 ; t<100000 ; t++ )
  {
    agents_controller( Flatworld ) ;
  }

  printf("main- terminating normally.\n") ;
}

