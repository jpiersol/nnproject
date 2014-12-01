/*
 *  Controller.c
 *  For the UNM Neural Networks class, this should be the only file you will need to modify.
 *  World and agent initialization code are located in the main().  An
 *  example of a non-neural controller is included here.
 *  Note that most all of the functions called here can be found in the 
 *  file FlatworldIICore.c
 *  
 *
 *  Created by Thomas Caudell on 9/15/09.
 *  Modified by Thomas Caudell on 9/30/2010
 *  Modified by Thomas Caudell on 9/13/2012
 *  Modified by Thomas Caudel on 9/10/14
 *  Copyright 2009 UNM. All rights reserved.
 *
 */
#include <algorithm>
#include <iterator>
#include <fstream>
#include <math.h>
// #include "engine.h"

using namespace std;

float eyeval[3] = {0,0,0};
float learn = 0.05;
float weight[4] = { distributions_uniform( -1,1),distributions_uniform( -1,1),distributions_uniform( -1,1),distributions_uniform( -1,1) };
float sumerror = 0;
int num_eats = 0;
ofstream* results = NULL;
ofstream* rms_error = NULL;

void agents_controller( WORLD_TYPE *w )
{ /* Adhoc function to test agents, to be replaced with NN controller. tpc */
	
  AGENT_TYPE *a ;
  int maxvisualreceptor = -1 ;
  int nacousticfrequencies ;
  float dfb , drl, dth, dh ;
  float headth ;
  float forwardspeed ;
  float maxvisualreceptordirection ;
  float bodyx, bodyy, bodyth ;
  float **eyevalues, **ear0values, **ear1values, **skinvalues ;
  float ear0mag=0.0, ear1mag=0.0 ;
  time_t now ;
  struct tm *date ;
  char timestamp[30] ;
  
    if (results == NULL) {
        results = new ofstream( "results.csv" );
        *results << "Lifetimes" << endl;
    }
    if (rms_error == NULL) {
        rms_error = new ofstream( "rms_error.csv" );
        *rms_error << "RMS Error" << endl;
    }

    /* Initialize */
    forwardspeed = 0.01 ;  
	a = w->agents[0] ; /* get agent pointer */
	
	/* test if agent is alive. if so, process sensors and actuators.  if not, report death and 
	   reset agent & world */
	if( a->instate->metabolic_charge>0.0 )
	{
		/* get current motor rates and body/head angles */
		read_actuators_agent( a, &dfb, &drl, &dth, &dh ) ;
		read_agent_body_position( a, &bodyx, &bodyy, &bodyth ) ;
		read_agent_head_angle( a, &headth ) ;
        
		/* read visual sensor to get R, G, B intensity values */ 
		read_visual_sensor( w, a) ;
		eyevalues = extract_visual_receptor_values_pointer( a, 0 ) ;
        
        //record the eyevalue
        float sumeyeval = eyevalues[15][0] + eyevalues[15][1] + eyevalues[15][2];
        if (sumeyeval == 0) sumeyeval = 0.000001;
        eyeval[0] = eyevalues[15][0] / sumeyeval;
        eyeval[1] = eyevalues[15][1] / sumeyeval;
        eyeval[2] = eyevalues[15][2] / sumeyeval;
				

        /* read somatic(touch) sensor for collision */  
		int collision_flag = read_soma_sensor(w, a) ; 	
        skinvalues = extract_soma_receptor_values_pointer( a ) ;
        int nsomareceptors = get_number_of_soma_receptors( a ) ;
        for( int k=0 ; k<nsomareceptors ; k++ )
        {
          if( (k==0 || k==1 || k==7 ) && skinvalues[k][0]>0.0 )
          {
            float y = weight[0] + weight[1] * eyeval[0] + weight[2] * eyeval[1] + weight[3] * eyeval[2];
//             if (y > 0) {
                float delta_energy = 10 * eat_colliding_object( w, a, k) ;
//              if (delta_energy != 0) {
//                  printf("Eyeval: %f, %f, %f\n", eyeval[0], eyeval[1], eyeval[2]);

                float error = delta_energy - y;
                weight[0] += learn*error;
                weight[1] += learn*error*eyeval[0];
                weight[2] += learn*error*eyeval[1];
                weight[3] += learn*error*eyeval[2];

                sumerror += pow(error, 2);
                *rms_error << sqrt(sumerror/(num_eats+1)) << endl;
                num_eats++;
                break;
//             }
//             }
          }
        }

//     
//     /* read hearing sensors and load spectra for each ear, and compute integrated sound magnitudes */
//     read_acoustic_sensor( w, a) ;
//     ear0values = extract_sound_receptor_values_pointer( a, 0 ) ;
//     ear1values = extract_sound_receptor_values_pointer( a, 1 ) ;
//     nacousticfrequencies = get_number_of_acoustic_receptors( a ) ;    
//     for( int i=0 ; i<nacousticfrequencies ; i++ )
//     {
//       ear0mag += ear0values[i][0] ;
//       ear1mag += ear1values[i][0] ;
//     }
//     //printf("simtime: %d ear0mag: %f ear1mag: %f\n",simtime,ear0mag,ear1mag) ;
//     
        
        

		
        /* find brights object in visual field */
        maxvisualreceptor = intensity_winner_takes_all( a ) ;
        if( maxvisualreceptor >= 0 ) 
            {
                /* use brightest visual receptor to determine how to turn body to center it in the field of view */
                maxvisualreceptordirection = visual_receptor_position( a->instate->eyes[0], maxvisualreceptor ) ;      
                /* rotate body to face brightes object */
                set_agent_body_angle( a, bodyth + maxvisualreceptordirection ) ;
        }
        else
        {
          printf("agents_controller-  No visible object, simtime: %d, changing direction.\n",simtime) ;
          read_agent_body_position( a, &bodyx, &bodyy, &bodyth ) ;
                set_agent_body_angle( a, bodyth + 45.0 ) ;
        }

        /* move the agents body */
        set_forward_speed_agent( a, forwardspeed ) ;
		move_body_agent( a ) ;
// 
//         printf("Charge: %f",a->instate->metabolic_charge);
		/* decrement metabolic charge by basil metabolism rate.  DO NOT REMOVE THIS CALL */
		basal_metabolism_agent( a ) ;
		simtime++ ;

	} /* end agent alive condition */
	else
	{
        float x0, y0, h0 ;
    
    /* Example of agent is dead condition */
// 		printf("agent_controller- Agent has died, eating %d objects. simtime: %d\n",a->instate->itemp[0], simtime ) ;
// 		now = time(NULL) ;
// 		date = localtime( &now ) ;
// 		strftime(timestamp, 30, "%y/%m/%d H: %H M: %M S: %S",date) ;
// 		printf("Death time: %s\n",timestamp) ;
		
		/* Example as to how to restore the world and agent after it dies. */
		restore_objects_to_world( Flatworld ) ;  /* restore all of the objects back into the world */
		reset_agent_charge( a ) ;               /* recharge the agent's battery to full */
		a->instate->itemp[0] = 0 ;              /* zero the number of object's eaten accumulator */
		x0 = distributions_uniform( Flatworld->xmin, Flatworld->xmax ) ; /* pick random starting position and heading */
		y0 = distributions_uniform( Flatworld->ymin, Flatworld->ymax ) ;
		h0 = distributions_uniform( -179.0, 179.0) ;
// 		printf("\nagent_controller- new coordinates after restoration:  x: %f y: %f h: %f\n",x0,y0,h0) ;
		set_agent_body_position( a, x0, y0, h0 ) ;    /* set new position and heading of agent */
    
		/* Accumulate lifetime statistices */
		avelifetime += (float)simtime ;
		*results << simtime;
        printf("Weights: %f %f %f %f\n", weight[0], weight[1], weight[2], weight[3]);
        
		simtime = 0 ;
        nlifetimes++ ;
        printf("Life: %i\n", nlifetimes);
		if( nlifetimes >= maxnlifetimes )
		{
			avelifetime /= (float)maxnlifetimes ;
			printf("\nAverage lifetime: %f\n",avelifetime) ;
            
            results->close();
            rms_error->close();
                        
            ofstream wout( "weights.csv" );
            wout << "Weights\n";
            copy( weight, weight + 4, ostream_iterator<float>( wout, "\n" ) );
            wout.close();
            
			exit(0) ;
		}
		
		
		
	} /* end agent dead condition */
	
  
}
