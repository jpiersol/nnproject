/* Distibutions_Funcs.h - A collection definitions for random number generators
   for various probability distributions .

Copyright (C) 2003, University of New Mexico.

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.
             
    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

*/

/* 
 * Maintenance history
 *    
 * Created: 3 Nov 1997  Thomas Caudell
 */

struct stats_packet {
	char *name ;	/* Charater array with name */
	int N ;		/* The number of samples accumulated so far */
	float xmax ;	/* Current Max value */
	float xmin ;	/* Current Min value */
	float xave ;	/* Computed ave */
	float xsig ; 	/* Computed standard deviation */
	float sumx ;	/* Accumulated sum of x so far */
	float sumxx ;	/* Accumulcated sum of x*x so far */
} ;
   
void distributions_rantest() ;
float distributions_uniform( float a, float b ) ;
void distributions_uniform_nD( int ndim, float *a , float *b, float *x ) ;
int distributions_spirals( int nspirals, float gain, float rmax, float *x ) ;
float distributions_normal(float mean, float sigma ) ;
void distributions_normal_nD( int ndim, float *mean , float *sigma, float *x ) ;   
float Normal_nD( int ndim, float *mean , float *sigma, float *x ) ;
float distributions_exponential(float mean) ;
float distributions_poisson( float theta ) ;

struct stats_packet *make_stats_packet(char *name ) ;
void init_stats_packet ( struct stats_packet *s ) ;
void accum_stats_in_packet( float x, struct stats_packet *s) ;
void compute_stats_from_packet( struct stats_packet *s ) ;
void print_stat_packet(  struct stats_packet *s ) ;
void fprint_stat_packet( FILE *fp, struct stats_packet *s ) ;

