/*
   Distibutions_Funcs.c - A collection of random number generators
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

void distributions_rantest()
{
	/* Call this function first to test if RAND_MAX is set
	   correctly for this machine.
	*/
	
	int i ;
	float x,sum,sum2,fmin,fmax ;

	sum = sum2 = 0.0 ;
	fmin = 1.0e35 ;
	fmax = -1.0e35 ;

	printf("distributions_rantest- Calculating...\n") ;
	for( i=0 ; i<10000 ; i++ ) {
		x= distributions_uniform( -1.0, 1.0 ) ;
		if( x>fmax ) {
			fmax = x ;
		}
		if( x<fmin ) {
			fmin = x ;
		}
		sum += x ;
		sum2 += ( x * x ) ;
	}
	sum /= 10000.0 ;
	sum2 = sqrt( sum2/10000.0 - sum*sum ) ;
	printf("distributions_rantest- n: 10000 max: %f min: %f ave: %f sigma: %f\n",fmax,fmin,sum,sum2) ;
	if( fmax<0.99 || fmax>1.0 || fmin<-1.0 || fmin>-0.99 ) {
		printf("                     - ERROR in random number generator, check RAND_MAX.\n") ;
		printf("distributions_rantest- Good bye!\n") ;
		exit(0) ;
	}
	printf("distributions_rantest- RAND_MAX ok!\n") ;
}

float distributions_uniform( float a, float b)
{
/*  This routine returns a 1D random float precision number in
    the range from a to b. tpc
*/
	return(a+(b-a)*((float)rand()/(float)RAND_MAX)) ;

	/* return(a+(b-a)*(1.0+(float)rand()/(float)RAND_MAX)*0.5) ; for MAC */
}

void distributions_uniform_nD( int ndim, float *a , float *b, float *x )
{
	/* This routine returns a uniform random distribution within a hyperbox
	   defined by the lower(min) vertix a and the upper(max) vertix b in ndim
	   dimensions.  Returns the random point in the vector x .
	*/
	int i ;
	
	for( i=0 ; i<ndim ; i++ ) {
		x[i] = distributions_uniform(a[i],b[i]) ;
	}
}

int distributions_spirals( int nspirals, float gain, float rmax, float *x )
{
/* This function produces a ndim=2 random vector and classifies it as part of
   a spiral region defined by the area between spirals defined by r = gain * theta.
   There are nspiral regions, the rate of growth determined by gain. Rmax is
   the largest radius of any spiral. x[0],x[1] contain the actual random number
   and the function returns an integer [0,nspiral-1] as the class. tpc
*/
	int i, k, kmax ;
	float rx=1.0e35, rik, deltar, dr, thetax, thetak, dtheta0 ;
	
	/* set up initial values */
	dtheta0 = PI2 / (float)nspirals ;
	kmax = 1+(int)(rmax/(gain*PI2)) ;
	deltar = dtheta0 * gain ;
	
	/* Calculate x such that it falls within the rmax circle and x[0]!=0 */
	x[0] = 1.0 ;
	while(x[0]!=0.0 && rx>rmax) {
		x[0] = distributions_uniform(-rmax,rmax) ;
		x[1] = distributions_uniform(-rmax,rmax) ;
		rx = sqrt( x[0]*x[0] + x[1]*x[1] ) ;
	}
	thetax = (float)atan2((float)x[1],(float)x[0]) ;
	thetax += PI ; 

	/* Start search of spiral arm that contains the point */
	for( k=0 ; k<=kmax ; k++ ) {
		thetak = k * PI2 ;
		for( i=0 ; i<nspirals ; i++ ) {
			rik = gain * ( thetax + thetak - dtheta0 * (float)i ) ;
			dr = rik - rx ;
			if( dr>=0.0 && dr<deltar)
				goto XX ;
		}
	}
	printf("distributions_spirals- ERROR, arm not found.\n") ;
	exit(0) ;
XX:	; 
	return( i ) ;
}

float distributions_normal(float mean, float sigma )
{
	/* This function uses Box-Muller method to return a normally distributed
       random number.
	*/
	float u1, u2, x,y ;
	
	u1 = distributions_uniform(0.0, 1.0) ;
	u2 = distributions_uniform(0.0, 1.0) ;
	
	x = (float)(cos(PI2*(float)u1)*sqrt(-2.0*log((float)u2))) ;
	y = sigma * x + mean ;
	while( fabs(y-mean) > 20.0*sigma ) { /* fixes the possibility that +/- INF happens */
		u1 = distributions_uniform(0.0, 1.0) ;
		u2 = distributions_uniform(0.0, 1.0) ;
		x = (float)(cos(PI2*(float)u1)*sqrt(-2.0*log((float)u2))) ;
		y = sigma * x + mean ;
	}

	return( y ) ;
}

void distributions_normal_nD( int ndim, float *mean , float *sigma, float *x )
{
	/* This routine returns a noraml random distribution 
	   defined by the mean vector and the sigma vector in ndim
	   dimensions.  Returns the random point in the vector x.
	   Assumes diagonal variance matrix (ie axis aligned elipses)
	*/
	int i ;
	
	for( i=0 ; i<ndim ; i++ ) {
		x[i] = distributions_normal(mean[i],sigma[i]) ;
	}
}

float Normal_nD( int ndim, float *mean , float *sigma, float *x )
{
	/* Assumes all off-diagonal terms of covariance matrix are zero */
	int i ;
	float p,srpi2 ;

	srpi2 = sqrt(PI2) ;
	p = 1.0 ;
	for( i=0 ; i<ndim ; i++ ) {
		p *= ( exp( -0.5*(x[i]-mean[i])*(x[i]-mean[i])/(sigma[i]*sigma[i]) ) / (srpi2*sigma[i]) ) ;
	}
	return( p ) ;
}

float distributions_exponential(float mean)
{
	/* Exponential distribution: f(x) = lamda * exp( -lamda*x )
	   where 1/lamda = mean 
	*/
	float u, x ;
	
	u = distributions_uniform(0.0, 1.0) ;
	
	x = -(float)( log(1.0-u) * mean ) ;

	return( x ) ;
}

float distributions_poisson( float theta )
{
	/* Poisson distribution: f(x) = exp(-theta) * theta^x / x!
	   where theta is the mean and variance.
	*/
	float x, m, u ;
	
	x = 1 ;
	m = (float)exp((float)theta) ;

	while( m>=1.0 ) {
		u = distributions_uniform(0.0, 1.0) ;
		x = x + 1.0 ;
		m = m * u ;
	}
	return(x) ;
}

struct stats_packet *make_stats_packet(char *name ) 
{
	struct stats_packet *s ;

	s = (struct stats_packet *)malloc( sizeof(struct stats_packet) ) ;
	if( s==NULL ) {
		printf("make_stats_packet- ERROR mallocing s.\n") ;
		exit(0) ;
	}
	s->name = (char *)malloc(30*sizeof(char)) ;
	if( s->name==NULL ) {
		printf("make_stats_packet- ERROR mallocing s->name.\n") ;
		exit(0) ;
	}	
	strcpy(s->name,name) ;
	s->N = 0 ;
	s->xmax = -1.0e35 ;
	s->xmin = 1.0e35 ;
	s->xave = 0.0 ;
	s->xsig = 0.0 ;
	s->sumx = 0.0 ;
	s->sumxx = 0.0 ;

	return( s ) ;
}

void init_stats_packet ( struct stats_packet *s ) 
{
	s->N = 0 ;
	s->xmax = -1.0e35 ;
	s->xmin = 1.0e35 ;
	s->xave = 0.0 ;
	s->xsig = 0.0 ;
	s->sumx = 0.0 ;
	s->sumxx = 0.0 ;
}

void accum_stats_in_packet( float x, struct stats_packet *s) 
{
	s->N = s->N + 1 ;
	s->sumx += x ;
	s->sumxx += (x*x) ;
	if( x>s->xmax )
		s->xmax = x ;
	if( x<=s->xmin )
		s->xmin = x ;
}

void compute_stats_from_packet( struct stats_packet *s ) 
{
	if( s->N==0 ) {
		printf("compute_stats_from_packet- ERROR, attempt to compute stats with no data, packet name: %s\n",s->name) ;
		return ;
	}
	s->xave = s->sumx / (float)(s->N) ;
	s->xsig = (float)sqrt( (s->sumxx/(float)s->N) - (s->xave*s->xave) ) ;
}

void print_stat_packet(  struct stats_packet *s ) 
{
	if( s->N==0 ) {
		printf("Variable: %20s | N: %5d | No data...\n",s->name,s->N) ;
		return ;
	}
	printf("Variable: %20s N: %5d av: %10.5f sg: %10.5f mx: %10.5f mn: %10.5f sx: %10.5f \n",
		s->name,s->N,s->xave,s->xsig,s->xmax,s->xmin,s->sumx) ; fflush(stdout) ;
}

void fprint_stat_packet( FILE *fp, struct stats_packet *s ) 
{
/* Note: requires a \n call afer all writes for a line in main program */
	if( s->N==0 ) {
		printf("Variable: %20s | N: %5d | No data...\n",s->name,s->N) ;
		return ;
	}
	fprintf(fp,"Variable: %20s N: %5d av: %10.5f sg: %10.5f mx: %10.5f mn: %10.5f sx: %10.5f ",
		s->name,s->N,s->xave,s->xsig,s->xmax,s->xmin,s->sumx) ;
}


/*
Test Programs for the Spiral Distributions

-------------------------------------------------------------------
The following program caluclates a set of spiraal arm points with a 
specific class 

#include <stdio.h>
#include <math.h>
#include "Macintosh HD:Tom:Development:Common:Common.h"
#include "Macintosh HD:Tom:Development:Common:Distributions_Funcs.h"

main()
{
	FILE *fpin, *fpout, *fopen() ;	
	int class,i ;
	float x[2] ;

	fpout = fopen("spirals3.out","w") ;
	for( i=0 ; i<8000 ; i++ ) {
		if( i%800==0 ) printf(".\n") ;
		class =distributions_spirals( 3, 1, 20.0, x ) ;
		if( class==1 )
			fprintf(fpout,"%f, %f\n",x[0],x[1]);
	}
	fclose(fpout) ;
}
#include "Macintosh HD:Tom:Development:Common:Distributions_Funcs.c"
-------------------------------------------------------------------
The following progam computes a table of spial boundaries for pubs.

#include <stdio.h>
#include <math.h>
#include "Macintosh HD:Tom:Development:Common:Common.h"

main()
{
	FILE *fpin, *fpout, *fopen() ;	
	float gain ;
	int i, t, tmax, nspirals ;
	float x,y,z,r,rmax,theta, dtheta0 ;
	
	tmax = 600 ;
	gain = 1.0 ;
	nspirals = 3 ;
	rmax = 20.0 ;
	dtheta0 = PI2 / (float)nspirals ;
	
	printf("main- Starting calculation.\n");
	fpout = fopen("spirals_boundaries","w") ;
	for( t=0 ; t<tmax ; t++ ) {
		theta = t * PI2 / 100.0 ;
		for( i=0 ; i<nspirals ; i++ ) {
			r = gain * ( theta - dtheta0 * (float)i ) ;
			if(r>0.0 && r<=rmax) {
				x = r * cos(theta) ;
				y = r * sin(theta) ;
				z = 0.0 ;
				fprintf(fpout,"%f, %f, %f, ",x,y,z ) ;
			} else {
				x = 0.0 ;
				y = 0.0;
				z = 0.0;
				fprintf(fpout,"%f, %f, %f, ",x,y,z ) ;
			}
		}
		fprintf(fpout,"\n") ;
	}
	fclose(fpout) ;
	printf("main- Done.\n");
}

-------------------------------------------------------------------
The following progam computes a table of test values drawn from a 
distribution.

#include <stdio.h>
#include <math.h>
#include "Macintosh HD:Tom:Development:Common:Common.h"
#include "Macintosh HD:Tom:Development:Common:Distributions_Funcs.h"

main()
{
	FILE *fpin, *fpout, *fopen() ;	
	int class,i ;
	float x ;

	fpout = fopen("poisson.out","w") ;
	for( i=0 ; i<800 ; i++ ) {
		if( i%80==0 ) printf("*\n") ;
		x = distributions_poisson( 10.0 ) ;
		fprintf(fpout,"%f\n",x);
	}
	fclose(fpout) ;
}
#include "Macintosh HD:Tom:Development:Common:Distributions_Funcs.c"
*/

