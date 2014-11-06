/*
 *  FlatworldIICore.c
 *  
 *
 *  Created by Thomas Caudell on 3/17/09.
 *  Copyright University of New Mexico 2009. All rights reserved.
 *
 */
  
/*------------------------ General functions -------------------------------------------------------------------*/
float min( float x, float y) 
{
  if( x<y ) 
    return(x);
  return(y) ;
}

float L2measure( float x0, float y0, float x1, float y1 ) 
{
  float dx, dy ;
  dx = x1 - x0 ;
  dy = y1 - y0 ;
  return((float)sqrt( dx*dx + dy*dy)) ;
}

void rotate2D( GEOMETRIC_SHAPE_TYPE *spout, GEOMETRIC_SHAPE_TYPE *spin, float xc, float yc, float angle )
{

  int i ;
  float sinangle, cosangle ;
  float x,y ;
  float **vin, **vout ;
  float deg2rad = 1.7453292e-02 ;

  angle *= deg2rad ;
  
  sinangle = (float)sin(-angle) ;
  cosangle = (float)cos(-angle) ;
  vin = spin->vertices ;
  vout = spout->vertices ; 
  for( i=0 ; i<spin->nvertices ; i++ )
  {
    x = vin[i][0] ;
    y = vin[i][1] ;
    vout[i][0] = x * cosangle + y * sinangle + xc ;
    vout[i][1] = y * cosangle - x * sinangle + yc ;
  }
}

float min_geometric_shape_distance( GEOMETRIC_SHAPE_TYPE *s1, GEOMETRIC_SHAPE_TYPE *s2 )
{  /* computes the min distance between two sets of 2D vertices in the tow shape structures */

  int i,j ;
  int n1,n2 ;
  float radius, minradius = 1.0e6 ;
  float **v1, **v2  ;
  
  v1 = s1->vertices ;
  v2 = s2->vertices ;
  n1 = s1->nvertices ;
  n2 = s2->nvertices ;
  
  for( i=0 ; i<n1 ; i++ ) 
  {
    for( j=0 ; j<n2 ; j++ ) 
    {
      radius = L2measure( v1[i][0], v1[i][1], v2[j][0], v2[j][1] ) ;
      if( radius < minradius )
        minradius = radius ;
    }
  }
  return( minradius) ;
}
    
void testm( void *e, char *message, char *variable ) 
{
 if( e==NULL ) {
    fprintf(stdout,"%s- ERROR mallocing %s\n",message,variable) ;
	exit(0) ;
  }
}

void testf( void *e, char *message, char *variable ) 
{
 if( e==NULL ) {
    fprintf(stdout,"%s- ERROR opening file: %s\n",message,variable) ;
	exit(0) ;
  }
}

GEOMETRIC_SHAPE_TYPE *make_geometric_shape_type( int nvertices, int nbands, float height, float scale )
{ /* allocate vertex list and initialize */

  GEOMETRIC_SHAPE_TYPE *r ;
  int i,j ;
  int ncomponents ;              
	
  ncomponents = 2 + nbands ;  /* assumes two positions and nbands colors */
  r = (GEOMETRIC_SHAPE_TYPE *)malloc(sizeof(GEOMETRIC_SHAPE_TYPE)) ;
  testm(r,"make_geometric_shape_type","r") ;
	
  r->nvertices = nvertices ;
  r->nbands = nbands ;
  r->height = height ;
  r->scale = scale ;
  r->vertices = (float **)malloc(nvertices*sizeof(float *)) ;
  testm(r->vertices,"make_geometric_shape_type","r->vertices") ;
	
  for( i=0 ; i<nvertices ; i++ )
  {
    r->vertices[i] = (float *)malloc( ncomponents*sizeof(float)) ;
	  testm(r->vertices[i],"make_geometric_shape_type","r->vertices[i]") ;
	  for( j=0 ; j<ncomponents ; j++ )
	    r->vertices[i][j] = 0.0 ;
  }
	
  return( r ) ;
}
	  
void free_geometric_shape_type( GEOMETRIC_SHAPE_TYPE *r )
{
  int i ;
  
  if( r==NULL )
    return ;
  for( i=0 ; i<r->nvertices ; i++ )
    free( r->vertices[i] ) ;
  free(r->vertices) ;
  free( r ) ;
}

ACOUSTIC_SHAPE_TYPE *make_acoustic_shape_type( int nfrequencies, int nbands )
{ /* allocate sound list and initialize */

  ACOUSTIC_SHAPE_TYPE *r ;
  int i,j ;
	
  r = (ACOUSTIC_SHAPE_TYPE *)malloc(sizeof(ACOUSTIC_SHAPE_TYPE)) ;
  testm(r,"make_acoustic_shape_type","r") ;
	
  r->nfrequencies = nfrequencies ;
  r->nbands = nbands ;
  r->freq = (float *)malloc(nfrequencies*sizeof(float)) ;
  testm(r->freq,"make_acoustic_shape_type","r->freq") ;  
  r->spectrum = (float **)malloc(nfrequencies*sizeof(float*)) ;
  testm(r->spectrum,"make_acoustic_shape_type","r->spectrum") ;
	
  for( i=0 ; i<nfrequencies ; i++ )
  {
    r->spectrum[i] = (float*)malloc(nbands*sizeof(float)) ;
    testm(r->spectrum[i], "make_acoustic_shape_type", "r->spectrum[i]") ;
    for( j=0 ; j<nbands ; j++ )
      r->spectrum[i][j] = 0.0 ;
    r->freq[i] = 0.0 ;
  }
  return( r ) ;
}

void free_acoustic_shape_type( ACOUSTIC_SHAPE_TYPE *r )
{
  int i ;

  if( r==NULL )
    return ;  
  for( i=0 ; i<r->nfrequencies ; i++ )
    free( r->spectrum[i] ) ;
  free(r->spectrum) ;
  free( r ) ;
}

GEOMETRIC_SHAPE_TYPE *read_geometric_shape_file( char *filename, int rscaleflag ) 
{ /* reads a geometric shape file, creates a geometric_shape_type and fills it. */

  GEOMETRIC_SHAPE_TYPE *s ;
  FILE *fpin ;
  int nvertices, nbands, i, k ;
  float **v, height, scale, rscale ;
  
  fpin = fopen(filename, "r" ) ;
  testf(fpin,"read_geometric_shape_file",filename) ;
  fscanf(fpin,"%d %d %f %f", &nvertices, &nbands, &height, &scale ) ;
  
  s = make_geometric_shape_type( nvertices, nbands, height, scale ) ;
  v = s->vertices ;
	rscale = distributions_uniform( 0.5, 1.5 ) ;
  for( i=0 ; i<nvertices ; i++ ) 
  {
    fscanf(fpin, "%f %f",  &(v[i][0]), &(v[i][1]) ) ;
    if( rscaleflag==0 )
		{
			v[i][0] *= scale ;
      v[i][1] *= scale ;
		}
		else if( rscaleflag==1 )
		{
			v[i][0] *= scale * rscale ;
      v[i][1] *= scale * rscale ;			
		}

	  for( k=0 ; k<nbands ; k++ )
	    fscanf(fpin, "%f",  &(v[i][k+2]) ) ;
  }
  fclose(fpin) ;
  
  return( s ) ;
}

ACOUSTIC_SHAPE_TYPE *read_acoustic_shape_file( char *filename ) 
{ /* reads a sound shape file, creates a acoustic_shape_type and fills it.  */

  ACOUSTIC_SHAPE_TYPE *s ;
  FILE *fpin ;
  int nfrequencies, nbands ;
  int i, j ;
  float **v,*f ;
  
  fpin = fopen(filename, "r" ) ;
  testf(fpin,"read_acoustic_shape_file",filename) ;
  fscanf(fpin,"%d %d", &nfrequencies, &nbands ) ;
  
  s = make_acoustic_shape_type( nfrequencies, nbands ) ;
  v = s->spectrum ;
  f = s->freq ;
  for( i=0 ; i<nfrequencies ; i++ )
  { 
    fscanf(fpin,"%f",&(f[i]));
    for( j=0 ; j<nbands; j++ )
    {
      fscanf(fpin, "%f", &(v[i][j])) ;
    }
  }  
  fclose(fpin) ;
  return( s ) ;
}

int intersect_beam_with_object(OBJECT_TYPE *obj, float rx, float ry, float sx, float sy, float *ximin, float *yimin, float *d, float *beta )
/* Loops over line segments in an object and tests if ray from sensor intersects any of them.  If so, then
the nearest intersetion is reported, and routine returns the lower vertex of hte colliding segment, not intersections 
return -1. Beam is defined as starting at (rx,ry) with another point on the beam being (sx,sy) defining the direction.
*/
{
  int i ;
  int nv ;
  int flag = -1 ;  // flag to indicate that at least one line segment of the object was intersected with
                  // FALSE = -1
  float dmin = 1.0e6 ;
  float dd,bb ;
  float x2,y2,x3,y3 ;
  float xi, yi ;
  float **v ;
  float ox, oy ;
  float dummyT, dummyL ;
  
  nv = obj->physical_shape->nvertices ;
  v = obj->physical_shape->vertices ;
  ox = obj->x ;
  oy = obj->y ;
  
  //printf("intersect_beam_with_object- Entry. obj: %d ox: %f oy: %f nv: %d\n",obj->index,ox,oy,nv) ;
  for( i=0 ; i<nv-1 ; i++ ) 
  {
    x2 = v[i][0]+ox ;
    y2 = v[i][1]+oy ;
    x3 = v[i+1][0]+ox ;
    y3 = v[i+1][1]+oy ;		
    if( 1 == line_to_line_seg_intersect( rx, ry, sx, sy, x2, y2, x3, y3, &xi, &yi, &dd, &bb, &dummyT, &dummyL ))   //TRUE = 1
	  {
      //printf("  seg: %d x2: %f y2: %f x3: %f y3: %f dd: %f\n",i,x2,y2,x3,y3,dd) ;
      if( dd<dmin )   // save the closest intersection point data
      {
        *d = dd ;
        flag = i ; //(TRUE)
        *beta = bb ;
        dmin = dd ;
        *ximin = xi ;
        *yimin = yi ;
      }
    }
    //    printf("  seg: %d x2: %f y2: %f x3: %f y3: %f *xi: %f *yi: %f dd: %f\n",x2,y2,x3,y3,*xi,*yi,dd) ;
  }
  // deal with vertex list wrap-around 
  x2 = v[nv-1][0]+ox ;
  y2 = v[nv-1][1]+oy ;
  x3 = v[0][0]+ox ;
  y3 = v[0][1]+oy ;		
  if( 1 == line_to_line_seg_intersect( rx, ry, sx, sy, x2, y2, x3, y3, &xi, &yi, &dd, &bb,&dummyT, &dummyL ))   // TRUE = 1
  {
    //printf("  seg: %d x2: %f y2: %f x3: %f y3: %f dd: %f\n",i,x2,y2,x3,y3,dd) ;
    if( dd<dmin ) 
    {  // save the closest intersection point data
      *d = dd ;
      flag = nv-1 ;
      *beta = bb ;
      dmin = dd ;
      *ximin = xi ;
      *yimin = yi ;
    }
  }
  return( flag ) ;
}

int intersect_beam_with_agent(AGENT_TYPE *a, float rx, float ry, float sx, float sy, float *ximin, float *yimin, float *d, float *beta )
/* Loops over line segments in an agent and tests if ray from sensor intersects any of them.  If so, then
the nearest intersetion is reported, and routine returns the lower vertex of hte colliding segment, not intersections 
return -1. Beam is defined as starting at (rx,ry) with another point on the beam being (sx,sy) defining the direction.
*/
{
  int i ;
  int nv ;
  int flag = -1 ;  // flag to indicate that at least one line segment of the object was intersected with
                  // FALSE = -1
  float dmin = 1.0e6 ;
  float dd,bb ;
  float x2,y2,x3,y3 ;
  float xi, yi ;
  float **v ;
  float ox, oy ;
  float dummyT, dummyL ;
  
  nv = a->outstate->physical_shape->nvertices ;
  v = a->outstate->physical_shape->vertices ;
  ox = a->outstate->x ;
  oy = a->outstate->y ;
  
  //printf("intersect_beam_with_object- Entry. obj: %d ox: %f oy: %f nv: %d\n",obj->index,ox,oy,nv) ;
  for( i=0 ; i<nv-1 ; i++ ) 
  {
    x2 = v[i][0]+ox ;
    y2 = v[i][1]+oy ;
    x3 = v[i+1][0]+ox ;
    y3 = v[i+1][1]+oy ;		
    if( 1 == line_to_line_seg_intersect( rx, ry, sx, sy, x2, y2, x3, y3, &xi, &yi, &dd, &bb, &dummyT, &dummyL ))   //TRUE = 1
	{
	  //printf("  seg: %d x2: %f y2: %f x3: %f y3: %f dd: %f\n",i,x2,y2,x3,y3,dd) ;
	  if( dd<dmin )   // save the closest intersection point data
	  {
	    *d = dd ;
	    flag = i ; //(TRUE)
	    *beta = bb ;
	    dmin = dd ;
		*ximin = xi ;
		*yimin = yi ;
	  }
	}
//    printf("  seg: %d x2: %f y2: %f x3: %f y3: %f *xi: %f *yi: %f dd: %f\n",x2,y2,x3,y3,*xi,*yi,dd) ;
  }
  // deal with vertex list wrap-around 
  x2 = v[nv-1][0]+ox ;
  y2 = v[nv-1][1]+oy ;
  x3 = v[0][0]+ox ;
  y3 = v[0][1]+oy ;		
  if( 1 == line_to_line_seg_intersect( rx, ry, sx, sy, x2, y2, x3, y3, &xi, &yi, &dd, &bb,&dummyT, &dummyL ))   // TRUE = 1
  {
    //printf("  seg: %d x2: %f y2: %f x3: %f y3: %f dd: %f\n",i,x2,y2,x3,y3,dd) ;
    if( dd<dmin ) 
    {  // save the closest intersection point data
      *d = dd ;
      flag = nv-1 ;
      *beta = bb ;
      dmin = dd ;
      *ximin = xi ;
      *yimin = yi ;
    }
  }
  return( flag ) ;
}

int line_to_line_seg_intersect( float x0, float y0, float x1, float y1, float x2, float y2, float x3, float y3,
                                float *xi, float *yi, float *d, float *beta, float *T, float *L)
/* x0,y0 is start of directed line, x1,y1 is another point on line,
   x2,y2 and x3,y3 are the end points of the target line segment, 
   xi,yi is the intersection point if it exists, and beta is the normal 
   of incidence. The method here uses parametric forms of lines:  x = (1-L)*x0 + L*x1
   A point on the line between the two end points has 0 < L < 1, while a point out beyond x1 has an L >1
   The ray from the sensor is represented as one directed line, and the line segment of 
   the object another.  Set equal, two equations in two unknowns L and T say, solve for one and test 
   intersection.  Finally compute intersection point, distance from sensor, and normal angle.
   The routine returns 1 (TRUE) if any segment intersected, and 0 (FALSE) otherwise.
*/
{
//  float L, T ; // line parameters for line segment and sensor ray respectively
  float x3x2x1x0, y3y2y1y0, y2y0y1y0, x2x0x1x0 ;
  float denom, ip, th ;
  float d32;
  float pi = M_PI;
 // float a, b ;
  
 // a = (float)fabs( x1 - x0 ) ;
 // b = (float)fabs( y1 - y0 ) ;
  
  //printf("line_to_line_seg_intersect- Entry.\n") ;
 // printf("   x0: %f x1: %f y0: %f y1: %f\n"
 //if( a==0.0 || b==0.0 )
   //printf("line_to_line_seg_intersect- fabs(x1-x0): %f fabs(y1-y0): %f\n",fabs(x1-x0),fabs(y1-y0)) ;
 
  if( (float)fabs( x1 - x0 ) != 0.0 && (float)fabs ( y1 - y0 ) != 0.0 ) 
  {
    //printf("line_to_line_seg_intersect- part 1.\n") ;
    x3x2x1x0 = (x3-x2) / (x1-x0) ;
	  y3y2y1y0 = (y3-y2) / (y1-y0) ;
	  y2y0y1y0 = (y2-y0) / (y1-y0) ;
	  x2x0x1x0 = (x2-x0) / (x1-x0) ;
	  denom = ( x3x2x1x0 - y3y2y1y0 );
	  if( (float)fabs( denom ) != 0.0 )
	  {
      *L = ( y2y0y1y0 - x2x0x1x0 ) / denom ;
	    *T = x2x0x1x0 + *L * x3x2x1x0 ;
	    //printf("line_to_line_seg_intersect- T: %f L: %f\n",T,L) ;
	    if( *L>=0.0 && *L<=1.0 && *T >= 0.0 ) // tests if intersection point is actually in line segment and is in FRONT of the sensor
	    {
	      *xi = ( 1.0 - *L ) * x2 + *L * x3 ;
		    *yi = ( 1.0 - *L ) * y2 + *L * y3 ;
		    //printf("line_to_line_seg_intersect- xi: %f yi: %f\n",*xi,*yi);
		    *d = (float)sqrt( ( *xi - x1 ) * ( *xi - x1 ) + ( *yi - y1 ) * ( *yi - y1 ) ) ;  // distance from sensor to intersection point

		    if( *d <= 0.0 )
		    {
		      return ( 0 ) ;  //FALSE
		    }
		    d32 = (float)sqrt( ( x3 - x2 ) * ( x3 - x2 ) + ( y3 - y2 ) * ( y3 - y2 ) ) ;  // length of line seg
		    ip = ( *xi - x1 ) * ( x3 - x2 ) + ( *yi - y1 ) * ( y3 - y2 ) ; // inner product between two vectors
        th = (float)acos( ip / ( *d * d32 ) ) ; // equation cos(th12) = ip12 / ( d1 * d2 )
		    th = th * 180.0 / pi ;
		    *beta = 90.0 - th ;  // normal angle of incidence
	      return( 1 ) ;  // TRUE
	    }
	    else
	    {
	    return(0) ;
	    }
	  }
	  else
	  {
	    //printf("line_to_line_seg_intersect- part 2.\n") ;
	    return( 0 ) ;  // FALSE
	  }
  } 
  else 
  {  // axis aligned cases 
    if( (float)fabs ( y1 - y0 ) == 0.0 && (float)fabs( x1 - x0 ) != 0.0 && (float)fabs( y3 - y2 ) != 0.0 ) // ray parallel to x-axis and line seg not parallel to x axis
	  {
      //printf("line_to_line_seg_intersect- part 3.\n") ;
      x3x2x1x0 = (x3-x2) / (x1-x0) ;
      x2x0x1x0 = (x2-x0) / (x1-x0) ;
      *L = ( y1 - y2 ) / ( y3 - y2 ) ;
      *T = x2x0x1x0 + *L * x3x2x1x0 ;
      //printf("line_to_line_seg_intersect- L: %f T: %f\n",L,T) ;
      if( *L>=0.0 && *L<=1.0 && *T >= 0.0 ) // tests if intersection point is actually in line segment and in FRONT of sensor
      {
        //printf("line_to_line_seg_intersect- part 3.1\n") ;
        *xi = ( 1.0 - *L ) * x2 + *L * x3 ;
        *yi = y1 ;
        //printf("line_to_line_seg_intersect- xi: %f yi: %f\n",*xi,*yi);
        *d = (float)sqrt( ( *xi - x1 ) * ( *xi - x1 ) + ( *yi - y1 ) * ( *yi - y1 ) ) ;  // distance fro sensor to intersection point
        if( *d <= 0.0 )
        {
          return ( 0 ) ;  //FALSE
        }
        d32 = (float)sqrt( ( x3 - x2 ) * ( x3 - x2 ) + ( y3 - y2 ) * ( y3 - y2 ) ) ;  // length of line seg
        ip = ( *xi - x1 ) * ( x3 - x2 ) + ( *yi - y1 ) * ( y3 - y2 ) ; // inner product between two vectors
        th = (float)acos( ip / ( *d * d32 ) ) ; // equation cos(th12) = ip12 / ( d1 * d2 )
        th = th * 180.0 / pi ;
        *beta = 90.0 - th ;  // normal angle of incidence
        return( 1 ) ;  // TRUE		
      }
      else
      {
        return(0) ;
      }
	  }
    else if( fabs ( x1 - x0 ) == 0.0 && fabs( y1 - y0 ) != 0.0 && fabs( x3 - x2 ) != 0.0 ) // ray parallel to y-axis and line seg not parallel to y axis
	  {
      //printf("line_to_line_seg_intersect- part 4.\n") ;
      y3y2y1y0 = (y3-y2) / (y1-y0) ;
      y2y0y1y0 = (y2-y0) / (y1-y0) ;
      *L = ( x1 - x2 ) / ( x3 - x2 ) ;
      *T = y2y0y1y0 + *L * y3y2y1y0 ;
      if( *L>=0.0 && *L<=1.0 && *T >= 0.0 ) // tests if intersection point is actually in line segment and in FRONT of sensor
      {
        *xi = x1 ;
        *yi = ( 1.0 - *L ) * y2 + *L * y3 ;
        //printf("line_to_line_seg_intersect- xi: %f yi: %f\n",*xi,*yi);
        *d = (float)sqrt( ( *xi - x1 ) * ( *xi - x1 ) + ( *yi - y1 ) * ( *yi - y1 ) ) ;  // distance fro sensor to intersection point
        if( *d <= 0.0 )
        {
          return ( 0 ) ;  //FALSE
        }
        d32 = (float)sqrt( ( x3 - x2 ) * ( x3 - x2 ) + ( y3 - y2 ) * ( y3 - y2 ) ) ;  // length of line seg, assume non zero
        ip = ( *xi - x1 ) * ( x3 - x2 ) + ( *yi - y1 ) * ( y3 - y2 ) ; // inner product between two vectors
        th = (float)acos( ip / ( *d * d32 ) ) ; // equation cos(th12) = ip12 / ( d1 * d2 )
        th = th * 180.0 / pi ;
        *beta = 90.0 - th ;  // normal angle of incidence
        return( 1 ) ;  // TRUE	
      }
      else
      {
        return(0) ;
      }
	  }
	  else
	  {
	    //printf("line_to_line_seg_intersect- part 5.\n") ;
      return ( 0 ) ;  //FALSE
	  }
  }
}
	
/*------------------------ Agent related functions -------------------------------------------------------------------------------*/

AGENT_TYPE *make_agent( int index, float x, float y, float body_angle, float head_radius, float mass ) 
{ /* allocates space for and initializes variables for an agent.  Note that index must be > 0. */

  AGENT_TYPE *a ;
  int i ;
  
  if( index<=0 ) 
  {
    fprintf(stdout, "make_agent- ERROR, agent index must be > 0. No agent built, returning NULL.\n") ;
    return( NULL ) ;
  }
  a = (AGENT_TYPE *)malloc(sizeof(AGENT_TYPE)) ;
  testm(a,"make_agent","a") ; 
  a->index = index ;
  a->inworld_flag = 1 ;
  
  a->outstate = (EXTERNAL_STATE_TYPE *)malloc(sizeof(EXTERNAL_STATE_TYPE)) ;
  testm(a->outstate, "make_agent", "a->outstate") ; 
  a->outstate->x = x ;
  a->outstate->y = y ;
  a->outstate->body_angle = body_angle ; 
  a->outstate->head_radius = head_radius ;
  a->outstate->head_angle = 0 ;
  a->outstate->physical_shape = NULL ; 
  a->outstate->sound_shape = NULL ; 
  
  a->instate = (INTERNAL_STATE_TYPE *)malloc(sizeof(INTERNAL_STATE_TYPE)) ;
  testm(a->instate, "make_agent", "a->instate") ; 
  a->instate->mass = mass ;
  a->instate->skin = NULL ;
  a->instate->encoders = NULL ;
  a->instate->cargo = NULL ;
  a->instate->bodyactuators = NULL ;
  a->instate->headactuators = NULL ;
  a->instate->brain = NULL ;
  a->instate->metabolic_charge = 1.0 ;
  a->instate->metabolic_burn_rate = 0.0 ;
  
  a->instate->maxneyes = 2 ;					/* <---------------------- HARD CODED LIMIT ------------------------<<<< */
  a->instate->neyes = 0 ;
  a->instate->eyes = (VISUAL_SENSOR_TYPE **)malloc(a->instate->maxneyes*sizeof(VISUAL_SENSOR_TYPE *)) ;
  testm(a->instate->eyes, "make_agent", "a->instate->eyes") ;
  
  a->instate->maxnears = 2 ;					/* <---------------------- HARD CODED LIMIT ------------------------<<<< */
  a->instate->nears = 0 ;
  a->instate->ears = (ACOUSTIC_SENSOR_TYPE **)malloc(a->instate->maxnears*sizeof(ACOUSTIC_SENSOR_TYPE *)) ;
  testm(a->instate->ears, "make_agent", "a->instate->ears") ;
  
  a->outstate->bounding_radius = 0 ;
  
  for( i=0 ; i<10 ; i++ )
  {
    a->instate->ftemp[i] = 0.0 ;
    a->instate->itemp[i] = 0 ;
  }
  
  return( a ) ;
}

void add_visual_sensor_to_agent( AGENT_TYPE *agent, int nreceptors, int nbands, float initial_values, float *receptor_locations,  
                               float *receptor_directions ) 
{ /* allocate a visual sensor, initial it, and add it to the agent */

  VISUAL_SENSOR_TYPE *r ;
  int i,j ;
  
  r = (VISUAL_SENSOR_TYPE *)malloc(sizeof(VISUAL_SENSOR_TYPE)) ;
  testm(r, "add_visual_sensor_to_agent", "v") ;
  
  r->nreceptors = nreceptors ;
  r->nbands = nbands ;
  
  r->values = (float **)malloc(nreceptors*sizeof(float *)) ;
  testm(r->values, "add_visual_sensor_to_agent", "r->values") ;
  for( i=0 ; i<nreceptors ; i++ )
  {
    r->values[i] = (float *)malloc(nbands*sizeof(float)) ;
    testm(r->values[i], "add_visual_sensor_to_agent", "r->values[i]") ;   
  }
    
  r->intensities = (float *)malloc(nreceptors*sizeof(float)) ;
  testm(r->intensities, "add_visual_sensor_to_agent", "r->intensities") ;
  for( i=0 ; i<nreceptors ; i++ )
  {
    r->intensities[i] = 0.0 ;   
  }

  r->seen_objects = (OBJECT_TYPE **)malloc(nreceptors*sizeof(OBJECT_TYPE *)) ;
  testm(r->seen_objects, "add_visual_sensor_to_agent", "r->seen_objects") ;
  for( i=0 ; i<nreceptors ; i++ )
  {
    r->seen_objects[i] = NULL ;   
  }

  r->receptor_locations = (float *)malloc(nreceptors*sizeof(float)) ;
  testm(r->receptor_locations, "add_visual_sensor_to_agent", "r->receptor_locations") ;
  r->receptor_directions = (float *)malloc(nreceptors*sizeof(float)) ;
  testm(r->receptor_directions, "add_visual_sensor_to_agent", "r->receptor_directions") ;
  
  for( i=0 ; i<nreceptors ; i++ )
  {
    r->receptor_locations[i] = receptor_locations[i] ;
    r->receptor_directions[i] = receptor_directions[i] ;
    for( j=0 ; j<nbands ; j++ ) 
	{
	  r->values[i][j] = initial_values ;  /* note band major ordering: ie ( b11,b12,b13, b21,b22,b23, b31,b32,b33) */
	}
  }
  if(  agent->instate->neyes < agent->instate->maxneyes )
  {
    agent->instate->eyes[agent->instate->neyes] = r ;
    agent->instate->neyes++ ;
  }
  else
    fprintf(stdout, "add_visual_sensor_to_agent- ERROR max neyes exceeded, no new eye added to agent #: %d\n",agent->index) ;

  return ;
}

void free_visual_sensor_type( VISUAL_SENSOR_TYPE *r )
{
  int i ;

  free( r->receptor_directions ) ;
  free( r->receptor_locations ) ;
  for( i=0 ; i<r->nreceptors ; i++ )
    free( r->values[i] ) ;
  free( r->values ) ;
  free( r ) ;
}

void add_acoustic_sensor_to_agent( AGENT_TYPE *agent, int nreceptors, int nbands, float initial_values, float receptor_location ) 
{ /* allocate a aural sensor, initial it, and add it to the agent */

  ACOUSTIC_SENSOR_TYPE *r ;
  int i,j ;
  
  r = (ACOUSTIC_SENSOR_TYPE *)malloc(sizeof(ACOUSTIC_SENSOR_TYPE)) ;
  testm(r, "add_acoustic_sensor_to_agent", "r") ;
  
  r->nreceptors = nreceptors ;
  r->nbands = nbands ;
  r->receptor_location = receptor_location ;
  r->intensity = 0.0 ;
  
  r->values = (float **)malloc(nreceptors*sizeof(float *)) ;
  testm(r->values, "add_acoustic_sensor_to_agent", "r->values") ;
  for( i=0 ; i<nreceptors ; i++ )
  {
    r->values[i] = (float *)malloc(nbands*sizeof(float)) ;
    testm(r->values[i], "add_acoustic_sensor_to_agent", "r->values[i]") ;   
  }
  
  for( i=0 ; i<nreceptors ; i++ )
  {
    for( j=0 ; j<nbands ; j++ ) 
	  {
	    r->values[i][j] = initial_values ;  /* note band major ordering: ie ( b11,b12,b13, b21,b22,b23, b31,b32,b33) */
	  }
  }
  if(  agent->instate->nears < agent->instate->maxnears )
  {
    agent->instate->ears[agent->instate->nears] = r ;
    agent->instate->nears++ ;
  }
  else
    fprintf(stdout, "add_acoustic_sensor_to_agent- ERROR max nears exceeded, no new ear added to agent #: %d\n",agent->index) ;

  return ;
}

void free_acoustic_sensor_type( ACOUSTIC_SENSOR_TYPE *r )
{
  int i ;

  for( i=0 ; i<r->nreceptors ; i++ )
    free( r->values[i] ) ;
  free( r->values ) ;
  free( r ) ;
}

void add_soma_sensor_to_agent( AGENT_TYPE *agent, int nbands, float initial_values, GEOMETRIC_SHAPE_TYPE *receptor_locations ) 
{ /* allocate a sensor, initial it, and add it to the agent */

  SOMA_SENSOR_TYPE *r ;
  int i,j, b;
  int nreceptors ;
  
  r = (SOMA_SENSOR_TYPE *)malloc(sizeof(SOMA_SENSOR_TYPE)) ;
  testm(r, "add_soma_sensor_to_agent", "r") ;

  nreceptors = receptor_locations->nvertices ;
  r->nreceptors = nreceptors ;
  r->nbands = nbands ;  /* receptor value bands */
  
  /* allocate space for skin cell values */
  r->values = (float **)malloc(nreceptors*sizeof(float *)) ;
  testm(r->values, "add_soma_sensor_to_agent", "r->values") ;
  for( i=0 ; i<nreceptors ; i++ )
  {
    r->values[i] = (float *)malloc(nbands*sizeof(float)) ;
    testm(r->values[i], "add_soma_sensor_to_agent", "r->values[i]") ;   
  }
  
  /* allocate space for skin cell object in collision list */
  r->touched_objects = (int *)malloc(nreceptors*sizeof(int)) ;
  testm(r->touched_objects, "add_soma_sensor_to_agent", "o->touched_objects") ;
  for( i=0 ; i<nreceptors ; i++ )
    r->touched_objects[i] = 0 ;

  /* Allocate space for geometric shpae, each facet of which is a skin cell */
  r->receptor_locations = make_geometric_shape_type( nreceptors, 5, 0.0, 1.0 ) ;
  
  /* Initialize values */
  for( i=0 ; i<nreceptors ; i++ )
  {
    for( j=0 ; j<5 ; j++ ) /* Copy vertex values and colors to soma sensor, 2 locations, 3 colors */
      r->receptor_locations->vertices[i][j] = receptor_locations->vertices[i][j] ;
    for( b=0 ; b<nbands ; b++ ) 
		{
	    r->values[i][b] = initial_values ;  /* note band major ordering: ie ( b11,b12,b13, b21,b22,b23, b31,b32,b33) */
	  }
  }
  agent->instate->skin = r ;

  return ;
}

void free_soma_sensor_type( SOMA_SENSOR_TYPE *r )
{
  int i ;

  free( r->touched_objects ) ;
  free_geometric_shape_type( r->receptor_locations ) ;
  for( i=0 ; i<r->nreceptors ; i++ )
    free( r->values[i] ) ;
  free( r->values ) ;
  free( r ) ;
}

void add_proprio_sensor_to_agent( AGENT_TYPE *agent, int nreceptors, int nbands, float initial_values, float *receptor_locations ) 
{ /* allocate a sensor, initial it, and add it to the agent */

  PROPRIO_SENSOR_TYPE *r ;
  int i,j ;
  
  r = (PROPRIO_SENSOR_TYPE *)malloc(sizeof(PROPRIO_SENSOR_TYPE)) ;
  testm(r, "add_proprio_sensor_to_agent", "r") ;
  
  r->nreceptors = nreceptors ;
  r->nbands = nbands ;
  
  r->values = (float **)malloc(nreceptors*sizeof(float *)) ;
  testm(r->values, "add_proprio_sensor_to_agent", "r->values") ;
  for( i=0 ; i<nreceptors ; i++ )
  {
    r->values[i] = (float *)malloc(nbands*sizeof(float)) ;
    testm(r->values[i], "add_proprio_sensor_to_agent", "r->values[i]") ;   
  }
 
  r->receptor_locations = (float *)malloc(nreceptors*sizeof(float)) ;
  testm(r->receptor_locations, "add_proprio_sensor_to_agent", "r->receptor_locations") ;
  
  for( i=0 ; i<nreceptors ; i++ )
  {
    r->receptor_locations[i] = receptor_locations[i] ;
    for( j=0 ; j<nbands ; j++ ) 
	{
	  r->values[i][j] = initial_values ;  /* note band major ordering: ie ( b11,b12,b13, b21,b22,b23, b31,b32,b33) */
	}
  }
  agent->instate->encoders = r ;

  return ;
}

void free_proprio_sensor_type( PROPRIO_SENSOR_TYPE *r )
{
  int i ;

  free( r->receptor_locations ) ;
  for( i=0 ; i<r->nreceptors ; i++ )
    free( r->values[i] ) ;
  free( r->values ) ;
  free( r ) ;
}

void add_cargo_manifest_type_to_agent( AGENT_TYPE *a, int maxnitems ) 
{ /* allocate a manifest, initial it, and add it to the agent */

  CARGO_MANIFEST_TYPE *r ;
  int i ;
  
  r = (CARGO_MANIFEST_TYPE *)malloc(sizeof(CARGO_MANIFEST_TYPE)) ;
  testm(r, "add_cargo_manifest_type_to_agent", "r") ;
  
  r->nitems = 0 ;
  r->maxnitems = maxnitems ;  					/* <---------------------- HARD CODED LIMIT ------------------------<<<< */
  
  r->manifest = (void **)malloc(r->maxnitems*sizeof(void *)) ;
  testm(r->manifest, "add_cargo_manifest_type_to_agent", "r->manifest") ;
  
  for( i=0 ; i<r->maxnitems ; i++ )
    r->manifest[i] = NULL ;

  a->instate->cargo = r ;
  
  return ;
}

void free_cargo_manifest_type( CARGO_MANIFEST_TYPE *r  )
{
  free( r->manifest ) ;
  free( r ) ;
}

void add_actuators_to_agent( AGENT_TYPE *a )
{ /* allocates memory and initializes body and head actuators */

  a->instate->bodyactuators = ( BODY_ACTUATOR_TYPE *)malloc(sizeof(BODY_ACTUATOR_TYPE)) ;
  testm(a->instate->bodyactuators, "add_actuators_to_agent", "a->instate->bodyactuators") ;
  a->instate->bodyactuators->deltaFB = 0.0 ;
  a->instate->bodyactuators->deltaRL = 0.0 ;
  a->instate->bodyactuators->deltaTH = 0.0 ;  
  
  a->instate->headactuators = ( HEAD_ACTUATOR_TYPE *)malloc(sizeof(HEAD_ACTUATOR_TYPE)) ;
  testm(a->instate->headactuators, "add_actuators_to_agent", "a->instate->headactuators") ;
  a->instate->headactuators->deltaH = 0.0 ;

}

void free_actuators_type( AGENT_TYPE *a ) 
{
  free( a->instate->bodyactuators ) ;
  free( a->instate->headactuators ) ; 
}

void add_physical_shape_to_agent( AGENT_TYPE *a, GEOMETRIC_SHAPE_TYPE *shape )
{ /* adds a physical shape to the agent. computes bounding shape information too */
  int i ;
  float x,y,radius,maxradius=0.0 ;
  float xc,yc ;
 
  a->outstate->physical_shape = shape ;
  x = a->outstate->x ;
  y = a->outstate->y ;
  a->outstate->xmaxbody = -1.0e6 ;
  a->outstate->xminbody = 1.0e6 ; 
  a->outstate->ymaxbody = -1.0e6 ; 
  a->outstate->yminbody = 1.0e6 ;

  for( i=0 ; i<shape->nvertices ; i++ ) 
  {
    if( shape->vertices[i][0]>a->outstate->xmaxbody )
      a->outstate->xmaxbody = shape->vertices[i][0] ;
    if( shape->vertices[i][0]<a->outstate->xminbody )
      a->outstate->xminbody = shape->vertices[i][0] ;
    if( shape->vertices[i][1]>a->outstate->ymaxbody )
      a->outstate->ymaxbody = shape->vertices[i][1] ;
    if( shape->vertices[i][1]<a->outstate->yminbody )
      a->outstate->yminbody = shape->vertices[i][1] ;
  }
  xc = (a->outstate->xmaxbody - a->outstate->xminbody) * 0.5 ;
  yc = (a->outstate->ymaxbody - a->outstate->yminbody) * 0.5 ;
  for( i=0 ; i<shape->nvertices ; i++ ) 
  {
    radius = L2measure( xc, yc, shape->vertices[i][0], shape->vertices[i][1] ) ;
    if( radius > maxradius )
      maxradius = radius ;
  }
  a->outstate->bounding_radius = maxradius ;

  return ;
}

void add_sound_shape_to_agent( AGENT_TYPE *a, ACOUSTIC_SHAPE_TYPE *shape )
{ /* adds a sound shape to the agent */

  a->outstate->sound_shape = shape ;

  return ;
}

/******************************  Reading Sensors *************************************************************/

void read_visual_sensor( WORLD_TYPE *w, AGENT_TYPE *a)
{ /* loops over eyes and objects/agents computing the color intensity of beam sensors. Note work is needed to optimize the computation */
  VISUAL_SENSOR_TYPE **eyes, *eye ;
  OBJECT_TYPE *obj ;
  OBJECT_TYPE **seen_objects ;
  AGENT_TYPE *agent ;
  int neyes, nrec, nbands ;
  int ne, i,j,k ;
  int nobjects ;
  int nagents ;
  int intersect_segment, iintersect_segment ;
  int iobject,iagent ;
  float **values, **ovalues ;
  float *loc, *dir ;
  float xi, yi ;
  float rt,rr ;
  float mx, my ;
  float ximin, yimin ;
  float sx, sy, st ;
  float tx, ty ;
  float *intensities ;

  float dmin, dmax = 50.0 ; /* <---------------------- HARD CODED LIMIT ------------------------<<<< */
  float ox, oy, dro, dd, beta, angle, angle_effect ;
  float visualsensorthreshold = 0.0 ;  // Visual sensor threshold, will not show detection if light level lower
  float deg2rad = 1.745329251994e-02 ;
  float pi = 3.1415926 ;
  
  nobjects = w->nobjects ;
  
  eyes = a->instate->eyes ;
  neyes = a->instate->neyes ;
  mx = a->outstate->x ;
  my = a->outstate->y ;
  rt = a->outstate->body_angle + a->outstate->head_angle ;
  rr = a->outstate->head_radius ;
  
  //printf("\n\nread_visual_sensor- Entry.\n") ;
  
  /* Loop over eyes */
  for( ne=0 ; ne<neyes ; ne++ )
  {
    eye = eyes[ne] ;
	  nrec = eye->nreceptors ;
	  nbands = eye->nbands ;
    values = eye->values ;
    intensities = eye->intensities ;
    seen_objects  = eye->seen_objects ;
	  loc = eye->receptor_locations ;
	  dir = eye->receptor_directions ;  /* deviation from normal radial */

    /* Loop over receptors in an eye */
    for( j=0 ; j<nrec ; j++ ) 
	  {
	    /* Initialize sensor values to zero */
	    for( k=0 ; k<nbands ; k++ )
	      values[j][k] = 0.0 ;
      intensities[j] = 0.0 ;
      seen_objects[j] = NULL ;
	
      /* calculating coordinates of sensor in world coodinates then loop over objects anf agents */
      st= loc[j] + rt ;                /* Loading the angle of the sensor j taking into account agents's head and body angle */
      sx = mx + cos(pi * st / 180.0) * rr ;
      sy = my - sin(pi * st / 180.0) * rr ;
      tx = sx + 0.1*cos( (st+dir[j])*deg2rad ) ;
      ty = sy - 0.1*sin( (st+dir[j])*deg2rad ) ;
	  	  
	    dmin = 1.0e6 ;
      /* Loop over objects in world */
	    for( i=0 ; i<nobjects ; i++ )
	    {
	      obj = w->objects[i] ;
        if( obj->type>0 && obj->inworld_flag==1 && obj->physical_shape!=NULL ) 
        {
          ox = obj->x ;
          oy = obj->y ;
          ovalues = obj->physical_shape->vertices ;
          dro = L2measure( ox, oy, sx, sy ) ;
          if( dro < dmax) 
          {
            dd = 0.0 ;
            beta = 0.0 ;
            intersect_segment = intersect_beam_with_object(obj, sx, sy, tx, ty, &xi, &yi, &dd, &beta ) ;
            if( dd<0.0 ) {
              //printf("read_visual_sensor- dd: neg obj: %d objtype: %d\n",i,obj->type) ;
              exit(0) ;
            } 
            if( intersect_segment >= 0 )        // TRUE >= 0
            {
              if( dd>0.0 && dd<dmin && dd<dmax ) 
              {
                angle =  beta * pi / 180.0 ;
                //angle_effect = cos ( angle ) / dd ;
                //angle_effect = ( dmax - dd) / dmax ;
                angle_effect = exp( - dd * 0.2763 ) ;
                
                dmin = dd ;
                ximin = xi ;
                yimin = yi ;
                iintersect_segment = intersect_segment ;
                iobject = i ;
                if( angle_effect >= visualsensorthreshold )
                {
                  //printf("read_visual_sensor- rec: %d obj: %d seg: %d dd: %f\n",j,i,intersect_segment,dd) ;
                  intensities[j] = 0.0 ;
                  for( k=0 ; k<nbands ; k++ )
                  {			
                    values[j][k] =  ovalues[intersect_segment][k+2] * angle_effect ;  /* the first two locations in ovalues[][] are the x,y vertex locations. */
                 
                    intensities[j] += (values[j][k]*values[j][k]) ;
                  }
                  intensities[j] = (float)sqrt(intensities[j]) ; /* total intensity of the spot on the object hit by the ray */
                  seen_objects[j] = obj ; /* save the pointer to the object that the ray is hitting */
                }
                else
                {
                  for( k=0 ; k<nbands ; k++ )
                  {			
                    values[j][k] = 0.0 ; 
                  }
                  intensities[j] = 0.0 ;
                  seen_objects[j] = NULL ;
                } 
              }
            }
          } /* end closeness test */
        } /* end object in world test */
      } /* End object loop */
      
      /* Begin loopin gover agents */
      nagents = w->nagents ;
	    for( i=0 ; i<nagents ; i++ )
	    {
	      agent = w->agents[i] ;
        if( agent->index!=a->index && agent->inworld_flag==1 &&  agent->outstate->physical_shape!=NULL) 
        {
          ox = agent->outstate->x ;
          oy = agent->outstate->y ;
          ovalues = agent->outstate->physical_shape->vertices ;
          dro = L2measure( ox, oy, sx, sy ) ;
          if( dro < dmax) 
          {
            dd = 0.0 ;
            beta = 0.0 ;
            intersect_segment = intersect_beam_with_agent(agent, sx, sy, tx, ty, &xi, &yi, &dd, &beta ) ;
            if( dd<0.0 ) {
              //printf("read_visual_sensor- dd: neg obj: %d objtype: %d\n",i,obj->type) ;
              exit(0) ;
            } 
            if( intersect_segment >= 0 )        // TRUE >= 0
            {
              if( dd>0.0 && dd<dmin ) 
              {
                angle =  beta * pi / 180.0 ;
                angle_effect = cos ( angle ) / dd  ;
                dmin = dd ;
                ximin = xi ;
                yimin = yi ;
                iintersect_segment = intersect_segment ;
                iagent = i ;
                if( angle_effect >= visualsensorthreshold )
                {
                  //printf("read_visual_sensor- rec: %d obj: %d seg: %d dd: %f angle_effect: %f\n",j,i,intersect_segment,dd,angle_effect) ;
                  intensities[j] = 0.0 ;
                  for( k=0 ; k<nbands ; k++ )
                  {			
                    angle_effect = 1.0 ;
                    values[j][k] =  ovalues[intersect_segment][k+2] * angle_effect ; /* the first two locations in ovalues[][] are the x,y vertex locations. */
                    //printf("k: %d values[j][k]: %f |",k,values[j][k]) ;
                    intensities[j] += (values[j][k]*values[j][k]) ;
                  }
                  //printf("\n") ;
                  intensities[j] = (float)sqrt(intensities[j]) ; /* total intensity */
                }
                else
                {
                  for( k=0 ; k<nbands ; k++ )
                  {			
                    values[j][k] = 0.0 ; 
                  }
                } 
              }
            }
          } /* end closeness test */
        } /* end agent in world test */
      } /* End agent loop */

    } /* end receptor loop */
  } /* End eyes loop */
}

float **extract_visual_receptor_values_pointer( AGENT_TYPE *a, int eye_index)
{ /* extracts the pointer for the receptor array for eye number eye_index.
  values is a 2D array the receptor values of size (nreceptors x nbands) */
  
  if( a->instate->eyes==NULL )
  {
    printf("extract_visual_receptor_values_pointer- ERROR, no eyes on this agent: %d\n",a->index) ;
    exit(0) ;
  }  
  if( eye_index >= a->instate->neyes )
  {
    printf("extract_visual_receptor_values_pointer- error, eye_index >= neyes. Nothing done.\n") ;
  }
  return(a->instate->eyes[eye_index]->values) ;
}

float visual_receptor_position( VISUAL_SENSOR_TYPE *eye, int recindex )
{ /* this function returns the pointning direction of an individual visual receptor in head coordinates. */

  return( eye->receptor_locations[recindex] + eye->receptor_directions[recindex] ) ;
}

int get_number_of_visual_receptors( AGENT_TYPE *a )
{ /* for one eye only. tpc 8/13/12 */
  
  return( a->instate->eyes[0]->nreceptors ) ;
}

int get_number_of_visual_bands( AGENT_TYPE *a )
{ /* for one eye only. tpc 8/13/12 */
  
  return( a->instate->eyes[0]->nbands ) ;
}

void read_acoustic_sensor( WORLD_TYPE *w, AGENT_TYPE *a)
{
  ACOUSTIC_SENSOR_TYPE **ears, *ear ;
  OBJECT_TYPE *obj ;
  AGENT_TYPE *agent ;
  int nears, nrec, nbands ;
  int ne, i,j,k ;
  int nobjects ;
  int nagents ;
  float **values ;
  float rt,rr ;
  float mx, my ;
  float sx, sy, st ;
  float ox, oy, dro, attenuation ;
  float deg2rad = 1.745329251994e-02 ;
  
  nobjects = w->nobjects ;
  
  ears = a->instate->ears ;
  nears = a->instate->nears ;
  mx = a->outstate->x ;
  my = a->outstate->y ;
  rt = a->outstate->body_angle + a->outstate->head_angle ;
  rr = a->outstate->head_radius ;
  
  /* Loop over ears */
  for( ne=0 ; ne<nears ; ne++ )
  {
    ear = ears[ne] ;
    nrec = ear->nreceptors ;
    nbands = ear->nbands ;
    values = ear->values ;
    ear->intensity = 0.0 ;
    for( j=0 ; j<nrec ; j++ ) 
      for( k=0 ; k<nbands ; k++ )
        values[j][k] = 0.0 ;
        
    /* Loop over objects */    
    for( i=0 ; i<nobjects ; i++ )
    {
      obj = w->objects[i] ;
      /*test if object is active */
      if( obj->inworld_flag==1 && obj->type>0 && obj->sound_shape!=NULL ) 
      {
        ox = obj->x ;
        oy = obj->y ;
        st= rt + ear->receptor_location ;  /* angle of the sensor j taking into account agents's 
                                                    head, body, and sensor angle */
        sx = mx + cos( st * deg2rad ) * rr ; /* positions of ear in world coordinates */
        sy = my - sin( st * deg2rad ) * rr ;
        dro = L2measure( ox, oy, sx, sy ) ;   /* disance from object center to ear */
        if( dro >= 1.0)
          attenuation = 1.0 / (dro*dro) ;
        else
          attenuation = 1.0 ;
        for( j=0 ; j<nrec ; j++ ) 
        {
          for( k=0 ; k<nbands ; k++ )
          {
            values[j][k] += obj->sound_shape->spectrum[j][k] * attenuation ; /* accumulate attenuated sounds from this object
                                                                                 assuming all bands are attenuated the same. */
            ear->intensity += (values[j][k]*values[j][k]) ;
          }
        }
      } /* end object active test */
    } /* end object loop */
    
    /* loop over other agents */
    nagents = w->nagents ;
    for( i=0 ; i<nagents ; i++ )
    {
      agent = w->agents[i] ;
      if( agent->index!=a->index && agent->inworld_flag==1 && agent->outstate->sound_shape!=NULL ) 
      {
        ox = agent->outstate->x ;
        oy = agent->outstate->y ;
        st= rt + ear->receptor_location ;  /* angle of the sensor j taking into account agents's 
                                                    head, body, and sensor angle */
        sx = mx + cos( st * deg2rad ) * rr ; /* positions of ear in world coordinates */
        sy = my - sin( st * deg2rad ) * rr ;
        dro = L2measure( ox, oy, sx, sy ) ;   /* disance from object center to ear */
        if( dro > 0.0)
          attenuation = 1.0 / dro ;
        else
          attenuation = 1.0 ;
        for( j=0 ; j<nrec ; j++ ) 
        {
          for( k=0 ; k<nbands ; k++ )
          {
            values[j][k] += agent->outstate->sound_shape->spectrum[j][k] * attenuation ; /* accumulate attenuated sounds from this object
                                                                                 assuming all bands are attenuated the same. */
            ear->intensity += (values[j][k]*values[j][k]) ;
          }
        }
      } /* end agent active test */
    } /* end agent loop */
    if( ear->intensity!=0.0)
      ear->intensity = log10f(ear->intensity) ;    
    
  } /* end ears loop */
}

float **extract_sound_receptor_values_pointer( AGENT_TYPE *a, int ear_index)
{ /* extracts the pointer for the receptor array for ear number ear_index.
  values is a 2D array the receptor values of size (nreceptors x nbands) */
  
  if( a->instate->ears==NULL )
  {
    printf("extract_sound_receptor_values_pointer- ERROR, no ears on this agent: %d\n",a->index) ;
    exit(0) ;
  }
  if( ear_index >= a->instate->nears )
  {
    printf("extract_sound_receptor_values_pointer- error, ear_index >= nears. Nothing done.\n") ;
  }  
  return(a->instate->ears[ear_index]->values) ;
}

int get_number_of_acoustic_receptors( AGENT_TYPE *a )
{ /* for one ear only. tpc 8/13/12 */
  
  return( a->instate->ears[0]->nreceptors ) ;
}

int get_number_of_acoustic_bands( AGENT_TYPE *a )
{ /* for one ear only. tpc 8/13/12 */
  
  return( a->instate->ears[0]->nbands ) ;
}

int read_soma_sensor( WORLD_TYPE *w, AGENT_TYPE *a)
{ /* returns the number of object or agent line segments that are in collision. array touched_object has positive element 
     for collision with "object" and negative index for collision with "agent". Assumes soma value[i][0] (band=0)is
     pressure sensor. Other bands could be pain or temperature.  */

  SOMA_SENSOR_TYPE *soma ;
  OBJECT_TYPE *obj ;
  AGENT_TYPE *agent ;
  int  nrec, nbands ;
  int i,j,k,m ;
  int nobjects, nagents ;
  int novertices, nsvertices ;
  int flag = 0 ;
  int collision_flag = 0 ;
  int *touched_objects ;
  float **values ;
  float **overtices, **svertices=NULL ;
  float xi, yi ;
  float rt ;
  float sx, sy ;
  float ox, oy, dd, beta;
  float L, T ;
  
  //printf("\n\nread_soma_sensor- Entry. simtime: %d.\n",simtime) ; fflush(stdout) ;
    
  soma = a->instate->skin ;
  sx = a->outstate->x ;
  sy = a->outstate->y ;
  rt = a->outstate->body_angle ;  
  touched_objects = soma->touched_objects ;
 
  nrec = soma->nreceptors ; /* same as nsvertices */
  nsvertices = soma->receptor_locations->nvertices ;
  if( nrec != nsvertices ) 
  {
    fprintf(stdout,"read_soma_sensor- ERROR, nrec != nsvertices. Exiting.\n") ;
    exit(0) ;
  }
  nbands = soma->nbands ;
  values = soma->values ;
  
  /* Initialize sensor values to zero  */
  for( j=0 ; j<nrec ; j++ ) 
    for( k=0 ; k<nbands ; k++ )
      values[j][k] = 0.0 ;
  
  /* Initialize touched objects to -1, no history of collisions */
  for( j=0 ; j<nrec ; j++ ) 
  {
    touched_objects[j] = 0 ;
  }
  
  /* Loop over world objects */
  nobjects = w->nobjects ;
  for( i=0 ; i<nobjects ; i++ )
  {  
    obj = w->objects[i] ;
    if( obj->inworld_flag==1 ) 
    {
      ox = obj->x ;
      oy = obj->y ;
      //printf("read_soma_sensor- Comparing with object: %d L2m: %f obr: %f abr: %f  b: %f\n",obj->index, L2measure( ox, oy, sx, sy ), obj->bounding_radius,a->outstate->bounding_radius, ( obj->bounding_radius + a->outstate->bounding_radius )) ;
      
      /* test if object is active  and is close enough to collide with */
      if( obj->type>0 &&  L2measure( ox, oy, sx, sy ) <= ( obj->bounding_radius + a->outstate->bounding_radius ) )
      {
        //printf("read_soma_sensor- Passed bounding cirlce test.\n") ;
        if( flag == 0 )
        {
          /* if at least one object is in possible collision, rotate agent vertices by body angle amount around agent center, use world's scratch GOEMETRIC_SHAPE_TYPE */
          if( soma->receptor_locations->nvertices > w->scratch_geo_shape_type->nvertices )
          {
            fprintf(stdout,"read_soma_sensor- ERROR, w->scratch_geo_shape_type too small for agent: %d shape.\n",a->index) ;
            exit(0) ;
          }
          rotate2D( w->scratch_geo_shape_type, soma->receptor_locations, sx, sy, -a->outstate->body_angle) ;
          svertices = w->scratch_geo_shape_type->vertices ;
          //printf("read_soma_sensor- completed first time rotation of agent shape.\n") ;
          flag = 1 ;
        }
        overtices = obj->physical_shape->vertices ;
        novertices = obj->physical_shape->nvertices ;
        dd = 0.0 ;
        beta = 0.0 ;
        
        /* Test each soma cell against each object segment */
        for( j=0 ; j<nsvertices-1 ; j++ )  /* agent's vertices */
        {
          for( m=0 ; m<novertices-1 ; m++ ) /* objects vertices */
          {
            line_to_line_seg_intersect( svertices[j][0], svertices[j][1], svertices[j+1][0], svertices[j+1][1],
                         overtices[m][0]+ox, overtices[m][1]+oy, overtices[m+1][0]+ox, overtices[m+1][1]+oy, &xi, &yi, &dd, &beta, 
                         &T, &L ) ;
            if( T>=0 && T<=1.0 && L>=0.0 && L<=1.0 ) /* condition for collision of one skin cell an done object segment */
            {
              values[j][0] += 1.0 ;  /* only one band at this time. integrate number of segments in collision with this cell */
              touched_objects[j] = i+1 ;  /* the idenity of the touched object. Not intended to be knowable by neural network */
              //printf("  j: %d m: %d collision detected a. dd: %f T: %f L: %f\n",j,m,dd,T,L) ;
              //printf("    sx0: %f sy0: %f sx1: %f sy1: %f\n",svertices[j][0], svertices[j][1], svertices[j+1][0], svertices[j+1][1]) ;
              //printf("    ox0: %f oy0: %f ox1: %f oy1: %f\n",overtices[m][0]+ox, overtices[m][1]+oy, overtices[m+1][0]+ox, overtices[m+1][1]+oy) ;
              collision_flag++ ;
            }
          }
          /* Handle wrap-around of vertices for object shape */
          line_to_line_seg_intersect( svertices[j][0], svertices[j][1], svertices[j+1][0], svertices[j+1][1],
                      overtices[novertices-1][0]+ox, overtices[novertices-1][1]+oy, overtices[0][0]+ox, overtices[0][1]+oy, &xi, &yi, &dd, &beta, 
                      &T, &L ) ;
          if( T>=0 && T<=1.0 && L>=0.0 && L<=1.0 ) /* condition for collision of one skin cell an done object segment */
          {
              values[j][0] += 1.0 ;  /* only one band at this time. integrate number of segments in collision with this cell */
              touched_objects[j] = i+1 ;  /* teh idenity of the touched object. Not intended to be knowable by neural network */
              //printf("  j: %d m: %d collision detected b. dd: %f T: %f L: %f\n",j,novertices-1,dd,T,L) ;         
              //printf("    sx0: %f sy0: %f sx1: %f sy1: %f\n",svertices[j][0], svertices[j][1], svertices[j+1][0], svertices[j+1][1]) ;
              //printf("    ox0: %f oy0: %f ox1: %f oy1: %f\n",overtices[novertices-1][0]+ox, overtices[novertices-1][1]+oy, overtices[0][0]+ox, overtices[0][1]+oy) ;
              collision_flag++ ;
          }
        } /* end agent n-1 vertices loop */
        /* Handle wrap-around of vertices for agent shape */
        for( m=0 ; m<novertices-1 ; m++ )
        {
          line_to_line_seg_intersect( svertices[nsvertices-1][0], svertices[nsvertices-1][1], svertices[0][0], svertices[0][1],
                       overtices[m][0]+ox, overtices[m][1]+oy, overtices[m+1][0]+ox, overtices[m+1][1]+oy, &xi, &yi, &dd, &beta, 
                       &T, &L ) ;
          if( T>=0 && T<=1.0 && L>=0.0 && L<=1.0 ) /* condition for collision of one skin cell an done object segment */
          {
            values[nsvertices-1][0] += 1.0 ;  /* only one band at this time. integrate number of segments in collision with this cell */
            touched_objects[j] = i+1 ;  /* teh idenity of the touched object. Not intended to be knowable by neural network */
            //printf("  j: %d m: %d collision detected c. dd: %f T: %f L: %f\n",nsvertices-1,m,dd,T,L) ;          
            //printf("    sx0: %f sy0: %f sx1: %f sy1: %f\n",svertices[nsvertices-1][0], svertices[nsvertices-1][1], svertices[0][0], svertices[0][1]) ;
            //printf("    ox0: %f oy0: %f ox1: %f oy1: %f\n",overtices[m][0]+ox, overtices[m][1]+oy, overtices[m+1][0]+ox, overtices[m+1][1]+oy) ;
            collision_flag++ ;
          }
        }
        /* Handle wrap-around of vertices for object and agent shape */
        line_to_line_seg_intersect( svertices[nsvertices-1][0], svertices[nsvertices-1][1], svertices[0][0], svertices[0][1],
                    overtices[novertices-1][0]+ox, overtices[novertices-1][1]+oy, overtices[0][0]+ox, overtices[0][1]+oy, &xi, &yi, &dd, &beta, 
                    &T, &L ) ;
        if( T>=0 && T<=1.0 && L>=0.0 && L<=1.0 ) /* condition for collision of one skin cell an done object segment */
        {
            values[nsvertices-1][0] += 1.0 ;  /* only one band at this time. integrate number of segments in collision with this cell */
            touched_objects[j] = i+1 ;  /* teh idenity of the touched object. Not intended to be knowable by neural network */
            //printf("  j: %d m: %d collision detected d. dd: %f T: %f L: %f\n",nsvertices-1,novertices-1,dd,T,L) ;         
            //printf("    sx0: %f sy0: %f sx1: %f sy1: %f\n",svertices[nsvertices-1][0], svertices[nsvertices-1][1], svertices[0][0], svertices[0][1]) ;
            //printf("    ox0: %f oy0: %f ox1: %f oy1: %f\n",overtices[novertices-1][0]+ox, overtices[novertices-1][1]+oy, overtices[0][0]+ox, overtices[0][1]+oy) ;
            collision_flag++ ;
        }
      } /* end pass-bounding circle test segment */
    } /* end active object test */
  } /* end object loop */
  
  /* Loop over world agents */
  nagents = w->nagents ;
  for( i=0 ; i<nagents ; i++ )
  {  
    agent = w->agents[i] ;
    if( agent->index!=a->index && agent->inworld_flag==1 ) 
    {
      ox = agent->outstate->x ;
      oy = agent->outstate->y ;
      //printf("read_soma_sensor- a->index: %d comparing with agent->index: %d L2m: %f sum: %f\n",a->index, agent->index, L2measure( ox, oy, sx, sy ), ( agent->outstate->bounding_radius + a->outstate->bounding_radius )) ;
      //printf("read_soma_sensor- ox: %f oy: %f sx: %f sy: %f agbr: %f abr: %f\n",ox,oy,sx,sy,agent->outstate->bounding_radius,a->outstate->bounding_radius) ;
      
      /* test if agent is not self and is close enough to collide with */
      if(  L2measure( ox, oy, sx, sy ) <= ( agent->outstate->bounding_radius + a->outstate->bounding_radius ) )
      {
        //printf("read_soma_sensor- Passed bounding cirlce test for agents. a->index: %d agent->index: %d\n",a->index, agent->index) ;
        if( flag == 0 )
        {
          /* if at least one object is in possible collision, rotate agent vertices by body angle amount around agent center, use world's scratch GOEMETRIC_SHAPE_TYPE */
          if( soma->receptor_locations->nvertices > w->scratch_geo_shape_type->nvertices )
          {
            fprintf(stdout,"read_soma_sensor- ERROR, w->scratch_geo_shape_type too small for agent: %d shape.\n",a->index) ;
            exit(0) ;
          }
          rotate2D( w->scratch_geo_shape_type, soma->receptor_locations, sx, sy, -a->outstate->body_angle) ;
          svertices = w->scratch_geo_shape_type->vertices ;
          //printf("read_soma_sensor- completed first time rotation of agent shape.\n") ;
          flag = 1 ;
        }
        if( agent->outstate->physical_shape->nvertices > w->scratch_geo_shape_type2->nvertices )
        {
          fprintf(stdout,"read_soma_sensor- ERROR, w->scratch_geo_shape_type2 too small for agent: %d shape.\n",agent->index) ;
          exit(0) ;
        }
        //overtices = agent->outstate->physical_shape->vertices ;
        rotate2D( w->scratch_geo_shape_type2, agent->outstate->physical_shape, 0.0, 0.0, -agent->outstate->body_angle) ;        
        overtices = w->scratch_geo_shape_type2->vertices ;
        
        novertices = agent->outstate->physical_shape->nvertices ;
        dd = 0.0 ;
        beta = 0.0 ;
        
        /* Test each soma cell against each object segment */
        for( j=0 ; j<nsvertices-1 ; j++ )  /* agent's vertices */
        {
          for( m=0 ; m<novertices-1 ; m++ ) /* objects vertices */
          {
            line_to_line_seg_intersect( svertices[j][0], svertices[j][1], svertices[j+1][0], svertices[j+1][1],
                         overtices[m][0]+ox, overtices[m][1]+oy, overtices[m+1][0]+ox, overtices[m+1][1]+oy, &xi, &yi, &dd, &beta, 
                         &T, &L ) ;
            if( T>=0 && T<=1.0 && L>=0.0 && L<=1.0 ) /* condition for collision of one skin cell an done object segment */
            {
              values[j][0] += 1.0 ;  /* only one band at this time. integrate number of segments in collision with this cell */
              touched_objects[j] = -(i+1) ;  /* teh idenity of the touched object. Not intended to be knowable by neural network */
              //printf("  j: %d m: %d collision detected a. dd: %f T: %f L: %f\n",j,m,dd,T,L) ;
              //printf("    sx0: %f sy0: %f sx1: %f sy1: %f\n",svertices[j][0], svertices[j][1], svertices[j+1][0], svertices[j+1][1]) ;
              //printf("    ox0: %f oy0: %f ox1: %f oy1: %f\n",overtices[m][0]+ox, overtices[m][1]+oy, overtices[m+1][0]+ox, overtices[m+1][1]+oy) ;
              collision_flag++ ;
            }
          }
          /* Handle wrap-around of vertices for object shape */
          line_to_line_seg_intersect( svertices[j][0], svertices[j][1], svertices[j+1][0], svertices[j+1][1],
                      overtices[novertices-1][0]+ox, overtices[novertices-1][1]+oy, overtices[0][0]+ox, overtices[0][1]+oy, &xi, &yi, &dd, &beta, 
                      &T, &L ) ;
          if( T>=0 && T<=1.0 && L>=0.0 && L<=1.0 ) /* condition for collision of one skin cell an done object segment */
          {
              values[j][0] += 1.0 ;  /* only one band at this time. integrate number of segments in collision with this cell */
              touched_objects[j] = -(i+1)  ;  /* teh idenity of the touched object. Not intended to be knowable by neural network */
              //printf("  j: %d m: %d collision detected b. dd: %f T: %f L: %f\n",j,novertices-1,dd,T,L) ;         
              //printf("    sx0: %f sy0: %f sx1: %f sy1: %f\n",svertices[j][0], svertices[j][1], svertices[j+1][0], svertices[j+1][1]) ;
              //printf("    ox0: %f oy0: %f ox1: %f oy1: %f\n",overtices[novertices-1][0]+ox, overtices[novertices-1][1]+oy, overtices[0][0]+ox, overtices[0][1]+oy) ;
              collision_flag++ ;
          }
        } /* end agent n-1 vertices loop */
        /* Handle wrap-around of vertices for agent shape */
        for( m=0 ; m<novertices-1 ; m++ )
        {
          line_to_line_seg_intersect( svertices[nsvertices-1][0], svertices[nsvertices-1][1], svertices[0][0], svertices[0][1],
                       overtices[m][0]+ox, overtices[m][1]+oy, overtices[m+1][0]+ox, overtices[m+1][1]+oy, &xi, &yi, &dd, &beta, 
                       &T, &L ) ;
          if( T>=0 && T<=1.0 && L>=0.0 && L<=1.0 ) /* condition for collision of one skin cell an done object segment */
          {
            values[nsvertices-1][0] += 1.0 ;  /* only one band at this time. integrate number of segments in collision with this cell */
            touched_objects[j] = -(i+1)  ;  /* teh idenity of the touched object. Not intended to be knowable by neural network */
            //printf("  j: %d m: %d collision detected c. dd: %f T: %f L: %f\n",nsvertices-1,m,dd,T,L) ;          
            //printf("    sx0: %f sy0: %f sx1: %f sy1: %f\n",svertices[nsvertices-1][0], svertices[nsvertices-1][1], svertices[0][0], svertices[0][1]) ;
            //printf("    ox0: %f oy0: %f ox1: %f oy1: %f\n",overtices[m][0]+ox, overtices[m][1]+oy, overtices[m+1][0]+ox, overtices[m+1][1]+oy) ;
            collision_flag++ ;
          }
        }
        /* Handle wrap-around of vertices for object and agent shape */
        line_to_line_seg_intersect( svertices[nsvertices-1][0], svertices[nsvertices-1][1], svertices[0][0], svertices[0][1],
                    overtices[novertices-1][0]+ox, overtices[novertices-1][1]+oy, overtices[0][0]+ox, overtices[0][1]+oy, &xi, &yi, &dd, &beta, 
                    &T, &L ) ;
        if( T>=0 && T<=1.0 && L>=0.0 && L<=1.0 ) /* condition for collision of one skin cell an done object segment */
        {
            values[nsvertices-1][0] += 1.0 ;  /* only one band at this time. integrate number of segments in collision with this cell */
            touched_objects[j] = -(i+1)  ;  /* teh idenity of the touched object. Not intended to be knowable by neural network */
            //printf("  j: %d m: %d collision detected d. dd: %f T: %f L: %f\n",nsvertices-1,novertices-1,dd,T,L) ;         
            //printf("    sx0: %f sy0: %f sx1: %f sy1: %f\n",svertices[nsvertices-1][0], svertices[nsvertices-1][1], svertices[0][0], svertices[0][1]) ;
            //printf("    ox0: %f oy0: %f ox1: %f oy1: %f\n",overtices[novertices-1][0]+ox, overtices[novertices-1][1]+oy, overtices[0][0]+ox, overtices[0][1]+oy) ;
            collision_flag++ ;
        }
      } /* end pass-bounding circle test segment */
    } /* end active agent test */
  } /* end agent loop */
  
  
  /* decay the active cells  
  for( j=0 ; j<nsvertices ; j++ )  
    for( k=0 ; k<nbands ; k++ )
    {
      values[j][k] *= decay_factor ;
      if(values[j][k]<0.05 )
        values[j][k] = 0.0 ;
    }
  */
  
  //printf("read_soma_sensor- Exit.\n") ;
  return(collision_flag) ;
}

float **extract_soma_receptor_values_pointer( AGENT_TYPE *a )
{ /* extracts the pointer for the receptor array for the skin.
  values is a 2D array the receptor values of size (nreceptors x nbands) */
  
  if( a->instate->skin==NULL )
  {
    printf("extract_soma_receptor_values_pointer- ERROR, no skin on this agent: %d\n",a->index) ;
    exit(0) ;
  }  
   
  return(a->instate->skin->values) ;
}

int get_number_of_soma_receptors( AGENT_TYPE *a )
{ /* tpc 8/13/12 */
  
  return( a->instate->skin->nreceptors ) ;
}

int get_number_of_soma_bands( AGENT_TYPE *a )
{ /* tpc 8/13/12 */
  
  return( a->instate->skin->nbands ) ;
}

void set_metabolic_burn_rate_agent(AGENT_TYPE *a, float m )
{
  a->instate->metabolic_burn_rate = m ;
}

float read_agent_metabolic_charge( AGENT_TYPE *a )
{
  return( a->instate->metabolic_charge ) ;
}

float read_agent_mass( AGENT_TYPE *a )
{
  return( a->instate->mass ) ;
}

void reset_agent_charge( AGENT_TYPE *a )
{
  printf("reset_agent_charge- Resetting agent: %d charge to 1.0\n",a->index) ;
  a->instate->metabolic_charge = 1.0 ;
}

void set_agent_body_position( AGENT_TYPE *a, float x, float y, float h )
{
  a->outstate->x = x ;
  a->outstate->y = y ;
  a->outstate->body_angle = h ;
}

void set_agent_body_angle( AGENT_TYPE *a, float h )
{
  a->outstate->body_angle = h ;
}
void read_agent_body_position( AGENT_TYPE *a, float *x, float *y, float *h )
{

  *x = a->outstate->x ;
  *y = a->outstate->y ;
  *h = a->outstate->body_angle ;
}

void read_agent_head_angle( AGENT_TYPE *a, float *th )
{
  *th =  a->outstate->head_angle  ;
}

void set_agent_head_angle( AGENT_TYPE *a, float th )
{
  a->outstate->head_angle = th ;
}



/******************************* Actuation of agent ***********************************************************/

float calc_soma_activation_direction( WORLD_TYPE *w, AGENT_TYPE *a )
{ /* computes the centroid of direction in body corrdinates of all active soma sensors band=0 only */
  SOMA_SENSOR_TYPE *s ;
  GEOMETRIC_SHAPE_TYPE *r ;
  int nreceptors ;
  int i ;
  float **values ;
  float **vertices ;
  float xmid, ymid,th ;
  float rad2deg = 57.295779 ;
  
  
  s = a->instate->skin ;
  nreceptors = s->nreceptors ;
  values = s->values ;
  r = s->receptor_locations ;
  vertices = r->vertices ;
  if( nreceptors!=r->nvertices )
  {
    fprintf(stdout,"calc_soma_activation_direction- ERROR. Agent: %d soma receptors do not match shape file.\n",a->index) ;
    exit(0) ;
  }
  for( i=0 ; i<nreceptors-1 ; i++ )
  {
    if( values[i][0]!=0.0 )
    {
      xmid = 0.5*(vertices[i][0] + vertices[i+1][0]) ;
      ymid = 0.5*(vertices[i][1] + vertices[i+1][1]) ;
      th = rad2deg * atan2f(-ymid,xmid) ;
      return( th ) ;
    }
  }
  if( values[nreceptors-1][0]!=0.0 )
  {
    xmid = 0.5*(vertices[nreceptors-1][0] + vertices[0][0]) ;
    ymid = 0.5*(vertices[nreceptors-1][1] + vertices[0][1]) ;
    th = rad2deg * atan2f(-ymid,xmid) ;
    return(th) ;
  }
  return(0) ;
}

float basal_metabolism_agent( AGENT_TYPE *a ) 
{  
  if( runflag==1 )
  {
    a->instate->metabolic_charge -= a->instate->metabolic_burn_rate ;
  }
  if( a->instate->metabolic_charge<0.0 )
  { 
    a->instate->metabolic_charge = 0.0 ;
    a->inworld_flag=0 ;
    //printf("basal_metabolism_agent- Agent: %d died.\n",a->index) ;
  }
  return(a->instate->metabolic_charge) ;
}

void set_forward_speed_agent( AGENT_TYPE *a, float dfb )
{
  float max_translation_delta ;
  
  max_translation_delta = a->instate->bodyactuators->max_translation_delta ;
  if( dfb > max_translation_delta ) dfb = max_translation_delta ;
  if( dfb < -max_translation_delta ) dfb = -max_translation_delta ;
  a->instate->bodyactuators->deltaFB = dfb ;
  
}

void set_movement_burn_rate_agent( AGENT_TYPE *a, float rate )
{
  a->instate->bodyactuators->movement_burn_rate = rate ;
  
}

void move_body_agent( AGENT_TYPE *a )
{ /* this function translates an agent as it rotates, dh in degrees */

  float dx,dy,th ;
  float deg2rad = 1.745329251994e-02 ;
  float cs,sn ;
  float dfb,drl ;
  
  th = a->outstate->body_angle * deg2rad ;
  
  cs = cos(th) ;
  sn = sin(th) ;
  dfb = a->instate->bodyactuators->deltaFB ;
  drl = a->instate->bodyactuators->deltaRL ;
  dy = -dfb * sn  -drl * cs ;
  dx = dfb * cs - drl * sn ;
  
  a->outstate->x += dx ;
  a->outstate->y += dy ;
  a->outstate->body_angle += a->instate->bodyactuators->deltaTH ;
  if(a->outstate->body_angle>360.0)
    a->outstate->body_angle -= 360.0 ;
  if( a->outstate->body_angle<0.0 )
    a->outstate->body_angle += 360.0 ;

/* add metabolic discharge for movement here */
  a->instate->metabolic_charge -= a->instate->bodyactuators->movement_burn_rate * ( fabs(dfb) + fabs(drl) ) ;
  if( a->instate->metabolic_charge<0.0 )
  { 
    a->instate->metabolic_charge = 0.0 ;
    a->inworld_flag=0 ;
  }

}

void move_head_agent1( AGENT_TYPE *a )
{
  a->outstate->head_angle += a->instate->headactuators->deltaH ;
  if( a->outstate->head_angle > 360.0 ) 
    a->outstate->head_angle -= 360.0 ;
  if( a->outstate->head_angle < 0.0 ) 
    a->outstate->head_angle += 360.0 ;
}

void rotate_agent_head1( AGENT_TYPE *a, float dh )
{
  printf("rotate_agent_head_angle- dh: %fbefore: %f ",dh, a->outstate->head_angle) ;
  a->outstate->head_angle += dh ;
  if( a->outstate->head_angle > 360.0 ) 
    a->outstate->head_angle -= 360.0 ;
  if( a->outstate->head_angle < 0.0 ) 
    a->outstate->head_angle += 360.0 ;
  printf("after: %f\n",a->outstate->head_angle) ;
}
void move_head_agent( AGENT_TYPE *a )
{
  if( a->outstate->head_angle<=180.0 && a->outstate->head_angle+a->instate->headactuators->deltaH>180.0)
    a->outstate->head_angle = a->outstate->head_angle + a->instate->headactuators->deltaH - 360.0 ;
  else if(a->outstate->head_angle>=-180.0 && a->outstate->head_angle+a->instate->headactuators->deltaH<-180.0)
    a->outstate->head_angle = a->outstate->head_angle + a->instate->headactuators->deltaH + 360.0 ;
  else
    a->outstate->head_angle += a->instate->headactuators->deltaH ;
}

void rotate_agent_head( AGENT_TYPE *a, float dh )
{
  //printf("rotate_agent_head_angle- dh: %fbefore: %f ",dh, a->outstate->head_angle) ;
  
  if( a->outstate->head_angle<=180.0 && a->outstate->head_angle+dh>180.0)
    a->outstate->head_angle = a->outstate->head_angle + dh - 360.0 ;
  else if(a->outstate->head_angle>=-180.0 && a->outstate->head_angle+dh<-180.0)
    a->outstate->head_angle = a->outstate->head_angle + dh + 360.0 ;
  else
    a->outstate->head_angle += dh ;

  //printf("after: %f\n",a->outstate->head_angle) ;
}

void scan_head_agent1( AGENT_TYPE *a, float thmax, float thmin, float period ) 
{ /* period in world dseconds, noting that the world runs at dseconds per second, one decasecond per sim cycles.
     Note that this function uses the sign of the period to control which direction the scan goes. 
     This function uses one of the temp int variables in the agent: itemp[9] 
     Note heading wrap around issues.  Heading angle positive CW from nose diretion.  No negative
     headings. */
     
  float dth ;
  
  if( period==0.0 )
    return ;
    
  dth = fabs( (thmin - thmax) / period ) ;
  
  if( a->outstate->head_angle==0.0 )
  {
    if( a->instate->itemp[9]==0 )
      a->outstate->head_angle += dth ;
    else
    {
      a->outstate->head_angle -= dth ;
      a->outstate->head_angle += 360.0 ;
    }
    return ;
  }  
  
  if( a->outstate->head_angle==360.0)
  {
    if( a->instate->itemp[9]==0 )
    {
      a->outstate->head_angle += dth  ;
      a->outstate->head_angle -= 360.0 ;
    }
    else
      a->outstate->head_angle -= dth ;
    return ;
  }  
  if( a->instate->itemp[9]==0 && a->outstate->head_angle<thmax && (a->outstate->head_angle+dth)>thmax)
  {
    a->instate->itemp[9]=1 ;
    return ;
  }
  if( a->instate->itemp[9]==1 && a->outstate->head_angle>thmin && (a->outstate->head_angle-dth)<thmin)
  {
    a->instate->itemp[9]=0 ;
    return ;
  }
  
  if( a->instate->itemp[9]==0 )
  {  
    a->outstate->head_angle += dth  ;
    if( a->outstate->head_angle>360.0)
      a->outstate->head_angle -= 360.0 ;
    return ;
  }

  if( a->instate->itemp[9]==1 )
  {  
    a->outstate->head_angle -= dth  ;
    if(  a->outstate->head_angle<0.0)
      a->outstate->head_angle += 360.0 ;
    return ;
  }

  if( a->instate->itemp[9]==0 && a->outstate->head_angle<thmax && (a->outstate->head_angle+dth)>thmax)
  {
    a->instate->itemp[9]=1 ;
    return ;
  }
  if( a->instate->itemp[9]==1 && a->outstate->head_angle>thmin && (a->outstate->head_angle-dth)<thmin)
  {
    a->instate->itemp[9]=0 ;
    return ;
  }
  
  
}

void scan_head_agent( AGENT_TYPE *a, float thmax, float thmin, float period ) 
{ /* period in world dseconds, noting that the world runs at dseconds per second, one decasecond per sim cycles.
     Note that this function uses the sign of the period to control which direction the scan goes. 
     This function uses one of the temp int variables in the agent: itemp[9] 
     Note heading wrap around issues.  Heading angle positive CW from nose diretion.  
     thmax > thmin, thmax < 180.0 and thmin > -180.0 */
     
  float dth ;
  
  if( period==0.0 )
    return ;
    
  dth = (thmax - thmin) / period ;
  
  if( a->instate->itemp[9]==0 && a->outstate->head_angle<thmax )
  {
   //printf("scan_head_agent- a.\n") ;
    a->outstate->head_angle += dth ;
    return ;
  }
  else if( a->instate->itemp[9]==0 && a->outstate->head_angle>=thmax )
  {
    //printf("scan_head_agent- b.\n") ;
    a->instate->itemp[9]=1 ;
    return ;
  }

  if( a->instate->itemp[9]==1 && a->outstate->head_angle>thmin )
  {
    //printf("scan_head_agent- c.\n") ;
    a->outstate->head_angle -= dth ;
    return ;
  }
  else if( a->instate->itemp[9]==1 && a->outstate->head_angle<thmax )
  {
    //printf("scan_head_agent- d.\n") ;
    a->instate->itemp[9]=0 ;
    return ;
  }
  //printf("scan_head_agent- e.\n") ;
}

void scan_head_agent_1( AGENT_TYPE *a, float thmax, float thmin, float period ) 
{ /* period in world dseconds, noting that the world runs at dseconds per second, one decasecond per sim cycles.
     Note that this function uses the sign of the period to control which direction the scan goes. 
     This function uses one of the temp int variables in the agent: itemp[9] 
     Note heading wrap around issues.  Heading angle positive CW from nose diretion.  No negative
     headings. */
     
  float dth ;
  
  if( period==0.0 )
    return ;
    
  dth = fabs( (thmin - thmax) / period ) ;
  
  if( a->outstate->head_angle==0.0 )
  {
    if( a->instate->itemp[9]==0 )
      a->outstate->head_angle += dth ;
    else
    {
      a->outstate->head_angle -= dth ;
      a->outstate->head_angle += 360.0 ;
    }
    return ;
  }  
  
  if( a->outstate->head_angle==360.0)
  {
    if( a->instate->itemp[9]==0 )
    {
      a->outstate->head_angle += dth  ;
      a->outstate->head_angle -= 360.0 ;
    }
    else
      a->outstate->head_angle -= dth ;
    return ;
  }  
  if( a->instate->itemp[9]==0 && a->outstate->head_angle<thmax && (a->outstate->head_angle+dth)>thmax)
  {
    a->instate->itemp[9]=1 ;
    return ;
  }
  if( a->instate->itemp[9]==1 && a->outstate->head_angle>thmin && (a->outstate->head_angle-dth)<thmin)
  {
    a->instate->itemp[9]=0 ;
    return ;
  }
  
  if( a->instate->itemp[9]==0 )
  {  
    a->outstate->head_angle += dth  ;
    if( a->outstate->head_angle>360.0)
      a->outstate->head_angle -= 360.0 ;
    return ;
  }

  if( a->instate->itemp[9]==1 )
  {  
    a->outstate->head_angle -= dth  ;
    if(  a->outstate->head_angle<0.0)
      a->outstate->head_angle += 360.0 ;
    return ;
  }

  if( a->instate->itemp[9]==0 && a->outstate->head_angle<thmax && (a->outstate->head_angle+dth)>thmax)
  {
    a->instate->itemp[9]=1 ;
    return ;
  }
  if( a->instate->itemp[9]==1 && a->outstate->head_angle>thmin && (a->outstate->head_angle-dth)<thmin)
  {
    a->instate->itemp[9]=0 ;
    return ;
  }
  
  
}

int intensity_winner_takes_all( AGENT_TYPE *a )
{ 
  VISUAL_SENSOR_TYPE **eyes = a->instate->eyes ;
  int nr,nb ;
  int i,j ;
  int maxintensityrec = -1 ;
  float intensity, maxintensity = -1.0 ;
  
  nr = eyes[0]->nreceptors ;
  nb = eyes[0]->nbands ;
  for( j=0 ; j<nr ; j++ ) 
  {
    intensity = 0 ;
    for( i=0 ; i<nb ; i++ )
      intensity += pow( eyes[0]->values[j][i], 2.0 ) ;
    intensity = sqrt( intensity ) ;
    
    if( intensity>0.0 && intensity>maxintensity ) 
    {
      maxintensityrec = j ;
      maxintensity = intensity ;
    }
  }
  if(eyes[0]->seen_objects[maxintensityrec]!=NULL )
  {
    //printf("intensity_winner_takes_all- simtime: %d maxintensityrec: %d  maxintensity: %f obj: %d\n",
         //simtime,maxintensityrec,maxintensity,eyes[0]->seen_objects[maxintensityrec]->index) ;
  }
  
  return( maxintensityrec ) ;

}

void process_visual_sensors_2( AGENT_TYPE *a, int *maxrec )
{ /* assumes one eye at this time even though it loops over eyes */
  VISUAL_SENSOR_TYPE **eyes = a->instate->eyes ;
  int ne = a->instate->neyes ;
  int nr,nb ;
  int i,j ;
  int maxintensityrec = -1 ;
  float maxintensity = 0.0 ;
  float threshold = 0.01 ;
  
  for( i=0 ; i<ne ; i++ ) 
  {
    nr = eyes[i]->nreceptors ;
    nb = eyes[i]->nbands ;
    maxintensity = 0.0 ;
    for( j=0 ; j<nr ; j++ ) 
    {
      if( eyes[i]->seen_objects[j]!=NULL )
      {
        //printf("process_visual_sensors_1- seen_object->type: %d\n", eyes[i]->seen_objects[j]->type) ;
        if( /*eyes[i]->seen_objects[j]->type==1 && */ eyes[i]->intensities[j] > maxintensity && eyes[i]->intensities[j] > threshold )
        {
          maxintensityrec = j ;
          maxintensity = eyes[i]->intensities[j] ;
        }
      }
    } /* end receptors */
  } /* end eyes */
  //printf("process_visual_sensors_1- maxintensityrec: %d maxintensity: %f\n",maxintensityrec,maxintensity) ;
  *maxrec = maxintensityrec ;
}

void set_max_translation_delta_agent( AGENT_TYPE *a, float max_translation_delta )
{
  a->instate->bodyactuators->max_translation_delta = max_translation_delta ;
}


void set_actuators_agent( AGENT_TYPE *a, float dfb, float drl, float dth, float dh )
{ /* sets body and head actuator values */

  float max_translation_delta ;

  max_translation_delta = a->instate->bodyactuators->max_translation_delta ;
  if( dfb > max_translation_delta ) dfb = max_translation_delta ;
  if( dfb < -max_translation_delta ) dfb = -max_translation_delta ;
  if( drl > max_translation_delta ) drl = max_translation_delta ;
  if( drl < -max_translation_delta ) drl = -max_translation_delta ;

  a->instate->bodyactuators->deltaFB = dfb ;
  a->instate->bodyactuators->deltaRL = drl ;
  a->instate->bodyactuators->deltaTH = dth ;  
  a->instate->headactuators->deltaH = dh ;
}
 
void read_actuators_agent( AGENT_TYPE *a, float *dfb, float *drl, float *dth, float *dh )
{ /* sets body and head actuator values */

  *dfb = a->instate->bodyactuators->deltaFB ;
  *drl = a->instate->bodyactuators->deltaRL ;
  *dth = a->instate->bodyactuators->deltaTH ;  
  *dh = a->instate->headactuators->deltaH ;
}
 

float agent_eat_object( WORLD_TYPE *w, AGENT_TYPE *a, OBJECT_TYPE *o )
{ /* charge is bounded between 0 0nd 1. If object is eaten, its inworld_flag is set to zero but type is not changed */

  //printf("agent_eat_object- Agent: %d eating object: %d. mc inital: %f ",a->index, o->index,a->instate->metabolic_charge) ;
  a->instate->metabolic_charge += o->food_value ;
  if( a->instate->metabolic_charge>1.0 )
    a->instate->metabolic_charge = 1.0 ;
  if( a->instate->metabolic_charge<0.0 )
    a->instate->metabolic_charge = 0.0 ;
  //printf("mc final: %f\n",a->instate->metabolic_charge) ;

  o->inworld_flag = 0 ;  /* remove eaten object from world */
  w->nactive_objects-- ; /* reduce active number of objects */
  
  return( o->food_value ) ;
}

float agent_eat_object_with_flag( WORLD_TYPE *w, AGENT_TYPE *a, OBJECT_TYPE *o, int flag)
{
  if( flag==1 )
    return( agent_eat_object( w, a, o ) ) ;
  return( 0.0 ) ;
}

float eat_colliding_object( WORLD_TYPE *w, AGENT_TYPE *a, int j )
{
  OBJECT_TYPE *o ;
  int i ;
  
  /* get index of colliding object that is in touch with this skin receptor */
  i = a->instate->skin->touched_objects[j] ;
  
  if(i > 0 )  /* the object in collision is an object not an agent, which would return a negative index from this func */
  {
    /* get pointer to object in collision. Object list in world data struct starts at index zero, but function adds one. */
    o = w->objects[i-1] ;
    
    if( o->inworld_flag!=0) /* test if object in collision is "in the world", i.e. not previously eaten */
    {
      agent_eat_object(  w, a, o ) ; /* eat the object */
      a->instate->itemp[0]++ ; /* increment the total number of objects eaten. this uses one of the user definable 
                                temp variable in the agent data struct. */
      printf("eat_colliding_object- Object: %3d of type: %1d eaten, food_value: %f. New charge: %5.4f total eaten: %1d simtime: %4d\n",
             o->index,o->type,o->food_value,a->instate->metabolic_charge,a->instate->itemp[0],simtime) ;
      return( o->food_value ) ; /* assumes only one object is in collision with skin at one time */
    }	
  }
  return(0.0) ;
}

int agent_pickup_object( WORLD_TYPE *w, AGENT_TYPE *a, OBJECT_TYPE *o )
{ /* trys to pick up an object and add it to manifest.  If already picked up, return -1, if hold full, return 0, succesful pick up, return 1 */

  CARGO_MANIFEST_TYPE *m ;

  m = a->instate->cargo ;

  //printf("agent_pickup_object- Entry. a->index: %d o->index: %d o->type: %d\n",a->index, o->index, o->type) ;
  
  if(  o->type<0 ) 
  {
    //printf("agent_pickup_object- No objected picked up.\n") ;
    return( -1 ) ;
  }
  //printf("agent_pickup_object- a->index: %d picking up o->index: %d\n", a->index, o->index) ;
  
  if( m->nitems == m->maxnitems ) 
  {
    //fprintf(stdout, "agent_pickup_object- WARNING, manifest full, agent: %d object: %d\n",a->index, o->index) ; /* Test if manifest is full */
    return( 0 ) ;
  }
  
  m->manifest[m->nitems] = (void *)o ; /* Add object pointer to list */
  m->nitems++ ;                         /* increment number of carried items */
  a->instate->mass += o->mass ;         /* increase the mass of the agent */
  w->nactive_objects-- ;                /* remove the object from active number of objects */
  o->type *= -1 ;                       /* negate lable to indicate the object carried, making it disappear from the world and appear in agent's cargo hold */
  //printf("agent_pickup_object- new o->type: %d\n",o->type) ;
  //printf("agent_pickup_object- Exit.\n") ;
  return( 1 ) ;
}

/*------------------------ Object related functions -------------------------------------------------------------------------------*/

OBJECT_TYPE *make_object( int index, int type, float x, float y, float mass, float food_value ) 
{ /* allocate an object and initial variables */

  OBJECT_TYPE *o ;
 
  if( index<=0 ) 
  {
    fprintf(stdout, "make_object- ERROR, object index must be > 0. No agent built, returning NULL.\n") ;
    return( NULL ) ;
  }
  
  o = (OBJECT_TYPE *)malloc(sizeof(OBJECT_TYPE)) ;
  testm(o, "make_object", "o") ;
  
  o->index = index ;
  o->inworld_flag = 1 ;
  o->type = type ;
  o->mass = mass ;
  o->food_value = food_value ;
  o->x = x ;
  o->y = y ;
  o->physical_shape = NULL ;
  o->sound_shape = NULL ;
  o->behavior_func = NULL ;
  o->bounding_radius = 0.0 ;

  return( o ) ;
}

void free_object( OBJECT_TYPE *o )
{
  free_geometric_shape_type( o->physical_shape ) ;
  free_acoustic_shape_type( o->sound_shape ) ;
  free( o ) ;
}

void add_physical_shape_to_object( OBJECT_TYPE *object, GEOMETRIC_SHAPE_TYPE *shape )
{ /* adds physical shape to object */
  int i ;
  float radius, maxradius = 0.0 ;
  
  object->physical_shape = shape ;
  
//  printf("add_physical_shape_to_object- Entry. Object#: %d\n",object->index); 
  for( i=0 ; i<shape->nvertices ; i++ ) 
  {
    radius = L2measure( 0.0, 0.0, shape->vertices[i][0], shape->vertices[i][1] ) ;
    if( radius > maxradius )
    {
      maxradius = radius ;
//      printf("add_physical_shape_to_object- vertex: %d radius: %f maxradius: %f\n",i,radius,maxradius) ;
    }
  }
  object->bounding_radius = maxradius ;
  
//  printf("add_physical_shape_to_object- Exit.\n") ; 

  return ;
}

void add_sound_shape_to_object( OBJECT_TYPE *object, ACOUSTIC_SHAPE_TYPE *shape )
{ /* adds sound shape to object */

  object->sound_shape = shape ;
  
  return ;
}

void add_behavior_to_object( OBJECT_TYPE *object, void (*behavior_func)(struct object_type *object)) 
{ /* adds a fucntion to the behavior of the object */

  object->behavior_func = behavior_func ;
  
  return ;
}


/*------------------------ World related functions -------------------------------------------------------------------------------*/

WORLD_TYPE *make_world( int index, int maxnagents, int maxnobjects, float xmax, float xmin, float ymax, float ymin, int rscaleflag )
{ /* allocates and parially initializes a world */

  WORLD_TYPE *w ;
  int i ;
  
  w = (WORLD_TYPE *)malloc(sizeof(WORLD_TYPE)) ;
  testm(w, "make_world", "w" ) ;
  
  w->index = index ;
  
  w->nagents = 0 ;
  w->maxnagents = maxnagents ;
  w->agents = (AGENT_TYPE **)malloc(maxnagents*sizeof(AGENT_TYPE *)) ;
  testm(w->agents, "make_world", "w->agents") ;
  for( i=0 ; i<maxnagents ; i++ )
    w->agents[i] = NULL ;
  
  w->nobjects = 0 ;
  w->nactive_objects = 0 ;
  w->maxnobjects = maxnobjects ;
  w->objects = (OBJECT_TYPE **)malloc(maxnobjects*sizeof(OBJECT_TYPE *)) ;  
  testm(w->objects, "make_world", "w->objects") ;
  for( i=0 ; i<maxnobjects ; i++ )
    w->objects[i] = NULL ;
  
  w->xmax = xmax ;
  w->xmin = xmin ;
  w->ymax = ymax ;
  w->ymin = ymin ;
  
  w->current_time = ( TIME_TYPE *)malloc(sizeof(TIME_TYPE)) ;
  testm(w->current_time, "make_world","w->current_time") ;
  w->current_time->second = 0 ;
  w->current_time->minute = 0 ;
  w->current_time->hour = 0 ;
  w->current_time->day = 0 ;
  w->current_time->year = 0 ;
 
  w->scratch_geo_shape_type = make_geometric_shape_type( 20, 5, 0.0, 1.0 ) ; /* <---------------------- HARD CODED LIMIT ------------------------<<<< */
  w->scratch_geo_shape_type2 = make_geometric_shape_type( 20, 5, 0.0, 1.0 ) ; /* <---------------------- HARD CODED LIMIT ------------------------<<<< */
  w->rscaleflag = rscaleflag ;
	
  return( w ) ;
}

void add_agent_to_world( WORLD_TYPE *world, AGENT_TYPE *agent ) 
{ /* add a agent to the population of a world */

  if( world->nagents < world->maxnagents )
  {
    world->agents[world->nagents++] = agent ;
  }
  else
  {
    fprintf(stdout, "add_agent_to_world- ERROR max agents exceeded, no new agent added.\n") ;
  }
  
  return ;
}
    
void add_object_to_world( WORLD_TYPE *world, OBJECT_TYPE *object ) 
{ /* add a object to the population of a world */

  if( world->nobjects < world->maxnobjects )
  {
    world->objects[world->nobjects++] = object ;
    object->inworld_flag = 1 ;
    world->nactive_objects++ ;
  }
  else
  {
    fprintf(stdout, "add_object_to_world- ERROR max objects exceeded, no new object added.\n") ;
  }
  
  return ;
}

void delete_object_from_world ( WORLD_TYPE *w, OBJECT_TYPE *o )
{ /* labeling an object type as -1 effectively removes it from the world, although no object is ACTUALLY deleted */

  o->type = -abs( o->type ) ;
  o->inworld_flag = 0;
  w->nactive_objects-- ;
}

void init_world_time( WORLD_TYPE *w )
{
  TIME_TYPE *t ;
  
  t = w->current_time ;
  t->dsecond = 0 ;
  t->second = 0 ;
  t->minute = 0 ;
  t->hour = 0 ;
  t->day = 0 ;
  t->week = 0 ;
  t->month = 0 ;
  t->year = 0 ;
}

void increment_world_clock( WORLD_TYPE *w )
{
  TIME_TYPE *t ;
  
  t = w->current_time ;
  t->dsecond++ ;
  if( t->dsecond>9 )
  {
    t->dsecond = 0 ;
    t->second++ ;
    if( t->second >59 ) 
    {
      t->second = 0 ;
      t->minute++ ;
      if( t->minute>59 )
      {
        t->minute = 0 ;
        t->hour++ ;
        if( t->hour > 23 ) 
        {
          t->hour = 0 ;
          t->day++ ;
          if( t->day > 6 )
          {
            t->day = 0 ;
            t->week++ ;
            if( t->week > 3 )
            {
              t->week = 0 ;
              t->month++ ;
              if( t->month > 11 ) 
              {
                t->month = 0 ;
                t->year++ ;
              }
            }
          }
        }
      }
    }
  }
}

unsigned long int seconds_from_start( WORLD_TYPE *w )
{
  unsigned long int sec ; /* note unsigned long int can contain 4,294,967,294 seconds, about 136 sim years. */
  TIME_TYPE *t ;
  
  t = w->current_time ;

  sec = t->second + 60*t->minute + 3600*t->hour + 86400*t->day + 604800*t->week + 2419200*t->month + 31449600*t->year ;
  
  return( sec ) ;
}

void print_world_time( WORLD_TYPE *w )
{
  TIME_TYPE *t ;
  unsigned long int sec ;
  
  sec = seconds_from_start( w ) ;
  t = w->current_time ;
  
  printf("print_world_time- Total: %ld | %4d:%2d:%2d:%2d:%2d:%2d:%2d.%1d\n",sec,t->year,t->month,t->week,t->day,t->hour,t->minute,t->second,t->dsecond) ;
}

void make_world_objects_specfile( WORLD_TYPE *w, char *filename, int nobjects )
{
  int i,j ;
  int ix=0, iy=0 ;
  int flag ;
  FILE *fp ;
  float *x, *y ;
  
  distributions_rantest() ;
	
	/* Create new world positions */
	
	for( i=0 ; i<1000 ; i++ )
		distributions_uniform( -1.0, 1.0 ) ;
  
  x = (float *)malloc( nobjects * sizeof( float) ) ;
  if( x==NULL ) 
  {
    printf("make_world_objects_specfile- ERROR mallocing x.\n") ;
    exit(0) ;
  }
  y= (float *)malloc( nobjects * sizeof( float) ) ;
  if( y==NULL ) 
  {
    printf("make_world_objects_specfile- ERROR mallocing y.\n") ;
    exit(0) ;
  }
  for( i=0 ; i<nobjects ; i++ )
  {
    x[i] = 0.0 ;
    y[i] = 0.0 ;
  }
  
  fp = fopen( filename, "w" ) ;
  if( fp==NULL ) 
  {
    printf("make_world_objects_specfile- ERROR opening file: %s\n",filename) ;
    exit(0) ;
  }
  printf("make_world_objects_specfile- Creating object list file: %s with nobjects: %d\n",filename, nobjects) ;
  fprintf(fp,"%d %d\n",nobjects, w->rscaleflag ) ;
  do
  {
	  ix = roundf(distributions_uniform( w->xmin, w->xmax ));
	  iy = roundf(distributions_uniform( w->ymin, w->ymax ));
  } while(ix < 10 && ix > -10 && iy < 10 && iy > -10 ); 
 
  x[0] = (float)ix ;
  y[0] = (float)iy ;

  fprintf(fp,"%f %f 1.0 -0.1 3 geoshapepoison.dat soundshapepoison.dat\n",x[0],y[0]) ;

  for( i=1 ; i<nobjects ; i++ )
  {
    flag = 1; /* assume there are duplicates to start */ 
    while( flag==1 )
    { 
      do 
      {
        ix = roundf(distributions_uniform( w->xmin, w->xmax ));
        iy = roundf(distributions_uniform( w->ymin, w->ymax ));
      } while(ix < 10 && ix > -10 && iy < 10 && iy > -10 );
      
      flag = 0 ; /* assume there are no duplicates and search to see if false. */
      for( j=0 ; j<i ; j++ )
      {
        if( ix==(int)x[j] && iy==(int)y[j]) 
        {
          flag = 1 ;
          //printf("  -i: %d j: %d duplicate object coordinates found, recomputing new random coordinates.\n",i,j) ;
        }
      }
    }
    x[i] = (float)ix ;
    y[i] = (float)iy ;
	 
    if( i%3==0 )
      fprintf(fp,"%f %f 1.0 -0.1 3 geoshapepoison.dat soundshapepoison.dat\n",x[i],y[i]) ;
    if( i%3==1 )
      fprintf(fp,"%f %f 1.0 0.0 2 geoshapeneutral.dat soundshapeneutral.dat\n",x[i],y[i]) ;
    if( i%3==2 )
      fprintf(fp,"%f %f 1.0 0.1 1 geoshapefood.dat soundshapefood.dat\n",x[i],y[i]) ;
  
    //  x y mass food_value gshapename sshapename
  }
  
	free( x ) ;
	free( y ) ;
	
  fclose( fp) ;
  printf("make_world_objects_specfile- Object list file: %s successfully created.\n",filename) ;
}

void read_object_spec_file( WORLD_TYPE *w, char *filename )
{ /* reads object properties, creates objects, and adds them to the world */
  FILE *fp ;
  int nobjects ;
  int i ;
  int type ;
	int rscaleflag ;
  float x,y,mass,foodvalue ;
  char gshapename[50], sshapename[50] ;
  OBJECT_TYPE *object ;
  GEOMETRIC_SHAPE_TYPE *gshape ;
  ACOUSTIC_SHAPE_TYPE *sshape ;
   
  fp = fopen( filename, "r" ) ;
  if( fp==NULL )
  {
    printf("read_object_spec_file- ERROR opening file: %s.\n", filename) ;
    exit(0) ;
  }
  
  fscanf(fp,"%d %d", &nobjects, &rscaleflag ) ;
  printf("read_object_spec_file- Opening object list file: %s with %d objects. rscaleflag: %d\n",filename, nobjects, rscaleflag) ;
  
  for( i=0 ; i<nobjects ; i++ ) 
  {
    fscanf(fp,"%f %f %f %f %d %s %s",&x,&y,&mass,&foodvalue,&type, gshapename,sshapename) ;
    object = make_object( i+1, type, x, y, mass, foodvalue ) ;
    gshape = read_geometric_shape_file( gshapename, rscaleflag ) ;
    add_physical_shape_to_object( object, gshape ) ;
    sshape = read_acoustic_shape_file( sshapename ) ;
    add_sound_shape_to_object( object, sshape ) ;
    //printf("  i: %d x: %f y: %f mass: %f foodvlaue: %f gshapename: %s sshapename; %s\n",i,x,y,mass,foodvalue, gshapename,sshapename) ;
    add_object_to_world( w, object ) ;    
  }

  printf("read_object_spec_file- Successfully added %d objects to the world.\n",w->nobjects) ;
   
  fclose( fp ) ;
  
}

void restore_objects_to_world( WORLD_TYPE *w) 
{ /* makes all world objects active, and therefore visable and eatable. */

  int i ;
  
  printf("restore_objects_to_world- before restore,  w->nactive_objects: %d w->nobjects: %d\n", w->nactive_objects, w->nobjects) ;
  for( i=0 ; i<w->nobjects ; i++ ) 
  {
    w->objects[i]->type = abs( w->objects[i]->type ) ;
    w->objects[i]->inworld_flag = 1 ;
  }
  w->nactive_objects = w->nobjects ;
  printf("restore_objects_to_world- restoration complete.\n") ;
}



