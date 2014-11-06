/*
 *  GraphicsCore.c
 *  
 *
 *  Created by Thomas Caudell on 3/18/09.
 *  Copyright University of New Mexico 2009. All rights reserved.
 *
 */
/* Graphics utility functions ----------------------------------------------*/

void draw_circle( float xc, float yc, float r, int nfacets ) 
{
  float x,y, th=0.0 ;
  float pi2 = 6.2831853 ;
  int j ;
  
  glBegin(GL_POLYGON) ;
    for( j=0 ; j<=nfacets ; j++ ) 
    {
      x = r * (float)cos( th ) ;
      y = r * (float)sin( th ) ;
      glVertex3f( x+xc, y+yc, 0.0 ) ;
      if( th > pi2) th -= pi2 ;
        th += pi2/(float)nfacets ;
    }
  glEnd() ;
}

void draw_cylinder( float xc, float yc, float h, float r ) 
{ /* draws a 10 sided cylinder with no bottom parallel to z-axis.  assumes material props are set outside function */

  float th=0.0 ;
  float pi2 = 6.2831853 ;
  float x[20], y[20] ;
  float p0[3], p1[3], p2[3], p3[3], n[3] ;
  int j ;
  int nv = 20 ; /* <----------------------- Hardcoded limit. ----------------------------<<< */
  
  /* Compute vertives */
  for( j=0 ; j<nv ; j++ ) 
  {      
    x[j] = r * (float)cos( th )+xc ;
    y[j] = r * (float)sin( th )+yc ;
    if( th > pi2) th -= pi2 ;
      th += pi2/(float)nv ;
  }
  
  /* Draw sides */
  for( j=0 ; j<nv-1 ; j++ ) 
  {  
    p0[0] = x[j] ;  p0[1] = y[j] ;  p0[2] = h ;
    p1[0] = x[j] ;  p1[1] = y[j] ;  p1[2] = 0.0 ;
    p2[0] = x[j+1] ;  p2[1] = y[j+1] ;  p2[2] = 0.0 ;
    p3[0] = x[j+1] ;  p3[1] = y[j+1] ;  p3[2] = h ;
    fl_calnormals( p0, p1, p2, n ) ;
    glNormal3fv( n ) ;
    glBegin(GL_POLYGON) ;
      glVertex3fv( p0 ) ;
      glVertex3fv( p1 ) ;
      glVertex3fv( p2 ) ;
      glVertex3fv( p3 ) ;
    glEnd() ;
  }
  p0[0] = x[nv-1] ;  p0[1] = y[nv-1] ;  p0[2] = h ;
  p1[0] = x[nv-1] ;  p1[1] = y[nv-1] ;  p1[2] = 0.0 ;
  p2[0] = x[0] ;  p2[1] = y[0] ;  p2[2] = 0.0 ;
  p3[0] = x[0] ;  p3[1] = y[0] ;  p3[2] = h ;
  fl_calnormals( p0, p1, p2, n ) ;
  glNormal3fv( n ) ;
  glBegin(GL_POLYGON) ;
    glVertex3fv( p0 ) ;
    glVertex3fv( p1 ) ;
    glVertex3fv( p2 ) ;
    glVertex3fv( p3 ) ;
  glEnd() ;
  
  /* Draw top */
  glNormal3f(0.0,0.0,1.0) ;
  glBegin(GL_POLYGON) ;
  for( j=0 ; j<nv ; j++ ) 
  {
    glVertex3f( x[j], y[j], h ) ;
  }
  glEnd() ;

}

void draw_rectangle( float xc, float yc, float th, float xhw, float yhw ) 
{ /* x half width and y half width centered on xc,yc, rotated by th */
  
  glTranslatef(xc,yc,0.0) ;
  glRotatef(th, 0.0,0.0,1.0) ;
  glNormal3f(0.0,0.0,1.0) ;
  glBegin(GL_POLYGON) ;
    glVertex3f(xhw, yhw,0.0) ;
    glVertex3f(-xhw, yhw,0.0) ;
    glVertex3f(-xhw, -yhw,0.0) ;
    glVertex3f(xhw, -yhw,0.0) ;
  glEnd() ;
  glRotatef(-th, 0.0,0.0,1.0) ;
  glTranslatef(-xc,-yc,0.0) ;
}

void linear_color_interpolation( float x, float *c0, float *c1, float *c)
{  /* linear interpolation of color vector between c0 (x=0) to c1 (x=1) output in c.  bounded if x>1 or x<0 */
  int i ;
  
  if( x>1.0 )
  {
    for( i=0 ; i<3 ; i++ ) 
      c[i] = c1[i] ;
    return ;
  }
  if( x<0.0 )
  {
    for( i=0 ; i<3 ; i++ ) 
      c[i] = c0[i] ;
    return ;
  }
  for( i=0 ; i<3 ; i++ ) 
    c[i] = c0[i]*(1.0-x) + c1[i]*x ;
}

void nonlinear_color_interpolation_1( float x, float *c0, float *c1, float *c)
{  /* x/1+x interpolation of color vector between c0 (x=0) to c1 output in c.  bounded if x<0 */
  int i ;
  float factor ;
  
  if( x<0.0 )
  {
    for( i=0 ; i<3 ; i++ ) 
      c[i] = c0[i] ;
    return ;
  }
  factor = x / ( 1.0 + x ) ;
  for( i=0 ; i<3 ; i++ ) 
    c[i] = c0[i]*(1.0-factor) + c1[i]*factor ;
}

void nonlinear_color_interpolation_2( float *x, float *c0, float *c1, float *c)
{  /* x/1+x interpolation of color vector between c0 (x=0) to c1 output in c component wise.  bounded if x<0 */
  int i ;
  float factor ;
  
  for( i=0 ; i<3 ; i++ ) 
  {
    if( x[i]<0.0 ) 
      x[i] = 0.0 ;
    factor = x[i] / ( 1.0 + x[i] ) ;
    c[i] = c0[i]*(1.0-factor) + c1[i]*factor ;
  }
}

void nonlinear_color_interpolation_3( float *x, float *c)
{  /* returns input if all components are less that one.  Otherwise scales all components to less that one. */
  int i ;
  float xmax=0.0 ;
  
  for( i=0 ; i<3 ; i++ ) 
  {
    if( x[i]>xmax )
      xmax = x[i] ;
  }
  if( xmax <=1.0 || xmax==0.0 )
  {
    for( i=0 ; i<3 ; i++ ) 
      c[i] = x[i] ;
    return ;
  }
  else
  {
    for( i=0 ; i<3 ; i++ ) 
      c[i] = x[i] / xmax ;
  }
}
  

void fl_calnormals( float *p0, float *p1, float *p2, float *n )
{
/* assume dim=3 */

	float norm ;
	float v1[3], v2[3];
	
	v1[0] = p1[0]-p0[0] ;
	v1[1] = p1[1]-p0[1] ;
	v1[2] = p1[2]-p0[2] ;
	v2[0] = p2[0]-p1[0] ;
	v2[1] = p2[1]-p1[1] ;
	v2[2] = p2[2]-p1[2] ;
	
	n[0] = v1[1]*v2[2] - v1[2]*v2[1] ;
	n[1] = -( v1[0]*v2[2] - v1[2]*v2[0] ) ;
	n[2] = v1[0]*v2[1] - v1[1]*v2[0] ;
	norm = (float)sqrt( n[0]*n[0] + n[1]*n[1] + n[2]*n[2] ) ;
	if( norm==0.0 ) 
  {
		printf("fl_calnormals- ERROR, norm=0.\n") ;
		exit(0) ;
	}
	n[0] /= norm ;
	n[1] /= norm ;
	n[2] /= norm ;
}

/* Drawing functions for Flatworld objects ----------------------------------------*/
/* Assumes there is a global Flatworld which is a pointer to a FlatworldII world structure. */

void draw_Flatworld()
{
  int i ;
  AGENT_TYPE *agent ;
  OBJECT_TYPE *object ;
  
  glPushMatrix() ;
  
  draw_world() ;
 
  for( i=0 ; i<Flatworld->nagents ; i++ )
  {
    agent = Flatworld->agents[i] ;
    draw_agent( agent ) ;
  }

  for( i=0 ; i<Flatworld->nobjects ; i++ ) 
  {
    object = Flatworld->objects[i] ;
    if( object->inworld_flag==1 && object->type>0 ) 
    {
      glTranslatef(object->x, object->y,0.0) ;
      draw_object( object ) ;
      glTranslatef(-object->x, -object->y,0.0) ;
    }
  }
  

  glPopMatrix() ;
}

void draw_world()
{
  int i ;
  int nx = 10 ;
  int ny = 10 ;
  int nx2, ny2 ;
  float xmax, xmin, ymax, ymin ;
  float x,y,dx, dy ;
  float delta = 0.1 ;
  float color[3] ;
  
  xmin = Flatworld->xmin ;
  xmax = Flatworld->xmax ;
  dx = xmax - xmin ;
  ymin = Flatworld->ymin ;
  ymax = Flatworld->ymax ;
  dy = ymax - ymin ;
  nx2 = nx / 2 ;
  ny2 = ny / 2 ;
  color[0] = 1.0 ;
  color[1] = 1.0 ;
  color[2] = 1.0 ;
  
  glLineWidth( 0.1 ) ;

  glMaterialfv(GL_FRONT,GL_AMBIENT,color) ;
  glMaterialfv(GL_FRONT,GL_DIFFUSE,color) ;
  glMaterialfv(GL_FRONT,GL_SPECULAR,color) ;
   
  for( i=0 ; i<nx+1 ; i++ ) 
  {
    x = (float)i*dx/(float)nx + xmin ;
    glBegin( GL_LINES ) ;
	  glVertex3f( x, ymax, -delta ) ;
	  glVertex3f( x, ymin, -delta ) ;
    glEnd() ;
  }
  for( i=0 ; i<ny+1 ; i++ ) 
  {
    y = (float)i*dy/(float)ny + ymin ;
    glBegin( GL_LINES ) ;
	  glVertex3f( xmax, y, -delta ) ;
	  glVertex3f( xmin, y, -delta ) ;
    glEnd() ;
  }
}

void draw_object( OBJECT_TYPE *o )
{

  float **v ;
  float p0[3], p1[3], p2[3], p3[3], n[3] ;
  float z ;
  int j, nv ;

  v = o->physical_shape->vertices ;
  z = o->physical_shape->height ;
  nv = o->physical_shape->nvertices ;
 
  for( j=0 ; j<nv-1 ; j++ ) 
  {      
    glMaterialfv(GL_FRONT,GL_AMBIENT,&(v[j][2])) ;
    glMaterialfv(GL_FRONT,GL_DIFFUSE,&(v[j][2])) ;
    glMaterialfv(GL_FRONT,GL_SPECULAR,&(v[j][2])) ;
    p0[0] = v[j][0] ;  p0[1] = v[j][1] ;  p0[2] = z ;
    p1[0] = v[j][0] ;  p1[1] = v[j][1] ;  p1[2] = 0.0 ;
    p2[0] = v[j+1][0] ;  p2[1] = v[j+1][1] ;  p2[2] = 0.0 ;
    p3[0] = v[j+1][0] ;  p3[1] = v[j+1][1] ;  p3[2] = z ;
    fl_calnormals( p0, p1, p2, n ) ;
    glNormal3fv( n ) ;
    glBegin(GL_POLYGON) ;
      glVertex3fv( p0 ) ;
      glVertex3fv( p1 ) ;
      glVertex3fv( p2 ) ;
      glVertex3fv( p3 ) ;
    glEnd() ;
  }
  glMaterialfv(GL_FRONT,GL_AMBIENT,&(v[nv-1][2])) ;
  glMaterialfv(GL_FRONT,GL_DIFFUSE,&(v[nv-1][2])) ;
  glMaterialfv(GL_FRONT,GL_SPECULAR,&(v[nv-1][2])) ; 
  p0[0] = v[nv-1][0] ;  p0[1] = v[nv-1][1] ;  p0[2] = z ;
  p1[0] = v[nv-1][0] ;  p1[1] = v[nv-1][1] ;  p1[2] = 0.0 ;
  p2[0] = v[0][0] ;  p2[1] = v[0][1] ;  p2[2] = 0.0 ;
  p3[0] = v[0][0] ;  p3[1] = v[0][1] ;  p3[2] = z ;
  fl_calnormals( p0, p1, p2, n ) ;
  glNormal3fv( n ) ;
  glBegin(GL_POLYGON) ;
    glVertex3fv( p0 ) ;
    glVertex3fv( p1 ) ;
    glVertex3fv( p2 ) ;
    glVertex3fv( p3 ) ;
  glEnd() ;
  
  /* Draw top */
  glTranslatef(0.0, 0.0, z) ;
  glNormal3f( 0.0, 0.0, 1.0 ) ;
  glBegin(GL_POLYGON) ;
  for( j=0 ; j<nv ; j++ ) {
    glMaterialfv(GL_FRONT,GL_AMBIENT,&(v[j][2])) ;
    glMaterialfv(GL_FRONT,GL_DIFFUSE,&(v[j][2])) ;
    glMaterialfv(GL_FRONT,GL_SPECULAR,&(v[j][2])) ;
  	glVertex3f( v[j][0], v[j][1], 0.0 ) ;
  }
  glEnd() ;
  glTranslatef(0.0,0.0,-z) ;

}

void draw_agent( AGENT_TYPE *a )
{
  float **v, xoffset, yoffset, body_angle, head_angle, head_radius ;
  int j, nv ;
  float color1[3] = {0.6, 0.6, 0.6 } ;
  float color2[3] = {0.0, 0.0, 0.0 } ;
  float color3[3] = {0.15, 0.15, 0.15} ;
  float dzbody, dzhead = 0.1, dzbump = 0.01 ;

  v = a->outstate->physical_shape->vertices ;
  dzbody = a->outstate->physical_shape->height ;
  nv = a->outstate->physical_shape->nvertices ;
  xoffset = a->outstate->x ;
  yoffset = a->outstate->y ;
  body_angle = a->outstate->body_angle ;
  head_angle = a->outstate->head_angle ;
  head_radius = a->outstate->head_radius ;
    
  /* Draw base body sides, assuming soma sensor cells are the same and body geo shape */
  glTranslatef(xoffset, yoffset,0.0) ;
  glRotatef( -body_angle, 0.0, 0.0, 1.0 ) ;
  draw_agent_skin( a, dzbody ) ;

  /* Draw base body top */
  glNormal3f(0.0,0.0,1.0) ;
  glBegin(GL_POLYGON) ;
    for( j=0 ; j<nv ; j++ ) 
    {
      if( a->instate->metabolic_charge>0.0 ) 
      {
        glMaterialfv(GL_FRONT,GL_AMBIENT,&(v[j][2])) ;
        glMaterialfv(GL_FRONT,GL_DIFFUSE,&(v[j][2])) ;
        glMaterialfv(GL_FRONT,GL_SPECULAR,&(v[j][2])) ;
      }
      else /* dead */
      {
        glMaterialfv(GL_FRONT,GL_AMBIENT,color3) ;
        glMaterialfv(GL_FRONT,GL_DIFFUSE,color3) ;
        glMaterialfv(GL_FRONT,GL_SPECULAR,color3) ;
      }
	    glVertex3f( v[j][0], v[j][1], dzbody ) ;
    }
  glEnd() ;
  
  glTranslatef(0.0, 0.0, dzbody) ;

  /* Draw cargo manifest */
  draw_agent_cargo_manifest( a ) ;
  
  /* draw thermometer of metabolic charge */
  draw_agent_metabolic_charge( a ) ;
 
  /* draw mouth */
  draw_agent_mouth( a ) ;

  /* Draw head */
  glRotatef( -head_angle, 0.0, 0.0, 1.0 ) ;
  glMaterialfv(GL_FRONT,GL_AMBIENT,color1) ;
  glMaterialfv(GL_FRONT,GL_DIFFUSE,color1) ;
  glMaterialfv(GL_FRONT,GL_SPECULAR,color1) ;
  draw_cylinder( 0.0, 0.0, dzhead, head_radius) ;

  glTranslatef(0.0, 0.0, dzhead) ;

  /* Draw arrow (nose) on head */
  glMaterialfv(GL_FRONT,GL_AMBIENT,color2) ;
  glMaterialfv(GL_FRONT,GL_DIFFUSE,color2) ;
  glMaterialfv(GL_FRONT,GL_SPECULAR,color2) ;
  glBegin(GL_POLYGON) ;
    glVertex3f(0.0,head_radius*0.5,dzbump) ;
    glVertex3f(0.0,-head_radius*0.5,dzbump) ;
    glVertex3f(head_radius*0.7,0.0,dzbump) ;
  glEnd() ;	
  
  /* Draw ears */
  draw_agent_ears( a ) ;
  
  /* return back to world coordinates */
  glRotatef( head_angle, 0.0, 0.0, 1.0 ) ;
  glRotatef( body_angle, 0.0, 0.0, 1.0 ) ;
  glTranslatef(-xoffset, -yoffset,-(dzhead+dzbody)) ;

 /* Draw eyes */
  glTranslatef(0.0, 0.0, (dzhead+dzbody) ) ;
  draw_agent_visual_sensor_rays_1( a ) ;
  glTranslatef(0.0, 0.0, -(dzhead+dzbody) ) ;
  
}

void draw_agent_metabolic_charge( AGENT_TYPE *a )
{
  float p0[3], p1[3], p2[3], p3[3], n[3] ;
  float dzbump = 0.001, b = 0.08, w = 0.1, s = 0.9 ;
  float xmin,xmax,ymin,ymax,y ;
  float color1[3] = {1.0, 0.0, 0.0} ;
  float color2[3] = {0.2, 0.2, 0.2} ;
  
 /* Draw the cargo hold platform */
  xmin = a->outstate->xminbody+s ;
  xmax = a->outstate->xminbody+w+s ;
  ymax = a->outstate->ymaxbody-b ;
  ymin = a->outstate->yminbody+b ;
  y = a->instate->metabolic_charge * (ymax - ymin) + ymin ;
  
  /* draw cargp hold.  This assumes that the body geometric shape is created facing +x direction and 
      that there is space behind the head for the cargo hold */
  if( y!=ymin ) 
  {
    p0[0] = xmax ;  p0[1] = y ;  p0[2] = dzbump ;
    p1[0] = xmin ;  p1[1] = y ;  p1[2] = dzbump ;
    p2[0] = xmin ;  p2[1] = ymin ;  p2[2] = dzbump ;
    p3[0] = xmax ;  p3[1] = ymin ;  p3[2] = dzbump ;
    fl_calnormals( p0, p1, p2, n ) ;
    glNormal3fv( n ) ;
    glMaterialfv(GL_FRONT,GL_AMBIENT,color1) ;
    glMaterialfv(GL_FRONT,GL_DIFFUSE,color1) ;
    glMaterialfv(GL_FRONT,GL_SPECULAR,color1) ;
    glBegin(GL_POLYGON) ;
      glVertex3fv( p0 ) ;
      glVertex3fv( p1 ) ;
      glVertex3fv( p2 ) ;
      glVertex3fv( p3 ) ;
    glEnd() ;    
  }

  if( y!=ymax )
  {
    p0[0] = xmax ;  p0[1] = ymax ;  p0[2] = dzbump ;
    p1[0] = xmin ;  p1[1] = ymax ;  p1[2] = dzbump ;
    p2[0] = xmin ;  p2[1] = y ;  p2[2] = dzbump ;
    p3[0] = xmax ;  p3[1] = y ;  p3[2] = dzbump ;
    fl_calnormals( p0, p1, p2, n ) ;
    glNormal3fv( n ) ;
    glMaterialfv(GL_FRONT,GL_AMBIENT,color2) ;
    glMaterialfv(GL_FRONT,GL_DIFFUSE,color2) ;
    glMaterialfv(GL_FRONT,GL_SPECULAR,color2) ;
    glBegin(GL_POLYGON) ;
      glVertex3fv( p0 ) ;
      glVertex3fv( p1 ) ;
      glVertex3fv( p2 ) ;
      glVertex3fv( p3 ) ;
    glEnd() ;
  }
}

void draw_agent_cargo_manifest( AGENT_TYPE *a )
{
  CARGO_MANIFEST_TYPE *m ;
  OBJECT_TYPE *obj ;  
  float p0[3], p1[3], p2[3], p3[3], n[3] ;
  float dzbump = 0.001, b = 0.08, w = 0.8 ;
  float xmin,xmax,ymin,ymax ;
  float dy,dx,ds,y,h,s ;
  float color[3] = {0.2, 0.2, 0.2} ;
  int i,nitems, maxnitems ;

  //printf("draw_cargo_manifest- entering.\n") ;
  
  /* Test is there is a manifest */
  if( a->instate->cargo==NULL ) 
    return ;
  m = a->instate->cargo ;

 /* Draw the cargo hold platform */
  xmin = a->outstate->xminbody+b ;
  xmax = a->outstate->xminbody+w ;
  ymax = a->outstate->ymaxbody-b ;
  ymin = a->outstate->yminbody+b ;
  
  /* draw cargp hold.  This assumes that the body geometric shape is created facing +x direction and 
      that there is space behind the head for the cargo hold */
  p0[0] = xmax ;  p0[1] = ymax ;  p0[2] = dzbump ;
  p1[0] = xmin ;  p1[1] = ymax ;  p1[2] = dzbump ;
  p2[0] = xmin ;  p2[1] = ymin ;  p2[2] = dzbump ;
  p3[0] = xmax ;  p3[1] = ymin ;  p3[2] = dzbump ;
  fl_calnormals( p0, p1, p2, n ) ;
  glNormal3fv( n ) ;
  glMaterialfv(GL_FRONT,GL_AMBIENT,color) ;
  glMaterialfv(GL_FRONT,GL_DIFFUSE,color) ;
  glMaterialfv(GL_FRONT,GL_SPECULAR,color) ;
  glBegin(GL_POLYGON) ;
    glVertex3fv( p0 ) ;
    glVertex3fv( p1 ) ;
    glVertex3fv( p2 ) ;
    glVertex3fv( p3 ) ;
  glEnd() ;
  
  /* draw the objects in the hold */
  nitems = m->nitems ;
  maxnitems = m->maxnitems ;
  if(nitems>0 ) 
  {
    dy = (ymax - ymin)/(float)maxnitems ;
    dx = w - b ;
    ds = min(dx, dy) ;
    //printf("draw_cargo_manifest- drawing objects. nitems: %d nmaxintems: %d\n",nitems,maxnitems) ;
    glTranslatef(xmin+0.5*w-b,0.0,0.0) ;
    y = ymax - 0.5*dy ;
    for( i=0 ; i<nitems ; i++ ) 
    {
      obj = (OBJECT_TYPE*)m->manifest[i] ;
      s = ds / ( 2.0*obj->bounding_radius) ;
      h = obj->physical_shape->height * s ;
      //printf("  obj: %d\n",obj->index) ;
      glTranslatef(0.0,y,h) ;
      glScalef(s,s,s) ;
        draw_object(obj) ;
      
      glScalef(1.0/s,1.0/s,1.0/s) ;
      glTranslatef(0.0,-y,-h) ;
      y -= dy ;
    }
    glTranslatef(-(xmin+0.5*w-b),0.0,0.0) ;

  }
  //printf("draw_cargo_manifest- exiting.\n") ;
}

void draw_agent_visual_sensor_rays( AGENT_TYPE *a )
{
  VISUAL_SENSOR_TYPE **eyes, *eye ;
  int neyes, nrec, nbands ;
  int ne,j;
  int nobjects ;
  float *values;
  float *loc, *dir ;
  float rt,rr ;
  float mx, my ;
  float sx, sy, st ;
  float tx, ty ;
  float dth ;
  float pi = 3.1415926 ;
  float deg2rad = 1.7453292e-02 ;
  float c[3] ;

  //printf("\n\ndraw_Agent_visual_sensor_rays_2- Entry.\n") ;
  nobjects = Flatworld->nobjects ;
  eyes = a->instate->eyes ;  
  if( eyes==NULL )
    return ;
    
  neyes = a->instate->neyes ;
  mx = a->outstate->x ;
  my = a->outstate->y ;
  rt = a->outstate->body_angle + a->outstate->head_angle ;
  rr = a->outstate->head_radius ;
  
  glLineWidth( 1 ) ;
 
  for( ne=0 ; ne<neyes ; ne++ )
  {
    eye = eyes[ne] ;
	  nrec = eye->nreceptors ;
	  nbands = eye->nbands ;
	  loc = eye->receptor_locations ;
	  dir = eye->receptor_directions ;  /* deviation from normal radial */

    dth = 0.5*deg2rad*rr*(loc[nrec-1]-loc[0]) / (float)nrec;
    for( j=0 ; j<nrec ; j++ ) 
	  {	
      /* calculating coordinates of sensor in world coodinates then loop over objects */
      st= loc[j] + rt ;  /* calculate the angle of the sensor j taking into account agents's head and body angle */
      sx = mx + cos(pi * st / 180.0) * rr ;
      sy = my - sin(pi * st / 180.0) * rr ;
	  
	  /* Compute a new start point for ray definition to correct for the receptor direction. */
      tx = sx + cos( (st+dir[j])*deg2rad ) ;
      ty = sy - sin( (st+dir[j])*deg2rad ) ;

      values = (float *)eye->values[j] ;
      nonlinear_color_interpolation_3( values, c) ;
      glMaterialfv(GL_FRONT,GL_AMBIENT,c) ;
      glMaterialfv(GL_FRONT,GL_DIFFUSE,c) ;
      glMaterialfv(GL_FRONT,GL_SPECULAR,c) ;
      draw_rectangle( sx, sy, -(st+dir[j]), 0.25*rr, dth ) ;
    }
  } /* End Eyes loop */
  //printf("draw_Agent_visual_sensor_rays_2- Exit.\n") ;
}

void draw_agent_visual_sensor_rays_1( AGENT_TYPE *a )
{
  VISUAL_SENSOR_TYPE **eyes, *eye ;
  OBJECT_TYPE *obj ;
  int neyes, nrec, nbands ;
  int ne, i,j,k ;
  int nobjects ;
  int iobject, iintersect_segment ;
  int intersect_segment ;
  int drawraysflag = 1 ;
  float values[10], **ovalues ;
  float *loc, *dir ;
  float xi,yi ;
  float ximin, yimin ;
  float rt,rr ;
  float mx, my ;
  float sx, sy, st ;
  float tx, ty ;
  float dmin, dmax = 50.0 ; /* <---------------------- HARD CODED LIMIT ------------------------<<<< */
  float ox, oy, dro, dd, beta, angle, angle_effect ;
  float dth ;
  float visualsensorthreshold = 0.0 ;  // Visual sensor threshold, will not show detection if light level lower
  float pi = 3.1415926 ;
  float deg2rad = 1.7453292e-02 ;
  float color[3] = {0.0, 0.0, 0.0 } ;
  float color1[3] = {1.0, 1.0, 1.0 } ;

  //printf("\n\ndraw_Agent_visual_sensor_rays_2- Entry.\n") ;
  nobjects = Flatworld->nobjects ;
  eyes = a->instate->eyes ;  
  if( eyes==NULL )
    return ;
    
  neyes = a->instate->neyes ;
  mx = a->outstate->x ;
  my = a->outstate->y ;
  rt = a->outstate->body_angle + a->outstate->head_angle ;
  rr = a->outstate->head_radius ;
  
  glLineWidth( 1 ) ;
 
  for( ne=0 ; ne<neyes ; ne++ )
  {
    eye = eyes[ne] ;
	  nrec = eye->nreceptors ;
	  nbands = eye->nbands ;
	  loc = eye->receptor_locations ;
	  dir = eye->receptor_directions ;  /* deviation from normal radial */

    dth = 0.5*deg2rad*rr*(loc[nrec-1]-loc[0]) / (float)nrec;
 //   printf("draw_Agent_visual_sensor_rays_2- eye: %d dth: %f\n",ne,dth) ;
    for( j=0 ; j<nrec ; j++ ) 
	  {
	    /* Initialize sensor values to zero */
	    ximin = 1.0e6 ;
	    yimin = 1.0e6 ;
	
      /* calculating coordinates of sensor in world coodinates then loop over objects */
      st= loc[j] + rt ;  /* calculate the angle of the sensor j taking into account agents's head and body angle */
      sx = mx + cos(pi * st / 180.0) * rr ;
      sy = my - sin(pi * st / 180.0) * rr ;
	  
	  /* Compue a new start point for ray definition to correct for the receptor direction. */
      tx = sx + cos( (st+dir[j])*deg2rad ) ;
      ty = sy - sin( (st+dir[j])*deg2rad ) ;
//	  printf("draw_Agent_visual_sensor_rays_2- st: %f sx: %f sy: %f tx: %f ty: %f\n",st,sx,sy,tx,ty) ;

	    dmin = 1.0e6 ;
	    iobject = -1  ;
	    iintersect_segment = -1 ;
	    for( i=0 ; i<nobjects ; i++ )
	    {
	      obj = Flatworld->objects[i] ;
        if( obj->inworld_flag==1 && obj->type>0 ) 
        {
          //printf("draw_Agent_visual_sensor_rays_2- Scanning obj: %d\n",obj->index) ;
          ox = obj->x ;
          oy = obj->y ;
          ovalues = obj->physical_shape->vertices ;
          dro = (float)sqrt( (ox-sx)*(ox-sx) + (oy-sy)*(oy-sy) ) ;
          if( dro < dmax) 
          {
           dd = 0.0 ;
            beta = 0.0 ;
            intersect_segment = intersect_beam_with_object(obj, sx, sy, tx, ty, &xi, &yi, &dd, &beta ) ;

            if( dd<0.0 ) {
              printf("draw_Agent_visual_sensor_rays_2- dd: neg obj: %d objtype: %d\n",i,obj->type) ;
              exit(0) ;
            } 
            if( intersect_segment >= 0 )        // TRUE >= 0
            {
              if( dd<dmin ) 
              {
                //printf("draw_Agent_visual_sensor_rays_2- found closer segment xi: %f yi: %f dd: %f intersect_segment: %d\n",xi,yi,dd,intersect_segment) ;
                angle =  beta * pi / 180.0 ;
                  angle_effect = 1.0 ;
                dmin = dd ;
                ximin = xi ;
                yimin = yi ;
                iobject = i ;
                iintersect_segment = intersect_segment ;
          
                if( angle_effect >= visualsensorthreshold )
                {
                  //printf("draw_Agent_visual_sensor_rays_2- rec: %d obj: %d seg: %d\n",j,i,intersect_segment) ;
                  for( k=0 ; k<nbands ; k++ )
                  {			
                    values[k] =  ovalues[intersect_segment][k+2] * angle_effect ; /* the first two locations in ovalues[][] are the x,y vertex locations. */
                    //printf("values[%d]: %f | ",k,values[k]) ;
                  }
                  //printf("\n") ;
                }
              }
            }
          }
        }
      }
      if( dmin == 1.0e6 ) 
      {
        tx = sx + cos(deg2rad * (st+dir[j]) ) * 50.0 ;
        ty = sy - sin(deg2rad * (st+dir[j]) ) * 50.0 ; 
        if( drawraysflag==1 ) 
        {
          glMaterialfv(GL_FRONT,GL_AMBIENT,color1) ;
          glMaterialfv(GL_FRONT,GL_DIFFUSE,color1) ;
          glMaterialfv(GL_FRONT,GL_SPECULAR,color1) ;
          glBegin( GL_LINES ) ;
            glVertex3f( sx, sy, 0.0 ) ;
            glVertex3f( tx, ty, 0.0 ) ;
          glEnd() ;
        }
        else
        {
          glMaterialfv(GL_FRONT,GL_AMBIENT,color) ;
          glMaterialfv(GL_FRONT,GL_DIFFUSE,color) ;
          glMaterialfv(GL_FRONT,GL_SPECULAR,color) ;
          draw_rectangle( sx, sy, -(st+dir[j]), 0.25*rr, dth ) ; 
        }
      }
      else
      {
        glMaterialfv(GL_FRONT,GL_AMBIENT,values) ;
        glMaterialfv(GL_FRONT,GL_DIFFUSE,values) ;
        glMaterialfv(GL_FRONT,GL_SPECULAR,values) ;
        if( drawraysflag==1 ) 
        {
          glBegin( GL_LINES ) ;
            glVertex3f( sx, sy, 0.0 ) ;
            glVertex3f( ximin, yimin, 0.0 ) ;
          glEnd() ;
        }
        else
        {
          draw_rectangle( sx, sy, -(st+dir[j]), 0.25*rr, dth ) ; 
        }
      }
    }
  } /* End Eyes loop */
  //printf("draw_Agent_visual_sensor_rays_2- Exit.\n") ;
}

void draw_agent_ears( AGENT_TYPE *a ) 
{
   
  ACOUSTIC_SENSOR_TYPE **ears ;
  int nears, ne ;
  float rr ;
  float intensity ;
  float earheight = 0.15, earwidth = 0.15 ;
  float dz = 0.01 ;
  float c[3] ;
  float c0[3] = { 0.0, 0.0, 0.0 } ;
  float c1[3] = { 1.0, 0.0, 0.0 } ;
  
  ears = a->instate->ears ;
  if( ears==NULL )
    return ;
  nears = a->instate->nears ;
  rr = a->outstate->head_radius ;
 
  glNormal3f( 0.0, 0.0, 1.0 ) ;
 
  for( ne=0 ; ne<nears ; ne++ ) 
  {
    intensity = ears[ne]->intensity / 2.0 ;  /* HARD CODED gain factor *****************************************/
    glRotatef( -ears[ne]->receptor_location, 0.0, 0.0, 1.0 ) ;
    linear_color_interpolation( intensity, c0, c1, c) ;
    glMaterialfv(GL_FRONT,GL_AMBIENT,c) ;
    glMaterialfv(GL_FRONT,GL_DIFFUSE,c) ;
    glMaterialfv(GL_FRONT,GL_SPECULAR,c) ;
    glBegin(GL_POLYGON) ;
      glVertex3f(rr,0.0,dz) ;
      glVertex3f(rr+earheight,earwidth,dz) ;
      glVertex3f(rr+earheight,-earwidth,dz) ;
    glEnd() ;	
    glRotatef( ears[ne]->receptor_location, 0.0, 0.0, 1.0 ) ;
  }
}

void draw_agent_skin( AGENT_TYPE *a, float dzbody ) 
{
  SOMA_SENSOR_TYPE *skin ;
  int j,nv ;
  float **v ;
  float **values ;
  float c[3] ;
  float c0[3] = { 0.6, 0.6, 0.6 } ;
  float c1[3] = { 1.0, 0.0, 0.0 } ;
  float p0[3], p1[3], p2[3], p3[3], n[3] ;
  
  skin = a->instate->skin ;
  if( skin==NULL )
    return ;
  v = skin->receptor_locations->vertices ;
  nv = skin->receptor_locations->nvertices ;
  values = skin->values ;
 
  for( j=0 ; j<nv-1 ; j++ ) 
  {      
    if( values[j][0] == 0.0 ) 
    {
      glMaterialfv(GL_FRONT,GL_AMBIENT,c0) ;
      glMaterialfv(GL_FRONT,GL_DIFFUSE,c0) ;
      glMaterialfv(GL_FRONT,GL_SPECULAR,c0) ;
    }
    else
    {
      linear_color_interpolation( values[j][0], c0, c1, c) ;
      glMaterialfv(GL_FRONT,GL_AMBIENT,c) ;
      glMaterialfv(GL_FRONT,GL_DIFFUSE,c) ;
      glMaterialfv(GL_FRONT,GL_SPECULAR,c) ;
    }    
    p0[0] = v[j][0] ;  p0[1] = v[j][1] ;  p0[2] = dzbody ;
    p1[0] = v[j][0] ;  p1[1] = v[j][1] ;  p1[2] = 0.0 ;
    p2[0] = v[j+1][0] ;  p2[1] = v[j+1][1] ;  p2[2] = 0.0 ;
    p3[0] = v[j+1][0] ;  p3[1] = v[j+1][1] ;  p3[2] = dzbody ;
    fl_calnormals( p0, p1, p2, n ) ;
    glNormal3fv( n ) ;
    glBegin(GL_POLYGON) ;
      glVertex3fv( p0 ) ;
      glVertex3fv( p1 ) ;
      glVertex3fv( p2 ) ;
      glVertex3fv( p3 ) ;
    glEnd() ;
  }
  if( values[nv-1][0] == 0.0 ) 
  {
    glMaterialfv(GL_FRONT,GL_AMBIENT,c0) ;
    glMaterialfv(GL_FRONT,GL_DIFFUSE,c0) ;
    glMaterialfv(GL_FRONT,GL_SPECULAR,c0) ;
  }
  else
  {
    linear_color_interpolation( values[j][0], c0, c1, c) ;
    glMaterialfv(GL_FRONT,GL_AMBIENT,c) ;
    glMaterialfv(GL_FRONT,GL_DIFFUSE,c) ;
    glMaterialfv(GL_FRONT,GL_SPECULAR,c) ;
  }    
  p0[0] = v[nv-1][0] ;  p0[1] = v[nv-1][1] ;  p0[2] = dzbody ;
  p1[0] = v[nv-1][0] ;  p1[1] = v[nv-1][1] ;  p1[2] = 0.0 ;
  p2[0] = v[0][0] ;  p2[1] = v[0][1] ;  p2[2] = 0.0 ;
  p3[0] = v[0][0] ;  p3[1] = v[0][1] ;  p3[2] = dzbody ;
  fl_calnormals( p0, p1, p2, n ) ;
  glNormal3fv( n ) ;
  glBegin(GL_POLYGON) ;
    glVertex3fv( p0 ) ;
    glVertex3fv( p1 ) ;
    glVertex3fv( p2 ) ;
    glVertex3fv( p3 ) ;
  glEnd() ;
}

void draw_agent_mouth( AGENT_TYPE *a ) 
{
  int j,k ;
  int nfrequencies, nbands ;
  int lj, hj ;
  float intensity[3] = {0.0, 0.0, 0.0 } ;
  float c[3] ;
  //float c0[3] = { 0.0, 0.0, 0.0 } ;
 // float c1[3] = { 1.0, 0.0, 0.0 } ;
  float **spectrum ;
  
  spectrum = a->outstate->sound_shape->spectrum ; 
  nfrequencies = a->outstate->sound_shape->nfrequencies ;
  nbands = a->outstate->sound_shape->nbands ;
  lj = (int)( (float)nfrequencies/3.0) ;
  hj = (int)( 2.0*(float)nfrequencies/3.0) ;

  for( j=0 ; j<lj ; j++ ) 
  {
    for( k=0 ; k<nbands ; k++ )
      intensity[0] += spectrum[j][k]*spectrum[j][k] ;
  }
  intensity[0] = (float)sqrt(intensity[0]/(float)lj) ;
  for( j=lj ; j<hj ; j++ ) 
  {
    for( k=0 ; k<nbands ; k++ )
      intensity[1] += spectrum[j][k]*spectrum[j][k] ;
  }  
  intensity[1] = (float)sqrt(intensity[1]/(float)(hj-lj)) ;
  for( j=hj ; j<nfrequencies ; j++ ) 
  {
    for( k=0 ; k<nbands ; k++ )
      intensity[2] += spectrum[j][k]*spectrum[j][k] ;
  }  
  intensity[2] = (float)sqrt(intensity[2]/(float)(nfrequencies-hj)) ;
    
  nonlinear_color_interpolation_3( intensity, c ) ;     
  glMaterialfv(GL_FRONT,GL_AMBIENT,c) ;
  glMaterialfv(GL_FRONT,GL_DIFFUSE,c) ;
  glMaterialfv(GL_FRONT,GL_SPECULAR,c) ;
  draw_cylinder( 0.82, 0.0, 0.05, 0.12 ) ;
}


   

