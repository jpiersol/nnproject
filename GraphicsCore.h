/*
 *  GraphicsCore.h
 *  
 *
 *  Created by Thomas Caudell on 3/18/09.
 *  Copyright University of New Mexico 2009. All rights reserved.
 *
 */
void draw_cylinder( float xc, float yc, float h, float r ) ;
void draw_rectangle( float xc, float yc, float th, float xhw, float yhw ) ;
void draw_circle( float xc, float yc, float r, int nfacets ) ;
void linear_color_interpolation( float x, float *c0, float *c1, float *c) ;
void nonlinear_color_interpolation_1( float x, float *c0, float *c1, float *c) ;
void nonlinear_color_interpolation_2( float *x, float *c0, float *c1, float *c) ;
void nonlinear_color_interpolation_3( float *x, float *c) ;
void fl_calnormals( float *p0, float *p1, float *p2, float *n ) ;
void draw_Flatworld() ;
void draw_world() ;
void draw_object( OBJECT_TYPE *o) ;
void draw_agent( AGENT_TYPE *a ) ;
void draw_agent_metabolic_charge( AGENT_TYPE *a ) ;
void draw_agent_cargo_manifest( AGENT_TYPE *a ) ;
void draw_agent_visual_sensor_rays( AGENT_TYPE *a ) ;
void draw_agent_visual_sensor_rays_1( AGENT_TYPE *a ) ;
void draw_agent_ears( AGENT_TYPE *a ) ;
void draw_agent_skin( AGENT_TYPE *a, float dzbody ) ;
void draw_agent_mouth( AGENT_TYPE *a )  ;