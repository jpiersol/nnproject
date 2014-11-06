/*
 *  FlatworldII.h
 *  
 *
 *  Created by Thomas Caudell on 3/17/09.
 *  Copyright University of New Mexico 2009. All rights reserved.
 *
 */
 
/* General purpose structures */
typedef struct geometric_shape_type
{
  float **vertices ;	/* list of 2D vertices and their colors that compose the surface of this object in object coordinate system.
						    The list is of size nvertices x 5. The five components are x, y, r, g, b. */
  float height ; /* height values fro graphics */
  int nbands ; /* number of color bands for each vertix */
  int nvertices ;		/* number of vertices in list. */
  float scale ;     /* A scale used by the graphics when drawing the objects */
} GEOMETRIC_SHAPE_TYPE ;
 
typedef struct acoustic_shape_type
{
  float *freq ;    /* list of frequencies for the spectrum */
  float **spectrum ; /* list of sound intensity values for a set of frequencies, emitted by this object 
							from the object coordinate system origin. */
  int nbands ; /* number of amplitudes for the sound at each frequencey.  useful for simulating phase. */
  int nfrequencies ;      /* number of frequencies in sound spectrum. */
} ACOUSTIC_SHAPE_TYPE ;

/* Structures for the world */
typedef struct world_type 
{
  int index ;			/* unique name of world. */
  struct agent_type **agents ;	/* list of pointers to agents */
  int nagents ;			/* number of agents in world. */
  int maxnagents ;		/* max allowable agents in world */
  struct object_type **objects ;	/* list of pointers to objects. */
  int nobjects ;			/* number of objects in world. */
  int nactive_objects ; /* the number of objects remaining active (i.e., not eaten) inthe world */
  int maxnobjects ;			/* max allowable objects in world */
  float xmax, xmin ;		/* max and min x bounds on world */
  float ymax, ymin ;		/* max and min y bounds on world */
  struct time_type *current_time ; /* current simulation world time. */
  struct geometric_shape_type *scratch_geo_shape_type ;  /* a scratch area for soma sensor calculations, size greater that nvertices of
                                                   and agent */
  struct geometric_shape_type *scratch_geo_shape_type2 ;  /* a scratch area for soma sensor calculations, size greater that nvertices of
                                                   and agent */
	int rscaleflag ;
} WORLD_TYPE ;

typedef struct object_type 
{
  int index ;         /* unique name of object, */
  int inworld_flag ;  /* 1 = object is physically in the world, 0 = out of world */
  int type ;          /* type of object. Objects on world have type number >0, <0 when being carried by an agent */
  float mass ;        /* the "weight" of the object. */
  float food_value ;  /* the amout of metabolic charge +/- an agent gets if this object is eaten */
  float x ;           /* x location of origin of object coordinate system in world coordinates. */
  float y ;           /* y location of origin of object coordinate system in world coordinates. */
  struct geometric_shape_type *physical_shape ;   /* the geometric shape of the object and its colors emmitted by the vertices */
  struct acoustic_shape_type *sound_shape ;       /* the sound spectrum emmitted by this object */
  void (*behavior_func)(struct object_type *object) ; /* function pointer to possible behavior of object,
							including changing the vertex colors and the sound intensity levels as well
							as the position ofhe object over time. */
  float bounding_radius ;
} OBJECT_TYPE ;

typedef struct time_type 
{ /* simulation time hierarchy */
  int dsecond ; /* 0-1 decasecond */
  int second ;	/* 0-59 */
  int minute ;	/* 0-59 */
  int hour ;	/* 0-23 */
  int day ;		/* 0-29 */
  int week ;	/* 0-3 */
  int month ;	/* 0-11 */
  int year ;	/* 0-inf */
} TIME_TYPE ;

/* Structures for an agent */
typedef struct agent_type 
{
  int index ;					/* unique name of agent. */
  int inworld_flag ;  /* 1 = agent is physically in the world, 0 = out of world */
  struct internal_state_type *instate ;	/* internal state information. */
  struct external_state_type *outstate ;	/* external state information */
} AGENT_TYPE;

typedef struct external_state_type 
{
  float x ;            /* x coordinate of orgin of agent's coordinate system in world coordinates. */
  float y ;            /* y coordinate of orgin of agent's coordinate system in world coordinates. */
  float body_angle ;   /* The body heading angle relative to True North, measured CW. */
  float head_radius ;		  /* radius of head, assumed circular, centered at origin of agent coordinate system. */
  float head_angle ;          /* head direction rotation angle, relative to front of body, in degrees, positive CW 
									around origin of agent coordinated system.*/
  struct geometric_shape_type *physical_shape ;  /* the geometric shape of the agent and the colors emitted by its vertices. */
  struct acoustic_shape_type *sound_shape ;	 /* the sound spectrum emitted by this agent. */
  float bounding_radius ;
  float xmaxbody, xminbody, ymaxbody, yminbody ; /* tight bounding rectangle */

} EXTERNAL_STATE_TYPE ;

typedef struct internal_state_type 
{
  float metabolic_charge ;    /* current charge on battery, max charge is unity. */	
  float metabolic_burn_rate ; /* the basal rate the metabolic charge is used. */
  float mass ;                /* current mass of the agent including all cargo. */
  float gender ;			  /* a variable to represent gender disposition. */
  struct visual_sensor_type **eyes ;		/* list of visual sensors, assumed to be attached to head. */
  int neyes ;						/* number of eyes. */
  int maxneyes ;					/* max number of eyes. */
  struct acoustic_sensor_type **ears ;		/* list of sound sensors, assumed to be attached to head. */
  int nears ;						/* number of ears. */
  int maxnears ;					/* max number of ears */
  struct soma_sensor_type *skin ;			/* touch sensors, assumed to be attached to body, one receptor for every 
                                               polyline in body shape. */
  struct proprio_sensor_type *encoders ;	/* muscle encoder sensors, assumed to be associated with body actuators. */
  struct cargo_manifest_type *cargo ;	/* manifest of cargo items, either objects or agents. */
  struct body_actuator_type *bodyactuators ; /* rate of change of locamotion actuators. */
  struct head_actuator_type *headactuators ; /* rate of change of locamotion actuators. */

  struct eloom_system_type *brain ;		/* the neural network controller. */
  float ftemp[10] ;                   /* floats temp variables */
  int itemp[10] ;                   /* integer temp variables */
  
} INTERNAL_STATE_TYPE ;

typedef struct visual_sensor_type 
{
  int nreceptors ;				/* the number of receptor complexes in the sensor. */
  int nbands ;					/* the number of frequency bands in the receptor complex. */
  float *receptor_locations ;	/* an array of positions of each receptor complex around the agent's head, in degrees
							       measured clockwise from the head front. */
  float *receptor_directions ;	/* an array of pointing directions relative to the radial pointing direction of each receptor outward from their 
                                   location on the agent's head, in degrees
							       measured clockwise from the radial direction.  Allows for overlapping receptive fields. */
  float **values ;				/* an 2D array the receptor values of size (nreceptors x nbands) */
  float *intensities ;    /* array of intensities for each receptor complex over bands*/
  OBJECT_TYPE **seen_objects ; /* Array of pointers to the objects intersected by the beams */
} VISUAL_SENSOR_TYPE ;

typedef struct acoustic_sensor_type 
{
  int nreceptors ;				/* the number of frequency receptor complexes in the sensor. */
  int nbands ;					/* the number of bands per frequency receptor ( nbands=2 => real & imag compoments). */
  float receptor_location ;	/* the position of the receptor complex on the agent's head, in degrees
								               measured clockwise from the head front. */
  float **values ;				/* an 2D array the receptor values of size (nreceptors x nbands) */
  float intensity ;       /* the total sound intensity for this sensor over all receptors */
} ACOUSTIC_SENSOR_TYPE ;

typedef struct soma_sensor_type 
{
  int nreceptors ;				/* the number of receptor complexes in the sensor, located at vertices of body shape. */
  int nbands ;					/* the number of quality (e.g. mechano, thermo, nocio) bands in the receptor complex. */
  struct geometric_shape_type *receptor_locations;	/* an array of vertex indices for each receptor complex around the agent body. */
  float **values ;				/* an 2D array the receptor values of size (nreceptors x nbands) */
  int *touched_objects ;    /* skin cell objec in collision list */
} SOMA_SENSOR_TYPE ;

typedef struct proprio_sensor_type 
{
  int nreceptors ;				/* the number of receptor complexes in the sensor. */
  int nbands ;					/* the number of quality bands in the receptor complex. */
  float *receptor_locations ;	/* an array of positions of each receptor complex in the motor system. */
  float **values ;				/* an 2D array the receptor values of size (nreceptors x nbands) */
} PROPRIO_SENSOR_TYPE ;

typedef struct cargo_manifest_type
{
  void **manifest ;	/* list of pointers to either objects or agents */
  
  int maxnitems ;	/* max number of items in manifest */
  int nitems ;		/* number of items in manifest */
} CARGO_MANIFEST_TYPE ;

typedef struct body_actuator_type 
{
  float deltaFB ; /* rate of change in body Front/Back position */
  float deltaRL ; /* rate of change in body Right/Left position */
  float deltaTH ;  /* rate of change in body heading ( degrees ) */
  float movement_burn_rate ; /* the cost per uint movement step.  Added 14 Aug 2012, tpc */
  float max_translation_delta ; /* magnitude of maximum allowable teanslation step size. dded 9/11/13, tpc */
} BODY_ACTUATOR_TYPE ;

typedef struct head_actuator_type 
{
  float deltaH ;  /* rate of change in head direction wrt body( degrees ) */
} HEAD_ACTUATOR_TYPE ;

typedef struct eloom_system_type 
{
  float dummy ;					/* will contain structure information about the eLoom II system. */
} ELOOM_SYSTEM_TYPE ;

/* Utility function prototypes */
float min( float x, float y) ;
float L2measure( float x0, float y0, float x1, float y1 ) ;
void rotate2D( GEOMETRIC_SHAPE_TYPE *spout, GEOMETRIC_SHAPE_TYPE *spin, float xc, float yc, float angle ) ;
float min_geometric_shape_distance( GEOMETRIC_SHAPE_TYPE *s1, GEOMETRIC_SHAPE_TYPE *s2 ) ;
void testm( void *e, char *message, char *variable ) ;
GEOMETRIC_SHAPE_TYPE *make_geometric_shape_type( int nvertices, int nbands, float height, float scale ) ;
void free_geometric_shape_type( GEOMETRIC_SHAPE_TYPE *r ) ;
ACOUSTIC_SHAPE_TYPE *make_acoustic_shape_type( int nfrequencies, int nbands ) ;
void free_acoustic_shape_type( ACOUSTIC_SHAPE_TYPE *r ) ;
int intersect_beam_with_object(OBJECT_TYPE *obj, float rx, float ry, float sx, float sy, float *xi, float *yi, float *d, float *beta ) ;
int intersect_beam_with_physical_shape(GEOMETRIC_SHAPE_TYPE *shape, float rx, float ry, float sx, float sy, float *ximin, float *yimin, float *d, float *beta ) ;
int line_to_line_seg_intersect( float x0, float y0, float x1, float y1, float x2, float y2, float x3, float y3,
                                float *xi, float *yi, float *d, float *beta, float *T, float *L);

/* Creator prototypes*/
AGENT_TYPE *make_agent( int index, float x, float y, float body_angle, float head_radius, float mass ) ;
void add_visual_sensor_to_agent( AGENT_TYPE *agent, int nreceptors, int nbands, float initial_values, float *receptor_locations,  
                               float *receptor_directions ) ;
void free_visual_sensor_type( VISUAL_SENSOR_TYPE *r ) ;
void add_acoustic_sensor_to_agent( AGENT_TYPE *agent, int nreceptors, int nbands, float initial_values, float receptor_location ) ;
void free_acoustic_sensor_type( ACOUSTIC_SENSOR_TYPE *r ) ;
void add_soma_sensor_to_agent( AGENT_TYPE *agent, int nbands, float initial_values, GEOMETRIC_SHAPE_TYPE *receptor_locations ) ;
void free_soma_sensor_type( SOMA_SENSOR_TYPE *r ) ;
void add_proprio_sensor_to_agent( AGENT_TYPE *agent, int nreceptors, int nbands, float initial_values, float *receptor_locations ) ;
void free_proprio_sensor_type( PROPRIO_SENSOR_TYPE *r ) ;
void add_cargo_manifest_type_to_agent( AGENT_TYPE *agent, int maxnitems ) ;
void free_cargo_manifest_type( CARGO_MANIFEST_TYPE *r  ) ;
void add_actuators_to_agent( AGENT_TYPE *a ) ;
void free_actuators_type( AGENT_TYPE *a ) ;
void add_physical_shape_to_agent( AGENT_TYPE *agent, GEOMETRIC_SHAPE_TYPE *shape );
void add_sound_shape_to_agent( AGENT_TYPE *agent, ACOUSTIC_SHAPE_TYPE *shape );

OBJECT_TYPE *make_object( int index, int type, float x, float y, float mass, float food_value )  ;
void free_object( OBJECT_TYPE *o ) ;
void add_physical_shape_to_object( OBJECT_TYPE *agent, GEOMETRIC_SHAPE_TYPE *shape );
void add_sound_sphape_to_object( OBJECT_TYPE *agent, ACOUSTIC_SHAPE_TYPE *shape );
void add_type_to_object( OBJECT_TYPE *object, int type );
void add_location_to_object( OBJECT_TYPE *object, float x, float y ) ;
void add_behavior_to_object( OBJECT_TYPE *object, void (*behavior_func)(struct object_type *object) ) ;

WORLD_TYPE *make_world( int index, int maxnagents, int maxnobjects, float xmax, float xmin, float ymax, float ymin, int rscaleflag ) ;
void add_agent_to_world( WORLD_TYPE *world, AGENT_TYPE *agent ) ;
void add_object_to_world( WORLD_TYPE *world, OBJECT_TYPE *object ) ;
void delete_object_from_world( WORLD_TYPE *w, OBJECT_TYPE *o ) ;
void init_world_time( WORLD_TYPE *w ) ;
void increment_world_clock( WORLD_TYPE *w ) ;
unsigned long int seconds_from_start( WORLD_TYPE *w ) ;
void print_world_time( WORLD_TYPE *w ) ;

/* Reading, setting, and movement function prototypes */
void read_visual_sensor( WORLD_TYPE *w, AGENT_TYPE *a) ;
float **extract_visual_receptor_values_pointer( AGENT_TYPE *a, int eye_index) ;
float visual_receptor_position( VISUAL_SENSOR_TYPE *eye, int recindex ) ;
void read_acoustic_sensor( WORLD_TYPE *w, AGENT_TYPE *a) ;
float **extract_sound_receptor_values_pointer( AGENT_TYPE *a, int ear_index) ;
int read_soma_sensor( WORLD_TYPE *w, AGENT_TYPE *a) ;
float **extract_soma_receptor_values_pointer( AGENT_TYPE *a ) ;
float read_agent_metabolic_charge( AGENT_TYPE *a ) ;
float read_agent_mass( AGENT_TYPE *a ) ;
void read_agent_body_position( AGENT_TYPE *a, float *x, float *y, float *h ) ;
void set_agent_body_position( AGENT_TYPE *a, float x, float y, float h ) ;
void rotate_agent_head( AGENT_TYPE *a, float h ) ;
void read_agent_head_angle( AGENT_TYPE *a, float *th ) ;
void set_agent_head_angle( AGENT_TYPE *a, float th ) ;
float calc_soma_activation_direction( WORLD_TYPE *w, AGENT_TYPE *a ) ;
void set_metabolic_burn_rate_agent(AGENT_TYPE *a, float m ) ;
void reset_agent_charge( AGENT_TYPE *a ) ;
void move_body_agent( AGENT_TYPE *a ) ;
float basal_metabolism_agent( AGENT_TYPE *a )  ;
void move_head_agent( AGENT_TYPE *a ) ;
void scan_head_agent( AGENT_TYPE *a, float thmax, float thmin, float period )  ;
void set_actuators_agent( AGENT_TYPE *a, float dfb, float drl, float dth, float dh ) ;
void read_actuators_agent( AGENT_TYPE *a, float *dfb, float *drl, float *dth, float *dh ) ;
float agent_eat_object( WORLD_TYPE *w, AGENT_TYPE *a, OBJECT_TYPE *o ) ;
float agent_eat_object_with_flag( WORLD_TYPE *w, AGENT_TYPE *a, OBJECT_TYPE *o, int flag) ;
int agent_pickup_object( WORLD_TYPE *w, AGENT_TYPE *a, OBJECT_TYPE *o ) ;
void agents_controller( WORLD_TYPE *w ) ;
void agents_controller_1( WORLD_TYPE *w ) ;

void make_world_objects_specfile( WORLD_TYPE *w, char *filename, int nobjects ) ;
void read_object_spec_file( WORLD_TYPE *w, char *filename ) ;

void process_visual_sensors_1( AGENT_TYPE *a, int *maxrec ) ;


