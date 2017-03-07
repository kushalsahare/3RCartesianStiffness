/*************************************************************************/
/* File:        object.c                                                 */
/* Description: all structures and dynamics specific to the objects      */
/* Author:      Rod Grupen                                               */
/* Date:        2-19-2017                                                */
/*************************************************************************/
#include <math.h>
#include "3Rarm.h"

extern Arm robot[NFRAMES];

// Obj object; // initialized in xrobot.c
Obj object ={ M_OBJECT, { 1.0, 1.0}, {0.0, 0.0}, {0.0, 0.0} };

// typedef struct _obj {
//   double mass;                 // intrinsic parameters 
//   double position[2];          // position of the centroid of the object
//   double velocity[2];          // velocity of the centroid of the object
//   double ext_force[2];         // written by the simulator: endpoint load
// } Obj;

void simulate_object()
{
  int i;
  double c1, s1, c12, s12, c123, s123, acc[2];

  acc[X] = object.ext_force[X]/object.mass - GRAVITY;
  acc[Y] = object.ext_force[Y]/object.mass;

  object.velocity[X] += acc[X] * DT;
  object.velocity[Y] += acc[Y] * DT;

  object.position[X] += -0.5*GRAVITY*SQR(DT) + object.velocity[X]*DT;
  object.position[Y] += object.velocity[Y]*DT;
  
  if ((object.position[X] < (MIN_X + R_OBJ)) && (object.velocity[X] < 0.0))
    object.velocity[X] *= -1.0;                                               
  if ((object.position[X] > (MAX_X - R_OBJ)) && (object.velocity[X] > 0.0))
    object.velocity[X] *= -1.0;                                               
  if ((object.position[Y] < (MIN_Y + R_OBJ)) && (object.velocity[Y] < 0.0))
    object.velocity[Y] *= -1.0;                                               
  if ((object.position[Y] > (MAX_Y - R_OBJ)) && (object.velocity[Y] > 0.0))
    object.velocity[Y] *= -1.0;      
}
