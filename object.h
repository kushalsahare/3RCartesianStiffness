/**************************************************************************/
/* File:        Object.h                                                     */
/* Description:  Object structure for robot                                  */
/* Author:      Rod Grupen                                                */
/* Date:        2-28-2016                                                 */
/* Edited :     Kushal Sahare                                             */
/* Date:        3-15-2017                                                 */
/**************************************************************************/


#ifndef OBJECT_H_
#define OBJECT_H_

typedef struct _obj {
  double mass;         // intrinsic parameters 
  double position[2];  // position of the centroid of the object
  double velocity[2];  // velocity of the centroid of the object
  double ext_force[2]; // written by the simulator: endpoint load
} Obj;

#endif //OBJECT_H_
