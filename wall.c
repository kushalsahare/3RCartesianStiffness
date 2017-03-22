/*************************************************************************/
/* File:        wall.c                                                 */
/* Description: all structures and dynamics specific to the wall      */
/* Author:      Kushal Sahare                                               */
/* Date:        3-19-2017                                                */
/*************************************************************************/
#include <math.h>
#include "wall.h"
#include "3Rarm.h"
#include "geometry.h"



Wall wall= {ZAXIS, /*axis*/
            M_WALL, /*Mass*/ 
            {2.0,2.0}, /*centroid*/
            { {2.0 - W_WALL*0.5, 2.0 + H_WALL*0.5 },
              {2.0 + W_WALL*0.5, 2.0 + H_WALL*0.5},
              {2.0 + W_WALL*0.5, 2.0 - H_WALL*0.5},
              {2.0 - W_WALL*0.5, 2.0 - H_WALL*0.5}}, /*vertices*/
              {0.0,0.0}, /*velocity*/
              {0.0,0.0}, /*ext_force*/
               0.0, /*theta*/
               0.0, /*theta_dot*/
               {{1.0,0.0, 2.0},
                  {0.0,1.0,2.0},
                  {0.0,0.0,1.0}} /*iTj*/ };
             

//{wall.centroid[X]-W_WALL*0.5 , wall.centroid[X]+W_WALL*0.5, wall.centroid[X]+W_WALL*0.5, wall.centroid[X]-W_WALL*0.5},
//{wall.centroid[Y]+H_WALL*0.5, wall.centroid[Y]+H_WALL*0.5, wall.centroid[Y]-H_WALL*0.5, wall.centroid[Y]-H_WALL*0.5} 

void simulate_wall()
{
  int i;
  double c1, s1, acc[2], alpha;

  acc[X] =  (!STATIONARY)*(wall.ext_force[X]/wall.mass - GRAVITY);
  acc[Y] = (!STATIONARY)*wall.ext_force[Y]/wall.mass;

  //alpha  = wall.ext_torque/wall.I;

  wall.velocity[X] += acc[X] * DT;
  wall.velocity[Y] += acc[Y] * DT;

 // wall.omega += alpha *DT;

  wall.centroid[X] += -0.5*(!STATIONARY)*GRAVITY*SQR(DT) + wall.velocity[X]*DT;
  wall.centroid[Y] += wall.velocity[Y]*DT;

 // wall.theta += wall
  
  if ((wall.centroid[X] < (MIN_X + R_OBJ)) && (wall.velocity[X] < 0.0))
    wall.velocity[X] *= -1.0;                                               
  if ((wall.centroid[X] > (MAX_X - R_OBJ)) && (wall.velocity[X] > 0.0))
    wall.velocity[X] *= -1.0;                                               
  if ((wall.centroid[Y] < (MIN_Y + R_OBJ)) && (wall.velocity[Y] < 0.0))
    wall.velocity[Y] *= -1.0;                                               
  if ((wall.centroid[Y] > (MAX_Y - R_OBJ)) && (wall.velocity[Y] > 0.0))
    wall.velocity[Y] *= -1.0; 
/*
  acc[X] = 0.0; //wall.ext_force[X]/wall.mass - GRAVITY;
  acc[Y] = 0.0; //wall.ext_force[Y]/wall.mass;

  wall.velocity[X] += acc[X] * DT;
  wall.velocity[Y] += acc[Y] * DT;

//  wall.centroid[X] += 0.0; //-0.5*GRAVITY*SQR(DT) + wall.velocity[X]*DT;
//  wall.centroid[Y] += 0.0;wall.velocity[Y]*DT;
  
  if ((wall.centroid[X] < (MIN_X + R_OBJ)) && (wall.velocity[X] < 0.0))
    wall.velocity[X] *= 0.0; //-1.0;                                               
  if ((wall.centroid[X] > (MAX_X - R_OBJ)) && (wall.velocity[X] > 0.0))
    wall.velocity[X] *= 0.0;//-1.0;                                               
  if ((wall.centroid[Y] < (MIN_Y + R_OBJ)) && (wall.velocity[Y] < 0.0))
    wall.velocity[Y] *= 0.0; //-1.0;                                               
  if ((wall.centroid[Y] > (MAX_Y - R_OBJ)) && (wall.velocity[Y] > 0.0))
    wall.velocity[Y] *= 0.0; //-1.0; 
*/

}

