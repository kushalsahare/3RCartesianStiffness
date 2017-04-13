/*
 **	control.c - a 3R Cartesian Stiffness controller
 **	rag -- February 2017
 */

#include <stdio.h>
#include <math.h>
#include "3Rarm.h"

#include "arm.h"
#define VERBOSE FALSE

extern Arm robot[NFRAMES];
int mode = FREEFALL;

void control_arm(clock)
  double clock;
{ 
  void arm_dynamics_3R(), matXvec(), calculate_gains();
  static double K[NJOINTS] = {KP_ARM, KP_ARM, KP_ARM}; 
  double B[NJOINTS] = {KD_ARM, KD_ARM, KD_ARM};
  int i, j, joint=0;
  double M[NJOINTS][NJOINTS],V[NJOINTS],G[NJOINTS],F[NJOINTS];
  double acc[NJOINTS],tmp[NJOINTS];
  double theta_error,torq;
  double dxdt1,dxdt2,dxdt3,dydt1,dydt2,dydt3,xdot,ydot,phi;
  double fx=0, fy=0, nz=0;

//  robot[NFRAMES-1].ext_force[X] = -10.0;
//  robot[NFRAMES-1].ext_force[Y] = -10.0;
  

  if (mode==FREEFALL) {
    for (i=1;i<=NJOINTS;++i) {
      robot[i].torque=0.0;
    }
  }
  else if (mode==PD_CONTROL) {
    //arm_dynamics_3R(fx,fy,nz, M,V,G,F);
//    printf("F[x]= %f F[Y]= %f\n", robot[NFRAMES-1].ext_force[X], robot[NFRAMES-1].ext_force[Y]);
    if(robot[NFRAMES-1].ext_force[X] || robot[NFRAMES-1].ext_force[Y]){
    calculate_gains(K,B);
    for(i=0; i < 3; i++)
     printf("K[%d] = %f B[%d]= %f\n ", i, K[i], i, B[i]);
    }

    for (i=0;i<NARMS;++i) {
      for (j=0;j<NFRAMES;++j){      
        if (robot[j].dof_type == REVOLUTE) {
          // PDcontrol - desired accelerations
          theta_error = robot[j].theta_ref - robot[j].theta;
          if (theta_error > M_PI) theta_error -= 2.0*M_PI;
          if (theta_error < -M_PI) theta_error += 2.0*M_PI;
          acc[joint++] = K[joint]*(theta_error) + 
            B[joint]*(robot[j].theta_dot_ref - robot[j].theta_dot);
        }
      }
      arm_dynamics_3R(fx,fy,nz, M,V,G,F);
      matXvec(M,acc, tmp);

      joint=0;
      for (j=0;j<NFRAMES;++j){
        if (robot[j].dof_type == REVOLUTE) {
          robot[j].torque = tmp[joint] +V[joint] + G[joint] + F[joint];
          joint++;
        }
      }
    }
  }
}
