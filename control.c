/*
**	control.c - a 3R Cartesian Stiffness controller
**	rag -- February 2017
*/

#include <stdio.h>
#include <math.h>
#include "3Rarm.h"

#define VERBOSE FALSE

extern Arm robot[NFRAMES];
int mode = FREEFALL;

void control_arm(clock)
double clock;
{ 
  void arm_dynamics_3R(), matXvec();
  int i, j, joint=0;
  double M[NJOINTS][NJOINTS],V[NJOINTS],G[NJOINTS],F[NJOINTS];
  double acc[NJOINTS],tmp[NJOINTS];
  double theta_error,torq;
  double dxdt1,dxdt2,dxdt3,dydt1,dydt2,dydt3,xdot,ydot,phi;
  double fx=-10, fy=0, nz=0;
  
  if (mode==FREEFALL) {
    for (i=1;i<=NJOINTS;++i) {
      robot[i].torque=0.0;
    }
  }
  else if (mode==PD_CONTROL) {

    for (i=0;i<NARMS;++i) {
      for (j=0;j<NFRAMES;++j){      
	if (robot[j].dof_type == REVOLUTE) {
	  // PDcontrol - desired accelerations
	  theta_error = robot[j].theta_ref - robot[j].theta;
	  if (theta_error > M_PI) theta_error -= 2.0*M_PI;
	  if (theta_error < -M_PI) theta_error += 2.0*M_PI;
	  acc[joint++] = KP_ARM*(theta_error) + 
	    KD_ARM*(robot[j].theta_dot_ref - robot[j].theta_dot);
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
