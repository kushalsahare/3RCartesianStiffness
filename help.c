/*
 **	help.c - to calculate the gains
 **	rag -- March 2017
 */

#include <stdio.h>
#include <math.h>
#include "3Rarm.h"

#define VERBOSE FALSE

extern Arm robot[NFRAMES];

void calculate_gains(K,B)
double K[NJOINTS],
B[NJOINTS];
{ 

  int i, j, joint=0;
  double f[2] ={robot[NFRAMES-1].ext_force[X], robot[NFRAMES-1].ext_force[Y]};
  double theta_error;
  double t[2];
  double J[NJOINTS][2]; // Jacobian
  double JT[2][NJOINTS]; // J^T
  double s1, c1, s2, c2, s3, c3, s12, c12, s23, c23, s123, c123;

  s1 = sin(robot[1].theta);
  c1 = cos(robot[1].theta);
  s2 = sin(robot[2].theta);
  c2 = cos(robot[2].theta);
  s3 = sin(robot[3].theta);
  c3 = cos(robot[3].theta);
  s12 = sin(robot[1].theta + robot[2].theta);
  c12 = cos(robot[1].theta + robot[2].theta);
  s23 = sin(robot[2].theta + robot[3].theta);
  c23 = cos(robot[2].theta + robot[3].theta);
  s123 = sin(robot[1].theta + robot[2].theta + robot[3].theta);
  c123 = cos(robot[1].theta + robot[2].theta + robot[3].theta);

  J[0][0] = -L1*s1-L2*s12-L3*s123;
  J[0][1] = -L2*s12-L3*s123;
  J[0][2] = -L3*s123;
  J[1][0] =  L1*c1+L2*c12+L3*c123;
  J[1][1] =  L2*c12+L3*c123;
  J[1][2] =  L3*c123;

  matrix_transpose(2, NJOINTS, J, JT);

  matrix_mult(NJOINTS, 2, JT, 2,f, t);

  for (i=0;i<NARMS;++i) {
    for (j=0;j<NFRAMES;++j){      
      if (robot[j].dof_type == REVOLUTE) {
        // PDcontrol - desired accelerations
        theta_error = robot[j].theta_ref - robot[j].theta;
        if (theta_error > M_PI) theta_error -= 2.0*M_PI;
        if (theta_error < -M_PI) theta_error += 2.0*M_PI;
        K[joint] = t[joint]/(theta_error) ;
        B[joint] = 2*sqrt(K[joint]);

      }
    }
  }



}
