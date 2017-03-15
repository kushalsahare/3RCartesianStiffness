/*
 **	help.c - to calculate the gains
 **	rag -- March 2017
 */

#include <stdio.h>
#include <math.h>
#include "3Rarm.h"

#define VERBOSE FALSE
#define M_DEBUG 0

extern Arm robot[NFRAMES];

void calculate_gains(K,B)
double K[NJOINTS],
B[NJOINTS];
{ 

  int i, j, jnt=0;
  double f[2] ={robot[NFRAMES-1].ext_force[X], robot[NFRAMES-1].ext_force[Y]};
  
  #ifdef M_DEBUG 
  printf("f[X]=%f f[Y]=%f\n",f[0], f[1]);
  #endif
  double theta_error[3];
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

  #ifdef M_DEBUG 
  printf("t[0]=%f t[2]=%f t[3]=%f\n",t[0], t[1], t[2]);
  #endif
/* for(i =0; i < 3 ; i++){
  theta_error[i] = robot[i+1].theta_ref - robot[i+1].theta;
        K[i] = t[i]/(theta_error[i]) ;
        B[i] = 2*sqrt(K[i]);
}*/

 /* theta_error[0] = robot[1].theta_ref - robot[1].theta;
        printf("tht_error[0]= %f ", theta_error[0]);
        K[0] = t[0]/(theta_error[0]) ;
        B[0] = 2*sqrt(K[0]);

    theta_error[1] = robot[2].theta_ref - robot[2].theta;
        printf("tht_error[1]= %f ", theta_error[1]);
        K[1] = t[1]/(theta_error[1]) ;
        B[1] = 2*sqrt(K[1]); 

        theta_error[2] = robot[3].theta_ref - robot[3].theta;
        printf("tht_error[2]= %f\n", theta_error[2]);
        K[2] = t[2]/(theta_error[2]) ;
        B[2] = 2*sqrt(K[2]); 
 */ 
 for (i=0;i<NARMS;++i) {
    for (j=0;j<NFRAMES;++j){      
      if (robot[j].dof_type == REVOLUTE) {
        // update gains
        theta_error[jnt] = robot[j].theta_ref - robot[j].theta;
        if (theta_error[jnt] > M_PI) theta_error[jnt] -= 2.0*M_PI;
        if (theta_error[jnt] < -M_PI) theta_error[jnt] += 2.0*M_PI;
          if(theta_error[jnt] < 0.01) continue;
        #ifdef M_DEBUG
        //printf("j= %d joint = %d\n",j, jnt);
        #endif
        K[jnt] = t[jnt]/(theta_error[jnt]) ;
        B[jnt] = 2*sqrt(K[jnt]);
        jnt++;
        #ifdef M_DEBUG
        //printf("K[%d]=%f B[%d]=%f\n", jnt, K[jnt], jnt, B[jnt]); 
        #endif  
      }
    }
  }



}
