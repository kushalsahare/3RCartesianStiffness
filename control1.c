/*
**	control.c - a 3R Cartesian Stiffness controller
**	rag -- February 2017
*/

#include <stdio.h>
#include <math.h>
#include "3Rarm.h"

#define VERBOSE FALSE

extern Arm robot[NFRAMES];
extern double P_des[2] = {0.4052, 0.2943};
extern double K[2][2]={{250.0,0.0},
                       { 0.0,1000.0}};

int mode = FREEFALL;
    

void control_arm(clock)
double clock;
{ 
  void arm_dynamics_3R(), matXvec();
  int i, j, joint=0;
  double M[NJOINTS][NJOINTS],V[NJOINTS],G[NJOINTS],F[NJOINTS];
  double acc[NJOINTS],tmp[NJOINTS], t[NJOINTS];
  double J[2][NJOINTS]; // manipulator Jacobian
  double JT[NJOINTS][2]; // Jacobian transpose
  double temp[NJOINTS][2];
  double Ktheta[NJOINTS][NJOINTS];
  double tht_error[NJOINTS];
  double theta_error,torq;
  double dxdt1,dxdt2,dxdt3,dydt1,dydt2,dydt3,xdot,ydot,phi;
  double s1, c1, s2, c2, s3, c3, s12, c12, s23, c23, s123, c123;
  double fx=10, fy=10, nz=0;

  double P_curr[2];

  double delta[2] ;
  
  if (mode==FREEFALL) {
    for (i=1;i<=NJOINTS;++i) {
      robot[i].torque=0.0;
    }
  }
  else if (mode==PD_CONTROL) {

   /* for (i=0;i<NARMS;++i) {
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
    }*/
    
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

   P_curr[X] = L1*c1+L2*c12+L3*c123;
   P_curr[Y] = L1*s1 +L2*s12+L3*s123;
   printf("Current Position %f %f\n", P_curr[X], P_curr[Y]);
   printf("Desired Position %f %f\n", P_des[X], P_des[Y]);

   delta[X] = P_des[X]-P_curr[X]; 
   delta[Y] = P_des[Y]-P_curr[Y];
   printf("Current Position %f %f\n", P_curr[X], P_curr[Y]);

    J[0][0] = -L1*s1 -L2*s12-L3*s123;
    J[0][1] = -L2*s12-L3*s123;
    J[0][2] = -L3*s123;
    
    J[1][0] =  L1*c1+L2*c12+L3*c123;
    J[1][1] =  L2*c12+L3*c123;
    J[1][2] =  L3*c123;
    
    matrix_transpose(2, NJOINTS, J, JT); // get transpose
    
    matrix_mult(NJOINTS, 2, JT, 2,K, temp);
    matrix_mult(NJOINTS, 2, temp, NJOINTS, J, Ktheta);
    
   /* for (i=0;i<NARMS;++i) {
        for (j=0;j<NFRAMES;++j){      
	     if (robot[j].dof_type == REVOLUTE) {
	        // PDcontrol - desired accelerations
	        theta_error = robot[j].theta_ref - robot[j].theta;
	            if (theta_error > M_PI) theta_error -= 2.0*M_PI;
	            if (theta_error < -M_PI) theta_error += 2.0*M_PI;
	            tht_error[joint++] = theta_error;
	    }
      }   
    */
    matrix_mult(NJOINTS, 2, JT, 2,delta, tht_error );
    printf("Current theta error %f %f %f\n", tht_error[0], tht_error[1], tht_error[2]);

    matrix_mult(NJOINTS, NJOINTS, Ktheta, NJOINTS, tht_error, tmp);
    arm_dynamics_3R(fx,fy,nz, M,V,G,F);
    //matXvec(M,acc, tmp);
    
    joint=0;
      for (j=0;j<NFRAMES;++j){
	if (robot[j].dof_type == REVOLUTE) {
	  robot[j].torque = tmp[joint] +V[joint] + G[joint] + F[joint];
	  joint++;
	}

    
  }
 }
 

  /*else if(mode == STIFF_CONTROL){

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

   P_curr[X] = L1*c1+L2*c12+L3*c123;
   P_curr[Y] = L1*s1 +L2*s12+L3*s123;

   delta[X] = P_des[X]-P_curr[X]; 
   delta[Y] = P_des[Y]-P_curr[Y];
   
    J[0][0] = -L1*s1 -L2*s12-L3*s123;
    J[0][1] = -L2*s12-L3*s123;
    J[0][2] = -L3*s123;
    
    J[1][0] =  L1*c1+L2*c12+L3*c123;
    J[1][1] =  L2*c12+L3*c123;
    J[1][2] =  L3*c123;
    
    matrix_transpose(2, NJOINTS, J, JT); // get transpose
    
    matrix_mult(NJOINTS, 2, JT, 2,K, temp);
    matrix_mult(NJOINTS, 2, temp, NJOINTS, J, Ktheta);
    
    for (i=0;i<NARMS;++i) {
        for (j=0;j<NFRAMES;++j){      
	     if (robot[j].dof_type == REVOLUTE) {
	        // PDcontrol - desired accelerations
	        theta_error = robot[j].theta_ref - robot[j].theta;
	            if (theta_error > M_PI) theta_error -= 2.0*M_PI;
	            if (theta_error < -M_PI) theta_error += 2.0*M_PI;
	            tht_error[joint++] = theta_error);
	    }
      }   
    
    matrix_mult(NJOINTS, NJOINTS, Ktheta, NJOINTS, tht_error, acc);
    arm_dynamics_3R(fx,fy,nz, M,V,G,F);
    matXvec(M,acc, tmp);
    
    joint=0;
      for (j=0;j<NFRAMES;++j){
	if (robot[j].dof_type == REVOLUTE) {
	  robot[j].torque = tmp[joint] +V[joint] + G[joint] + F[joint];
	  joint++;
	}

    
    }*/
}