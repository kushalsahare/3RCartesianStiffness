/*
**	control.c - a 3R Cartesian Stiffness controller
**	rag -- February 2017
*/

#include <stdio.h>
#include <math.h>
#include "3Rarm.h"
#include "arm.h"
#include <stdlib.h>

#define VERBOSE FALSE

extern Arm robot[NFRAMES];
double P_des[2];
double K[2][2];
double Kv[3][3];

//FILE *ofp;
int mode = FREEFALL;

//char * commandsForGnuplot[] = {"set title \"Force\"", "plot 'robot.ext_force[X]'"};
//FILE * gnuplotPipe = popen ("gnuplot -persistent", "w");
double phi;

void printMatrix( int rows, int cols, double A[rows][cols]){

int i=0;
int j=0;

for(i =0; i < rows ; i++){
   for(j=0; j< cols ; j++){
   printf("%f ", A[i][j]);  
   }
   printf("\n"); 
  }
}   

void printArray( int n, double A[n]){

int i=0;

for(i =0; i < n ; i++){
   printf("%f ", A[i]);  
}
   printf("\n"); 

}  

double linearlyInterpolate(double time, double startTime, 
			   double endTime, double startValue, double endValue) {
  return startValue + 
    (time - startTime)*
    (endValue - startValue)/(endTime - startTime);
}


void fk(double theta[3], double P[X]){

 double s1 = sin(theta[0]), c1 = cos(theta[0]),
        s2 = sin(theta[1]), c2 = cos(theta[1]),
        s3 = sin(theta[2]), c3 = cos(theta[2]);

double  s12 = sin(theta[0] + theta[1]), c12 = cos(theta[0] + theta[1]); 
double  s23 = sin(theta[1] + theta[2]), c23 = cos(theta[1] + theta[2]);
double  s123 = sin(theta[0] + theta[1] + theta[2]), c123 = cos(theta[0] + theta[1] + theta[2]);

P[X] = L1*c1 + L2*c12 + L3*c123;
P[Y] = L1*s1 + L2*s12 + L3*s123;

}

void Jacobian(double J[2][3]){

 double  s1 = sin(robot[1].theta),
  c1 = cos(robot[1].theta),
  s2 = sin(robot[2].theta),
  c2 = cos(robot[2].theta),
  s3 = sin(robot[3].theta),
  c3 = cos(robot[3].theta),
  s12 = sin(robot[1].theta + robot[2].theta),
  c12 = cos(robot[1].theta + robot[2].theta), 
  s23 = sin(robot[2].theta + robot[3].theta),
  c23 = cos(robot[2].theta + robot[3].theta),
  s123 = sin(robot[1].theta + robot[2].theta + robot[3].theta),
  c123 = cos(robot[1].theta + robot[2].theta + robot[3].theta);

    J[0][0] = -L1*s1 -L2*s12-L3*s123;
    J[0][1] = -L2*s12-L3*s123;
    J[0][2] = -L3*s123;
    
    J[1][0] =  L1*c1+L2*c12+L3*c123;
    J[1][1] =  L2*c12+L3*c123;
    J[1][2] =  L3*c123;
}

void control_arm(clock)
double clock;
{ 
  void arm_dynamics_3R(), matXvec();
  int i, j, jnt, jnt1, joint=0;
  double M[NJOINTS][NJOINTS],V[NJOINTS],G[NJOINTS],F[NJOINTS];
  double acc[NJOINTS],tmp[NJOINTS], t[NJOINTS], w[NJOINTS], tw[NJOINTS], damp[NJOINTS];
  double J[2][NJOINTS]; // manipulator Jacobian
  double JT[NJOINTS][2]; // Jacobian transpose
  double temp[NJOINTS][2];
  double Ktheta[NJOINTS][NJOINTS];
  

  double kp2 = 0.1;
  double kp3 = 0.2;
 
  double Jxy[2][1];
  double Jc[1][NJOINTS];

  double f[2] ;
  double del_f[2];
  double del_torq[2];

  
  double torque[2];

  double f_ref[2];

  double K_joint[NJOINTS];
  double B_joint[NJOINTS];

  double Jc_pinv[NJOINTS][1];

  double tht_error[NJOINTS], tht_mag;
  double theta_error,torq;
  double dxdt1,dxdt2,dxdt3,dydt1,dydt2,dydt3,xdot,ydot,phi;
  double s1, c1, s2, c2, s3, c3, s12, c12, s23, c23, s123, c123;
  double fx=0, fy=0, nz=0;

  double P_curr[2];

  double delta[2] ;


  if (mode==FREEFALL) {
    for (i=1;i<=NJOINTS;++i) {
      robot[i].torque=0.0;
    }
  }
  else if (mode==PD_CONTROL) {

/**************** PID ******************************************************/


 //  ofp = fopen("force_data.txt", "a+");
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
    
/******************************Cartesian stiffness controller***********************************************/
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

   P_curr[X] = L1*c1 + L2*c12 + L3*c123;
   P_curr[Y] = L1*s1 + L2*s12 + L3*s123;

  printf("Current Position %f %f\n", P_curr[X], P_curr[Y]);
 /* for (j=0;j<NFRAMES;++j){      
	if (robot[j].dof_type == REVOLUTE) {
	  theta_error = robot[j].theta_ref - robot[j].theta;
	  if (theta_error > M_PI) theta_error -= 2.0*M_PI;
	  if (theta_error < -M_PI) theta_error += 2.0*M_PI;
      w[joint++] = theta_error;
	    
	}
      }
*/
 


 /*  //printf("Current force %f %f\n", robot[NFRAMES-1].ext_force[X], robot[NFRAMES-1].ext_force[Y]);
   //fprintf(ofp, "%f %f\n", robot[NFRAMES-1].ext_force[X], robot[NFRAMES-1].ext_force[Y] );
   delta[X] = P_des[X]-P_curr[X]; 
   delta[Y] = P_des[Y]-P_curr[Y];
   //printf("Current delta %f %f\n", delta[X], delta[Y]);
    
    Jxy[0] = -2*delta[X];
    Jxy[1] = -2*delta[Y];

    J[0][0] = -L1*s1 -L2*s12-L3*s123;
    J[0][1] = -L2*s12-L3*s123;
    J[0][2] = -L3*s123;
    
    J[1][0] =  L1*c1+L2*c12+L3*c123;
    J[1][1] =  L2*c12+L3*c123;
    J[1][2] =  L3*c123;
    
    //printf("J = ");
    //printMatrix(NJOINTS, 2, J); 
    matrix_transpose(2, NJOINTS, J, JT); // get transpose
     
    matrix_mult(1,2, Jxy, J, NJOINTS, Jc); // Jxy
    
    void pseudoinverse(int m, int n, double A[m][n], double Jc_pinv[n][m]) 
    //printf("Jc= ");
    //printMatrix(1, NJOINTS, Jc);
    
    matrix_mult(NJOINTS, 2, JT, 2,K, temp); // temp = J^T*K
    matrix_mult(NJOINTS, 2, temp, NJOINTS, J, Ktheta); // Ktheta = temp*J = J^T*K*J
   // printf("Ktheta= ");
    //printMatrix( NJOINTS, NJOINTS, Ktheta);
    //
   // for (i=0;i<NARMS;++i) {
        for (j=0;j<NFRAMES;++j){      
	     if (robot[j].dof_type == REVOLUTE) {
	        // PDcontrol - desired accelerations
	        theta_error = robot[j].theta_ref - robot[j].theta;
	            if (theta_error > M_PI) theta_error -= 2.0*M_PI;
	            if (theta_error < -M_PI) theta_error += 2.0*M_PI;
	            tht_error[joint++] = theta_error;
	    }
      }   
    //
    matrix_mult(NJOINTS, 2, JT, 2,delta, tht_error ); // delta_theta = J^T*delta; 

    //printf("Current theta error %f %f %f\n", tht_error[0], tht_error[1], tht_error[2]);
    //tht_mag = sqrt(tht_error[0]*tht_error[0]+tht_error[1]*tht_error[1]+tht_error[2]*tht_error[2]);
    //if(tht_mag < 0.001)
    //   return;     
    

    matrix_mult(NJOINTS, NJOINTS, Ktheta, NJOINTS, w, tmp);
    //printf("Torque: ");
    //printArray(3,tmp);
    
    arm_dynamics_3R(fx,fy,nz, M,V,G,F);
    
    //matXvec(M,acc, tmp);
    //matXvec(M,w, tw); // M*(theta_des-theta)
    //matrix_mult(NJOINTS, NJOINTS, Kv, NJOINTS,tw, damp );
    
    joint=0;
      for (j=0;j<NFRAMES;++j){
	if (robot[j].dof_type == REVOLUTE) {
	  robot[j].torque = tmp[joint] + V[joint] + G[joint]+ F[joint]+ 10*(robot[j].theta_dot_ref - robot[j].theta_dot);
	  joint++;
	}

    
  }*/ 

/***************************** Method I ***************************************/
  
   P_curr[X] = L1*c1 + L2*c12 + L3*c123;
   P_curr[Y] = L1*s1 + L2*s12 + L3*s123;

   delta[X] = P_des[X]-P_curr[X]; 
   delta[Y] = P_des[Y]-P_curr[Y];

   f[X] = robot[NFRAMES-1].ext_force[X]; 
   f[Y] = robot[NFRAMES-1].ext_force[Y];

   printf("Force %f %f\n", f[X], f[Y]);
    
   // Jxy[0][0] = -2*delta[X];
   // Jxy[1][0] = -2*delta[Y];

    Jacobian(J);

    //printf("J = ");
    //printMatrix(NJOINTS, 2, J); 

    matrix_transpose(2, NJOINTS, J, JT); // get transpose
     
    matrix_mult(1,2, Jxy, J, NJOINTS, Jc); // Jxy
    
    //void pseudoinverse(int m, int n, double A[m][n], double Jc_pinv[n][m]) 
    //printf("Jc= ");
    //printMatrix(1, NJOINTS, Jc);
    
    matrix_mult(NJOINTS, 2, JT, 2,K, temp); // temp = J^T*K
    matrix_mult(NJOINTS, 2, temp, NJOINTS, J, Ktheta); // Ktheta = temp*J = J^T*K*J

    theta_error = 0.174; // in radian ( = 10 degrees)
    
    jnt  = 0;
     //jnt1 = 0;
     
     // if force then calculate the pid gains
     if(robot[NFRAMES-1].ext_force[X] || robot[NFRAMES-1].ext_force[X]) {
     
     matrix_mult(NJOINTS, 2, JT, 2,f, torque);
     //printMatrix(1, NJOINTS, torque);
    
     //printf("Printing K values \n");
     for( j =0; j < 3; j++){
      K_joint[j] = torque[j]/ theta_error;
      B_joint[j] = 2*sqrt(fabs(K_joint[j]));
      printf("T= %f K= %f , B= %f ", torque[j], K_joint[j], B_joint[j]);

     }

     printf("\n");
     //printf("Torque= ");
    // for (j=0;j<NFRAMES;++j){      
	//     if (robot[j].dof_type == REVOLUTE) {
        //        printf("%f ", torque[jnt]); 
	//        K_joint[jnt] = torque[jnt]/ theta_error;
        //        printf(" K = %f T/theta = %f ", K_joint[jnt], torque[jnt]/theta_error);
        //        B_joint[jnt] = 2*sqrt(K_joint[jnt]);
        //        printf(" B = %f\n ", B_joint[jnt]);
        //        jnt++;
	//    }
      //}//
 
     printf("\n");
     //calculate the change in delta because of the delta force
     
     jnt = 0;
     del_f[X] = f_ref[X]+ K[0][0]*delta[X] - robot[NFRAMES-1].ext_force[X];
     del_f[Y] = f_ref[Y]+ K[1][1]*delta[Y] - robot[NFRAMES-1].ext_force[Y];

     matrix_mult(NJOINTS, 2, JT, 2,f, del_torq); 
     printf("delta torque= ");
   
     for( j =0; j < 3; j++){
     K_joint[j] += kp2*del_torq[j]/ theta_error;
     B_joint[j]  = 2*sqrt(fabs(K_joint[j]));
      printf("T= %f K= %f , B= %f ", del_torq[j], K_joint[j], B_joint[j]);

     }


 //for (j=0;j<NFRAMES;++j){      
	  //   if (robot[j].dof_type == REVOLUTE) {
          //      printf("%f ", del_torq[jnt]); 
	  //      K_joint[jnt] += kp2*del_torq[jnt]/ theta_error;
          //      B_joint[jnt]  = 2*sqrt(K_joint[jnt]);
          //      jnt++;
	  //  }
 // } 
    
   printf("\n");   

   joint = 0;
   printf("acc= ");
    for (i=0;i<NARMS;++i) {
      for (j=0;j<NFRAMES;++j){      
	if (robot[j].dof_type == REVOLUTE) {
	  // PDcontrol - desired accelerations
	  theta_error = robot[j].theta_ref - robot[j].theta;
	  if (theta_error > M_PI) theta_error -= 2.0*M_PI;
	  if (theta_error < -M_PI) theta_error += 2.0*M_PI;
	  acc[joint] = K_joint[joint]*(theta_error) + 
	    B_joint[joint]*(robot[j].theta_dot_ref - robot[j].theta_dot);
          printf("%f %f \n", K_joint[joint], B_joint[joint]);          
          printf("%f ", acc[joint]);
        
          joint++;
	}
        
      }
      printf("\n");
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
