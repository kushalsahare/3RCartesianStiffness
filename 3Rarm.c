/*************************************************************************/
/* File:        arm.c                                                    */
/* Description: all data structures and dynamics specific to the arm     */
/* Author:      Rod Grupen/Bryan Thibodeau                               */
/* Date:        2-21-91                                                  */
/*************************************************************************/
#include <stdio.h>
#include <math.h>
#include "3Rarm.h"
#include "arm.h"

extern double clock;
double P_des[2];
double K[2][2]={{1.0,0.0},
                       { 0.0,1000.0}};
double Kv[3][3]={{10.000,  0.0, 0.0},
                      { 0.0, 10.000, 0.0},
                        0.0,  0.0, 10.000};

static double l[3] = {L1, L2, L3};
static double m[3] = {M1, M2, M3};

Arm robot[NFRAMES] =
  { { { { 1.0, 0.0, 0.0, 0.0 },
	{ 0.0, 1.0, 0.0, 0.0 },         /* world to frame 0 */
	{ 0.0, 0.0, 1.0, 0.0 },
	{ 0.0, 0.0, 0.0, 1.0 } }, NOTYPE, NOAXIS,
      0.0, 0.0, 0.0, 0.0, 0.0, {0.0,0.0,0.0}},
    { { { 1.0, 0.0, 0.0, 0.0 },         /* frame 0 to frame 1 */
	{ 0.0, 1.0, 0.0, 0.0 },
	{ 0.0, 0.0, 1.0, 0.0 },
	{ 0.0, 0.0, 0.0, 1.0 } }, REVOLUTE, ZAXIS,
      THETA1_HOME, 0.0, THETA1_REF, 0.0, 0.0, {0.0,0.0,0.0}},
    { { { 1.0, 0.0, 0.0, L1  },  /* frame 1 to frame 2 */
	{ 0.0, 1.0, 0.0, 0.0 },
	{ 0.0, 0.0, 1.0, 0.0 },
	{ 0.0, 0.0, 0.0, 1.0 } }, REVOLUTE, ZAXIS,
      THETA2_HOME, 0.0, THETA2_REF, 0.0, 0.0, {0.0,0.0,0.0}},
    { { { 1.0, 0.0, 0.0, L2  },  /* frame 1 to frame 2 */
	{ 0.0, 1.0, 0.0, 0.0 },
	{ 0.0, 0.0, 1.0, 0.0 },
	{ 0.0, 0.0, 0.0, 1.0 } }, REVOLUTE, ZAXIS,
      THETA3_HOME, 0.0, THETA3_REF, 0.0, 0.0, {0.0,0.0,0.0}},
    { { { 1.0, 0.0, 0.0, L3  },  /* frame 1 to frame 2 */
	{ 0.0, 1.0, 0.0, 0.0 },
	{ 0.0, 0.0, 1.0, 0.0 },
	{ 0.0, 0.0, 0.0, 1.0 } }, NOTYPE, NOAXIS,
      0.0, 0.0, 0.0, 0.0, 0.0, {0.0,0.0,0.0}} };

void simulate_arm()
{
  int i;
  void arm_dynamics_3R();
  void arm_accelerations(), euler_arm();
  double c123, s123;
  double M[NJOINTS][NJOINTS], V[NJOINTS], G[NJOINTS], F[NJOINTS];
  double arm_acc[NJOINTS];
  double fx=0,fy=0,nz=0;
 
  s123 = sin(robot[1].theta + robot[2].theta + robot[3].theta);
  c123 = cos(robot[1].theta + robot[2].theta + robot[3].theta);

  // convert Cartesian endpt forces in frame 0 to endpoint frame (frame 4)
  fx= robot[NFRAMES-1].ext_force[X]*c123 + robot[NFRAMES-1].ext_force[Y]*s123;
  fy= -robot[NFRAMES-1].ext_force[X]*s123 + robot[NFRAMES-1].ext_force[Y]*c123;

  arm_dynamics_3R(fx,fy,nz, M,V,G,F);
  //  arm_dynamics3R(M,V,G,F);
  arm_accelerations(M,V,G,F, arm_acc);

  //  printf("acc = [ %lf  %lf  %lf ]\n", 
  //    	 arm_acc[0], arm_acc[1], arm_acc[2]);

  euler_arm(arm_acc);
}

// the original 3R dynamics in the 3Rpitch code: strange behavior, breaks when
// length of L3 is altered to be the same as L1 and L2
// altered to incorporate GRAVITY, ignores endpoint (contact) forces
// ONE of these models of dynamics must be named 3R_arm_dynamics() to
//     compile into the simulator 
//void arm_dynamics_3R(fx,fy,nz, M,V,G,F)
void arm_dynamics_3R_pitch(fx,fy,nz, M,V,G,F)
double fx, fy, nz;
double M[NJOINTS][NJOINTS],V[NJOINTS], G[NJOINTS],F[NJOINTS];
{
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

  M[0][0] = 2*M1*SQR(L1) + M2*(SQR(L1)+2.0*L1*L2*c2+2*SQR(L2))
    + M3*(SQR(L1)+2.0*L1*L2*c2+2.0*L1*L3*c23*SQR(L2)+2.0*L2*L3*c3+SQR(L3));
  M[0][1] = M2*(L1*L2*c2 + 2*SQR(L2)) +
    M3*(L1*L2*c2+L1*L3*c23+SQR(L2)+2.0*L2*L3*c3+2*SQR(L3));
  M[0][2] = M3*(L1*L3*c23 + L2*L3*c3 + 2*SQR(L3));

  M[1][0] = M[0][1];
  M[1][1] = 2*M2*SQR(L2) + M3*(SQR(L2) + 2.0*L2*L3*c3 + 2*SQR(L3));
  M[1][2] = M3*(L2*L3*c3 + 2*SQR(L3));

  M[2][0] = M[0][2];
  M[2][1] = M[1][2];
  M[2][2] = 2*M3*SQR(L3);

  V[0] = -((M2+M3)*L1*L2*s2 + M3*L1*L3*s23)*SQR(robot[2].theta_dot) -
    M3*(L1*L3*s23 + L2*L3*s3)*SQR(robot[3].theta_dot) -
    2.0*((M2+M3)*L1*L2*s2+M3*L1*L3*s23)*robot[1].theta_dot*robot[2].theta_dot-
    2.0*M3*(L1*L3*s23 + L2*L3*s3)*robot[1].theta_dot*robot[3].theta_dot -
    2.0*M3*(L1*L3*s23 + L2*L3*s3)*robot[2].theta_dot*robot[3].theta_dot;

  V[1] = ((M2+M3)*L1*L2*s2 + M3*L1*L3*s23)*SQR(robot[1].theta_dot) -
    (M3*L2*L3*s3)*SQR(robot[3].theta_dot) -
    (2.0*M3*L2*L3*s3)*robot[1].theta_dot*robot[3].theta_dot -
    (2.0*M3*L2*L3*s3)*robot[2].theta_dot*robot[3].theta_dot;
              
  V[2] = M3*(L1*L3*s23 + L2*L3*s3)*SQR(robot[1].theta_dot) +
    M3*(L2*L3*s3)*SQR(robot[2].theta_dot) +
    2.0*M3*L2*L3*s3*robot[1].theta_dot*robot[2].theta_dot;
    
  G[0] = (M1*L1*c1+M2*(L1*c1+L2*c12)+M3*(L1*c1+L2*c12+L3*c123))*GRAVITY;
  G[1] = ((M2+M3)*L2*c12 + M3*L3*c123)*GRAVITY;
  G[2] = M3*L3*c123*GRAVITY;

  F[0] = F[1] = F[2] = 0.0;
}

// the version of 3R dynamics extracted from the whole-body dynamics:
// strange behavior, doesn't look right
// no GRAVITY, incorporates endpoint (contact) forces
// ONE of these models of dynamics must be named 3R_arm_dynamics() to
//     compile into the simulator 
//void arm_dynamics_3R(fx,fy,nz, M,V,G,F)
void arm_dynamics_3R_extract(fx,fy,nz, M,V,G,F)
double fx, fy, M[NJOINTS][NJOINTS],V[NJOINTS], G[NJOINTS],F[NJOINTS];
{
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

  M[0][0] = (M1+M2+M3)*SQR(L1) + (2.0*M2+M3)*SQR(L2) + 2.0*M3*SQR(L3)
    - 2.0*(M2+M3)*L1*L2*s2 - 2.0*M3*L1*L3*s23 + 2.0*M3*L2*L3*c3;
  M[0][1] =(2.0*M2+M3)*SQR(L2) + 2.0*M3*SQR(L3) - (M2+M3)*L1*L2*s2 
    - M3*L1*L3*s23 + 2.0*M3*L2*L3*c3;
  M[0][2] = 2.0*M3*SQR(L3) - M3*L1*L3*s23 + M3*L2*L3*c3;

  M[1][0] = (2.0*M2+M3)*SQR(L2) + 2.0*M3*L2*L3*c2 + 2.0*M3*SQR(L3)
    - (M2+M3)*L2*L1*s2 - M3*L3*L1*s23;
  M[1][1] = (2.0*M2+M3)*SQR(L2) + 2.0*M3*L2*L3*c2 + 2.0*M3*SQR(L3);
  M[1][2] = 2.0*M3*SQR(L3) + M3*L2*L3*c2;

  M[2][0] = 2.0*M3*SQR(L3) - M3*L1*L3*s23 + M3*L2*L3*c3;
  M[2][1] = 2.0*M3*SQR(L3) + M3*L2*L3*c3;
  M[2][2] = 2*M3*SQR(L3);


  V[0] = (-(M2+M3)*L1*L2*c2 - M3*L1*L3*c23)*SQR(robot[2].theta_dot)
    - M3*(L1*L3*c23 + L2*L3*s3)*SQR(robot[3].theta_dot)
    - 2.0*((M2+M3)*L1*L2*c2+M3*L1*L3*c23)*robot[1].theta_dot*robot[2].theta_dot
    - 2.0*M3*(L1*L3*c23 + L2*L3*s3)*robot[1].theta_dot*robot[3].theta_dot
    - 2.0*M3*(L1*L3*c23 + L2*L3*s3)*robot[2].theta_dot*robot[3].theta_dot;

  V[1] = ((M2+M3)*L1*L2*c2 + M3*L1*L3*c23)*SQR(robot[1].theta_dot) -
    (M3*L2*L3*s3)*SQR(robot[3].theta_dot) -
    (2.0*M3*L2*L3*s3)*robot[1].theta_dot*robot[3].theta_dot -
    (2.0*M3*L2*L3*s3)*robot[2].theta_dot*robot[3].theta_dot;

  V[2] = M3*(L1*L3*c23 + L2*L3*s3)*SQR(robot[1].theta_dot) +
    M3*(L2*L3*s3)*SQR(robot[2].theta_dot) +
    2.0*M3*L2*L3*s3*robot[1].theta_dot*robot[2].theta_dot;


  G[0] = G[1] = G[2] = 0.0;


  F[0] = (L1*c23 + L2*s3)*fx + (-L1*s23 + L2*c3 + L3)*fy;
  F[1] = (L2*s3)*fx + (L2*c3 + L3)*fy;
  F[2] = L3*fy;
}

// the 3R arm dynamics derived from scratch
// untested, incorporates both GRAVITY and endpoint (contact) forces
//    seems to work for no GRAVITY and no contact forces
//    seems to work for GRAVITY and no contact forces
// ONE of these models of dynamics must be named 3R_arm_dynamics() to
//     compile into the simulator 
void arm_dynamics_3R(fx,fy,nz, M,V,G,F)
//void arm_dynamics_3R_derived(fx,fy,nz, M,V,G,F)
double fx, fy, nz;
double M[NJOINTS][NJOINTS],V[NJOINTS], G[NJOINTS],F[NJOINTS];
{
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

  M[0][0] = (M1+M2+M3)*SQR(L1)
    + M3*(2.0*L1*L3*c23 + L2*L3*c3 + SQR(L3)) + (M2+M3)*L1*L2*c2;
  M[0][1] = M3*(L1*L3*c23 + L2*L3*c3 + SQR(L3)) + (M2+M3)*L1*L2*c2;
  M[0][2] = M3*(L1*L3*c23 + SQR(L3));

  M[1][0] = M3*L1*L3*c23 + 2.0*M3*L2*L3*c3 + M3*SQR(L3) 
    + (M2+M3)*(SQR(L2) + L1*L2*c2);
  M[1][1] = M3*(SQR(L2) + SQR(L3) + 2.0*L2*L3*c3) + M2*SQR(L2);
  M[1][2] = M3*(SQR(L3) + L2*L3*c3);

  M[2][0] = M3*SQR(L3) + M3*L1*L3*c23 + M3*L2*L3*c3;
  M[2][1] = M3*SQR(L3) + M3*L2*L3*c3;
  M[2][2] = M3*SQR(L3);


  V[0] = M3*L1*L3*s23*SQR(robot[1].theta_dot)
    + M3*L2*L3*s23*SQR(robot[1].theta_dot + robot[2].theta_dot)
    - (M2+M3)*L1*L2*s2*SQR(robot[1].theta_dot + robot[2].theta_dot)
    - M3*L1*L3*s23*SQR(robot[1].theta_dot + robot[2].theta_dot 
		       + robot[3].theta_dot);
  V[1] = ((M2+M3)*L1*L2*s2 + M3*L1*L3*s23)*SQR(robot[1].theta_dot)
    + M3*L2*L3*s3
       *(SQR(robot[1].theta_dot + robot[2].theta_dot) -
         SQR(robot[1].theta_dot + robot[2].theta_dot + robot[3].theta_dot));
  V[2] = M3*L1*L3*s23*SQR(robot[1].theta_dot) 
    + M3*L2*L3*SQR(robot[1].theta_dot+robot[2].theta_dot)*s3;

  G[0] = -(M1+M2+M3)*GRAVITY*L1*s1 - M3*GRAVITY*L3*s123;
  G[1] = -M2*GRAVITY*L2*s12 - M3*GRAVITY*(L2*s12 + L3*s123);
  G[2] = -M3*L3*GRAVITY*s123;

  F[0] = nz + (L1*c23 + L3)*fy + L1*s23*fx;
  F[1] = nz + (L2*c3 + L3)*fy + (L2*s3)*fx;
  F[2] = nz + L3*fy;
}

void arm_accelerations(M,V,G,F, theta_ddot)
double M[NJOINTS][NJOINTS],V[NJOINTS],G[NJOINTS],F[NJOINTS];
double theta_ddot[NJOINTS];
{
  void invert(), matXvec();
  int i, j;
  double arg[NJOINTS], Minv[NJOINTS][NJOINTS], tmp[NJOINTS][NJOINTS];

  invert(M,Minv);

  //  printf("M:\n");
  //  for (i=0;i<NJOINTS;++i) {
  //    for (j=0;j<NJOINTS;++j) {
  //      printf(" %lf", M[i][j]);
  //    }
  //    printf("\n");
  //  }
  
  //  printf("Minv:\n");
  //  for (i=0;i<NJOINTS;++i) {
  //    for (j=0;j<NJOINTS;++j) {
  //      printf(" %lf", Minv[i][j]);
  //    }
  //    printf("\n");
  //  }

  //  matXmat(M, Minv, tmp);
  //  
  //  printf("M Minv:\n");
  //  for (i=0;i<NJOINTS;++i) {
  //    for (j=0;j<NJOINTS;++j) {
  //      printf(" %lf", tmp[i][j]);
  //    }
  //
  //  printf("\n");
  //  }

  for (i=0; i<NJOINTS; ++i) {
    arg[i] = robot[i+1].torque - V[i] - G[i] - F[i];
  }

  matXvec(Minv,arg,theta_ddot);

  //  printf("torque:\n");
  //  for (i=0;i<NJOINTS;++i) {
  //    printf(" %lf", robot[i+1].torque);
  //  }
  //  printf("\n");
  //  printf("V:\n");
  //  for (i=0;i<NJOINTS;++i) {
  //    printf(" %lf", V[i]);
  //  }
  //  printf("\n");
  //  printf("G:\n");
  //  for (i=0;i<NJOINTS;++i) {
  //    printf(" %lf", G[i]);
  //  }
  //  printf("\n");
  //  printf("theta_ddot:\n");
  //  for (i=0;i<NJOINTS;++i) {
  //    printf(" %lf", theta_ddot[i]);
  //  }
  //  printf("\n");
}

void euler_arm(acc)
double acc[NJOINTS];
{
  void rectify_theta(), update_state();
  int i;
  int joint = 0;

  /* update positions and velocities of roger's manipulator */
  for (i=0;i<NFRAMES;++i) {
    if (robot[i].dof_type != NOTYPE) {
      if (robot[i].dof_type == REVOLUTE) {
	robot[i].theta += 0.5*acc[joint]*DT*DT + robot[i].theta_dot*DT;
	robot[i].theta_dot += acc[joint]*DT;
	++joint;
      }
      else if (robot[i].dof_type == PRISMATIC) {
	robot[i].theta += 0.5*acc[joint]*DT*DT + robot[i].theta_dot*DT;
	robot[i].theta_dot += acc[joint]*DT;
	if (robot[i].theta < 0.0) { 
	  robot[i].theta = 0.0; robot[i].theta_dot = 0.0;
	}
	if (robot[i].theta > l[joint+1]) {
	  robot[i].theta = l[joint+1];
	  robot[i].theta_dot = 0.0;
	}
	++joint;
      }
    }
  }
  rectify_theta();
  update_state();
}

void update_state()
{
  int i;

  /* update transforms that describe configuration of robot */
  for(i=1;i<NFRAMES;++i) {
    switch(robot[i].axis) {
      case(XAXIS):   /* dof axis is local x axis */
	if (robot[i].dof_type == REVOLUTE) {
	  robot[i].iTj[1][1] =  cos(robot[i].theta);
	  robot[i].iTj[1][2] = -sin(robot[i].theta);
	  robot[i].iTj[2][1] = -robot[i].iTj[1][2];
	  robot[i].iTj[2][2] = 	robot[i].iTj[1][1];
	}
	else if (robot[i].dof_type == PRISMATIC) {
	  robot[i].iTj[0][3] = robot[i].theta;
	}
      break;

      case(YAXIS):   /* dof axis is local y axis */
	if (robot[i].dof_type == REVOLUTE) {
	  robot[i].iTj[0][0] = cos(robot[i].theta);
	  robot[i].iTj[0][2] = sin(robot[i].theta);
	  robot[i].iTj[2][0] = -robot[i].iTj[0][2];
	  robot[i].iTj[2][2] =  robot[i].iTj[0][0];
	}
	else if (robot[i].dof_type == PRISMATIC) {
	  robot[i].iTj[1][3] = robot[i].theta;
	}
      break;

      case(ZAXIS):   /* dof axis is local z axis */
	if (robot[i].dof_type == REVOLUTE) {
	  robot[i].iTj[0][0] = cos(robot[i].theta);
	  robot[i].iTj[0][1] = -sin(robot[i].theta);
	  robot[i].iTj[1][0] = -robot[i].iTj[0][1];
	  robot[i].iTj[1][1] =  robot[i].iTj[0][0];
	}
	else if (robot[i].dof_type == PRISMATIC) {
	  robot[i].iTj[2][3] = l[i-1] + robot[i].theta;
	}
      break;
    }
  }
}

void rectify_theta()
{
  int i;

  for (i=1;i<=NJOINTS;++i) {
    while (robot[i].theta < (-M_PI)) {
      robot[i].theta += 2.0*M_PI;
      // printf("\n+ %lf +\n", robot[i].theta);

    }
    while (robot[i].theta >= M_PI) {
      robot[i].theta -= 2.0*M_PI;
      // printf("\n- %lf -\n", robot[i].theta);
    }
  }
}

void invert(A,Ainv)
double A[NJOINTS][NJOINTS], Ainv[NJOINTS][NJOINTS];
{
   double det;

   switch(NJOINTS) {
   case (1):
     Ainv[0][0] = 1.0/A[0][0];
     break;
   case (2):
     det = 1.0 / (A[0][0]*A[1][1] - A[1][0]*A[0][1]);
      
     Ainv[0][0] = det * A[1][1];
     Ainv[1][0] = -1.0 * det * A[1][0];
     Ainv[0][1] = -1.0 * det * A[0][1];
     Ainv[1][1] = det * A[0][0];
     break;
   
   case(3):
     det = 1.0 / (A[0][0]*A[1][1]*A[2][2] + A[0][1]*A[1][2]*A[2][0] +
		  A[0][2]*A[1][0]*A[2][1] - A[2][0]*A[1][1]*A[0][2] -
		  A[2][1]*A[1][2]*A[0][0] - A[2][2]*A[1][0]*A[0][1]) ;
     
     Ainv[0][0] = det * (A[1][1]*A[2][2]-A[2][1]*A[1][2]);
     Ainv[0][1] = det * (A[2][1]*A[0][2]-A[0][1]*A[2][2]);
     Ainv[0][2] = det * (A[0][1]*A[1][2]-A[1][1]*A[0][2]);
     Ainv[1][0] = det * (A[2][0]*A[1][2]-A[1][0]*A[2][2]);
     Ainv[1][1] = det * (A[0][0]*A[2][2]-A[2][0]*A[0][2]);
     Ainv[1][2] = det * (A[1][0]*A[0][2]-A[0][0]*A[1][2]);
     Ainv[2][0] = det * (A[1][0]*A[2][1]-A[2][0]*A[1][1]);
     Ainv[2][1] = det * (A[2][0]*A[0][1]-A[0][0]*A[2][1]);
     Ainv[2][2] = det * (A[0][0]*A[1][1]-A[1][0]*A[0][1]);
     break;
   }
 }

void matXvec(A,x,y)
double A[NJOINTS][NJOINTS],x[NJOINTS],y[NJOINTS];
{
  int i,j;
  for (i=0; i<NJOINTS; ++i) {
    y[i] = 0.0;
    for (j=0; j<NJOINTS; ++j) {
      y[i] += A[i][j] * x[j];
    }
  }
}

/*********************************************************************/
void matXmat(t1,t2,result)
double t1[NJOINTS][NJOINTS],t2[NJOINTS][NJOINTS],result[NJOINTS][NJOINTS];
{
  int i,j,k;
  for (i=0; i<NJOINTS; ++i) {
    for (j=0; j<NJOINTS; ++j) {
      result[i][j] = 0.0;
      for (k=0; k<NJOINTS; ++k) {
	result[i][j] += t1[i][k] * t2[k][j];
      }
    }
  }
}

