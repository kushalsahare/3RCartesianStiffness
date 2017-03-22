/**************************************************************************/
/* File:        arm.h                                                     */
/* Description:  arm structure for robot                                  */
/* Author:      Rod Grupen                                                */
/* Date:        2-28-2016                                                 */
/* Edited :     Kushal Sahare                                             */
/* Date:        3-15-2017                                                 */
/**************************************************************************/


#ifndef ARM_H_
#define ARM_H_

typedef struct _arm {
  double iTj[4][4];
  int dof_type;     /* revolute or prismatic type */
  int axis;         /* XAXIS, YAXIS, ZAXIS */
  double theta;
  double theta_dot;
  double theta_ref; /* for the dof associated with this link */
  double theta_dot_ref;
  double torque;
  double ext_force[3]; /* (fx,fy) force on distal endpoint of this link */
} Arm;

#endif //ARM_H_
