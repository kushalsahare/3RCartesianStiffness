/**************************************************************************/
/* File:        3Rarm.h                                                   */
/* Description: compile time constants for 1R robot pitcher               */
/* Author:      Rod Grupen                                                */
/* Date:        2-28-2016                                                 */
/* Edited : Kushal Sahare                                                 */
/* Date: 3-15-2017                                                        */

/**************************************************************************/

/********************* simulator X display **********************/
// simulator panel
#define WIDTH             580
#define HEIGHT	          580

// button geometry
#define BOXH	           18
#define BOXW	           ((int) WIDTH/4 - 4)

// sub-panel geometry and placement
#define CENTER_X          290
#define CENTER_Y          290


// #define SCALE              60.0
#define SCALE             350.0

/* world to pixmap panels (drawing) transformations */
#define W2DR(num)          (((int)(SCALE*1000.0*(num))/1000))
#define W2DX(num)	   (CENTER_X+((int)(SCALE*1000.0*((num)))/1000))
#define W2DY(num)	   (CENTER_Y-((int)(SCALE*1000.0*((num)))/1000))
/* pixmap to world */
#define D2WR(num)	   ((double)(num*1000)/(SCALE*1000.0))
#define D2WX(num)	   ((double)((num-CENTER_X)*1000)/(SCALE*1000.0))
#define D2WY(num)	   ((double)((CENTER_Y-num)*1000)/(SCALE*1000.0))

/********************* simulation constants **********************/
#define TIMEOUT            60        /* seconds worth of simulation       */
#define DT                  0.0005   /* the time increment between frames */
#define RENDER_RATE         5        /* render every fourth state         */
#define SERVO_RATE          1        /* servo rate at 1000Hz (1 msec)     */
#define TIMER_UPDATE        5
#define GRAVITY             9.8

#define NOFILL	            0
#define FILL                1

#define FREEFALL            0
#define PD_CONTROL          1
#define THROW               2
#define STIFFNESS_CONTROL   3
#define TORQUE_MAX         20.0

// Robot
#define NARMS           1
#define NLINKS          3
#define NJOINTS         3
#define NFRAMES         5
#define R_JOINT         0.015  /* the radius of a joint */
#define R_ENDPT         0.05 /*0.0450*/
#define L1              0.2020  /* the length of link 1 */ 
#define L2              0.2020  /* the length of link 2 */ 
#define L3              0.1010    /* the length of link 3 */ 
#define M1              1.0
#define M2              1.0
#define M3              0.8
#define THETA1_HOME          (M_PI/4.0)-0.4 //0.0          //
#define THETA1_REF           (M_PI/4.0)     //(M_PI/2.0)    //
#define THETA2_HOME          (M_PI/4.0)-0.4 // 0.0          // 
#define THETA2_REF           (M_PI/4.0)     //(5.0*M_PI/8.0)// 
#define THETA3_HOME          (M_PI/5.0)-0.4//0.0            //
#define THETA3_REF           (M_PI/5.0)    //(M_PI/2.0)    //

#define RELEASE_POINT        (M_PI/4.0)

#define MIN_X     -0.85
#define MIN_Y     -0.85
#define MAX_X     0.85
#define MAX_Y     0.85

// Baseball
#define R_OBJ               0.0508 /* m - regulation baseball */
#define M_OBJECT            0.1421 /* kg - regulation baseball */

//Wall

#define H_WALL       0.25 /* m  - height of the wall */
#define W_WALL       0.25 /* m  - width of the wall */
#define M_WALL       0.50 /* kg - mass of the wall */

#define K_COLLIDE    55000.0
#define B_COLLIDE      5.5

#define X                   0
#define Y                   1
#define XDOT                2
#define YDOT                3
#define TRUE                1
#define FALSE               0

/***** INVERSE KINEMATICS **********************************************/
#define OUT_OF_REACH       -1
#define IN_REACH            1

/***** PD CONTROL ******************************************************/
//#define KP_ARM           9000.0
//#define KD_ARM            200.0
#define KP_ARM           350.0
#define KD_ARM            20.0

#define SGN(x)   (((x) > 0) ? (1.0) : (-1.0))
#define SQR(x)   (((x)*(x)))
#define MIN(x,y) (((x)<(y)) ? (x) : (y))
#define MAX(x,y) (((x)>(y)) ? (x) : (y))

/***********************************************************************/
/* CONSTANTS AND STRUCTURES THAT DEFINE MECHANISMS FOR THE SIMULATOR   */
#define NOTYPE             -1        /* KINEMATIC SPECIFICATIONS       */
#define NOAXIS             -1
#define REVOLUTE            0
#define PRISMATIC           1
#define XAXIS               0
#define YAXIS               1
#define ZAXIS               2

/*typedef struct _arm {
  double iTj[4][4];
  int dof_type;     // revolute or prismatic type
  int axis;         // XAXIS, YAXIS, ZAXIS 
  double theta;
  double theta_dot;
  double theta_ref; // for the dof associated with this link 
  double theta_dot_ref;
  double torque;
  double ext_force[3]; // (fx,fy) force on distal endpoint of this link 
} Arm;
*/
/*typedef struct _obj {
  double mass;                 // intrinsic parameters 
  double position[2];          // position of the centroid of the object
  double velocity[2];          // velocity of the centroid of the object
  double ext_force[2];         // written by the simulator: endpoint load
} Obj;*/
