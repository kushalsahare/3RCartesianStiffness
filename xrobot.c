/****************************************************************************/
/** xrobot.c: simulates Cartesian Stiffness control in the planar 3R robot **/
/** author:   Grupen                                                       **/
/** date:     February 2017                                                **/
/****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h> /* For INT_MAX */
#include <float.h>  /* For DBL_MAX */
#include "Xkw/Xkw.h"

#include "3Rarm.h"
#include "arm.h"
#include "wall.h"
#include "object.h"

#define DEBUG 1

extern Arm robot[NFRAMES];
extern Obj object;
extern Wall wall; // wall 
extern int mode;

double clock=0.0;

double theta_home[3] = {THETA1_HOME, THETA2_HOME, THETA3_HOME};

void  x_canvas_proc(), x_start_proc(), x_reset_proc(), x_mode_proc();
int   x_quit_proc();
void  x_Slider_proc(), x_timer_proc();

Display   *display;
Window    window;
Pixmap    pixmap;
XtAppContext  app_con;
GC    gc;
int   screen;
Widget  canvas_w, reset_w, mode_w, start_w, popup_w = NULL;
XtIntervalId    timer = 0;
int             width = WIDTH, height = HEIGHT, depth;
unsigned long   foreground, background;

int arm_color, object_color, wall_color, force_color, grey[100];

int main(argc,argv)
    int argc;
    char **argv;
{
    int i;
    void x_clear(), x_init_colors(), update_state(), simulate_arm();
    void simulate_object(), simulate_wall(), draw_all(), print_help();
    char string[] = "Hello World" ;

    static String fallback_resources[]={
        "*title:  RRR Cartesian Stiffness Simulator",
        "*Roger-Pitching*x: 100",
        "*Roger-Pitching*y: 100",
        NULL,
    };
    Widget toplevel, form, widget;

    toplevel = XtAppInitialize(&app_con, "Roger-Pitching", NULL, ZERO,
            &argc, argv, fallback_resources, NULL, ZERO);
    form = XkwMakeForm(toplevel);
    widget = NULL;
    start_w = widget = XkwMakeCommand(form, NULL, widget, x_start_proc,
            "Start", BOXW, BOXH);
    reset_w = widget = XkwMakeCommand(form, NULL, widget, x_reset_proc,
            "Reset", BOXW, BOXH);
    mode_w = widget = XkwMakeCommand(form, NULL, widget, x_mode_proc,
            "FREEFALL", BOXW, BOXH);
    widget = XkwMakeCommand(form, NULL, widget, x_quit_proc,
            "Quit", BOXW, BOXH);


    canvas_w = widget = XkwMakeCanvas(form, widget, NULL,
            x_canvas_proc, width, height);

    XtRealizeWidget(toplevel);
    display = XtDisplay(canvas_w);
    window = XtWindow(canvas_w);
    screen = DefaultScreen(display);
    depth = DefaultDepth(display, screen);
    foreground = BlackPixel(display, screen);
    background = WhitePixel(display, screen);

    gc = XCreateGC(display, window, 0, NULL);
    XSetFunction(display, gc, GXcopy);
    XSetForeground(display, gc, foreground);
    XSetBackground(display, gc, background);

    pixmap = XCreatePixmap(display, window, width, height, depth);
    x_clear();

    x_init_colors();

    update_state();
    simulate_arm();
    simulate_object();
    simulate_wall();
    

    XDrawString(display, window, gc, 0, 0, string , strlen(string)); 
    //  printf("theta = [ %lf  %lf  %lf ]\n", 
    //       robot[1].theta, robot[2].theta, robot[3].theta);

    draw_all();

    print_help();

    XtAppMainLoop(app_con);
}

void print_help()
{
    printf("with mouse in Roger's Cartesian window:\n");
    printf("     \"h\" --- help (this menu)\n");
    printf("     \"c\" --- clear graphics\n");
    printf("     \"t\" --- check contact sensors\n");
    printf("     \"q\" --- quit\n\n");
}

void x_init_colors()
{
    int i;
    char color_name[128];

    //  printf("initializing grey scale colors..."); fflush(stdout);
    for(i=0; i<100; i++) { // 0 => black; 100 => white
        sprintf(color_name, "grey%d", i);
        grey[i] = XkwParseColor(display, color_name);
    }
    object_color = XkwParseColor(display, "red");
    wall_color =XkwParseColor(display, "grey");
    force_color = XkwParseColor(display, "green");
    arm_color = XkwParseColor(display, "blue");
}

/***********************************************/

void x_start_proc(w, client_data, call_data)
    Widget w;
    XtPointer client_data, call_data;
{
    if(!timer) {
        XkwSetWidgetLabel(start_w, "Stop");
        timer = XtAppAddTimeOut(app_con, TIMER_UPDATE,
                x_timer_proc, (XtPointer) NULL);
    }
    else {
        XkwSetWidgetLabel(start_w, "Start");
        XtRemoveTimeOut(timer);
        timer = 0;
    }
}

int change_input_mode()
{
   static int input_mode=0;

   input_mode = (input_mode + 1) % N_INPUT_MODES;
   //init_input_flag = TRUE;
   return (input_mode);
}


/*************************************************/

void x_mode_proc(w, client_data, call_data)
    Widget w;
    XtPointer client_data, call_data;
{ 
  /*   if (mode==FREEFALL) {
        mode = PD_CONTROL;
        XkwSetWidgetLabel(mode_w, "PD_CONTROL");
    }
    else if (mode==PD_CONTROL) {
        mode = FREEFALL;
        XkwSetWidgetLabel(mode_w, "FREEFALL");
    }
*/

mode = change_input_mode();
printf("Printing Input mode = %d\n" , mode);

switch (mode) {
     case FREEFALL:
       XkwSetWidgetLabel(mode_w, "FREEFALL"); break;
     case PD_CONTROL:
       XkwSetWidgetLabel(mode_w, "PD_CONTROL"); break;
     case STIFFNESS_CONTROL:
       XkwSetWidgetLabel(mode_w, "STIFFNESS_CONTROL"); break;
     //case TOY_INPUT:
     //  XkwSetWidgetLabel(input_mode_w, "Input: Ball position"); break;
     //case MAP_INPUT:
     //  XkwSetWidgetLabel(input_mode_w, "Input: Map Editor"); break;
     default: break;
}


}
/*************************************************/

void x_reset_proc(w, client_data, call_data)
    Widget w;
    XtPointer client_data, call_data;
{ 
    void update_state();
    int i;
    double c1, s1, c12, s12, c123, s123;
    double A = 0.1;
    double omega = 1;

    for (i=0;i<NJOINTS; ++i) {
        robot[i+1].theta = theta_home[i];
        robot[i+1].theta_dot = 0.0;
    }
    update_state();

    c1 = cos(robot[1].theta);
    s1 = sin(robot[1].theta);
    s12 = sin(robot[1].theta + robot[2].theta);
    c12 = cos(robot[1].theta + robot[2].theta); 
    s123 = sin(robot[1].theta + robot[2].theta + robot[3].theta);
    c123 = cos(robot[1].theta + robot[2].theta + robot[3].theta);

    //object.position[X] = 0.2;//L1*c1+L2*c12+L3*c123;
    //object.position[Y] = 0.0; //L1*s1+L2*s12+L3*c123;
    //object.velocity[X] = 0.0;
    //object.velocity[Y] = 0.0;
    wall.centroid[X] = 0.024108;//(for link 3 = 0.0101 --> 0.083469; //0.0
    wall.centroid[Y] = 0.508257;//(for link 3 = --//-- --> 0.426546; //0.4

    wall.vertices[0][X] = wall.centroid[X] - W_WALL*0.5;
    wall.vertices[0][Y] = wall.centroid[Y] + H_WALL*0.5;
    
    wall.vertices[1][X] = wall.centroid[X] + W_WALL*0.5;
    wall.vertices[1][Y] = wall.centroid[Y] + H_WALL*0.5;

    wall.vertices[2][X] = wall.centroid[X] + W_WALL*0.5;
    wall.vertices[2][Y] = wall.centroid[Y] - H_WALL*0.5;

    wall.vertices[3][X] = wall.centroid[X] - W_WALL*0.5;
    wall.vertices[3][Y] = wall.centroid[Y] - H_WALL*0.5;

    wall.velocity[X] = 0.0;
    wall.velocity[Y] = 0.0;

}
/*************************************************/

int x_quit_proc(w, client_data, call_data)
    Widget w;
    XtPointer call_data, client_data;
{
    XFreePixmap(display, pixmap);
    XFreeGC(display, gc);
    XtDestroyApplicationContext(app_con);
    exit(0);
}
/*************************************************/

void x_canvas_proc(w, client_data, call_data)
    Widget w;
    XtPointer call_data, client_data;
{
    void x_expose(), print_help(), x_clear(),x_expose();
    int x_quit_proc();

    XEvent *event = ((XEvent *) call_data);
    char text[10];
    KeySym key;
    double x,y,theta1,theta2;
    int c;
    static Dimension nwidth = WIDTH, nheight = HEIGHT;

    switch(event->type) {

        case ConfigureNotify:
            nwidth = event->xconfigure.width;
            nheight = event->xconfigure.height;
            break;

        case Expose:
            if(nwidth == width && nheight == height)
                x_expose();
            else { 
                /* width = nwidth; height = nheight; */
                x_expose();
            }
            break;

        case ButtonPress:
            break;

        case ButtonRelease:
            break;

        case KeyPress:
            c = XLookupString((XKeyEvent *) event, text, 10, &key, 0);
            if (c == 1) switch (text[0]) {

                case 'h':
                    print_help();
                    break;

                case 'c':
                    x_clear();
                    x_expose();
                    break;

                case 't':
                    break;

                case 'q':
                    x_quit_proc(w, client_data, call_data);
            }
    }
}
/*************************************************/

void x_expose()
{
    XCopyArea(display, pixmap, window, gc, 0, 0, width, height, 0, 0);
}
/*************************************************/

void x_clear()
{
    XSetForeground(display, gc, background);
    XFillRectangle(display, pixmap, gc, 0, 0, width, height);
}

void x_timer_proc(w, client_data, call_data)
    Widget w;
    XtPointer client_data, call_data;
{
    void control_arm(), simulate_arm(), simulate_wall(), draw_all(); //simulate_object(), 
    void compute_contact_forces();
    double x_endpt,y_endpt,x_wall,y_wall;
    double d;
    static int render=RENDER_RATE, servo=SERVO_RATE;

    //  printf("1"); fflush(stdout);
    if (servo++ == SERVO_RATE) {
        control_arm(clock);
        servo = 1;
    }

    compute_contact_forces();

    //  printf("2"); fflush(stdout);
    simulate_arm();
     // printf("2a"); fflush(stdout);  
    //simulate_object();
    //  printf("3"); fflush(stdout);
    simulate_wall();
    //    printf("3"); fflush(stdout);
    if (render++ == RENDER_RATE) {
        draw_all();
        render = 1;
    }
    clock += DT;
    //  printf("4"); fflush(stdout);

    timer = XtAppAddTimeOut(app_con, TIMER_UPDATE,
            x_timer_proc, (XtPointer) NULL);
}


/*****************************************************************/
double dist_to_seg(double x1, double y1, double x2, double y2, double pointX, double pointY)
{
    double diffX = x2 - x1;
    float diffY = y2 - y1;
    if ((diffX == 0) && (diffY == 0))
    {
        diffX = pointX - x1;
        diffY = pointY - y1;
        return sqrt(diffX * diffX + diffY * diffY);
    }

    float t = ((pointX - x1) * diffX + (pointY - y1) * diffY) / (diffX * diffX + diffY * diffY);

    if (t < 0)
    {
        //point is nearest to the first point i.e x1 and y1
        diffX = pointX - x1;
        diffY = pointY - y1;
    }
    else if (t > 1)
    {
        //point is nearest to the end point i.e x2 and y2
        diffX = pointX - x2;
        diffY = pointY - y2;
    }
    else
    {
        //if perpendicular line intersect the line segment.
        diffX = pointX - (x1 + t * diffX);
        diffY = pointY - (y1 + t * diffY);
    }

    //returning shortest distance
    return sqrt(diffX * diffX + diffY * diffY);
}
/*************************************************/

// compute object.ext_force[2], and robot[NRAMES-1].ext_force[3]
// due to a possible collision between them
void compute_contact_forces()
{
    double x_endpt, y_endpt, x_wall, y_wall, dx, dy, mag, d, x_ball , y_ball ;//
    circle circ;
    rect w;
    double f[2] = {0.0, 0.0};
    double dist[4];
    int i;
    int ii, jj;

    char fx[4];
    char fy[4];

    x_endpt = L1*cos(robot[1].theta) + L2*cos(robot[1].theta + robot[2].theta)
        + L3*cos(robot[1].theta + robot[2].theta + robot[3].theta);
    y_endpt = L1*sin(robot[1].theta) + L2*sin(robot[1].theta + robot[2].theta)
        + L3*sin(robot[1].theta + robot[2].theta + robot[3].theta);
    
   // printf(" Hand: %f %f\n", x_endpt, y_endpt);
    // save the data in circle struct
    circ.r = R_ENDPT;
    circ.c[X] = x_endpt;
    circ.c[Y] = y_endpt;
    // rect structure
    for( ii=0; ii < 4 ; ii ++){
        for(jj =0 ; jj < 2 ; jj++){

            w[ii][jj] = wall.vertices[ii][jj];
           // printf("%f ", r[ii][jj]);
        }
        //printf("\n");
    }
        wall.ext_force[X]=wall.ext_force[Y] = 0.0;
        robot[NFRAMES-1].ext_force[0]
            = robot[NFRAMES-1].ext_force[1]
            = robot[NFRAMES-1].ext_force[2] = 0.0;


        intersect_circle_poly(circ, w, f); // find the intersection points

        // #ifdef DEBUG
        // printf("%f %f \n" , f[X], f[Y]);
        //XDrawPoint(display, pixmap, gc, W2DX(0.1), W2DY(0.1));
        //XDrawLine(display, pixmap, gc,
        //        W2DX(0.1), W2DY(0.1), W2DX(0.2), W2DY(0.2));

        wall.ext_force[X] = f[X];
        wall.ext_force[Y] = f[Y];

        // force from link NFRAMES-1 on link NFRAMES 
        robot[NFRAMES-1].ext_force[X] = wall.ext_force[X];
        robot[NFRAMES-1].ext_force[Y] = wall.ext_force[Y];
    }
    /*************************************************/



    void draw_all()
    {
        void x_clear(), draw_wall(), draw_robot(), x_expose(), draw_object();
        x_clear();

        // draw_object(); 
        draw_robot();
        draw_wall();
        // draw_ellipsoid();
        // draw_ruler();
        x_expose();
    }
    /*************************************************/

    void draw_circle(cu, cv, r, fill)
        int cu, cv, r, fill;
    {
        if(fill == NOFILL)
            XDrawArc(display, pixmap, gc, cu-r, cv-r, 2*r, 2*r, 0, 64*360);
        else
            XFillArc(display, pixmap, gc, cu-r, cv-r, 2*r, 2*r, 0, 64*360);
    }
    /*************************************************/

    void draw_rect(xu, xv, w, h)
        int xu, xv, w, h;
    {
        XFillRectangle(display, pixmap, gc, xu, xv, w, h);
        //XDrawRectangle(display, pixmap, gc, xu, xv, w, h);

    }
    /*************************************************/

    void draw_frame()
    {
    #define FRAME_L 0.08
    #define FRAME_T 0.09

        XSetForeground(display,gc,foreground);

        /* x-axis */
        XDrawLine(display, pixmap, gc,
                W2DX(0.0), W2DY(0.0), W2DX(FRAME_L), W2DY(0.0));
        XDrawString(display, pixmap, gc,
                W2DX(FRAME_T), W2DY(0.0), "x", 1);

        /* y-axis */
        XDrawLine(display, pixmap, gc,
                W2DX(0.0), W2DY(0.0), W2DX(0.0), W2DY(FRAME_L)); 
        XDrawString(display, pixmap, gc,
                W2DX(0.0), W2DY(FRAME_T), "y", 1);

#undef FRAME_L // 0.04
#undef FRAME_T // 0.045
    }
    /*************************************************/

    void draw_object()
    {
        XSetForeground(display,gc,object_color);
        draw_circle(W2DX(object.position[X]), W2DY(object.position[Y]),
                W2DR(R_OBJ), FILL);
    }
    /*************************************************/

    void draw_wall()
    {
        XSetForeground(display,gc,wall_color);
        draw_rect(W2DX(wall.vertices[0][X]), W2DY(wall.vertices[0][Y]),
                W2DR(W_WALL), W2DR(H_WALL));
    }


    void draw_robot()
    {
        void copy_matrix4D(), matXmat4D(), draw_frame(), draw_circle();
        int i, j;
        double theta1, theta2, mag;
        double temp0[4][4], temp1[4][4];
        double x_endpt, y_endpt; 
        int magnitude;
        double u[2];

        x_endpt = L1*cos(robot[1].theta) + L2*cos(robot[1].theta + robot[2].theta)
            + L3*cos(robot[1].theta + robot[2].theta + robot[3].theta);
        y_endpt = L1*sin(robot[1].theta) + L2*sin(robot[1].theta + robot[2].theta)
            + L3*sin(robot[1].theta + robot[2].theta + robot[3].theta);

        XSetForeground(display,gc,foreground);
        draw_frame();

        XSetForeground(display,gc,arm_color);

        copy_matrix4D(robot[0].iTj, temp0);
        for(i=1; i<NFRAMES; i++) {
            matXmat4D(temp0, robot[i].iTj, temp1);
            XDrawLine(display, pixmap, gc,
                    W2DX(temp0[0][3]), W2DY(temp0[1][3]),
                    W2DX(temp1[0][3]), W2DY(temp1[1][3]));
            if (i<(NFRAMES-1))
                draw_circle(W2DX(temp1[0][3]),W2DY(temp1[1][3]),W2DR(R_JOINT), FILL);
            else {
                XSetForeground(display,gc,object_color);
                draw_circle(W2DX(temp1[0][3]), W2DY(temp1[1][3]),W2DR(R_ENDPT), FILL);
                if(robot[NFRAMES-1].ext_force[X] || robot[NFRAMES-1].ext_force[Y]){
                   
                    magnitude = sqrt(robot[NFRAMES-1].ext_force[X]*robot[NFRAMES-1].ext_force[X] + 
                            robot[NFRAMES-1].ext_force[Y]*robot[NFRAMES-1].ext_force[Y]) ;
                    u[X] = robot[NFRAMES-1].ext_force[X] / magnitude;
                    u[Y] = robot[NFRAMES-1].ext_force[Y] / magnitude;

                    XSetForeground(display,gc, force_color);
                    XDrawLine(display, pixmap, gc,
                            W2DX(x_endpt), W2DY(y_endpt),
                            W2DY( (x_endpt - u[X] ) ), W2DY((y_endpt - u[Y] ) ) );
                }
            }
            copy_matrix4D(temp1, temp0);
        }
    }

/****************************************************************************************/

    void draw_ellipsoid()
    {
        void arm_dynamics_3R(), matmult2x2(),transpose2x2(), invert2x2();
        void matxvec2(), draw_circle();

        double M[2][2], V[2], G[2], F[2];
        double J[2][3], Jt[3][2], Msquared[3][3], M2inv[3][3], tmp[3][3], JMJt[2][2];
        double Minv[3][3], grav_bias[2], vel_bias[2];
        double a,b,c, root[2], dx, dy, mag, span, ex, ey;
        double eigenvalue[2], eigenvector[2][2];
        double t1, t2, t3, v1, v2, v3;
        double posx, posy, theta, dx0, dy0, dx1, dy1;
        double fx=0, fy=0, nz=0;

        t1=robot[1].theta; t2=robot[2].theta; t3=robot[3].theta;
        v1=robot[1].theta_dot; v2 = robot[2].theta_dot; v3 = robot[3].theta_dot;

        J[0][0] = -L1*sin(t1)-L2*sin(t1+t2)-L3*sin(t1+t2+t3);
        J[0][1] = -L2*sin(t1+t2)-L3*sin(t1+t2+t3);
        J[0][2] = -L3*sin(t1+t2+t3);

        J[1][0] =  L1*cos(t1)+L2*cos(t1+t2)+L3*cos(t1+t2+t3);
        J[1][1] =  L2*cos(t1+t2)+L3*cos(t1+t2+t3);
        J[1][2] =  L3*cos(t1+t2+t3);

        arm_dynamics_3R(fx,fy,nz, M,V,G,F);

        //      matcopy(M, tmp);
        matmult2x2(M, M, Msquared);
        invert2x2(Msquared, M2inv);

        transpose2x2(J,Jt);
        matmult2x2(M2inv, Jt, tmp);
        matmult2x2(J, tmp, JMJt);

        // cov = [A  B] => det |JJt| = a(lambda)^2 + b(lambda) +c
        //       [B  C]
        a = 1.0;
        b = -(JMJt[0][0] + JMJt[1][1]);
        c = JMJt[0][0]*JMJt[1][1] - JMJt[1][0]*JMJt[0][1];

        //  printf("a = %lf b = %lf c = %lf\n", a,b,c);

        root[0] = (-b + sqrt(SQR(b)-4.0*a*c)) / (2.0*a);
        root[1] = (-b - sqrt(SQR(b)-4.0*a*c)) / (2.0*a);

        // the eigenvector for root 0
        dy = 1.0;
        dx = -JMJt[0][1]/(JMJt[0][0] - root[0]);
        mag = sqrt(SQR(dx)+SQR(dy));
        eigenvalue[0] = sqrt(root[0]);
        eigenvector[0][0] = dx/mag; eigenvector[0][1] = dy/mag;
        printf("     e0 = %6.4lf: ", eigenvalue[0]);
        printf("e_vec=(%6.4lf %6.4lf)\n", eigenvector[0][0], eigenvector[0][1] );

        // the eigenvector for root 1
        dy = 1.0;
        dx = -JMJt[0][1]/(JMJt[0][0] - root[1]);
        mag = sqrt(SQR(dx)+SQR(dy));
        eigenvalue[1] = sqrt(root[1]);
        eigenvector[1][0] = dx/mag; eigenvector[1][1] = dy/mag;
        printf("     e1 = %6.4lf: ", eigenvalue[1]);
        printf("e_vec=(%lf %lf)\n", eigenvector[1][0], eigenvector[1][1] );

        // the bias terms
        invert2x2(M,Minv);              // first term
        matmult2x2(J, Minv, tmp); // tmp contains J M^{-1}

    // gravity bias  -J M^{-1} G
    matxvec2(tmp, G, grav_bias);  grav_bias[0] *= -1.0; grav_bias[1] *= -1.0;
    printf("     gravity bias=(%lf %lf)\n", grav_bias[0], grav_bias[1]);

    // velocity bias  -J M^{-1} V + d/dt(J} qdot
    matxvec2(tmp, V, vel_bias); vel_bias[0] *= -1.0; vel_bias[1] *= -1.0;

    // second term p. 22 Nakamura
    vel_bias[0] += (-L1*cos(t1)-L2*cos(t1+t2))*SQR(v1) -
        (L2*cos(t1+t2)*(2.0*v1*v2 + SQR(v2)));
    vel_bias[1] += (-L1*sin(t1)-L2*sin(t1+t2))*SQR(v1) -
        (L2*sin(t1+t2)*(2.0*v1*v2 + SQR(v2)));

    printf("     velocity bias=(%lf %lf)\n", vel_bias[0],vel_bias[1]);

    // phi=atan2((s[1]*eigenvalue[1]),(s[0]*eigenvalue[0]));
    // ex = eigenvalue[0]*cos(phi)*eigenvector[0][X] + 
    //      eigenvalue[1]*sin(phi)*eigenvector[1][Y];
    // ey = eigenvalue[0]*cos(phi)*eigenvector[0][Y] + 
    //      eigenvalue[1]*sin(phi)*eigenvector[1][Y];

    // direction span of ellipsoid
    // span = sqrt(SQR(ex)+SQR(ey));
    // printf("span in direction (%6.4lf,%6.4lf): %6.4lf\n\n", 
    //         s[0],s[1],span);

    // DRAW THE ELLIPSE AT THE VEL OFFSET FROM THE CURRENT ENDPOINT POSITION
    posx = L1*cos(t1) + L2*cos(t1+t2) + L3*cos(t1+t2+t3);
    //  posx += grav_bias[0];
    posy = L1*sin(t1) + L2*sin(t1+t2) + L3*sin(t1+t2+t3);
    //  posy += grav_bias[1];
    draw_circle(W2DX(posx), W2DY(posy), W2DR(R_JOINT), NOFILL);

    dx0 = eigenvalue[0]*eigenvector[0][X];
    dy0 = eigenvalue[0]*eigenvector[0][Y];
    for (theta=M_PI/20.0; theta<2*M_PI; theta += M_PI/20.0) {
        dx1 = (eigenvalue[0]*cos(theta))*eigenvector[0][X] +
            (eigenvalue[1]*sin(theta))*eigenvector[1][X];
        dy1 = (eigenvalue[0]*cos(theta))*eigenvector[0][Y] +
            (eigenvalue[1]*sin(theta))*eigenvector[1][Y];
        XDrawLine(display, pixmap, gc,
                W2DX((posx + dx0)), 
                W2DY((posy + dy0)), 
                W2DX((posx + dx1)), 
                W2DY((posy + dy1)));
        dx0 = dx1; dy0 = dy1;
    }
}

void matmult2x2(A1, A2, ANS)
    double A1[2][2], A2[2][2], ANS[2][2];
{
    ANS[0][0] = A1[0][0]*A2[0][0] + A1[0][1]*A2[1][0];
    ANS[0][1] = A1[0][0]*A2[0][1] + A1[0][1]*A2[1][1];

    ANS[1][0] = A1[1][0]*A2[0][0] + A1[1][1]*A2[1][0];
    ANS[1][1] = A1[1][0]*A2[0][1] + A1[1][1]*A2[1][1];
}

void matxvec2(A, in, out)
    double A[2][2],in[2],out[2];
{
    out[0] = A[0][0]*in[0] + A[0][1]*in[1];
    out[1] = A[1][0]*in[0] + A[1][1]*in[1];
}

void invert2x2(A,Ainv)
    double A[2][2], Ainv[2][2];
{
    double det;

    det = 1.0 / (A[0][0]*A[1][1] - A[1][0]*A[0][1]);

    Ainv[0][0] = det * A[1][1];
    Ainv[1][0] = -1.0 * det * A[1][0];
    Ainv[0][1] = -1.0 * det * A[0][1];
    Ainv[1][1] = det * A[0][0];
}

void transpose2x2(A, At)
    double A[2][2], At[2][2];
{
    At[0][0] = A[0][0];
    At[0][1] = A[1][0];
    At[1][0] = A[0][1];
    At[1][1] = A[1][1];
}

void matcopy2x2(A, B) // A -> B
    double A[2][2], B[2][2];
{
    B[0][0] = A[0][0];
    B[0][1] = A[0][1];
    B[1][0] = A[1][0];
    B[1][1] = A[1][1];
}


