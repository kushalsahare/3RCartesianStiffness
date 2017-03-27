#include<stdio.h>
#include<math.h>

/*
   typedef struct{
   double x;
   double y;
   } point;
   */
#define X 0
#define Y 1
#define EPSILON 0.000001
#define DIMENSION 2

#define DEBUG 1

typedef double point[2];
typedef point rect[4];

typedef struct{
    point c;
    double r;
} circle;

/*typedef struct{
  point v[4];
  } rect;
  */

typedef struct{
    double a;
    double b;
    double c;
} line ; // line defined by a, b and c


double distance(point a, point b)
{
    int i;          /* counter */
    double d=0.0;       /* accumulated distance */

    for (i=0; i<DIMENSION; i++)
        d = d + (a[i]-b[i]) * (a[i]-b[i]);

    return( sqrt(d) );
}

double InvSqrt(double x){

    double xhalf = 0.5f*x;

    int i = *(int*) &x;

    i = 0x5f3759df - (i >> 1);

    x = *(double *) &i;

    x = x*(1.5 - xhalf*x*x);

    return x;

}

double signed_triangle_area(point a, point b, point c)
{
    return( (a[X]*b[Y] - a[Y]*b[X] + a[Y]*c[X]
                - a[X]*c[Y] + b[X]*c[Y] - c[X]*b[Y]) / 2.0 );
}

int collinear(point a, point b, point c)
{
    double signed_triangle_area();

    return (fabs(signed_triangle_area(a,b,c)) <= EPSILON);
}


intersect(circle circ, rect r, double f[2]){


    int i = 0 ;
    int j = 0 ;

    point A, B, E, F, G;

    double R = circ.r; // radius
    point C; //center
    C[X] = circ.c[X];
    C[Y] = circ.c[Y];

    double dx, dy, dr, D, discmt, inv_dr, dt;
    double inv_len; // 1/ L(P1-P2)
    int sign_dy;

    double t; // 0 <= t <=1
    double lAB;
    double lEC;

    double d;
    double Ex, Ey, mag;

    for (i =0;  i < 4 ; i++){

        A[X] = r[i%4][X];
        A[Y]=  r[i%4][Y];

        B[X] = r[(i+1)%4][X];
        B[Y]=  r[(i+1)%4][Y];

        lAB = sqrt( (B[X]-A[X])*(B[X]-A[X]) + (B[Y]-A[Y])*(B[Y]-A[Y]) );

        dx = (B[X] - A[X]) / lAB;

        dy = (B[Y] - A[Y]) / lAB;

        t = dx*(C[X]-A[X]) + dy*(C[Y]-A[Y]);  

        E[X] = t*dx+A[X];
        E[Y] = t*dy+A[Y];

        lEC = sqrt( (E[X]-C[X])*( E[X] -C[X]) +(E[Y]-C[Y])*(E[Y]-C[Y]) );

        if( lEC < R )
        {
            // compute distance from t to circle intersection point
            dt = sqrt( R*R - lEC*lEC);

            d= R - lEC ;
            // compute first intersection point
            F[X] = (t-dt)*dx + A[X];
            F[Y] = (t-dt)*dy + A[Y];

            if(t-dt >0 && t-dt < 1){
            f[X]  += 1000* d * (E[X]-C[X]);
            f[Y]  += 1000* d * (E[Y]-C[Y]);
            }
            printf("First point ( %f, %f) \n", F[X], F[Y]);

            // compute second intersection point
            G[X] = (t+dt)*dx + A[X];
            G[Y] = (t+dt)*dy + A[Y];
            printf("Second point ( %f, %f) \n", G[X], G[Y]);

            if(t+dt > 0 && t+dt < 1){
            f[X]  += 1000* d * (E[X]-C[X]);
            f[Y]  += 1000* d * (E[Y]-C[Y]);
            }

        }

        printf("Force ( %f, %f) \n", f[X], f[Y]);

    }
}

int main(){

    rect r;
    r[0][X] = 1.0;   r[0][Y] = 4.0;

    r[1][X] = 4.0;   r[1][Y] = 4.0;

    r[2][X] = 4.0;   r[2][Y] = 1.0;

    r[3][X] = 1.0;   r[3][Y] = 1.0;


    circle circ;

    circ.r = 2.5;
    circ.c[X] = 15.0;
    circ.c[Y] = 0.75;

    double p[2];

    /***********************************/

    intersect(circ, r, p);
    // printf("First  point ( %f, %f) \n", p[0][X], p[0][Y]);
    // printf("second point ( %f, %f) \n", p[1][X], p[1][Y]);

    return 0;



}
